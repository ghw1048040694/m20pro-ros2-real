"""Pure contract for the unified-navigation floor-switch transaction.

The Web node performs ROS and SSH side effects; this module owns the state
machine decisions and the evidence required to commit or roll back a switch.
Keeping these rules pure prevents a second set of ad-hoc conditions from
appearing in the background worker.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional


ACTIVE_STATES = {"PREPARED", "APPLYING", "RELOCALIZING", "ROLLING_BACK"}
TERMINAL_STATES = {"COMMITTED", "ROLLED_BACK", "RECOVERED", "FAILED", "UNCERTAIN"}
RECOVERABLE_STATES = ACTIVE_STATES | {"FAILED"}


def _text(value: Any) -> str:
    return str(value or "").strip()


def _error(code: str, message: str, **extra: Any) -> Dict[str, Any]:
    return {"ok": False, "code": code, "message": message, **extra}


def _positive_float(value: Any) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return 0.0
    return parsed if parsed > 0.0 else 0.0


def next_map_epoch(settings: Dict[str, Any]) -> int:
    """Allocate a monotonic transaction epoch from persisted settings."""
    try:
        current = int(settings.get("floor_switch_map_epoch", 0) or 0)
    except (TypeError, ValueError):
        current = 0
    return max(0, current) + 1


def recover_interrupted_transaction(
    transaction: Any,
    *,
    now_text: Optional[str] = None,
    now_unix_s: Optional[float] = None,
) -> Dict[str, Any]:
    """Convert an unfinished transaction found after restart to UNCERTAIN."""
    if not isinstance(transaction, dict):
        return {"changed": False, "transaction": None}
    state = _text(transaction.get("state"))
    if state == "UNCERTAIN" and not _positive_float(transaction.get("uncertain_at_unix")):
        updated = dict(transaction)
        updated.update(
            {
                "updated_at": now_text or time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                "uncertain_at_unix": float(now_unix_s if now_unix_s is not None else time.time()),
            }
        )
        return {"changed": True, "transaction": updated}
    if state not in RECOVERABLE_STATES:
        return {"changed": False, "transaction": dict(transaction)}
    updated = dict(transaction)
    updated.update(
        {
            "state": "UNCERTAIN",
            "status": "uncertain",
            "code": "floor_switch_interrupted_restart",
            "message": "进程在跨楼层地图事务中重启，物理地图和楼层状态不可自动推断",
            "updated_at": now_text or time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "uncertain_at_unix": float(now_unix_s if now_unix_s is not None else time.time()),
        }
    )
    return {"changed": True, "transaction": updated}


def begin_transaction(
    *,
    request: Dict[str, Any],
    context: Dict[str, Any],
    source_map_digest: Optional[str],
    target_map_digest: Optional[str],
    map_epoch: int,
    now_text: str,
    source_factory_identity: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    request_id = _text(request.get("request_id"))
    if not request_id:
        return _error("floor_switch_request_id_missing", "切层请求缺少 request_id")
    plan_id = _text(request.get("plan_id"))
    if not plan_id:
        return _error("floor_switch_plan_id_missing", "切层请求缺少 plan_id")
    try:
        normalized_epoch = int(map_epoch)
    except (TypeError, ValueError, OverflowError):
        normalized_epoch = 0
    if normalized_epoch <= 0:
        return _error("floor_switch_map_epoch_invalid", "切层请求缺少有效 map_epoch")
    task_id = _text(context.get("task_id"))
    if not task_id:
        return _error("floor_switch_task_id_missing", "切层事务缺少活动任务 ID")
    source_map_id = _text(context.get("source_map_id"))
    target_map_id = _text(context.get("target_map_id"))
    if not source_map_id or not target_map_id:
        return _error("floor_switch_map_identity_missing", "切层事务缺少源/目标地图身份")
    transaction = {
        "request_id": request_id,
        "plan_id": plan_id,
        "map_epoch": normalized_epoch,
        "task_id": task_id,
        "route_id": _text(context.get("route", {}).get("id")) if isinstance(context.get("route"), dict) else "",
        "source_floor": _text(context.get("source_floor")),
        "target_floor": _text(context.get("target_floor")),
        "source_map_id": source_map_id,
        "target_map_id": target_map_id,
        "source_platform": dict(context.get("route", {}).get("source_platform") or {})
        if isinstance(context.get("route"), dict)
        else {},
        "target_platform": dict(context.get("route", {}).get("target_platform") or {})
        if isinstance(context.get("route"), dict)
        else {},
        "source_map_digest": source_map_digest or None,
        "target_map_digest": target_map_digest or None,
        "source_factory_active_path": _text(
            (source_factory_identity or {}).get("resolved_path")
        ) or None,
        "source_factory_active_digest": _text(
            (source_factory_identity or {}).get("content_digest")
        ) or None,
        "source_factory_identity_mode": _text(
            (source_factory_identity or {}).get("identity_mode")
        ) or None,
        "state": "PREPARED",
        "status": "running",
        "message": "跨楼层事务已准备，等待目标地图切换",
        "created_at": now_text,
        "updated_at": now_text,
    }
    return {"ok": True, "transaction": transaction}


def mark_uncertain_transaction(
    transaction: Dict[str, Any],
    *,
    message: str,
    now_text: str,
    now_unix_s: Optional[float] = None,
    **evidence: Any,
) -> Dict[str, Any]:
    """Persist an emergency fail-closed state without a second ad-hoc path.

    A floor switch can fail after an external side effect has started.  In
    that case ``FAILED`` is not an adequate statement about the physical
    robot: the active map, pose, or Nav2 lifecycle may be unknown.  This
    helper is deliberately the only supported way to record that condition.
    A committed or successfully rolled-back transaction is immutable and is
    never rewritten by a late exception.
    """
    if not isinstance(transaction, dict):
        return _error("floor_switch_transaction_missing", "切层事务不存在，无法记录不确定状态")
    current = _text(transaction.get("state"))
    if current in {"COMMITTED", "ROLLED_BACK", "RECOVERED"}:
        return _error(
            "floor_switch_transaction_terminal",
            "切层事务已安全结束，不能改写为不确定状态",
            state=current,
        )
    if current == "UNCERTAIN":
        updated = dict(transaction)
        updated.update({"message": message, "updated_at": now_text})
        if not _positive_float(updated.get("uncertain_at_unix")):
            updated["uncertain_at_unix"] = float(
                now_unix_s if now_unix_s is not None else time.time()
            )
        updated.update(evidence)
        return {"ok": True, "transaction": updated, "already_uncertain": True}
    if current not in ACTIVE_STATES and current != "FAILED":
        return _error(
            "floor_switch_transaction_state_invalid",
            "切层事务当前状态无法安全收口",
            state=current,
        )
    updated = dict(transaction)
    updated.update(
        {
            "state": "UNCERTAIN",
            "status": "uncertain",
            "code": "floor_switch_state_uncertain",
            "message": message,
            "updated_at": now_text,
            "uncertain_at_unix": float(now_unix_s if now_unix_s is not None else time.time()),
        }
    )
    updated.update(evidence)
    return {"ok": True, "transaction": updated}


def recover_uncertain_transaction(
    transaction: Dict[str, Any],
    *,
    map_id: Any,
    expected_map_digest: Any,
    observed_map_digest: Any,
    factory_active_confirmed: bool,
    localization_status: Dict[str, Any],
    navigation_readiness: Dict[str, Any],
    relocalization_time: Optional[float],
    now_text: str,
) -> Dict[str, Any]:
    """Resolve ``UNCERTAIN`` only from a fully re-proven fixed-map state."""
    if not isinstance(transaction, dict) or _text(transaction.get("state")) != "UNCERTAIN":
        return _error(
            "floor_switch_recovery_state_invalid",
            "只有 UNCERTAIN 跨层事务需要人工恢复",
            state=_text((transaction or {}).get("state")) if isinstance(transaction, dict) else None,
        )
    selected_map_id = _text(map_id)
    if not selected_map_id:
        return _error("floor_switch_recovery_map_missing", "人工恢复必须选中一张固定地图")
    expected_digest = _text(expected_map_digest).lower()
    observed_digest = _text(observed_map_digest).lower()
    if not expected_digest or observed_digest != expected_digest:
        return _error(
            "floor_switch_recovery_map_mismatch",
            "104 当前 /map 内容与人工确认地图不一致",
        )
    if not factory_active_confirmed:
        return _error(
            "floor_switch_recovery_factory_unconfirmed",
            "106 active 地图尚未确认，不能解除不确定状态",
        )
    localization = dict(localization_status or {})
    navigation = dict(navigation_readiness or {})
    if not bool(localization.get("confirmed")) or localization.get("map_relocalization_required"):
        return _error(
            "floor_switch_recovery_localization_unconfirmed",
            "必须在人工确认地图上重新完成 2101 定位",
        )
    relocalized_at = _positive_float(relocalization_time)
    uncertain_at = _positive_float(transaction.get("uncertain_at_unix"))
    if uncertain_at <= 0.0:
        return _error(
            "floor_switch_recovery_barrier_missing",
            "UNCERTAIN 事务缺少时间屏障，必须重新启动服务后再执行新的 2101",
        )
    if relocalized_at <= 0.0 or relocalized_at < uncertain_at:
        return _error(
            "floor_switch_recovery_relocalization_stale",
            "人工恢复需要 UNCERTAIN 之后的新 2101 成功证据",
            uncertain_at_unix=uncertain_at or None,
            relocalization_time=relocalized_at or None,
        )
    if not bool(navigation.get("ready")):
        return _error(
            "floor_switch_recovery_navigation_not_ready",
            "人工恢复后的 /scan、代价地图或 Nav2 尚未就绪",
            navigation_readiness=navigation,
        )
    updated = dict(transaction)
    updated.update(
        {
            "state": "RECOVERED",
            "status": "completed",
            "code": "floor_switch_recovered_manually",
            "message": "人工确认地图、2101 定位和 Nav2 全链路后已解除不确定状态",
            "recovered_map_id": selected_map_id,
            "recovered_at": now_text,
            "updated_at": now_text,
            "recovery": {
                "expected_map_digest": expected_digest,
                "observed_map_digest": observed_digest,
                "factory_active_confirmed": True,
                "relocalization_time": relocalized_at,
                "navigation_readiness": navigation,
            },
        }
    )
    return {"ok": True, "transaction": updated}


def advance_transaction(
    transaction: Dict[str, Any],
    state: str,
    *,
    message: str,
    now_text: str,
    **evidence: Any,
) -> Dict[str, Any]:
    """Apply one legal phase update to a persisted transaction."""
    current = _text(transaction.get("state"))
    target = _text(state)
    legal = {
        "PREPARED": {"APPLYING", "FAILED", "ROLLING_BACK"},
        "APPLYING": {"RELOCALIZING", "ROLLING_BACK", "FAILED"},
        "RELOCALIZING": {"COMMITTED", "ROLLING_BACK", "FAILED"},
        "ROLLING_BACK": {"ROLLED_BACK", "UNCERTAIN", "FAILED"},
    }
    if current in TERMINAL_STATES:
        return _error("floor_switch_transaction_terminal", "切层事务已结束，不能再次推进", state=current)
    if target not in legal.get(current, set()):
        return _error(
            "floor_switch_transaction_transition_invalid",
            f"切层事务不能从 {current} 推进到 {target}",
            state=current,
            requested_state=target,
        )
    updated = dict(transaction)
    updated.update(
        {
            "state": target,
            "status": "completed" if target in {"COMMITTED", "ROLLED_BACK"} else ("uncertain" if target == "UNCERTAIN" else "running"),
            "message": message,
            "updated_at": now_text,
        }
    )
    updated.update(evidence)
    return {"ok": True, "transaction": updated}


def request_admission(transaction: Any, request_id: Any) -> Dict[str, Any]:
    """Reject concurrent/replayed requests using persisted transaction state."""
    request = _text(request_id)
    if not isinstance(transaction, dict):
        return {"ok": True, "mode": "new"}
    state = _text(transaction.get("state"))
    existing = _text(transaction.get("request_id"))
    if state in ACTIVE_STATES:
        return _error("floor_switch_busy", "已有跨楼层事务正在执行", active_request_id=existing or None)
    if state in {"UNCERTAIN", "FAILED"}:
        return _error(
            "floor_switch_recovery_required",
            "上一次跨楼层事务未完成，必须先人工确认地图并重定位",
            active_request_id=existing or None,
            map_epoch=transaction.get("map_epoch"),
        )
    if existing and existing == request and state in TERMINAL_STATES:
        return _error("floor_switch_request_replayed", "已忽略已结束切层请求的重放", state=state)
    return {"ok": True, "mode": "new"}


def commit_decision(
    transaction: Dict[str, Any],
    *,
    task_active: bool,
    target_map_id: Any,
    observed_map_digest: Any,
    factory_active_confirmed: bool,
    relocalization: Dict[str, Any],
    navigation_readiness: Dict[str, Any],
    factory_active_identity: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Return commit evidence; no single subsystem can commit alone."""
    if _text(transaction.get("state")) != "RELOCALIZING":
        return _error("floor_switch_commit_phase_invalid", "切层事务尚未进入目标层重定位阶段")
    if not task_active:
        return _error("floor_switch_task_cancelled", "跨楼层任务已停止")
    if _text(target_map_id) != _text(transaction.get("target_map_id")):
        return _error("floor_switch_target_map_mismatch", "目标层地图身份与事务不一致")
    if not factory_active_confirmed:
        return _error("floor_switch_factory_map_unconfirmed", "106 active 地图未完成后验确认")
    if factory_active_identity is not None:
        # Web confirmation uses explicit expected/active names so an operator
        # can see both sides of the comparison; pure callers may provide the
        # canonical content_digest directly.  Normalize both forms here so
        # the execution path and the contract cannot disagree on field names.
        direct_digest = _text(factory_active_identity.get("content_digest")).lower()
        active_digest = _text(factory_active_identity.get("active_content_digest")).lower()
        expected_identity_digest = _text(
            factory_active_identity.get("expected_content_digest")
        ).lower()
        actual_digest = direct_digest or active_digest
        if not actual_digest:
            return _error(
                "floor_switch_factory_identity_missing",
                "106 active 地图内容摘要缺失，不能提交切层事务",
            )
        if expected_identity_digest and actual_digest != expected_identity_digest:
            return _error(
                "floor_switch_factory_identity_mismatch",
                "106 active 地图内容摘要与事务确认摘要不一致，不能提交切层事务",
                expected_content_digest=expected_identity_digest,
                active_content_digest=actual_digest,
            )
    expected_digest = _text(transaction.get("target_map_digest"))
    if not expected_digest or _text(observed_map_digest) != expected_digest:
        return _error("floor_switch_nav2_map_content_mismatch", "104 /map 内容摘要与目标地图不一致")
    if not bool(relocalization.get("confirmed")) or not bool(relocalization.get("navigation_ready")):
        return _error(
            "floor_switch_target_navigation_not_ready",
            "目标层必须同时通过 2101 重定位和 Nav2 全链路就绪检查",
            relocalization=dict(relocalization or {}),
            navigation_readiness=dict(navigation_readiness or {}),
        )
    if not bool(navigation_readiness.get("ready")):
        return _error("floor_switch_navigation_not_ready", "目标层 Nav2、/scan 或代价地图尚未就绪", navigation_readiness=navigation_readiness)
    return {
        "ok": True,
        "code": "floor_switch_commit_ready",
        "message": "目标层地图内容、106 active、2101 重定位和 Nav2 全链路均已确认",
    }


def rollback_decision(
    transaction: Dict[str, Any],
    *,
    source_map_id: Any,
    observed_map_digest: Any,
    factory_active_confirmed: bool,
    relocalization: Dict[str, Any],
    navigation_readiness: Dict[str, Any],
) -> Dict[str, Any]:
    """Only a fully re-localized source map is a successful rollback."""
    if _text(transaction.get("state")) != "ROLLING_BACK":
        return _error("floor_switch_rollback_phase_invalid", "切层事务尚未进入回滚阶段")
    if _text(source_map_id) != _text(transaction.get("source_map_id")):
        return _error("floor_switch_rollback_map_mismatch", "回滚后的源地图身份不一致")
    expected_digest = _text(transaction.get("source_map_digest"))
    if not expected_digest or _text(observed_map_digest) != expected_digest:
        return _error("floor_switch_rollback_map_content_mismatch", "回滚后的源地图内容摘要不一致")
    if not factory_active_confirmed:
        return _error("floor_switch_rollback_factory_unconfirmed", "回滚后的 106 active 地图未确认")
    if not bool(relocalization.get("confirmed")) or not bool(relocalization.get("navigation_ready")):
        return _error("floor_switch_rollback_navigation_not_ready", "回滚后源层重定位或 Nav2 尚未恢复")
    if not bool(navigation_readiness.get("ready")):
        return _error("floor_switch_rollback_nav_not_ready", "回滚后源层导航链路尚未恢复")
    return {"ok": True, "code": "floor_switch_rolled_back", "message": "源地图、源层重定位和导航链路已完整恢复"}
