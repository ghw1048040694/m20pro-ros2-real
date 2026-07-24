"""Minimal persisted state for one coordinated map switch.

The runtime owns only the ordering and correlation needed to switch maps and
relocalize.  Map assets are prepared before a mission; the robot must not sit
in a transition area while the system computes hashes or rebuilds readiness
proofs.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional


ACTIVE_STATES = {"SWITCHING_MAP", "RELOCALIZING"}
TERMINAL_STATES = {"COMMITTED", "FAILED"}
_ALLOWED_TRANSITIONS = {
    "SWITCHING_MAP": {"RELOCALIZING", "FAILED"},
    "RELOCALIZING": {"COMMITTED", "FAILED"},
}


def _text(value: Any) -> str:
    return str(value or "").strip()


def _error(code: str, message: str, **extra: Any) -> Dict[str, Any]:
    return {"ok": False, "code": code, "message": message, **extra}


def next_map_epoch(settings: Dict[str, Any]) -> int:
    """Allocate a monotonic request generation for rejecting stale replies."""
    try:
        current = int(settings.get("floor_switch_map_epoch", 0) or 0)
    except (TypeError, ValueError, OverflowError):
        current = 0
    return max(0, current) + 1


def recover_interrupted_transaction(
    transaction: Any,
    *,
    now_text: Optional[str] = None,
    now_unix_s: Optional[float] = None,
) -> Dict[str, Any]:
    """Close an interrupted switch as failed so it cannot resume after restart."""
    if not isinstance(transaction, dict):
        return {"changed": False, "transaction": None}
    state = _text(transaction.get("state")).upper()
    if state not in ACTIVE_STATES:
        return {"changed": False, "transaction": dict(transaction)}
    updated = dict(transaction)
    updated.update(
        {
            "state": "FAILED",
            "status": "failed",
            "code": "floor_switch_interrupted_restart",
            "message": "地图切换期间服务重启，任务已停止；请人工确认当前地图并重新定位",
            "updated_at": now_text
            or time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "completed_at_unix": float(
                now_unix_s if now_unix_s is not None else time.time()
            ),
        }
    )
    return {"changed": True, "transaction": updated}


def begin_transaction(
    *,
    request: Dict[str, Any],
    context: Dict[str, Any],
    map_epoch: int,
    now_text: str,
    now_unix_s: Optional[float] = None,
) -> Dict[str, Any]:
    request_id = _text(request.get("request_id"))
    plan_id = _text(request.get("plan_id"))
    task_id = _text(context.get("task_id"))
    route = context.get("route") if isinstance(context.get("route"), dict) else {}
    try:
        epoch = int(map_epoch)
    except (TypeError, ValueError, OverflowError):
        epoch = 0
    if not request_id:
        return _error("floor_switch_request_id_missing", "切图请求缺少 request_id")
    if not plan_id:
        return _error("floor_switch_plan_id_missing", "切图请求缺少 plan_id")
    if epoch <= 0:
        return _error("floor_switch_map_epoch_invalid", "切图请求缺少有效 map_epoch")
    if not task_id:
        return _error("floor_switch_task_id_missing", "切图请求缺少活动任务 ID")
    source_map_id = _text(context.get("source_map_id"))
    target_map_id = _text(context.get("target_map_id"))
    if not source_map_id or not target_map_id:
        return _error("floor_switch_map_identity_missing", "切图请求缺少源/目标地图")
    started_at_unix = float(now_unix_s if now_unix_s is not None else time.time())
    transaction = {
        "request_id": request_id,
        "route_id": _text(route.get("id")),
        "plan_id": plan_id,
        "map_epoch": epoch,
        "task_id": task_id,
        "source_floor": _text(context.get("source_floor")),
        "target_floor": _text(context.get("target_floor")),
        "source_map_id": source_map_id,
        "target_map_id": target_map_id,
        "state": "SWITCHING_MAP",
        "status": "running",
        "code": "floor_switch_started",
        "message": "正在并行激活104和106目标地图",
        "started_at": str(now_text),
        "updated_at": str(now_text),
        "started_at_unix": started_at_unix,
    }
    return {
        "ok": True,
        "code": "floor_switch_started",
        "message": transaction["message"],
        "transaction": transaction,
    }


def advance_transaction(
    transaction: Dict[str, Any],
    state: str,
    *,
    message: str,
    now_text: str,
    now_unix_s: Optional[float] = None,
    code: Optional[str] = None,
    **evidence: Any,
) -> Dict[str, Any]:
    current = _text(transaction.get("state")).upper()
    target = _text(state).upper()
    if target not in _ALLOWED_TRANSITIONS.get(current, set()):
        return _error(
            "floor_switch_phase_invalid",
            f"切图阶段不能从 {current or 'UNKNOWN'} 进入 {target or 'UNKNOWN'}",
        )
    updated = dict(transaction)
    now_unix = float(now_unix_s if now_unix_s is not None else time.time())
    updated.update(
        {
            "state": target,
            "status": "completed" if target == "COMMITTED" else ("failed" if target == "FAILED" else "running"),
            "code": code or ("floor_switch_committed" if target == "COMMITTED" else ("floor_switch_failed" if target == "FAILED" else "floor_switch_relocalizing")),
            "message": str(message),
            "updated_at": str(now_text),
            **evidence,
        }
    )
    if target in TERMINAL_STATES:
        updated["completed_at_unix"] = now_unix
        try:
            updated["duration_s"] = max(
                0.0, now_unix - float(updated.get("started_at_unix") or now_unix)
            )
        except (TypeError, ValueError):
            updated["duration_s"] = None
    return {
        "ok": True,
        "code": updated["code"],
        "message": updated["message"],
        "transaction": updated,
    }


def request_admission(transaction: Any, request_id: Any) -> Dict[str, Any]:
    """Allow one switch at a time; completed or failed attempts never block later runs."""
    requested = _text(request_id)
    if not requested:
        return _error("floor_switch_request_id_missing", "切图请求缺少 request_id")
    if not isinstance(transaction, dict):
        return {"ok": True, "code": "floor_switch_admitted"}
    state = _text(transaction.get("state")).upper()
    if state in ACTIVE_STATES:
        return _error(
            "floor_switch_busy",
            "已有地图切换正在执行",
            active_request_id=transaction.get("request_id"),
            state=state,
        )
    return {"ok": True, "code": "floor_switch_admitted"}


def completion_decision(
    transaction: Dict[str, Any],
    *,
    task_active: bool,
    target_map_id: Any,
    map_activation: Dict[str, Any],
    relocalization: Dict[str, Any],
) -> Dict[str, Any]:
    """Require only the four observable results needed before navigation resumes."""
    if _text(transaction.get("state")).upper() != "RELOCALIZING":
        return _error("floor_switch_phase_invalid", "地图切换尚未进入重定位阶段")
    if not task_active:
        return _error("floor_switch_task_cancelled", "任务已停止")
    if _text(target_map_id) != _text(transaction.get("target_map_id")):
        return _error("floor_switch_target_map_mismatch", "目标地图与当前切图请求不一致")
    if not bool(map_activation.get("ok")):
        return _error("floor_switch_map_failed", "104/106目标地图未全部激活")
    if not bool((map_activation.get("nav2_load_map") or {}).get("ok")):
        return _error("floor_switch_nav2_map_failed", "104目标地图加载失败")
    if not bool((map_activation.get("factory_apply_map") or {}).get("ok")):
        return _error("floor_switch_factory_map_failed", "106目标地图激活失败")
    verification = (
        relocalization.get("verification")
        if isinstance(relocalization.get("verification"), dict)
        else {}
    )
    if not bool(relocalization.get("confirmed")) or not bool(
        verification.get("factory_pose_accepted")
    ):
        return _error(
            "floor_switch_relocalization_failed",
            "目标地图2101重定位或目标图位姿未确认",
        )
    return {
        "ok": True,
        "code": "floor_switch_commit_ready",
        "message": "104/106目标地图和2101目标图位姿已确认",
    }


__all__ = [
    "ACTIVE_STATES",
    "TERMINAL_STATES",
    "advance_transaction",
    "begin_transaction",
    "completion_decision",
    "next_map_epoch",
    "recover_interrupted_transaction",
    "request_admission",
]
