"""Pure active-task state helpers for M20Pro single-floor task execution."""

from __future__ import annotations

import math
import uuid
from typing import Any, Callable, Dict, Optional, Sequence


NowText = Callable[[], str]


GOAL_SENT_RESET_KEYS = (
    "has_nav_feedback",
    "last_nav_status",
    "last_nav_status_at",
    "last_nav_feedback",
    "last_nav_feedback_at",
    "last_nav_feedback_monotonic",
    "last_nav_feedback_recoveries",
    "last_nav_distance_remaining_m",
    "last_nav_goal_match",
    "last_nav_goal_seq",
    "last_ignored_nav_status",
    "last_ignored_nav_goal_match",
    "last_wait_code",
    "last_wait_at",
    "near_goal_started_monotonic",
    "near_goal_started_at",
    "stall_started_monotonic",
    "stall_age_s",
    "stall_warned",
    "last_progress_monotonic",
    "last_progress_at",
    "last_progress_pose",
    "last_progress_distance_m",
    "last_progress_moved_m",
    "last_progress_distance_delta_m",
    "last_nav_accepted_monotonic",
    "last_nav_accepted_at",
    "last_floor_goal_published_at",
    "last_floor_goal_published_monotonic",
    "last_floor_goal_annotation_id",
    "last_floor_goal_label",
    "last_floor_goal_pose",
    "last_floor_goal_source_floor",
    "last_floor_goal_target_floor",
    "last_floor_goal_cross_floor",
    "last_transition_nav_status",
    "last_transition_nav_status_at",
    "last_transition_nav_payload",
    "last_transition_nav_goal_status",
    "last_transition_nav_label",
    "last_transition_nav_monotonic",
    "connector_request_id",
    "connector_route_id",
    "connector_plan_id",
    "connector_map_epoch",
    "connector_state",
    "connector_source_floor",
    "connector_target_floor",
    "connector_started_at",
    "connector_started_monotonic",
    "connector_last_status_at",
    "connector_last_status_monotonic",
    "connector_status_code",
    "plan_goal_verified",
    "plan_goal_error_m",
    "plan_path_version",
)

NEXT_WAYPOINT_RESET_KEYS = GOAL_SENT_RESET_KEYS + (
    "last_goal_attempt_id",
    "last_goal_pose",
    "last_goal_label",
    "goal_sent_path_version",
)

CONNECTOR_ACTIVE_STATES = {
    "PREPARING",
    "ENTRY_NAVIGATION",
    "TRAVERSING",
    "PLATFORM_HOLD",
    "EXIT_NAVIGATION",
}


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def connector_owns_navigation_status(active: Dict[str, Any]) -> bool:
    """Return whether connector internals, not the waypoint runner, own Nav2 status."""
    return bool(
        active.get("status") == "running"
        and str(active.get("connector_request_id") or "").strip()
        and str(active.get("connector_state") or "").strip().upper()
        in CONNECTOR_ACTIVE_STATES
    )


def apply_connector_status_state(
    active: Dict[str, Any],
    status: Any,
    *,
    now_text: str,
    now_monotonic: float,
) -> Dict[str, Any]:
    """Apply only a status belonging to the active connector request."""
    if active.get("status") != "running" or not isinstance(status, dict):
        return {"matched": False, "active": dict(active), "reason": "task_or_status_invalid"}
    request_id = str(status.get("request_id") or "").strip()
    expected_request_id = str(active.get("connector_request_id") or "").strip()
    if not request_id or request_id != expected_request_id:
        return {"matched": False, "active": dict(active), "reason": "connector_request_mismatch"}
    identity_pairs = (
        ("route_id", "connector_route_id"),
        ("plan_id", "connector_plan_id"),
    )
    if any(
        not str(status.get(status_key) or "").strip()
        or str(status.get(status_key) or "").strip()
        != str(active.get(active_key) or "").strip()
        for status_key, active_key in identity_pairs
    ):
        return {"matched": False, "active": dict(active), "reason": "connector_identity_mismatch"}
    try:
        status_epoch = int(status.get("map_epoch"))
        active_epoch = int(active.get("connector_map_epoch"))
    except (TypeError, ValueError, OverflowError):
        return {"matched": False, "active": dict(active), "reason": "connector_identity_mismatch"}
    if status_epoch <= 0 or status_epoch != active_epoch:
        return {"matched": False, "active": dict(active), "reason": "connector_identity_mismatch"}
    state = str(status.get("state") or "").strip().upper()
    if state not in CONNECTOR_ACTIVE_STATES | {"COMPLETED", "FAILED", "STOPPED"}:
        return {"matched": False, "active": dict(active), "reason": "connector_state_invalid"}
    updated = dict(active)
    code = str(status.get("code") or "").strip()
    message = str(status.get("message") or "").strip()
    persistent_changed = any(
        (
            str(updated.get("connector_state") or "") != state,
            str(updated.get("connector_status_code") or "") != code,
            bool(message) and str(updated.get("status_message") or "") != message,
        )
    )
    updated["connector_state"] = state
    updated["connector_status_code"] = code or None
    updated["connector_last_status_at"] = now_text
    updated["connector_last_status_monotonic"] = float(now_monotonic)
    if message:
        updated["status_message"] = message
    return {
        "matched": True,
        "active": updated,
        "state": state,
        "terminal": state in {"COMPLETED", "FAILED", "STOPPED"},
        "failed": state in {"FAILED", "STOPPED"},
        "persistent_changed": persistent_changed,
    }


def waypoint_goal_payload(annotation: Dict[str, Any]) -> Dict[str, Any]:
    pose = annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {}
    floor = str(annotation.get("floor") or "").strip()
    x = _finite_float(pose.get("x"))
    y = _finite_float(pose.get("y"))
    z = _finite_float(pose.get("z", 0.0))
    yaw = _finite_float(pose.get("yaw", 0.0))
    if not floor:
        return {
            "ok": False,
            "message": "当前任务点楼层为空，已停止任务",
            "reason": "bad_waypoint_floor",
            "annotation_id": annotation.get("id"),
            "label": annotation.get("label"),
        }
    if x is None or y is None or z is None or yaw is None:
        return {
            "ok": False,
            "message": "当前任务点坐标无效，已停止任务",
            "reason": "bad_waypoint_pose",
            "annotation_id": annotation.get("id"),
            "label": annotation.get("label"),
            "pose": pose,
        }
    return {
        "ok": True,
        "floor": floor,
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
    }


def waypoint_goal_failure_extra(annotation: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "pose": annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {},
    }


def resolve_runtime_floor_goal(
    goal: Dict[str, Any],
    *,
    current_floor: Any,
    multi_floor: bool,
) -> Dict[str, Any]:
    """Use the runtime floor-manager label for a same-map, single-floor goal."""
    resolved = dict(goal)
    annotation_floor = str(goal.get("floor") or "").strip()
    runtime_floor = str(current_floor or "").strip()
    if not multi_floor and runtime_floor:
        resolved["floor"] = runtime_floor
    target_floor = str(resolved.get("floor") or "").strip()
    return {
        "goal": resolved,
        "annotation_floor": annotation_floor or None,
        "runtime_floor": target_floor or None,
        "floor_overridden": bool(
            annotation_floor and target_floor and annotation_floor != target_floor
        ),
        "multi_floor": bool(multi_floor),
    }


def task_uses_multiple_floors(
    task: Dict[str, Any],
    annotations: Any,
) -> bool:
    if bool(task.get("multi_floor")):
        return True
    floors = {
        str(item.get("floor") or "").strip()
        for item in annotations if isinstance(item, dict)
        if str(item.get("floor") or "").strip()
    }
    return len(floors) > 1


def goal_dispatch_decision(
    active: Dict[str, Any],
    annotation: Optional[Dict[str, Any]],
    *,
    force: bool,
    now_monotonic: float,
    resend_interval_s: float,
) -> Dict[str, Any]:
    if active.get("status") != "running":
        return {"action": "idle", "reason": "task_not_running"}
    if annotation is None:
        return {"action": "idle", "reason": "missing_annotation"}

    annotation_id = annotation.get("id")
    phase = active.get("phase") or "navigating"
    if not force and active.get("last_goal_annotation_id") == annotation_id:
        nav_status = str(active.get("last_nav_goal_status") or "")
        if nav_status in ("accepted", "succeeded"):
            return {"action": "publish_status", "phase": phase, "nav_status": nav_status}
        last_sent = _as_float(active.get("last_goal_sent_monotonic"), 0.0)
        age_s = max(0.0, float(now_monotonic) - last_sent)
        if nav_status != "sent" or age_s < max(1.0, float(resend_interval_s)):
            return {"action": "publish_status", "phase": phase, "nav_status": nav_status, "age_s": age_s}
        resend_payload = {
            "annotation_id": annotation.get("id"),
            "label": annotation.get("label"),
            "last_nav_goal_status": nav_status,
            "age_s": age_s,
        }
        return {
            "action": "send_goal",
            "resend": True,
            "operator_event": "补发当前任务点",
            "operator_payload": resend_payload,
            "resend_event": resend_payload,
        }

    return {"action": "send_goal", "resend": False}


def stale_goal_dispatch_payload(
    active: Dict[str, Any],
    requested_annotation: Dict[str, Any],
    current_annotation: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "event": "goal_dispatch_ignored",
        "message": "任务点已切换，忽略过期目标下发",
        "event_extra": {
            "task_id": active.get("task_id"),
            "requested_annotation_id": requested_annotation.get("id"),
            "requested_label": requested_annotation.get("label"),
            "current_annotation_id": current_annotation.get("id"),
            "current_label": current_annotation.get("label"),
            "index": active.get("index"),
        },
    }


def create_active_task_state(
    task: Dict[str, Any],
    *,
    task_map_id: str,
    now_text: str,
) -> Dict[str, Any]:
    stored_plan = task.get("navigation_plan")
    navigation_plan = dict(stored_plan) if isinstance(stored_plan, dict) else None
    planned_annotation_ids = (
        list(navigation_plan.get("annotation_ids") or [])
        if isinstance(navigation_plan, dict) and navigation_plan.get("ok")
        else list(task.get("annotation_ids") or [])
    )
    active = {
        "task_id": task.get("id"),
        "task_name": task.get("name"),
        # A reusable task definition gets a fresh runtime identity every time
        # it starts, so radar/navigation state cannot leak into the next run.
        "run_id": uuid.uuid4().hex,
        "map_id": task_map_id,
        "multi_floor": bool(task.get("multi_floor")),
        "navigation_plan": navigation_plan,
        "status": "running",
        "index": 0,
        "annotation_ids": planned_annotation_ids,
        "started_at": now_text,
        "last_goal_annotation_id": None,
        "last_goal_sent_monotonic": 0.0,
        "waypoint_started_monotonic": 0.0,
        "total_goal_send_count": 0,
        "waypoint_goal_send_count": 0,
        "resend_goal_count": 0,
        "phase": "navigating",
        "last_nav_goal_status": "idle",
        "status_message": "任务已创建，准备下发第一个点位",
    }
    task_id = active.get("task_id")
    annotation_ids = list(active.get("annotation_ids") or [])
    return {
        "active": active,
        "event": "task_started",
        "message": active["status_message"],
        "event_extra": {"task_id": task_id, "waypoints": len(annotation_ids)},
        "operator_event": "启动前端任务",
        "operator_payload": {"task_id": task_id},
    }


def mark_active_task_stopped_state(
    active: Dict[str, Any],
    *,
    message: str = "任务已手动停止/复位",
) -> Dict[str, Any]:
    updated = dict(active)
    updated["status_message"] = message
    return updated


def normalize_stop_task_request(payload: Dict[str, Any]) -> Dict[str, Any]:
    reason = str(payload.get("reason") or "web_manual_stop").strip() or "web_manual_stop"
    return {
        "reason": reason,
        "is_reset": reason == "web_manual_reset",
    }


def stop_task_state(
    active: Dict[str, Any],
    *,
    reason: str,
    message: str = "任务已手动停止/复位",
) -> Dict[str, Any]:
    updated = mark_active_task_stopped_state(active, message=message)
    operator = stop_task_operator_event_payload(task_id=updated.get("task_id"), reason=reason)
    return {
        "active": updated,
        "task_id": updated.get("task_id"),
        "event": "task_stopped",
        "message": updated.get("status_message") or message,
        "event_extra": {"reason": reason},
        "result_status": "stopped",
        "operator_event": operator["operator_event"],
        "operator_payload": operator["operator_payload"],
    }


def idle_stop_task_response(reason: str = "web_manual_stop") -> Dict[str, Any]:
    is_reset = str(reason or "") == "web_manual_reset"
    return {
        "ok": True,
        "active_task": None,
        "stopped_task_id": None,
        "reset_navigation": True,
        "reason": reason,
        "message": "已显式复位导航状态" if is_reset else "当前没有前端任务在执行，已发送导航取消/复位指令",
    }


def task_terminal_event_payload(
    *,
    event: str,
    task_id: Any = None,
    message: Optional[str] = None,
    reason: Optional[str] = None,
    status_text: Optional[str] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    payload: Dict[str, Any] = {"task_id": task_id}
    if message:
        payload["message"] = message
    if reason:
        payload["reason"] = reason
    if status_text:
        payload["status"] = status_text
    if extra:
        payload.update(extra)
    event_name = {
        "completed": "前端任务完成",
        "stopped": "停止前端任务",
        "failed": "前端任务停止",
        "nav_failed": "前端任务因导航状态停止",
    }.get(str(event or ""), str(event or "前端任务事件"))
    return {
        "event": event_name,
        "payload": payload,
    }


def stop_task_operator_event_payload(*, task_id: Any = None, reason: str) -> Dict[str, Any]:
    terminal = task_terminal_event_payload(event="stopped", task_id=task_id, reason=reason)
    return {
        "operator_event": terminal["event"],
        "operator_payload": terminal["payload"],
    }


def mark_active_task_failed_state(
    active: Dict[str, Any],
    *,
    message: str,
) -> Dict[str, Any]:
    updated = dict(active)
    updated["last_error"] = message
    updated["status_message"] = message
    return updated


def fail_active_task_state(
    active: Dict[str, Any],
    *,
    message: str,
    event_extra: Optional[Dict[str, Any]] = None,
    terminal_event: str = "failed",
    terminal_status_text: Optional[str] = None,
) -> Dict[str, Any]:
    updated = mark_active_task_failed_state(active, message=message)
    terminal_event_name = str(terminal_event or "failed")
    operator = task_terminal_event_payload(
        event=terminal_event_name,
        task_id=updated.get("task_id"),
        message=message if terminal_event_name == "failed" else None,
        status_text=terminal_status_text,
        extra=dict(event_extra or {}) if terminal_event_name == "failed" else None,
    )
    return {
        "active": updated,
        "task_id": updated.get("task_id"),
        "event": "task_failed",
        "message": message,
        "event_extra": dict(event_extra or {}),
        "result_status": "error",
        "operator_event": operator["event"],
        "operator_payload": operator["payload"],
    }


def mark_goal_sent(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    goal: Dict[str, Any],
    *,
    now_text: str,
    now_monotonic: float,
    path_version: Any,
    goal_attempt_id: str,
    goal_semantics: Dict[str, Any],
) -> Dict[str, Any]:
    updated = dict(active)
    annotation_id = annotation.get("id")
    is_new_waypoint = updated.get("last_goal_annotation_id") != annotation_id
    updated["last_goal_annotation_id"] = annotation_id
    updated["last_goal_label"] = annotation.get("label")
    updated["last_goal_sent_at"] = now_text
    updated["last_goal_sent_monotonic"] = now_monotonic

    if is_new_waypoint or _as_float(updated.get("waypoint_started_monotonic"), 0.0) <= 0.0:
        updated["waypoint_started_monotonic"] = now_monotonic
        updated["waypoint_started_at"] = now_text
        updated["waypoint_goal_send_count"] = 0
        for key in GOAL_SENT_RESET_KEYS:
            updated.pop(key, None)

    updated["total_goal_send_count"] = int(updated.get("total_goal_send_count", 0) or 0) + 1
    updated["waypoint_goal_send_count"] = int(updated.get("waypoint_goal_send_count", 0) or 0) + 1
    if not is_new_waypoint:
        updated["resend_goal_count"] = int(updated.get("resend_goal_count", 0) or 0) + 1

    updated["last_goal_attempt_id"] = goal_attempt_id
    updated["last_goal_pose"] = {
        "floor": goal["floor"],
        "x": goal["x"],
        "y": goal["y"],
        "z": goal["z"],
        "yaw": goal["yaw"],
    }
    updated["goal_sent_path_version"] = path_version
    updated["phase"] = "navigating"
    updated["last_nav_goal_status"] = "sent"
    updated["status_message"] = "已下发当前点位，等待 Nav2 接收"
    updated["last_goal_semantics"] = goal_semantics

    event_extra = {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "goal": {
            "floor": annotation.get("floor"),
            "pose": dict(annotation.get("pose") or {}),
        },
        "resend": not is_new_waypoint,
        "goal_attempt_id": goal_attempt_id,
        "goal_sent_path_version": path_version,
        "goal_send_count": updated.get("waypoint_goal_send_count"),
        "total_goal_send_count": updated.get("total_goal_send_count"),
        "resend_goal_count": updated.get("resend_goal_count"),
    }
    return {
        "active": updated,
        "is_new_waypoint": is_new_waypoint,
        "event": "waypoint_goal_sent",
        "message": updated["status_message"],
        "event_extra": event_extra,
    }


def prepare_goal_send_state(
    active: Dict[str, Any],
    requested_annotation: Dict[str, Any],
    current_annotation: Optional[Dict[str, Any]],
    goal: Dict[str, Any],
    *,
    now_text: str,
    now_monotonic: float,
    path_version: Any,
    goal_attempt_id: str,
    goal_semantics: Dict[str, Any],
) -> Dict[str, Any]:
    if active.get("status") != "running" or active.get("task_id") is None:
        return {"action": "idle", "reason": "task_not_running", "active": dict(active)}
    if current_annotation is None:
        return {
            "action": "fail",
            "failure": active_annotation_missing_failure(active),
            "active": dict(active),
        }
    if current_annotation.get("id") != requested_annotation.get("id"):
        stale = stale_goal_dispatch_payload(active, requested_annotation, current_annotation)
        return {
            "action": "record_stale",
            "active": dict(active),
            "event": stale["event"],
            "message": stale["message"],
            "event_extra": dict(stale["event_extra"]),
        }
    mark = mark_goal_sent(
        active,
        requested_annotation,
        goal,
        now_text=now_text,
        now_monotonic=now_monotonic,
        path_version=path_version,
        goal_attempt_id=goal_attempt_id,
        goal_semantics=goal_semantics,
    )
    return {
        "action": "send_goal",
        "active": mark["active"],
        "event": mark["event"],
        "message": mark["message"],
        "event_extra": dict(mark["event_extra"]),
        "is_new_waypoint": mark["is_new_waypoint"],
    }


def mark_floor_goal_published_state(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    goal: Dict[str, Any],
    *,
    now_text: str,
    now_monotonic: float,
    source_floor: Optional[str] = None,
) -> Dict[str, Any]:
    if active.get("status") != "running":
        return {"changed": False, "active": dict(active), "reason": "task_not_running"}
    updated = dict(active)
    updated["last_floor_goal_published_at"] = now_text
    updated["last_floor_goal_published_monotonic"] = float(now_monotonic)
    updated["last_floor_goal_annotation_id"] = annotation.get("id")
    updated["last_floor_goal_label"] = annotation.get("label")
    updated["floor_goal_publish_count"] = int(updated.get("floor_goal_publish_count", 0) or 0) + 1
    target_floor = str(goal.get("floor") or "").strip()
    normalized_source_floor = str(source_floor or "").strip()
    updated["last_floor_goal_source_floor"] = normalized_source_floor or None
    updated["last_floor_goal_target_floor"] = target_floor or None
    updated["last_floor_goal_cross_floor"] = bool(
        normalized_source_floor and target_floor and normalized_source_floor != target_floor
    )
    updated["last_floor_goal_pose"] = {
        "floor": goal["floor"],
        "x": goal["x"],
        "y": goal["y"],
        "z": goal["z"],
        "yaw": goal["yaw"],
    }
    updated["status_message"] = "已发布 /m20pro/floor_goal，等待 floor_manager/Nav2 接收"
    event_extra = {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "goal_attempt_id": updated.get("last_goal_attempt_id"),
        "goal_send_count": updated.get("waypoint_goal_send_count"),
        "total_goal_send_count": updated.get("total_goal_send_count"),
        "floor_goal_publish_count": updated.get("floor_goal_publish_count"),
        "goal": dict(updated["last_floor_goal_pose"]),
        "source_floor": updated.get("last_floor_goal_source_floor"),
        "target_floor": updated.get("last_floor_goal_target_floor"),
        "cross_floor": updated.get("last_floor_goal_cross_floor"),
    }
    return {
        "changed": updated != active,
        "active": updated,
        "event": "floor_goal_published",
        "message": updated["status_message"],
        "event_extra": event_extra,
    }


def mark_connector_started_state(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    goal: Dict[str, Any],
    *,
    transition: Dict[str, Any],
    request_id: str,
    plan_id: Optional[str],
    map_epoch: Any,
    now_text: str,
    now_monotonic: float,
) -> Dict[str, Any]:
    """Record a connector dispatch without pretending it is a floor goal.

    The active task remains the single owner of waypoint order.  Connector
    identity is runtime state only; route geometry continues to come from the
    current validated route registry.
    """
    if active.get("status") != "running":
        return {"changed": False, "active": dict(active), "reason": "task_not_running"}
    connector_request_id = str(request_id or "").strip()
    connector_route_id = str(transition.get("route_id") or "").strip()
    connector_plan_id = str(plan_id or "").strip()
    try:
        connector_epoch = int(map_epoch)
    except (TypeError, ValueError, OverflowError):
        connector_epoch = 0
    if (
        not connector_request_id
        or not connector_route_id
        or not connector_plan_id
        or connector_epoch <= 0
    ):
        return {
            "changed": False,
            "active": dict(active),
            "reason": "connector_identity_invalid",
        }
    updated = dict(active)
    updated["last_goal_annotation_id"] = annotation.get("id")
    updated["last_goal_label"] = annotation.get("label")
    updated["last_goal_sent_at"] = now_text
    updated["last_goal_sent_monotonic"] = float(now_monotonic)
    updated["last_goal_attempt_id"] = connector_request_id
    updated["last_goal_pose"] = {
        "floor": goal.get("floor"),
        "x": goal.get("x"),
        "y": goal.get("y"),
        "z": goal.get("z"),
        "yaw": goal.get("yaw"),
    }
    updated["phase"] = "navigating"
    updated["last_nav_goal_status"] = "sent"
    updated["connector_request_id"] = connector_request_id
    updated["connector_route_id"] = connector_route_id
    updated["connector_plan_id"] = connector_plan_id
    updated["connector_map_epoch"] = connector_epoch
    updated["connector_state"] = "PREPARING"
    updated["connector_source_floor"] = transition.get("source_floor")
    updated["connector_target_floor"] = transition.get("target_floor")
    updated["connector_started_at"] = now_text
    updated["connector_started_monotonic"] = float(now_monotonic)
    updated["last_floor_goal_published_at"] = now_text
    updated["last_floor_goal_published_monotonic"] = float(now_monotonic)
    updated["last_floor_goal_annotation_id"] = annotation.get("id")
    updated["last_floor_goal_label"] = annotation.get("label")
    updated["last_floor_goal_source_floor"] = transition.get("source_floor")
    updated["last_floor_goal_target_floor"] = transition.get("target_floor")
    updated["last_floor_goal_cross_floor"] = True
    updated["last_floor_goal_pose"] = dict(updated["last_goal_pose"])
    updated["status_message"] = "已启动楼梯连接边，正在导航至楼梯入口"
    return {
        "changed": updated != active,
        "active": updated,
        "event": "stair_connector_started",
        "message": updated["status_message"],
        "event_extra": {
            "annotation_id": annotation.get("id"),
            "request_id": connector_request_id,
            "route_id": connector_route_id,
            "plan_id": connector_plan_id,
            "map_epoch": connector_epoch,
            "source_floor": transition.get("source_floor"),
            "target_floor": transition.get("target_floor"),
        },
    }


def dwell_tick_decision(active: Dict[str, Any], *, now_time: float) -> Dict[str, Any]:
    if active.get("phase") != "dwelling":
        return {"action": "not_dwelling"}
    until = _as_float(active.get("dwell_until"), 0.0)
    if float(now_time) < until:
        return {
            "action": "wait",
            "remaining_s": max(0.0, until - float(now_time)),
        }
    return {"action": "advance", "remaining_s": 0.0}


def remaining_dwell_s(active: Dict[str, Any], *, now_time: float) -> float:
    if active.get("phase") != "dwelling":
        return 0.0
    try:
        return max(0.0, float(active.get("dwell_until", 0.0)) - float(now_time))
    except (TypeError, ValueError):
        return 0.0


def active_waypoint_elapsed_s(active: Dict[str, Any], *, now_monotonic: float) -> Optional[float]:
    try:
        started = float(active.get("waypoint_started_monotonic", 0.0) or 0.0)
    except (TypeError, ValueError):
        return None
    if started <= 0.0:
        return None
    return max(0.0, float(now_monotonic) - started)


def append_active_task_timeline_event_state(
    active: Dict[str, Any],
    *,
    event: str,
    message: str,
    now_text: str,
    now_monotonic: float,
    max_events: int,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    updated = dict(active)
    timeline = list(updated.get("timeline") or [])
    item: Dict[str, Any] = {
        "event": event,
        "message": message,
        "time": now_text,
        "index": int(updated.get("index", 0) or 0),
        "phase": updated.get("phase"),
        "nav_goal_status": updated.get("last_nav_goal_status"),
    }
    elapsed_s = active_waypoint_elapsed_s(updated, now_monotonic=now_monotonic)
    if elapsed_s is not None:
        item["elapsed_s"] = elapsed_s
    if extra:
        item.update(extra)
    limit = max(1, int(max_events))
    timeline.append(item)
    updated["timeline"] = timeline[-limit:]
    updated["last_timeline_event"] = item
    return updated


def active_annotation_missing_failure(active: Dict[str, Any]) -> Dict[str, Any]:
    resolution = active_annotation_resolution(active)
    annotation_ids = resolution["annotation_ids"]
    index = resolution["index"]
    annotation_id = resolution["annotation_id"]
    return {
        "action": "fail",
        "reason": "active_waypoint_missing",
        "message": "当前任务点位已删除或索引越界，已停止任务；请重新生成任务",
        "task_id": active.get("task_id"),
        "task_name": active.get("task_name"),
        "index": index,
        "annotation_id": annotation_id,
        "annotation_ids": annotation_ids,
    }


def active_annotation_resolution(active: Dict[str, Any]) -> Dict[str, Any]:
    annotation_ids = [str(item) for item in (active.get("annotation_ids") or [])]
    try:
        index = int(active.get("index", 0) or 0)
    except (TypeError, ValueError):
        index = 0
    annotation_id = annotation_ids[index] if 0 <= index < len(annotation_ids) else None
    return {
        "ok": annotation_id is not None,
        "index": index,
        "annotation_id": annotation_id,
        "annotation_ids": annotation_ids,
    }


def active_annotation_from_list(
    active: Dict[str, Any],
    annotations: Sequence[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    resolution = active_annotation_resolution(active)
    annotation_id = resolution.get("annotation_id")
    if not annotation_id:
        return None
    for annotation in annotations:
        if str(annotation.get("id") or "") == annotation_id:
            return dict(annotation)
    return None


def active_task_failure_payload(
    failure: Dict[str, Any],
    *,
    default_message: Optional[str] = None,
    task_id: Any = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    payload_extra = dict(failure)
    message = str(payload_extra.pop("message", "") or default_message or "任务执行失败，已停止任务")
    payload_extra.pop("action", None)
    if extra:
        payload_extra.update(extra)
    return {
        "task_id": task_id if task_id is not None else failure.get("task_id"),
        "message": message,
        "extra": payload_extra,
    }


def mark_active_task_waiting_state(
    active: Dict[str, Any],
    *,
    code: str,
    message: str,
    now_text: str,
) -> Dict[str, Any]:
    updated = dict(active)
    previous_wait_code = updated.get("last_wait_code")
    previous_message = updated.get("status_message")
    updated["last_wait_code"] = code
    updated["status_message"] = message
    updated["last_wait_at"] = now_text
    should_record_event = (
        previous_wait_code in (None, "")
        or str(previous_wait_code or "") != str(code)
        or str(previous_message or "") != str(message)
    )
    return {
        "active": updated,
        "changed": updated != active,
        "should_record_event": should_record_event,
        "event": "task_waiting",
        "message": message,
        "event_extra": {
            "code": code,
        },
    }


def begin_waypoint_dwell_state(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    *,
    dwell_s: float,
    now_text: str,
    now_time: float,
    reason: str,
) -> Dict[str, Any]:
    if active.get("status") != "running":
        return {"changed": False, "active": dict(active), "reason": "task_not_running"}
    updated = dict(active)
    dwell = max(0.0, float(dwell_s))
    if dwell <= 0.0:
        return {"changed": False, "active": updated, "reason": "zero_dwell"}
    updated["phase"] = "dwelling"
    updated["dwell_s"] = dwell
    updated["dwell_until"] = float(now_time) + dwell
    updated["last_reached_at"] = now_text
    updated["last_reached_annotation_id"] = annotation.get("id")
    updated["last_reached_reason"] = reason
    updated["status_message"] = "已到达点位，正在停留 %.1f 秒" % dwell
    event_extra = {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "dwell_s": dwell,
        "reason": reason,
    }
    return {
        "changed": True,
        "active": updated,
        "event": "waypoint_dwell_started",
        "message": updated["status_message"],
        "event_extra": event_extra,
        "operator_event": "到达点位并开始停留",
        "operator_payload": event_extra,
    }


def advance_active_task_state(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    *,
    now_text: str,
) -> Dict[str, Any]:
    if active.get("status") != "running":
        return {"changed": False, "active": dict(active), "completed": False}

    updated = dict(active)
    updated["index"] = int(updated.get("index", 0)) + 1
    updated["phase"] = "navigating"
    updated["dwell_s"] = 0.0
    updated["dwell_until"] = None
    updated["last_reached_at"] = now_text
    updated["last_reached_annotation_id"] = annotation.get("id")

    if updated["index"] >= len(updated.get("annotation_ids") or []):
        updated["status"] = "completed"
        updated["status_message"] = "任务已完成"
        operator = task_terminal_event_payload(event="completed", task_id=updated.get("task_id"))
        return {
            "changed": True,
            "active": updated,
            "completed": True,
            "task_id": updated.get("task_id"),
            "event": "task_completed",
            "message": "任务已完成",
            "event_extra": {"last_annotation_id": annotation.get("id"), "label": annotation.get("label")},
            "result_status": "completed",
            "operator_event": operator["event"],
            "operator_payload": operator["payload"],
        }

    updated["last_goal_annotation_id"] = None
    updated["last_goal_sent_monotonic"] = 0.0
    updated["waypoint_started_monotonic"] = 0.0
    updated["waypoint_goal_send_count"] = 0
    updated["last_nav_goal_status"] = "idle"
    for key in NEXT_WAYPOINT_RESET_KEYS:
        updated.pop(key, None)
    updated["status_message"] = "准备下发下一个点位"
    return {
        "changed": True,
        "active": updated,
        "completed": False,
        "event": "waypoint_advanced",
        "message": "准备下发下一个点位",
        "event_extra": {
            "last_annotation_id": annotation.get("id"),
            "label": annotation.get("label"),
            "next_index": updated["index"],
        },
    }
