"""Minimal state machine for one directed stair connector.

The reducer owns the execution order while the ROS node applies its closed
action set through the existing gait, velocity, Nav2 and floor-switch links.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


ACTIVE_STATES = {
    "ENTRY_NAVIGATION",
    "TRAVERSING",
    "PLATFORM_HOLD",
    "EXIT_NAVIGATION",
}
TERMINAL_STATES = {"COMPLETED", "STOPPED", "FAILED"}

NAV_FAILURE_REASONS = {
    "duplicate_floor_goal",
    "floor_mission_active",
    "navigate_action_unavailable",
    "nav_cancel_failed",
    "nav_goal_failed",
    "nav_goal_rejected",
    "nav_goal_request_failed",
    "nav_result_failed",
    "no_current_floor_for_goal",
    "stair_execution_retired",
}
NAV_PRE_ACCEPT_FAILURE_REASONS = {
    "duplicate_floor_goal",
    "floor_mission_active",
    "navigate_action_unavailable",
    "nav_goal_rejected",
    "nav_goal_request_failed",
    "no_current_floor_for_goal",
    "stair_execution_retired",
}


def _text(value: Any) -> str:
    return str(value or "").strip()


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _pose_ready(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    return all(_finite(value.get(key)) is not None for key in ("x", "y", "yaw"))


def _action(kind: str, **payload: Any) -> Dict[str, Any]:
    return {"kind": kind, **payload}


def _failure(
    execution: Dict[str, Any],
    *,
    code: str,
    message: str,
    reason: Optional[str] = None,
) -> Dict[str, Any]:
    updated = dict(execution)
    updated.update(
        {
            "state": "FAILED",
            "status": "failed",
            "failure_code": code,
            "failure_reason": reason or code,
            "status_message": message,
        }
    )
    return {
        "ok": False,
        "code": code,
        "message": message,
        "execution": updated,
        "actions": [_action("stop_motion", reason=reason or code)],
    }


def _route_validation(route: Any) -> Dict[str, Any]:
    if not isinstance(route, dict):
        return {"ok": False, "code": "connector_route_invalid", "message": "楼梯连接边不是对象"}
    required_text = ("id", "source_floor", "target_floor", "source_map_id", "target_map_id")
    if any(not _text(route.get(key)) for key in required_text):
        return {"ok": False, "code": "connector_route_identity_missing", "message": "楼梯连接边缺少地图或楼层身份"}
    if _text(route.get("source_floor")) == _text(route.get("target_floor")):
        return {"ok": False, "code": "connector_route_same_floor", "message": "楼梯连接边不能连接同一楼层"}
    direction = _text(route.get("direction")).lower()
    if direction not in {"up", "down"}:
        return {"ok": False, "code": "connector_route_direction_invalid", "message": "楼梯连接边方向必须为 up 或 down"}
    for key in ("entry", "source_platform", "target_platform", "post_exit"):
        if not _pose_ready(route.get(key)):
            return {"ok": False, "code": "connector_route_pose_missing", "message": f"楼梯连接边缺少 {key} 坐标"}
    return {"ok": True}


def connector_route_activation_decision(route: Any) -> Dict[str, Any]:
    """Validate the geometry needed by the minimal connector loop."""
    return _route_validation(route)


def connector_nav_status_event(
    status_text: Any,
    *,
    expected_goal_seq: Optional[int],
) -> Dict[str, Any]:
    """Translate floor_manager status into one connector navigation event."""
    text = _text(status_text)
    fields: Dict[str, str] = {}
    for token in text.replace(",", " ").split():
        key, separator, value = token.partition("=")
        if separator and key:
            fields[key.strip()] = value.strip()
    if fields.get("label") != "floor_goal":
        return {"action": "ignore", "code": "connector_nav_status_unrelated"}
    reason = fields.get("reason", "")
    try:
        goal_seq = int(fields.get("goal_seq", ""))
    except (TypeError, ValueError):
        if reason in NAV_PRE_ACCEPT_FAILURE_REASONS:
            return {
                "action": "failed",
                "code": reason,
                "goal_seq": None,
            }
        return {"action": "ignore", "code": "connector_nav_status_sequence_missing"}
    if goal_seq <= 0:
        return {"action": "ignore", "code": "connector_nav_status_sequence_invalid"}
    if text.startswith("nav_goal_accepted"):
        if expected_goal_seq is not None and goal_seq != int(expected_goal_seq):
            return {
                "action": "ignore",
                "code": "connector_nav_status_stale",
                "goal_seq": goal_seq,
            }
        return {
            "action": "accepted",
            "code": "connector_nav_goal_accepted",
            "goal_seq": goal_seq,
        }
    if expected_goal_seq is None:
        if reason in NAV_PRE_ACCEPT_FAILURE_REASONS:
            return {
                "action": "failed",
                "code": reason,
                "goal_seq": goal_seq,
            }
        return {
            "action": "ignore",
            "code": "connector_nav_status_sequence_unestablished",
            "goal_seq": goal_seq,
        }
    if goal_seq != int(expected_goal_seq):
        return {
            "action": "ignore",
            "code": "connector_nav_status_stale",
            "goal_seq": goal_seq,
        }
    if text.startswith("nav_goal_succeeded"):
        return {
            "action": "reached",
            "code": "connector_nav_goal_succeeded",
            "goal_seq": goal_seq,
        }
    if (
        reason in NAV_FAILURE_REASONS
        or text.startswith("nav_goal_failed")
        or text.startswith("nav_goal_rejected")
        or text.startswith("nav_goal_cancelled")
    ):
        return {
            "action": "failed",
            "code": reason or "connector_nav_goal_failed",
            "goal_seq": goal_seq,
        }
    return {
        "action": "ignore",
        "code": "connector_nav_status_nonterminal",
        "goal_seq": goal_seq,
    }


def connector_motion_decision(
    *,
    current_pose: Any,
    target_pose: Any,
    pose_age_s: Any,
    pose_timeout_s: float,
    tolerance_m: float,
    speed_mps: float,
    direction: Any,
) -> Dict[str, Any]:
    """Return wait, move, reached or stop for the straight stair segment."""
    target = target_pose if _pose_ready(target_pose) else None
    if target is None:
        return {"action": "stop", "code": "connector_motion_target_invalid"}
    age = _finite(pose_age_s)
    timeout = max(0.1, float(pose_timeout_s))
    if not _pose_ready(current_pose):
        if age is not None and age <= timeout:
            return {"action": "wait", "code": "connector_motion_waiting_pose"}
        return {"action": "stop", "code": "connector_motion_pose_timeout"}
    if age is None or age > timeout:
        return {"action": "stop", "code": "connector_motion_pose_timeout"}
    dx = float(target["x"]) - float(current_pose["x"])
    dy = float(target["y"]) - float(current_pose["y"])
    distance = math.hypot(dx, dy)
    tolerance = max(0.01, float(tolerance_m))
    if distance <= tolerance:
        return {
            "action": "reached",
            "code": "connector_platform_reached",
            "distance_m": distance,
        }
    speed = _finite(speed_mps)
    travel_direction = _text(direction).lower()
    if speed is None or speed <= 0.0 or travel_direction not in {"up", "down"}:
        return {"action": "stop", "code": "connector_motion_config_invalid"}
    return {
        "action": "move",
        "code": "connector_motion_command",
        "distance_m": distance,
        "linear_x": speed if travel_direction == "up" else -speed,
    }


def _positive_epoch(value: Any) -> Optional[int]:
    try:
        epoch = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return epoch if epoch > 0 else None


def create_connector_execution(
    route: Dict[str, Any],
    *,
    request_id: Any,
    plan_id: Any = None,
    map_epoch: Any = None,
    now_monotonic: float,
    stage_timeout_s: float = 180.0,
) -> Dict[str, Any]:
    """Create a single connector execution and its first semantic action."""
    route_check = _route_validation(route)
    request = _text(request_id)
    plan = _text(plan_id)
    epoch = _positive_epoch(map_epoch)
    if not request:
        route_check = {"ok": False, "code": "connector_request_id_missing", "message": "楼梯执行缺少 request_id"}
    elif not plan:
        route_check = {"ok": False, "code": "connector_plan_id_missing", "message": "楼梯执行缺少 plan_id"}
    elif epoch is None:
        route_check = {"ok": False, "code": "connector_map_epoch_invalid", "message": "楼梯执行缺少有效 map_epoch"}
    if not route_check.get("ok"):
        execution = {
            "request_id": request or None,
            "plan_id": plan or None,
            "map_epoch": epoch,
            "route_id": _text(route.get("id")) if isinstance(route, dict) else "",
            "state": "FAILED",
            "status": "failed",
        }
        return {
            **route_check,
            "execution": execution,
            "actions": [_action("stop_motion", reason=str(route_check.get("code")))],
        }
    timeout = max(1.0, float(stage_timeout_s))
    execution = {
        "request_id": request,
        "plan_id": plan,
        "map_epoch": epoch,
        "route_id": _text(route.get("id")),
        "source_floor": _text(route.get("source_floor")),
        "target_floor": _text(route.get("target_floor")),
        "source_map_id": _text(route.get("source_map_id")),
        "target_map_id": _text(route.get("target_map_id")),
        "direction": _text(route.get("direction")).lower(),
        "entry": dict(route.get("entry") or {}),
        "source_platform": dict(route.get("source_platform") or {}),
        "target_platform": dict(route.get("target_platform") or {}),
        "post_exit": dict(route.get("post_exit") or {}),
        "state": "ENTRY_NAVIGATION",
        "status": "running",
        "stage_started_monotonic": float(now_monotonic),
        "stage_timeout_s": timeout,
        "status_message": "导航至楼梯入口",
    }
    return {
        "ok": True,
        "code": "connector_entry_requested",
        "message": execution["status_message"],
        "execution": execution,
        "actions": [
            _action("set_gait", gait="flat"),
            _action(
                "dispatch_entry_goal",
                pose=dict(execution["entry"]),
                map_id=execution["source_map_id"],
            )
        ],
    }


def _event_identity_matches(execution: Dict[str, Any], event: Dict[str, Any]) -> bool:
    for key in ("request_id", "route_id", "plan_id"):
        if not _text(event.get(key)) or _text(event.get(key)) != _text(execution.get(key)):
            return False
    event_epoch = _positive_epoch(event.get("map_epoch"))
    return event_epoch is not None and event_epoch == _positive_epoch(execution.get("map_epoch"))


def step_connector_execution(
    execution: Dict[str, Any],
    event: Dict[str, Any],
    *,
    now_monotonic: float,
) -> Dict[str, Any]:
    """Apply one event; stale/malformed events cannot grant a new action."""
    if not isinstance(execution, dict) or not isinstance(event, dict):
        return {"ok": False, "code": "connector_event_invalid", "message": "楼梯执行事件无效", "execution": execution, "actions": []}
    current = dict(execution)
    state = _text(current.get("state"))
    event_type = _text(event.get("type"))
    if state in TERMINAL_STATES:
        return {"ok": True, "code": "connector_terminal_ignored", "message": "楼梯执行已结束，忽略迟到事件", "execution": current, "actions": []}
    if not _event_identity_matches(current, event):
        if (
            not _text(event.get("request_id"))
            or not _text(event.get("route_id"))
            or not _text(event.get("plan_id"))
            or _positive_epoch(event.get("map_epoch")) is None
        ):
            return {"ok": True, "code": "connector_event_identity_missing", "message": "楼梯执行事件缺少完整身份，已忽略", "execution": current, "actions": []}
        return {"ok": True, "code": "connector_stale_event_ignored", "message": "忽略其他楼梯执行的迟到事件", "execution": current, "actions": []}
    stage_started = _finite(current.get("stage_started_monotonic"))
    elapsed = float(now_monotonic) - (stage_started if stage_started is not None else float(now_monotonic))
    if elapsed > max(1.0, float(current.get("stage_timeout_s") or 180.0)):
        return _failure(current, code="connector_stage_timeout", message="楼梯连接边阶段超时，已停止")
    if event_type in {"stop_requested", "communication_timeout", "navigation_failed"}:
        current.update({"state": "STOPPED", "status": "stopped", "status_message": "楼梯连接边收到停止/安全事件"})
        return {
            "ok": True,
            "code": "connector_stopped",
            "message": current["status_message"],
            "execution": current,
            "actions": [_action("stop_motion", reason=event_type)],
        }

    if state == "ENTRY_NAVIGATION":
        if event_type == "entry_reached":
            gait = "stair_down" if _text(current.get("direction")) == "down" else "stair_up"
            current.update({"state": "TRAVERSING", "stage_started_monotonic": float(now_monotonic), "gait": gait, "status_message": "已到楼梯入口，开始通过楼梯"})
            return {
                "ok": True,
                "code": "connector_traverse_started",
                "message": current["status_message"],
                "execution": current,
                "actions": [
                    _action("set_gait", gait=gait),
                    _action(
                        "start_connector_motion",
                        route_id=current["route_id"],
                        direction=current["direction"],
                        target_pose=dict(current["source_platform"]),
                    ),
                ],
            }
        return {"ok": True, "code": "connector_entry_waiting", "message": "等待楼梯入口确认", "execution": current, "actions": []}

    if state == "TRAVERSING":
        if event_type == "platform_reached":
            current.update({"state": "PLATFORM_HOLD", "stage_started_monotonic": float(now_monotonic), "status_message": "已到共享平台，保持停止并请求切层"})
            return {"ok": True, "code": "connector_floor_switch_requested", "message": current["status_message"], "execution": current, "actions": [_action("stop_motion", reason="platform_reached"), _action("request_floor_switch", request_id=current["request_id"], route_id=current["route_id"], source_floor=current["source_floor"], target_floor=current["target_floor"], target_map_id=current["target_map_id"])]}
        return {"ok": True, "code": "connector_traverse_waiting", "message": "楼梯通过中", "execution": current, "actions": []}

    if state == "PLATFORM_HOLD":
        if event_type != "floor_switch_result":
            return {"ok": True, "code": "connector_waiting_floor_switch", "message": current["status_message"], "execution": current, "actions": []}
        if not bool(event.get("ok")):
            return _failure(current, code="floor_switch_failed", message="切层事务失败，楼梯执行已停止")
        if _text(event.get("target_floor")) != _text(current.get("target_floor")) or _text(event.get("target_map_id")) != _text(current.get("target_map_id")):
            return _failure(current, code="floor_switch_result_mismatch", message="切层回执与楼梯连接边不一致")
        current.update({"state": "EXIT_NAVIGATION", "stage_started_monotonic": float(now_monotonic), "status_message": "目标层地图与定位已确认，导航离开楼梯"})
        return {"ok": True, "code": "connector_exit_requested", "message": current["status_message"], "execution": current, "actions": [_action("dispatch_exit_goal", pose=dict(current["post_exit"]), map_id=current["target_map_id"])]}

    if state == "EXIT_NAVIGATION":
        if event_type != "exit_reached":
            return {"ok": True, "code": "connector_waiting_exit", "message": current["status_message"], "execution": current, "actions": []}
        current.update({"state": "COMPLETED", "status": "completed", "status_message": "楼梯连接边完成，已恢复平地导航"})
        return {
            "ok": True,
            "code": "connector_completed",
            "message": current["status_message"],
            "execution": current,
            "actions": [_action("set_gait", gait="flat")],
        }

    return {"ok": True, "code": "connector_event_waiting", "message": "等待楼梯执行阶段事件", "execution": current, "actions": []}
