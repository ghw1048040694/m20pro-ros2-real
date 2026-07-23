"""Fail-closed state machine contract for one certified stair connector.

The reducer emits semantic actions only.  It never publishes ``cmd_vel``,
gait commands, map changes, or raw point clouds; the ROS adapter routes its
request/status actions through the 106 terrain topic and existing command
arbiter/floor-switch transaction.  The default route profile is
shadow/stop-only and is rejected before any stair motion is possible.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


ACTIVE_STATES = {
    "PREPARING",
    "ENTRY_NAVIGATION",
    "TRAVERSING",
    "PLATFORM_HOLD",
    "EXIT_NAVIGATION",
}
TERMINAL_STATES = {"COMPLETED", "STOPPED", "FAILED"}


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


def _profile(route: Dict[str, Any]) -> Dict[str, Any]:
    raw = route.get("terrain_guard") if isinstance(route.get("terrain_guard"), dict) else {}
    profile_id = _text(raw.get("profile_id")) or f"{_text(route.get('id'))}:terrain"
    corridor_version = _text(raw.get("corridor_version")) or "shadow-v1"
    policy = _text(raw.get("motion_policy")) or "stop_only"
    return {
        "profile_id": profile_id,
        "corridor_version": corridor_version,
        "motion_policy": policy,
        "certified_motion": bool(raw.get("certified_motion", False))
        and policy == "certified_connector",
        "corridor": dict(raw.get("corridor") or {}) if isinstance(raw.get("corridor"), dict) else None,
    }


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
        "actions": [_action("stop", reason=reason or code)],
    }


def _route_validation(route: Any) -> Dict[str, Any]:
    if not isinstance(route, dict):
        return {"ok": False, "code": "connector_route_invalid", "message": "楼梯连接边不是对象"}
    required_text = ("id", "source_floor", "target_floor", "source_map_id", "target_map_id")
    if any(not _text(route.get(key)) for key in required_text):
        return {"ok": False, "code": "connector_route_identity_missing", "message": "楼梯连接边缺少地图或楼层身份"}
    if _text(route.get("source_floor")) == _text(route.get("target_floor")):
        return {"ok": False, "code": "connector_route_same_floor", "message": "楼梯连接边不能连接同一楼层"}
    for key in ("entry", "source_platform", "target_platform", "post_exit"):
        if not _pose_ready(route.get(key)):
            return {"ok": False, "code": "connector_route_pose_missing", "message": f"楼梯连接边缺少 {key} 坐标"}
    profile = _profile(route)
    if not profile["certified_motion"]:
        return {
            "ok": False,
            "code": "stair_execution_retired",
            "message": "楼梯连接边尚未完成现场认证，保持 stop-only",
            "terrain_guard": profile,
        }
    if not isinstance(profile.get("corridor"), dict) or not profile["corridor"].get("width_m") or not profile["corridor"].get("lookahead_m"):
        return {
            "ok": False,
            "code": "connector_terrain_profile_missing",
            "message": "认证楼梯连接边缺少已标定的三维走廊几何",
            "terrain_guard": profile,
        }
    return {"ok": True, "profile": profile}


def create_connector_execution(
    route: Dict[str, Any],
    *,
    request_id: Any,
    now_monotonic: float,
    stage_timeout_s: float = 180.0,
) -> Dict[str, Any]:
    """Create a single connector execution and its first semantic action."""
    route_check = _route_validation(route)
    request = _text(request_id)
    if not request:
        route_check = {"ok": False, "code": "connector_request_id_missing", "message": "楼梯执行缺少 request_id"}
    if not route_check.get("ok"):
        execution = {
            "request_id": request or None,
            "route_id": _text(route.get("id")) if isinstance(route, dict) else "",
            "state": "FAILED",
            "status": "failed",
        }
        return {
            **route_check,
            "execution": execution,
            "actions": [_action("stop", reason=str(route_check.get("code")))],
        }
    timeout = max(1.0, float(stage_timeout_s))
    profile = dict(route_check["profile"])
    execution = {
        "request_id": request,
        "route_id": _text(route.get("id")),
        "source_floor": _text(route.get("source_floor")),
        "target_floor": _text(route.get("target_floor")),
        "source_map_id": _text(route.get("source_map_id")),
        "target_map_id": _text(route.get("target_map_id")),
        "direction": _text(route.get("direction")) or "up",
        "entry": dict(route.get("entry") or {}),
        "post_exit": dict(route.get("post_exit") or {}),
        "terrain_guard": profile,
        "state": "PREPARING",
        "status": "running",
        "stage_started_monotonic": float(now_monotonic),
        "stage_timeout_s": timeout,
        "status_message": "等待楼梯 terrain_guard 新鲜可通行证据",
    }
    return {
        "ok": True,
        "code": "connector_prepared",
        "message": execution["status_message"],
        "execution": execution,
        "actions": [
            _action(
                "request_terrain_guard",
                request_id=request,
                route_id=execution["route_id"],
                profile_id=profile["profile_id"],
                corridor_version=profile["corridor_version"],
                direction=execution["direction"],
                corridor=profile.get("corridor"),
            )
        ],
    }


def _event_request_matches(execution: Dict[str, Any], event: Dict[str, Any]) -> bool:
    event_request = _text(event.get("request_id"))
    return bool(event_request) and event_request == _text(execution.get("request_id"))


def _terrain_ready(execution: Dict[str, Any], status: Any, now_monotonic: float) -> Dict[str, Any]:
    if not isinstance(status, dict):
        return {"ok": False, "code": "terrain_guard_status_missing", "message": "缺少楼梯 terrain_guard 状态"}
    profile = execution.get("terrain_guard") if isinstance(execution.get("terrain_guard"), dict) else {}
    status_request_id = _text(status.get("request_id"))
    if not status_request_id:
        return {"ok": False, "code": "terrain_guard_request_id_missing", "message": "楼梯 terrain_guard 状态缺少请求身份"}
    if status_request_id != _text(execution.get("request_id")):
        return {"ok": False, "code": "terrain_guard_request_mismatch", "message": "楼梯 terrain_guard 状态不是当前请求"}
    status_profile = _text(status.get("profile_id"))
    if not status_profile:
        status_profile = f"{_text(status.get('route_id'))}:terrain"
    if status_profile != _text(profile.get("profile_id")) or _text(status.get("corridor_version")) != _text(profile.get("corridor_version")):
        return {"ok": False, "code": "terrain_guard_profile_mismatch", "message": "楼梯 terrain_guard profile 不匹配"}
    if _text(status.get("state")).lower() != "traversable":
        return {"ok": False, "code": "terrain_guard_not_traversable", "message": "楼梯 terrain_guard 未确认可通行"}
    age = _finite(status.get("age_s"))
    if age is None:
        age = _finite(status.get("cloud_age_s"))
    if age is None or age > 1.0:
        return {"ok": False, "code": "terrain_guard_status_stale", "message": "楼梯 terrain_guard 状态过期"}
    if bool(profile.get("certified_motion")) and not bool(status.get("certified_motion")):
        return {"ok": False, "code": "terrain_guard_not_certified", "message": "楼梯 terrain_guard 尚未授予认证运动证据"}
    return {"ok": True, "code": "terrain_guard_ready", "message": "楼梯 terrain_guard 可通行"}


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
    if not _event_request_matches(current, event):
        if not _text(event.get("request_id")):
            return {"ok": True, "code": "connector_event_request_missing", "message": "楼梯执行事件缺少请求身份，已忽略", "execution": current, "actions": []}
        return {"ok": True, "code": "connector_stale_event_ignored", "message": "忽略其他楼梯执行的迟到事件", "execution": current, "actions": []}
    stage_started = _finite(current.get("stage_started_monotonic"))
    elapsed = float(now_monotonic) - (stage_started if stage_started is not None else float(now_monotonic))
    if elapsed > max(1.0, float(current.get("stage_timeout_s") or 180.0)):
        return _failure(current, code="connector_stage_timeout", message="楼梯连接边阶段超时，已停止")
    if event_type in {"stop_requested", "arbiter_lost", "localization_lost"}:
        current.update({"state": "STOPPED", "status": "stopped", "status_message": "楼梯连接边收到停止/安全事件"})
        return {"ok": True, "code": "connector_stopped", "message": current["status_message"], "execution": current, "actions": [_action("stop", reason=event_type)]}

    if state == "PREPARING":
        if event_type != "terrain_status":
            return {"ok": True, "code": "connector_waiting_terrain", "message": current["status_message"], "execution": current, "actions": []}
        terrain = _terrain_ready(current, event.get("status"), now_monotonic)
        if not terrain.get("ok"):
            raw_status = event.get("status")
            raw_state = _text(raw_status.get("state")).lower() if isinstance(raw_status, dict) else ""
            if raw_state in {"unknown", "stale"} and terrain.get("code") == "terrain_guard_not_traversable":
                current["status_message"] = "等待楼梯 terrain_guard 新鲜可通行证据"
                return {"ok": True, "code": "connector_waiting_terrain", "message": current["status_message"], "execution": current, "actions": []}
            return _failure(current, code=str(terrain["code"]), message=str(terrain["message"]))
        current.update({"state": "ENTRY_NAVIGATION", "stage_started_monotonic": float(now_monotonic), "status_message": "terrain_guard 通过，导航至楼梯入口"})
        return {"ok": True, "code": "connector_entry_requested", "message": current["status_message"], "execution": current, "actions": [_action("dispatch_entry_goal", pose=dict(current["entry"]), map_id=current["source_map_id"])]}

    if state == "ENTRY_NAVIGATION":
        if event_type == "entry_reached":
            gait = "stair_down" if _text(current.get("direction")) == "down" else "stair_up"
            current.update({"state": "TRAVERSING", "stage_started_monotonic": float(now_monotonic), "gait": gait, "status_message": "已到楼梯入口，开始沿认证走廊通过"})
            return {"ok": True, "code": "connector_traverse_started", "message": current["status_message"], "execution": current, "actions": [_action("set_gait", gait=gait), _action("start_connector_motion", route_id=current["route_id"])]}
        if event_type == "terrain_status":
            terrain = _terrain_ready(current, event.get("status"), now_monotonic)
            if not terrain.get("ok"):
                return _failure(current, code=str(terrain["code"]), message=str(terrain["message"]))
        return {"ok": True, "code": "connector_entry_waiting", "message": "等待楼梯入口确认", "execution": current, "actions": []}

    if state == "TRAVERSING":
        if event_type == "terrain_status":
            terrain = _terrain_ready(current, event.get("status"), now_monotonic)
            if not terrain.get("ok"):
                return _failure(current, code=str(terrain["code"]), message=str(terrain["message"]))
            return {"ok": True, "code": "connector_traverse_hold", "message": "楼梯 terrain_guard 持续通过", "execution": current, "actions": []}
        if event_type == "platform_reached":
            terrain = _terrain_ready(current, event.get("terrain_status"), now_monotonic)
            if not terrain.get("ok"):
                return _failure(current, code=str(terrain["code"]), message=str(terrain["message"]))
            current.update({"state": "PLATFORM_HOLD", "stage_started_monotonic": float(now_monotonic), "status_message": "已到共享平台，保持停止并请求切层"})
            return {"ok": True, "code": "connector_floor_switch_requested", "message": current["status_message"], "execution": current, "actions": [_action("stop", reason="platform_reached"), _action("request_floor_switch", request_id=current["request_id"], route_id=current["route_id"], source_floor=current["source_floor"], target_floor=current["target_floor"], target_map_id=current["target_map_id"])]}
        return {"ok": True, "code": "connector_traverse_waiting", "message": "沿楼梯认证走廊通过中", "execution": current, "actions": []}

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
        if event_type == "terrain_status":
            terrain = _terrain_ready(current, event.get("status"), now_monotonic)
            if not terrain.get("ok"):
                return _failure(current, code=str(terrain["code"]), message=str(terrain["message"]))
            return {"ok": True, "code": "connector_exit_hold", "message": "目标层楼梯 terrain_guard 持续通过", "execution": current, "actions": []}
        if event_type != "exit_reached":
            return {"ok": True, "code": "connector_waiting_exit", "message": current["status_message"], "execution": current, "actions": []}
        current.update({"state": "COMPLETED", "status": "completed", "status_message": "楼梯连接边完成，已恢复平地导航"})
        return {"ok": True, "code": "connector_completed", "message": current["status_message"], "execution": current, "actions": [_action("set_gait", gait="flat"), _action("resume_flat_navigation")]}

    return {"ok": True, "code": "connector_event_waiting", "message": "等待楼梯执行阶段事件", "execution": current, "actions": []}
