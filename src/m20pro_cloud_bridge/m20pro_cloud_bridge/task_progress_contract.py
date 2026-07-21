"""Pure task progress and timeout helpers for active waypoint execution."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _optional_float(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def task_start_localization_gate_decision(
    *,
    localization_ok: Any,
    pose: Dict[str, Any],
    pose_age: Optional[float],
    pose_timeout_s: float,
    map_relocalization_required: Any = None,
) -> Dict[str, Any]:
    """Decide whether a task may create its first navigation goal.

    This is deliberately stricter than the runtime tick gate.  A task must
    never be created and then wait for localization: doing so lets a forced
    first dispatch publish a goal before the normal timer has observed the
    failed localization state.
    """
    if map_relocalization_required:
        return {
            "action": "reject",
            "code": "map_relocalization_required",
            "message": "当前地图仍要求重定位，未启动任务；请先完成重定位并确认定位成功",
        }
    if localization_ok is not True:
        return {
            "action": "reject",
            "code": "localization_not_confirmed",
            "message": "当前定位未确认，未启动任务；请先完成重定位并确认定位成功",
        }
    if not isinstance(pose, dict):
        return {
            "action": "reject",
            "code": "pose_missing_or_invalid",
            "message": "当前地图位姿不可用，未启动任务；请先等待位姿恢复",
        }
    try:
        values = [float(pose[key]) for key in ("x", "y", "z", "yaw")]
    except (KeyError, TypeError, ValueError):
        return {
            "action": "reject",
            "code": "pose_missing_or_invalid",
            "message": "当前地图位姿不可用，未启动任务；请先等待位姿恢复",
        }
    if not all(math.isfinite(value) for value in values):
        return {
            "action": "reject",
            "code": "pose_missing_or_invalid",
            "message": "当前地图位姿不是有效数值，未启动任务；请先等待位姿恢复",
        }
    timeout_s = max(0.5, float(pose_timeout_s))
    try:
        age_value = None if pose_age is None else float(pose_age)
    except (TypeError, ValueError):
        age_value = None
    if age_value is None or not math.isfinite(age_value) or age_value > timeout_s:
        age_text = "未知" if age_value is None else "%.1f" % age_value
        return {
            "action": "reject",
            "code": "pose_stale",
            "message": "当前地图位姿已过期（%s 秒），未启动任务；请先等待位姿恢复" % age_text,
            "pose_age_s": age_value,
            "pose_timeout_s": timeout_s,
        }
    return {
        "action": "pass",
        "code": "task_start_ready",
        "message": "任务启动定位门禁通过",
        "pose_age_s": age_value,
        "pose_timeout_s": timeout_s,
    }


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def update_active_task_progress_state(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    pose: Dict[str, Any],
    *,
    distance: float,
    navigation_status: Any,
    now_monotonic: float,
    now_text: str,
    goal_tolerance_m: float,
    min_pose_movement_m: float,
    min_distance_delta_m: float,
    min_yaw_delta_rad: float = 0.10,
) -> Dict[str, Any]:
    updated = dict(active)
    last_progress_pose = (
        dict(updated.get("last_progress_pose"))
        if isinstance(updated.get("last_progress_pose"), dict)
        else {}
    )
    last_progress_distance = updated.get("last_progress_distance_m")
    progress_reference_time = _as_float(updated.get("last_progress_monotonic"), 0.0)
    has_progress_reference = bool(progress_reference_time > 0.0 and last_progress_pose)

    moved_m = None
    yaw_delta_rad = None
    distance_delta_m = None
    if has_progress_reference:
        try:
            moved_m = math.hypot(
                float(pose.get("x", 0.0)) - float(last_progress_pose.get("x", 0.0)),
                float(pose.get("y", 0.0)) - float(last_progress_pose.get("y", 0.0)),
            )
        except (TypeError, ValueError):
            moved_m = None
        try:
            yaw_delta_rad = abs(
                _wrap_angle(float(pose.get("yaw", 0.0)) - float(last_progress_pose.get("yaw", 0.0)))
            )
        except (TypeError, ValueError):
            yaw_delta_rad = None
        try:
            distance_delta_m = float(last_progress_distance) - float(distance)
        except (TypeError, ValueError):
            distance_delta_m = None

    made_progress = (
        not has_progress_reference
        or (moved_m is not None and moved_m >= min_pose_movement_m)
        or (yaw_delta_rad is not None and yaw_delta_rad >= min_yaw_delta_rad)
        or (distance_delta_m is not None and distance_delta_m >= min_distance_delta_m)
    )
    if made_progress:
        updated["last_progress_monotonic"] = now_monotonic
        updated["last_progress_at"] = now_text
        updated["last_progress_pose"] = {
            "x": pose.get("x"),
            "y": pose.get("y"),
            "yaw": pose.get("yaw"),
        }
        updated["last_progress_distance_m"] = float(distance)
        updated["stall_started_monotonic"] = 0.0
        for key in (
            "stall_age_s",
            "stall_warned",
            "last_progress_moved_m",
            "last_progress_yaw_delta_rad",
            "last_progress_distance_delta_m",
        ):
            updated.pop(key, None)
    else:
        if not updated.get("stall_started_monotonic"):
            updated["stall_started_monotonic"] = progress_reference_time or now_monotonic
        stall_started = _as_float(updated.get("stall_started_monotonic"), now_monotonic)
        updated["stall_age_s"] = max(0.0, now_monotonic - stall_started)
        updated["last_progress_moved_m"] = moved_m
        updated["last_progress_yaw_delta_rad"] = yaw_delta_rad
        updated["last_progress_distance_delta_m"] = distance_delta_m

    updated["last_distance_m"] = float(distance)
    updated["last_robot_pose"] = {
        "x": pose.get("x"),
        "y": pose.get("y"),
        "yaw": pose.get("yaw"),
    }
    updated["last_pose_update_at"] = now_text
    updated["last_navigation_status"] = navigation_status
    if updated.get("last_goal_annotation_id") != annotation.get("id"):
        updated["status_message"] = "准备下发当前点位"
    elif float(distance) <= float(goal_tolerance_m):
        updated["status_message"] = "已接近目标点，等待 Nav2 到达确认"
    else:
        updated["status_message"] = "正在前往当前点位，距离 %.2f m" % float(distance)

    return {
        "active": updated,
        "made_progress": made_progress,
        "moved_m": moved_m,
        "yaw_delta_rad": yaw_delta_rad,
        "distance_delta_m": distance_delta_m,
    }


def active_task_tick_gate_decision(
    *,
    active: Optional[Dict[str, Any]] = None,
    pose: Dict[str, Any],
    annotation: Dict[str, Any],
    current_floor: Any,
    localization_ok: Any,
    pose_age: Optional[float],
    pose_timeout_s: float,
) -> Dict[str, Any]:
    if not pose:
        return {
            "action": "wait_and_monitor_localization",
            "code": "waiting_pose",
            "reason": "no_pose",
            "message": "任务执行中未收到地图位姿；如果持续超过几秒，请先停止任务并重新定位",
        }
    if localization_ok is False:
        return {
            "action": "wait_and_monitor_localization",
            "code": "localization_lost",
            "reason": "localization_lost",
            "message": "任务执行中定位状态变为异常；已暂停下发新目标",
        }
    if pose_age is None or float(pose_age) > float(pose_timeout_s):
        age_text = pose_age if pose_age is not None else -1.0
        return {
            "action": "wait_and_monitor_localization",
            "code": "pose_stale",
            "reason": "pose_stale",
            "message": "任务执行中地图位姿已过期 %.1f 秒；已暂停下发新目标" % age_text,
            "pose_age_s": pose_age,
            "pose_timeout_s": pose_timeout_s,
        }
    target_floor = annotation.get("floor")
    if current_floor and target_floor and current_floor != target_floor:
        active_state = dict(active or {})
        if active is not None and active_state.get("last_goal_annotation_id") != annotation.get("id"):
            return {
                "action": "pass_cross_floor",
                "code": "cross_floor_dispatch",
                "reason": "target_floor_differs",
                "message": "当前点位在 %s，先下发跨楼层目标给 floor_manager" % target_floor,
                "current_floor": current_floor,
                "target_floor": target_floor,
            }
        return {
            "action": "wait",
            "code": "cross_floor_transitioning",
            "reason": "waiting_floor_switch",
            "message": "跨楼层目标已下发，等待从 %s 切换到 %s" % (current_floor, target_floor),
            "current_floor": current_floor,
            "target_floor": target_floor,
        }
    return {"action": "pass", "reason": "ready"}


def active_task_distance_decision(
    *,
    pose: Dict[str, Any],
    annotation: Dict[str, Any],
) -> Dict[str, Any]:
    target = annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {}
    try:
        robot_x = float(pose.get("x"))
        robot_y = float(pose.get("y"))
    except (TypeError, ValueError):
        return {
            "action": "wait_and_monitor_localization",
            "code": "pose_invalid",
            "reason": "pose_invalid",
            "message": "任务执行中机器人地图位姿无效；已暂停下发新目标",
            "pose": pose,
        }
    try:
        goal_x = float(target.get("x"))
        goal_y = float(target.get("y"))
    except (TypeError, ValueError):
        return {
            "action": "fail",
            "reason": "active_waypoint_pose_invalid",
            "message": "当前任务点坐标无效，已停止任务；请重新标点并生成任务",
            "annotation_id": annotation.get("id"),
            "label": annotation.get("label"),
            "pose": target,
        }
    if not all(math.isfinite(value) for value in (robot_x, robot_y, goal_x, goal_y)):
        return {
            "action": "wait_and_monitor_localization",
            "code": "pose_invalid",
            "reason": "pose_invalid",
            "message": "任务执行中机器人或目标地图位姿不是有限数；已暂停下发新目标",
            "pose": pose,
            "target_pose": target,
        }
    return {
        "action": "pass",
        "reason": "distance_ready",
        "distance_m": math.hypot(robot_x - goal_x, robot_y - goal_y),
    }


def active_task_pre_dispatch_decision(
    *,
    active: Optional[Dict[str, Any]] = None,
    pose: Dict[str, Any],
    annotation: Dict[str, Any],
    current_floor: Any,
    localization_ok: Any,
    pose_age: Optional[float],
    pose_timeout_s: float,
) -> Dict[str, Any]:
    """Combine the pre-dispatch tick gate and distance decision.

    Web code should not own the ordering between localization/floor checks and
    active-waypoint distance checks. That ordering is part of task execution
    semantics.
    """

    gate = active_task_tick_gate_decision(
        active=active,
        pose=pose,
        annotation=annotation,
        current_floor=current_floor,
        localization_ok=localization_ok,
        pose_age=pose_age,
        pose_timeout_s=pose_timeout_s,
    )
    if gate.get("action") == "pass_cross_floor":
        return {**gate, "stage": "tick_gate"}
    if gate.get("action") != "pass":
        return {**gate, "stage": "tick_gate"}

    distance = active_task_distance_decision(pose=pose, annotation=annotation)
    if distance.get("action") != "pass":
        return {**distance, "stage": "distance"}

    return {
        "action": "pass",
        "reason": "pre_dispatch_ready",
        "stage": "ready",
        "distance_m": float(distance.get("distance_m")),
    }


def task_stall_decision(
    active: Dict[str, Any],
    *,
    distance: float,
    now_monotonic: float,
    warn_timeout_s: float,
    stop_timeout_s: float,
) -> Dict[str, Any]:
    nav_status = str(active.get("last_nav_goal_status") or "")
    if nav_status not in ("accepted", "sent"):
        return {"action": "pass", "reason": "nav_goal_not_active"}
    stall_started = _as_float(active.get("stall_started_monotonic"), 0.0)
    if stall_started <= 0.0:
        return {"action": "pass", "reason": "not_stalled"}
    stall_age = max(0.0, float(now_monotonic) - stall_started)
    base = {
        "distance_m": float(distance),
        "stall_age_s": stall_age,
        "last_nav_status": active.get("last_nav_status"),
        "last_nav_goal_status": active.get("last_nav_goal_status"),
        "last_nav_feedback": active.get("last_nav_feedback"),
        "last_robot_pose": active.get("last_robot_pose"),
        "last_progress_pose": active.get("last_progress_pose"),
        "last_progress_distance_m": active.get("last_progress_distance_m"),
        "last_progress_moved_m": active.get("last_progress_moved_m"),
        "last_progress_yaw_delta_rad": active.get("last_progress_yaw_delta_rad"),
        "last_progress_distance_delta_m": active.get("last_progress_distance_delta_m"),
    }
    if stall_age >= float(stop_timeout_s):
        return {
            "action": "fail",
            "reason": "waypoint_stalled",
            "message": "当前点位 %.0f 秒内没有有效位移/距离收敛，已停止任务；最后距离 %.2f m"
            % (stall_age, float(distance)),
            **base,
        }
    if stall_age >= float(warn_timeout_s) and not active.get("stall_warned"):
        return {
            "action": "warn",
            "reason": "waypoint_stall_warning",
            "message": "当前点位 %.0f 秒内进展很小，继续观察；若持续到 %.0f 秒会停止任务"
            % (stall_age, float(stop_timeout_s)),
            **base,
        }
    return {"action": "pass", "reason": "stall_under_threshold", **base}


def apply_stall_warning_state(
    active: Dict[str, Any],
    decision: Dict[str, Any],
) -> Dict[str, Any]:
    updated = dict(active)
    updated["stall_warned"] = True
    updated["status_message"] = str(decision.get("message") or "当前点位进展过慢，继续观察")
    return updated


def stall_warning_event_payload(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    decision: Dict[str, Any],
) -> Dict[str, Any]:
    """Build timeline and operator-event payloads for a stall warning."""

    event_extra = {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "distance_m": decision.get("distance_m"),
        "stall_age_s": decision.get("stall_age_s"),
        "last_nav_status": active.get("last_nav_status"),
        "last_nav_feedback": active.get("last_nav_feedback"),
    }
    operator_payload = {
        "task_id": active.get("task_id"),
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "distance_m": decision.get("distance_m"),
        "stall_age_s": decision.get("stall_age_s"),
    }
    return {
        "timeline_event": "waypoint_stall_warning",
        "timeline_message": str(active.get("status_message") or decision.get("message") or "当前点位进展过慢"),
        "timeline_extra": event_extra,
        "operator_event": "任务点位进展过慢",
        "operator_payload": operator_payload,
    }


def stall_failure_extra(annotation: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
    }


def localization_lost_timeout_decision(
    active: Dict[str, Any],
    *,
    reason: str,
    now_monotonic: float,
    timeout_s: float,
) -> Dict[str, Any]:
    started = _as_float(active.get("localization_lost_started_monotonic"), 0.0)
    if started <= 0.0:
        return {
            "action": "start_timer",
            "reason": reason,
            "started_monotonic": float(now_monotonic),
            "timeout_s": float(timeout_s),
            "message": "定位/位姿暂时丢失，%.1f 秒内未恢复将停止任务" % float(timeout_s),
        }
    age_s = max(0.0, float(now_monotonic) - started)
    if age_s < float(timeout_s):
        return {
            "action": "wait",
            "reason": reason,
            "age_s": age_s,
            "timeout_s": float(timeout_s),
        }
    return {
        "action": "fail",
        "reason": reason,
        "age_s": age_s,
        "timeout_s": float(timeout_s),
        "message": "任务执行中定位/位姿丢失超过 %.1f 秒，已停止任务" % float(timeout_s),
    }


def apply_localization_lost_start_state(
    active: Dict[str, Any],
    decision: Dict[str, Any],
    *,
    fallback_monotonic: float,
) -> Dict[str, Any]:
    updated = dict(active)
    updated["localization_lost_started_monotonic"] = float(
        decision.get("started_monotonic") or fallback_monotonic
    )
    updated["status_message"] = str(decision.get("message") or "定位/位姿暂时丢失")
    return {
        "active": updated,
        "changed": updated != active,
    }


def localization_lost_start_event_payload(
    active: Dict[str, Any],
    decision: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "event": "localization_lost_waiting",
        "message": str(
            active.get("status_message")
            or decision.get("message")
            or "定位/位姿暂时丢失"
        ),
        "extra": {
            "reason": decision.get("reason"),
            "timeout_s": decision.get("timeout_s"),
            "started_monotonic": active.get("localization_lost_started_monotonic"),
        },
    }


def localization_lost_failure_extra(decision: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "localization_lost_age_s": decision.get("age_s"),
    }


def goal_accept_timeout_decision(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    *,
    now_monotonic: float,
    timeout_s: float,
) -> Dict[str, Any]:
    if active.get("last_goal_annotation_id") != annotation.get("id"):
        return {"action": "pass", "reason": "different_goal"}
    if str(active.get("last_nav_goal_status") or "") != "sent":
        return {"action": "pass", "reason": "nav_goal_not_sent"}
    first_sent = _as_float(
        active.get("waypoint_started_monotonic")
        or active.get("last_goal_sent_monotonic")
        or 0.0,
        0.0,
    )
    age_s = None if first_sent <= 0.0 else max(0.0, float(now_monotonic) - first_sent)
    if first_sent <= 0.0 or age_s is None or age_s < float(timeout_s):
        return {
            "action": "pass",
            "reason": "goal_accept_under_timeout",
            "age_s": age_s,
            "timeout_s": float(timeout_s),
        }
    return {
        "action": "fail",
        "reason": "goal_accept_timeout",
        "message": "当前点位下发 %.1f 秒后 Nav2 仍未接收，已停止任务；请检查 Nav2 lifecycle、floor_manager 和 /m20pro/floor_goal"
        % float(timeout_s),
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "last_nav_status": active.get("last_nav_status"),
        "age_s": age_s,
        "timeout_s": float(timeout_s),
    }


def near_goal_wait_decision(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    *,
    distance: float,
    goal_tolerance_m: float,
    now_monotonic: float,
    now_text: str,
) -> Dict[str, Any]:
    current_goal_sent = active.get("last_goal_annotation_id") == annotation.get("id")
    current_nav_status = str(active.get("last_nav_goal_status") or "")
    base = {
        "distance_m": float(distance),
        "goal_tolerance_m": float(goal_tolerance_m),
        "current_goal_sent": current_goal_sent,
        "last_nav_goal_status": current_nav_status,
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
    }
    if float(distance) > float(goal_tolerance_m):
        return {"action": "dispatch_goal", "reason": "not_near_goal", **base}
    if not current_goal_sent:
        return {"action": "dispatch_goal", "reason": "current_goal_not_sent", **base}
    if current_nav_status not in ("sent", "accepted"):
        return {"action": "dispatch_goal", "reason": "nav_goal_not_active", **base}

    updated = dict(active)
    started = _as_float(updated.get("near_goal_started_monotonic"), 0.0)
    changed = False
    if started <= 0.0:
        updated["near_goal_started_monotonic"] = float(now_monotonic)
        updated["near_goal_started_at"] = now_text
        changed = True
    return {
        "action": "wait_for_nav2",
        "reason": "near_goal_waiting_nav2",
        "message": "已接近目标点，等待 Nav2 返回到达确认",
        "active": updated,
        "changed": changed,
        **base,
    }


def apply_near_goal_wait_state(active: Dict[str, Any], decision: Dict[str, Any]) -> Dict[str, Any]:
    updated = dict(active)
    if decision.get("action") != "wait_for_nav2":
        return {"active": updated, "changed": False}
    source = decision.get("active") if isinstance(decision.get("active"), dict) else {}
    changed = False
    for key in ("near_goal_started_monotonic", "near_goal_started_at"):
        if key in source and updated.get(key) != source.get(key):
            updated[key] = source.get(key)
            changed = True
    return {"active": updated, "changed": changed}


def prepare_near_goal_wait_update(
    current: Dict[str, Any],
    expected_active: Dict[str, Any],
    decision: Dict[str, Any],
) -> Dict[str, Any]:
    if current.get("status") != "running":
        return {"action": "ignore", "reason": "task_not_running", "active": dict(current)}
    if current.get("task_id") != expected_active.get("task_id"):
        return {"action": "ignore", "reason": "task_changed", "active": dict(current)}
    result = apply_near_goal_wait_state(current, decision)
    if not result.get("changed"):
        return {"action": "no_change", "reason": "near_goal_wait_unchanged", "active": result["active"]}
    return {"action": "update", "reason": "near_goal_wait_updated", "active": result["active"]}


def waypoint_timeout_decision(
    active: Dict[str, Any],
    *,
    distance: float,
    now_monotonic: float,
    timeout_s: float,
) -> Dict[str, Any]:
    started = _as_float(active.get("waypoint_started_monotonic"), 0.0)
    age_s = None if started <= 0.0 else max(0.0, float(now_monotonic) - started)
    if started <= 0.0 or age_s is None or age_s < float(timeout_s):
        return {"action": "pass", "reason": "waypoint_under_timeout", "age_s": age_s, "timeout_s": timeout_s}
    return {
        "action": "fail",
        "reason": "waypoint_timeout",
        "message": "当前点位执行超过 %.0f 秒仍未完成，已停止任务；最后距离 %.2f m"
        % (float(timeout_s), float(distance)),
        "distance_m": float(distance),
        "age_s": age_s,
        "timeout_s": timeout_s,
        "last_nav_status": active.get("last_nav_status"),
        "last_nav_goal_status": active.get("last_nav_goal_status"),
    }


def near_goal_timeout_decision(
    active: Dict[str, Any],
    *,
    distance: float,
    now_monotonic: float,
    goal_tolerance_m: float,
    timeout_s: float,
) -> Dict[str, Any]:
    if float(distance) > float(goal_tolerance_m):
        return {"action": "pass", "reason": "not_near_goal"}
    if str(active.get("last_nav_goal_status") or "") not in ("accepted", "sent"):
        return {"action": "pass", "reason": "nav_goal_not_active"}
    started = _as_float(active.get("near_goal_started_monotonic"), 0.0)
    age_s = None if started <= 0.0 else max(0.0, float(now_monotonic) - started)
    if started <= 0.0 or age_s is None or age_s < float(timeout_s):
        return {"action": "pass", "reason": "near_goal_under_timeout", "age_s": age_s, "timeout_s": timeout_s}
    return {
        "action": "fail",
        "reason": "near_goal_no_nav2_result",
        "message": "机器人已进入目标容差 %.1f 秒但 Nav2 未返回到达，已停止任务；请检查目标朝向/代价地图/goal_checker"
        % float(timeout_s),
        "distance_m": float(distance),
        "age_s": age_s,
        "timeout_s": timeout_s,
        "last_nav_status": active.get("last_nav_status"),
        "last_nav_goal_status": active.get("last_nav_goal_status"),
        "last_nav_feedback": active.get("last_nav_feedback"),
    }


def timeout_failure_extra(annotation: Dict[str, Any], decision: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "reason": decision.get("reason"),
        "distance_m": decision.get("distance_m"),
        "age_s": decision.get("age_s"),
        "timeout_s": decision.get("timeout_s"),
    }
