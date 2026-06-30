"""Pure task plan-verification helpers for single-floor task execution."""

from __future__ import annotations

import math
from typing import Any, Dict


def _as_int(value: Any) -> int:
    return int(value)


def _goal_pose(annotation: Dict[str, Any]) -> Dict[str, Any]:
    return annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {}


def path_goal_error_m(path_last_point: Dict[str, Any], annotation: Dict[str, Any]) -> Any:
    pose = _goal_pose(annotation)
    try:
        return math.hypot(
            float(path_last_point.get("x")) - float(pose.get("x")),
            float(path_last_point.get("y")) - float(pose.get("y")),
        )
    except (TypeError, ValueError):
        return None


def task_plan_match_decision(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    path: Dict[str, Any],
    *,
    required: bool,
    now_monotonic: float,
    timeout_s: float,
    tolerance_m: float,
) -> Dict[str, Any]:
    if not required:
        return {"action": "pass", "reason": "not_required"}
    if active.get("last_goal_annotation_id") != annotation.get("id"):
        return {"action": "pass", "reason": "annotation_not_current"}
    if str(active.get("last_nav_goal_status") or "") != "accepted":
        return {"action": "pass", "reason": "nav_goal_not_accepted"}

    sent_version = active.get("goal_sent_path_version")
    if sent_version is None:
        return {"action": "pass", "reason": "missing_sent_path_version"}

    path_version = path.get("version")
    path_last_point = path.get("last_point") if isinstance(path.get("last_point"), dict) else {}
    try:
        has_new_path = _as_int(path_version) > _as_int(sent_version)
    except (TypeError, ValueError):
        has_new_path = False

    base = {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "path_version": path_version,
        "goal_sent_path_version": sent_version,
        "path_last_point": path_last_point,
        "last_goal_pose": active.get("last_goal_pose"),
        "last_nav_status": active.get("last_nav_status"),
        "last_nav_goal_status": active.get("last_nav_goal_status"),
        "last_nav_feedback": active.get("last_nav_feedback"),
    }

    if has_new_path:
        error_m = path_goal_error_m(path_last_point, annotation)
        if error_m is not None and error_m <= tolerance_m:
            return {
                "action": "verify",
                "reason": "path_goal_verified",
                "path_goal_error_m": float(error_m),
                "tolerance_m": tolerance_m,
                **base,
            }
        return {
            "action": "fail",
            "reason": "path_goal_mismatch",
            "path_goal_error_m": error_m,
            "tolerance_m": tolerance_m,
            "message": "Nav2 规划路径终点与当前任务点不一致，已停止任务；路径差 %.2f m"
            % (error_m if error_m is not None else -1.0),
            **base,
        }

    started = float(active.get("last_nav_accepted_monotonic") or active.get("last_goal_sent_monotonic") or 0.0)
    age_s = None if started <= 0.0 else max(0.0, float(now_monotonic) - started)
    if started <= 0.0 or age_s is None or age_s < timeout_s:
        return {
            "action": "wait",
            "reason": "waiting_for_new_plan",
            "age_s": age_s,
            "timeout_s": timeout_s,
            **base,
        }
    return {
        "action": "fail",
        "reason": "plan_update_timeout",
        "timeout_s": timeout_s,
        "age_s": age_s,
        "message": "Nav2 已接收当前点位 %.1f 秒，但前端未收到对应的新规划路径，已停止任务" % timeout_s,
        **base,
    }


def apply_plan_goal_verified_state(
    active: Dict[str, Any],
    decision: Dict[str, Any],
) -> Dict[str, Any]:
    updated = dict(active)
    updated["plan_goal_verified"] = True
    updated["plan_goal_error_m"] = float(decision.get("path_goal_error_m"))
    updated["plan_path_version"] = decision.get("path_version")
    updated["status_message"] = "规划路径已匹配当前点位，继续执行"
    return updated


def plan_goal_verified_event_payload(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    decision: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "event": "plan_goal_verified",
        "message": str(active.get("status_message") or "规划路径已匹配当前点位，继续执行"),
        "extra": {
            "annotation_id": annotation.get("id"),
            "label": annotation.get("label"),
            "path_version": decision.get("path_version"),
            "path_last_point": decision.get("path_last_point"),
            "path_goal_error_m": float(decision.get("path_goal_error_m")),
        },
    }
