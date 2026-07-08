"""Pure active-waypoint contract helpers for task/radar interfaces."""

from __future__ import annotations

import time
from typing import Any, Dict, Optional


def pose_age_sec(pose: Dict[str, Any], now: Optional[float] = None) -> Optional[float]:
    if pose.get("last_update") is None:
        return None
    try:
        current = time.time() if now is None else float(now)
        return max(0.0, current - float(pose.get("last_update")))
    except (TypeError, ValueError):
        return None

def _active_waypoint_elapsed_s(active: Dict[str, Any], now_monotonic: float) -> Optional[float]:
    try:
        started = float(active.get("waypoint_started_monotonic", 0.0) or 0.0)
    except (TypeError, ValueError):
        return None
    if started <= 0.0:
        return None
    return max(0.0, float(now_monotonic) - started)


def _remaining_dwell_s(active: Dict[str, Any], now_time: float) -> float:
    if active.get("phase") != "dwelling":
        return 0.0
    try:
        return max(0.0, float(active.get("dwell_until", 0.0)) - float(now_time))
    except (TypeError, ValueError):
        return 0.0


def build_active_waypoint_payload(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    *,
    phase: str,
    now_text: str,
    now_time: float,
    now_monotonic: float,
    waypoint: Dict[str, Any],
) -> Dict[str, Any]:
    pose = annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {}
    return {
        "task_id": active.get("task_id"),
        "task_name": active.get("task_name"),
        "phase": phase,
        "index": int(active.get("index", 0)),
        "remaining_dwell_s": _remaining_dwell_s(active, now_time),
        "elapsed_s": _active_waypoint_elapsed_s(active, now_monotonic),
        "distance_m": active.get("last_distance_m"),
        "robot_pose": active.get("last_robot_pose"),
        "goal_pose": {
            "x": pose.get("x"),
            "y": pose.get("y"),
            "z": pose.get("z"),
            "yaw": pose.get("yaw"),
        },
        "nav_goal_status": active.get("last_nav_goal_status"),
        "waypoint_started_at": active.get("waypoint_started_at"),
        "last_goal_sent_at": active.get("last_goal_sent_at"),
        "status_message": active.get("status_message"),
        "waypoint": waypoint,
        "updated_at": now_text,
    }


def build_idle_waypoint_payload(*, reason: str, now_text: str) -> Dict[str, Any]:
    return {
        "phase": "idle",
        "reason": str(reason or "idle"),
        "updated_at": now_text,
    }
