"""Pure task snapshot helpers for frontend task diagnostics."""

from __future__ import annotations

import time
import math
from typing import Any, Callable, Dict, Optional


NowText = Callable[[], str]

TASK_TOPIC_KEYS = (
    "localization_ok",
    "navigation_status",
    "scan",
    "lidar_points",
    "lidar_relay_status",
    "map",
    "local_costmap",
    "global_costmap",
)

RUNTIME_ACTIVE_KEYS = (
    "index",
    "phase",
    "last_goal_annotation_id",
    "last_goal_label",
    "last_goal_pose",
    "last_goal_attempt_id",
    "last_floor_goal_published_at",
    "last_floor_goal_annotation_id",
    "last_floor_goal_label",
    "last_floor_goal_pose",
    "floor_goal_publish_count",
    "last_nav_goal_status",
    "last_nav_status",
    "last_nav_feedback",
    "last_nav_goal_match",
    "last_ignored_nav_status",
    "last_ignored_nav_goal_match",
    "last_distance_m",
    "last_robot_pose",
    "goal_sent_path_version",
    "plan_goal_verified",
    "plan_goal_error_m",
    "plan_path_version",
    "stall_age_s",
    "runtime_guard",
    "runtime_guard_lost_age_s",
    "status_message",
)

RESULT_ACTIVE_KEYS = (
    "last_goal_annotation_id",
    "last_goal_label",
    "last_goal_pose",
    "last_goal_attempt_id",
    "last_floor_goal_published_at",
    "last_floor_goal_annotation_id",
    "last_floor_goal_label",
    "last_floor_goal_pose",
    "floor_goal_publish_count",
    "goal_sent_path_version",
    "plan_goal_verified",
    "plan_goal_error_m",
    "plan_path_version",
    "last_nav_goal_status",
    "last_nav_status",
    "last_nav_feedback",
    "last_nav_goal_match",
    "last_ignored_nav_status",
    "last_ignored_nav_goal_match",
    "last_robot_pose",
    "last_distance_m",
    "last_progress_at",
    "stall_age_s",
    "runtime_guard",
    "runtime_guard_lost_age_s",
    "start_readiness",
    "post_reset_navigation_readiness",
    "status_message",
)

RESULT_EXTRA_KEYS = (
    "reason",
    "nav_status",
    "annotation_id",
    "label",
    "distance_m",
    "path_version",
    "goal_sent_path_version",
    "path_last_point",
    "path_goal_error_m",
    "timeout_s",
    "runtime_guard",
)


def pose_age_sec(pose: Dict[str, Any], now: Optional[float] = None) -> Optional[float]:
    if pose.get("last_update") is None:
        return None
    try:
        current = time.time() if now is None else float(now)
        return max(0.0, current - float(pose.get("last_update")))
    except (TypeError, ValueError):
        return None


def _age_from_last_update(payload: Dict[str, Any], now: float) -> Optional[float]:
    last_update = payload.get("last_update")
    if last_update is None:
        return None
    try:
        return max(0.0, now - float(last_update))
    except (TypeError, ValueError):
        return None


def _dict_payload(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, dict) else {}


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


def _is_plausible_pose_dict(pose: Dict[str, Any]) -> bool:
    for key in ("x", "y"):
        try:
            if not math.isfinite(float(pose.get(key))):
                return False
        except (TypeError, ValueError):
            return False
    return True


def build_active_waypoint_payload(
    active: Dict[str, Any],
    annotation: Dict[str, Any],
    state: Dict[str, Any],
    *,
    phase: str,
    now_text: str,
    now_time: float,
    now_monotonic: float,
    waypoint: Dict[str, Any],
) -> Dict[str, Any]:
    pose = annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {}
    path_snapshot = _dict_payload(state.get("path"))
    state_pose = _dict_payload(state.get("pose"))
    state_pose_age_s = pose_age_sec(state_pose, now_time)
    nav_feedback_age_s = None
    try:
        feedback_monotonic = float(active.get("last_nav_feedback_monotonic", 0.0) or 0.0)
        if feedback_monotonic > 0.0:
            nav_feedback_age_s = max(0.0, float(now_monotonic) - feedback_monotonic)
    except (TypeError, ValueError):
        nav_feedback_age_s = None
    path_last_point = path_snapshot.get("last_point")
    path_goal_error_m = None
    if isinstance(path_last_point, dict):
        try:
            path_goal_error_m = math.hypot(
                float(path_last_point.get("x")) - float(pose.get("x")),
                float(path_last_point.get("y")) - float(pose.get("y")),
            )
        except (TypeError, ValueError):
            path_goal_error_m = None
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
        "path_last_point": path_last_point,
        "path_goal_error_m": path_goal_error_m,
        "path_point_count": path_snapshot.get("point_count"),
        "path_raw_point_count": path_snapshot.get("raw_point_count"),
        "path_version": path_snapshot.get("version"),
        "goal_sent_path_version": active.get("goal_sent_path_version"),
        "plan_goal_verified": active.get("plan_goal_verified"),
        "plan_goal_error_m": active.get("plan_goal_error_m"),
        "plan_path_version": active.get("plan_path_version"),
        "nav_goal_status": active.get("last_nav_goal_status"),
        "nav_goal_seq": active.get("last_nav_goal_seq"),
        "nav_status": active.get("last_nav_status"),
        "nav_feedback": active.get("last_nav_feedback"),
        "nav_goal_match": active.get("last_nav_goal_match"),
        "last_ignored_nav_status": active.get("last_ignored_nav_status"),
        "last_ignored_nav_goal_match": active.get("last_ignored_nav_goal_match"),
        "goal_attempt_id": active.get("last_goal_attempt_id"),
        "goal_send_count": active.get("waypoint_goal_send_count"),
        "total_goal_send_count": active.get("total_goal_send_count"),
        "resend_goal_count": active.get("resend_goal_count"),
        "last_floor_goal_published_at": active.get("last_floor_goal_published_at"),
        "last_floor_goal_annotation_id": active.get("last_floor_goal_annotation_id"),
        "last_floor_goal_label": active.get("last_floor_goal_label"),
        "last_floor_goal_pose": active.get("last_floor_goal_pose"),
        "floor_goal_publish_count": active.get("floor_goal_publish_count"),
        "waypoint_started_at": active.get("waypoint_started_at"),
        "last_goal_sent_at": active.get("last_goal_sent_at"),
        "last_progress_at": active.get("last_progress_at"),
        "stall_age_s": active.get("stall_age_s"),
        "runtime_guard": active.get("runtime_guard"),
        "runtime_guard_lost_age_s": active.get("runtime_guard_lost_age_s"),
        "last_progress_moved_m": active.get("last_progress_moved_m"),
        "last_progress_yaw_delta_rad": active.get("last_progress_yaw_delta_rad"),
        "last_progress_distance_delta_m": active.get("last_progress_distance_delta_m"),
        "state_pose": state_pose if _is_plausible_pose_dict(state_pose) else None,
        "state_pose_age_s": state_pose_age_s,
        "nav_feedback_age_s": nav_feedback_age_s,
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


def build_task_runtime_snapshot(
    active: Dict[str, Any],
    state: Dict[str, Any],
    *,
    camera_proxy_status: Optional[Dict[str, Any]] = None,
    now: Optional[float] = None,
) -> Dict[str, Any]:
    current_time = time.time() if now is None else float(now)
    pose = _dict_payload(state.get("pose"))
    path = _dict_payload(state.get("path"))
    scan = _dict_payload(state.get("scan"))
    lidar_points = _dict_payload(state.get("lidar_points"))
    lidar_relay_status = _dict_payload(state.get("lidar_relay_status"))
    raw_topics = _dict_payload(state.get("topics"))
    topics = {
        key: dict(value)
        for key, value in raw_topics.items()
        if key in TASK_TOPIC_KEYS and isinstance(value, dict)
    }

    snapshot: Dict[str, Any] = {
        "floor": state.get("floor"),
        "localization_ok": state.get("localization_ok"),
        "navigation_status": state.get("navigation_status"),
        "pose": pose if pose else None,
        "pose_age_sec": pose_age_sec(pose, current_time),
        "path": {
            "version": path.get("version"),
            "point_count": path.get("point_count"),
            "raw_point_count": path.get("raw_point_count"),
            "last_point": path.get("last_point"),
        },
        "scan": {
            "finite_ranges": scan.get("finite_ranges"),
            "frame_id": scan.get("frame_id"),
            "age_sec": _age_from_last_update(scan, current_time),
        },
        "lidar_points": {
            "width": lidar_points.get("width"),
            "height": lidar_points.get("height"),
            "source": lidar_points.get("source"),
            "frame_id": lidar_points.get("frame_id"),
            "age_sec": _age_from_last_update(lidar_points, current_time),
        },
        "lidar_relay_status": {
            "output_width": lidar_relay_status.get("output_width"),
            "output_height": lidar_relay_status.get("output_height"),
            "output_stride": lidar_relay_status.get("output_stride"),
            "downsample_method": lidar_relay_status.get("downsample_method"),
            "input_rate_hz": lidar_relay_status.get("input_rate_hz"),
            "publish_rate_hz": lidar_relay_status.get("publish_rate_hz"),
            "skip_ratio": lidar_relay_status.get("skip_ratio"),
            "max_output_points": lidar_relay_status.get("max_output_points"),
            "min_publish_interval_s": lidar_relay_status.get("min_publish_interval_s"),
            "age_sec": _age_from_last_update(lidar_relay_status, current_time),
        },
        "topics": topics,
        "camera_proxy": dict(camera_proxy_status or {}),
    }

    for active_key in RUNTIME_ACTIVE_KEYS:
        output_key = "active_index" if active_key == "index" else active_key
        snapshot[output_key] = active.get(active_key)
    return snapshot


def build_task_result_snapshot(
    active: Dict[str, Any],
    *,
    status: str,
    waypoint: Optional[Dict[str, Any]],
    runtime_snapshot: Dict[str, Any],
    now_text: str,
    message: Optional[str] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    timeline = list(active.get("timeline") or [])
    result: Dict[str, Any] = {
        "task_id": active.get("task_id"),
        "task_name": active.get("task_name"),
        "status": status,
        "message": message or active.get("status_message") or active.get("last_error"),
        "saved_at": now_text,
        "map_id": active.get("map_id"),
        "index": int(active.get("index", 0) or 0),
        "phase": active.get("phase"),
        "waypoint": waypoint,
        "last_error": active.get("last_error"),
        "last_event": timeline[-1] if timeline else None,
        "timeline_tail": timeline[-12:],
        "runtime_snapshot": runtime_snapshot,
    }
    for key in RESULT_ACTIVE_KEYS:
        result[key] = active.get(key)
    if extra:
        result["extra"] = dict(extra)
        for key in RESULT_EXTRA_KEYS:
            if key in extra:
                result[key] = extra.get(key)
    return result


def apply_task_result_persistence(
    task: Dict[str, Any],
    active: Dict[str, Any],
    *,
    status: str,
    result: Dict[str, Any],
    message: Optional[str],
    now_text: str,
) -> Dict[str, Any]:
    updated = dict(task)
    updated["status"] = str(status)
    updated["last_result"] = result
    updated["last_timeline"] = list(active.get("timeline") or [])
    updated["last_error"] = None if status == "completed" else (message or active.get("last_error"))
    updated["updated_at"] = now_text
    return updated


def apply_task_result_to_tasks(
    tasks: Any,
    *,
    task_id: Any,
    active: Dict[str, Any],
    status: str,
    result: Dict[str, Any],
    message: Optional[str],
    now_text: str,
) -> Dict[str, Any]:
    if not isinstance(tasks, list):
        return {
            "ok": False,
            "tasks": [],
            "task": None,
            "changed": False,
        }
    target_id = str(task_id or "").strip()
    updated_tasks = []
    updated_task = None
    changed = False
    for task in tasks:
        item = dict(task) if isinstance(task, dict) else {}
        if target_id and str(item.get("id") or "").strip() == target_id:
            item = apply_task_result_persistence(
                item,
                active,
                status=status,
                result=result,
                message=message,
                now_text=now_text,
            )
            updated_task = dict(item)
            changed = True
        updated_tasks.append(item)
    return {
        "ok": changed,
        "tasks": updated_tasks,
        "task": updated_task,
        "changed": changed,
    }


def last_task_result_payload(tasks: Any) -> Optional[Dict[str, Any]]:
    if not isinstance(tasks, list):
        return None

    def sort_key(task: Dict[str, Any]) -> str:
        return str(task.get("updated_at") or task.get("created_at") or "")

    candidates = [
        task
        for task in tasks
        if isinstance(task, dict) and (task.get("last_result") or task.get("last_error") or task.get("last_timeline"))
    ]
    if not candidates:
        return None
    task = max(candidates, key=sort_key)
    timeline = list(task.get("last_timeline") or [])
    return {
        "task_id": task.get("id"),
        "task_name": task.get("name"),
        "status": task.get("status"),
        "updated_at": task.get("updated_at") or task.get("created_at"),
        "last_result": task.get("last_result"),
        "last_error": task.get("last_error"),
        "last_event": timeline[-1] if timeline else None,
    }
