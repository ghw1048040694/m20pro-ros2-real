"""Pure task-contract helpers for the M20Pro web dashboard.

This module intentionally has no ROS dependency.  It owns the data-only rules
that decide whether frontend task payloads still match the task/waypoint data
the backend is about to execute.
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Dict, Iterable, Optional


NowText = Callable[[], str]


def default_now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def contract_error(message: str, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    payload: Dict[str, Any] = {"ok": False, "message": message}
    if extra:
        payload.update(extra)
    return payload


def readiness_success(
    message: str,
    extra: Optional[Dict[str, Any]] = None,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    if extra:
        payload.update(extra)
    payload.update(
        {
            "ready": True,
            "code": "ready",
            "message": message,
            "updated_at": (now_text or default_now_text)(),
        }
    )
    return payload


def readiness_failure(
    code: str,
    message: str,
    extra: Optional[Dict[str, Any]] = None,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    if extra:
        payload.update(extra)
    payload.update(
        {
            "ready": False,
            "code": code,
            "message": message,
            "updated_at": (now_text or default_now_text)(),
        }
    )
    return payload


def payload_age_sec(payload: Dict[str, Any], now: float) -> Optional[float]:
    last_update = payload.get("last_update")
    if last_update is None:
        return None
    try:
        return max(0.0, now - float(last_update))
    except (TypeError, ValueError):
        return None


def readiness_error_payload(readiness: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "code": readiness.get("code"),
        "task_readiness": readiness,
    }


def task_status_allows_start(status: Any) -> bool:
    normalized = str(status or "ready").strip() or "ready"
    return normalized in {"ready", "stopped", "completed", "error"}


def readiness_waypoint_payload(annotation: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not annotation:
        return None
    pose = annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {}
    return {
        "id": annotation.get("id"),
        "label": annotation.get("label"),
        "floor": annotation.get("floor"),
        "pose": {
            "x": pose.get("x"),
            "y": pose.get("y"),
            "z": pose.get("z"),
            "yaw": pose.get("yaw"),
        },
    }


def task_waypoint_payload(
    annotation_id: str,
    annotation: Optional[Dict[str, Any]],
    index: int,
) -> Dict[str, Any]:
    if not annotation:
        return {"id": annotation_id, "index": index, "missing": True}
    payload = readiness_waypoint_payload(annotation) or {"id": annotation_id}
    payload["index"] = index
    payload["manual_point_type"] = annotation.get("manual_point_type")
    payload["dwell_s"] = annotation.get("dwell_s")
    payload["building"] = annotation.get("building")
    payload["unit"] = annotation.get("unit")
    payload["house"] = annotation.get("house")
    payload["area"] = annotation.get("area")
    payload["room"] = annotation.get("room")
    payload["scan_point"] = annotation.get("scan_point")
    payload["radar"] = annotation.get("radar")
    return payload


def is_finite_pose_dict(pose: Dict[str, Any]) -> bool:
    if not isinstance(pose, dict):
        return False
    required = ("x", "y", "z", "yaw", "yaw_deg")
    if any(key not in pose for key in required):
        return False
    try:
        return all(math.isfinite(float(pose.get(key, 0.0))) for key in required)
    except (TypeError, ValueError):
        return False


def is_plausible_pose_dict(pose: Dict[str, Any], max_abs_position: float = 10000.0) -> bool:
    if not is_finite_pose_dict(pose):
        return False
    try:
        return all(abs(float(pose.get(key, 0.0))) <= max_abs_position for key in ("x", "y", "z"))
    except (TypeError, ValueError):
        return False


def is_plausible_waypoint_pose_dict(pose: Dict[str, Any], max_abs_position: float = 10000.0) -> bool:
    if not isinstance(pose, dict):
        return False
    required = ("x", "y", "z", "yaw")
    if any(key not in pose for key in required):
        return False
    try:
        return all(
            math.isfinite(float(pose.get(key, 0.0)))
            and (key not in ("x", "y", "z") or abs(float(pose.get(key, 0.0))) <= max_abs_position)
            for key in required
        )
    except (TypeError, ValueError):
        return False


def pose_distance_m(a: Optional[Dict[str, Any]], b: Optional[Dict[str, Any]]) -> Optional[float]:
    if not isinstance(a, dict) or not isinstance(b, dict):
        return None
    try:
        return math.hypot(float(a.get("x")) - float(b.get("x")), float(a.get("y")) - float(b.get("y")))
    except (TypeError, ValueError):
        return None


def task_pose_readiness_payload(
    pose: Dict[str, Any],
    first_annotation: Optional[Dict[str, Any]],
    *,
    task_id: Optional[str],
    task_map_id: Optional[str],
    selected_map_id: Optional[str],
    localization_ok: Any,
    current_floor: Any,
    navigation_status: Any,
    pose_age_sec: Optional[float],
    pose_timeout_s: float,
    require_localization_ok: bool,
    warn_first_waypoint_distance_m: float,
    max_first_waypoint_distance_m: float,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    base = {
        "task_id": task_id,
        "task_map_id": task_map_id,
        "selected_map_id": selected_map_id,
        "localization_ok": localization_ok,
        "pose_ok": is_plausible_pose_dict(pose),
        "pose_age_sec": pose_age_sec,
        "pose_timeout_s": max(0.5, float(pose_timeout_s)),
        "current_floor": current_floor,
        "navigation_status": navigation_status,
        "first_waypoint": readiness_waypoint_payload(first_annotation),
    }
    if first_annotation is not None:
        target_floor = str(first_annotation.get("floor") or "").strip()
        normalized_current_floor = str(current_floor or "").strip()
        cross_floor_first_waypoint = bool(
            normalized_current_floor and target_floor and normalized_current_floor != target_floor
        )
        base["target_floor"] = target_floor
        base["first_waypoint_cross_floor"] = cross_floor_first_waypoint
        if cross_floor_first_waypoint:
            base["first_waypoint_distance_skipped"] = "cross_floor"
        else:
            first_pose = first_annotation.get("pose") if isinstance(first_annotation.get("pose"), dict) else {}
            first_distance = pose_distance_m(pose, first_pose)
            if first_distance is not None:
                base["first_waypoint_distance_m"] = first_distance
                base["first_waypoint_distance_warn_m"] = max(0.0, float(warn_first_waypoint_distance_m))
                base["first_waypoint_distance_max_m"] = max(0.0, float(max_first_waypoint_distance_m))
    if require_localization_ok and localization_ok is not True:
        return readiness_failure(
            "localization_not_confirmed",
            "定位未确认，先在网页定位页完成重定位，再开始任务",
            base,
            now_text=now_text,
        )
    if not base["pose_ok"] or pose_age_sec is None or pose_age_sec > base["pose_timeout_s"]:
        return readiness_failure(
            "pose_invalid_or_stale",
            "地图位姿无效或已过期，先重定位并确认机器人位置稳定",
            {**base, "pose": pose},
            now_text=now_text,
        )
    return readiness_success("任务位姿检查通过", base, now_text=now_text)


def battery_readiness_payload(
    battery: Dict[str, Any],
    *,
    required: bool,
    min_level: int,
    timeout_s: float,
    now: float,
    success_message: str,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    if not required:
        return readiness_success("任务电池检查已关闭", {"required": False}, now_text=now_text)
    primary = battery.get("primary") if isinstance(battery.get("primary"), dict) else None
    age_s = payload_age_sec(battery, now)
    base = {
        "required": True,
        "min_level": max(0, int(min_level)),
        "timeout_s": max(1.0, float(timeout_s)),
        "age_sec": age_s,
        "battery": primary,
    }
    if primary is None:
        return readiness_failure("battery_missing", "未收到电池数据，不能开始/继续任务", base, now_text=now_text)
    if age_s is None or age_s > base["timeout_s"]:
        return readiness_failure(
            "battery_stale",
            "电池数据已过期 %.1f 秒，不能开始/继续任务" % (age_s if age_s is not None else -1.0),
            base,
            now_text=now_text,
        )
    try:
        level = int(primary.get("level"))
    except (TypeError, ValueError):
        level = -1
    base["level"] = level
    if level < base["min_level"]:
        return readiness_failure(
            "battery_low",
            "电量 %d%% 低于任务要求 %d%%，请先充电再执行任务" % (level, base["min_level"]),
            base,
            now_text=now_text,
        )
    return readiness_success(success_message, base, now_text=now_text)


def perception_readiness_payload(
    scan: Dict[str, Any],
    lidar: Dict[str, Any],
    *,
    require_scan: bool,
    require_lidar: bool,
    timeout_s: float,
    min_scan_ranges: int,
    min_lidar_points: int,
    now: float,
    success_message: str,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    if not require_scan and not require_lidar:
        return readiness_success("任务感知检查已关闭", {"required": False}, now_text=now_text)
    bounded_timeout = max(0.5, float(timeout_s))
    bounded_min_scan = max(1, int(min_scan_ranges))
    bounded_min_lidar = max(1, int(min_lidar_points))
    scan_age = payload_age_sec(scan, now)
    lidar_age = payload_age_sec(lidar, now)
    finite_ranges = int(scan.get("finite_ranges", 0) or 0)
    lidar_points = int(lidar.get("width", 0) or 0) * max(1, int(lidar.get("height", 1) or 1))
    scan_ok = bool(scan_age is not None and scan_age <= bounded_timeout and finite_ranges >= bounded_min_scan)
    lidar_ok = bool(lidar_age is not None and lidar_age <= bounded_timeout and lidar_points >= bounded_min_lidar)
    checks = {
        "scan": {
            "required": require_scan,
            "ok": scan_ok,
            "age_sec": scan_age,
            "finite_ranges": finite_ranges,
            "min_finite_ranges": bounded_min_scan,
            "frame_id": scan.get("frame_id"),
        },
        "lidar_points": {
            "required": require_lidar,
            "ok": lidar_ok,
            "age_sec": lidar_age,
            "points": lidar_points,
            "min_points": bounded_min_lidar,
            "source": lidar.get("source"),
            "frame_id": lidar.get("frame_id"),
        },
    }
    base = {
        "required": True,
        "timeout_s": bounded_timeout,
        "checks": checks,
    }
    if require_scan and not scan_ok:
        return readiness_failure(
            "perception_scan_unavailable",
            "任务启动/执行要求新鲜 /scan，但当前有效距离 %d、数据年龄 %s"
            % (
                finite_ranges,
                "%.1fs" % scan_age if scan_age is not None else "未知",
            ),
            base,
            now_text=now_text,
        )
    if require_lidar and not lidar_ok:
        return readiness_failure(
            "perception_lidar_unavailable",
            "任务启动/执行要求前端可见点云 relay，但当前点数 %d、数据年龄 %s"
            % (
                lidar_points,
                "%.1fs" % lidar_age if lidar_age is not None else "未知",
            ),
            base,
            now_text=now_text,
        )
    return readiness_success(success_message, base, now_text=now_text)


def runtime_guard_readiness_payload(
    *,
    battery_readiness: Dict[str, Any],
    perception_readiness: Dict[str, Any],
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    if not battery_readiness.get("ready"):
        return readiness_failure(
            str(battery_readiness.get("code") or "battery_not_ready"),
            str(battery_readiness.get("message") or "电池状态不满足任务运行要求"),
            {"battery_readiness": battery_readiness},
            now_text=now_text,
        )
    if not perception_readiness.get("ready"):
        return readiness_failure(
            str(perception_readiness.get("code") or "perception_not_ready"),
            str(perception_readiness.get("message") or "感知链路不满足任务运行要求"),
            {
                "battery_readiness": battery_readiness,
                "perception_readiness": perception_readiness,
            },
            now_text=now_text,
        )
    return readiness_success(
        "任务运行关键链路可用",
        {
            "battery_readiness": battery_readiness,
            "perception_readiness": perception_readiness,
        },
        now_text=now_text,
    )


def task_runtime_readiness_payload(
    *,
    map_relocalization_readiness: Optional[Dict[str, Any]],
    pose_readiness: Dict[str, Any],
    battery_readiness: Dict[str, Any],
    perception_readiness: Dict[str, Any],
    success_message: str,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    if map_relocalization_readiness is not None:
        return map_relocalization_readiness
    base = {
        key: value
        for key, value in dict(pose_readiness or {}).items()
        if key not in ("ready", "code", "message", "updated_at")
    }
    if not pose_readiness.get("ready"):
        return pose_readiness
    if not battery_readiness.get("ready"):
        return readiness_failure(
            str(battery_readiness.get("code") or "battery_not_ready"),
            str(battery_readiness.get("message") or "电池状态不满足任务启动要求"),
            {
                **base,
                "battery_readiness": battery_readiness,
            },
            now_text=now_text,
        )
    if not perception_readiness.get("ready"):
        return readiness_failure(
            str(perception_readiness.get("code") or "perception_not_ready"),
            str(perception_readiness.get("message") or "感知链路不满足任务启动要求"),
            {
                **base,
                "battery_readiness": battery_readiness,
                "perception_readiness": perception_readiness,
            },
            now_text=now_text,
        )
    return readiness_success(
        success_message,
        {
            **base,
            "battery_readiness": battery_readiness,
            "perception_readiness": perception_readiness,
        },
        now_text=now_text,
    )


def current_task_readiness_payload(
    *,
    active_task: Dict[str, Any],
    runtime_readiness: Dict[str, Any],
    nav_readiness: Optional[Dict[str, Any]],
    require_nav_ready: bool,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    active = dict(active_task or {})
    if active.get("status") == "running":
        return readiness_failure(
            "task_running",
            str(active.get("status_message") or "任务正在执行中"),
            {
                "task_id": active.get("task_id"),
                "task_name": active.get("task_name"),
                "index": active.get("index"),
                "phase": active.get("phase"),
                "distance_m": active.get("last_distance_m"),
                "nav_goal_status": active.get("last_nav_goal_status"),
                "last_nav_status": active.get("last_nav_status"),
                "running": True,
            },
            now_text=now_text,
        )
    runtime = dict(runtime_readiness or {})
    if runtime.get("ready") and require_nav_ready:
        nav_ready = dict(nav_readiness or {})
        if not nav_ready.get("ready"):
            return readiness_failure(
                "navigation_not_ready",
                str(nav_ready.get("message") or "导航链路尚未就绪，请等待 Nav2/costmap 恢复"),
                {
                    **runtime,
                    "navigation_readiness": nav_ready,
                },
                now_text=now_text,
            )
        return readiness_success(
            "定位、位姿和导航链路已就绪；具体任务点位请看任务列表的执行条件",
            {
                **runtime,
                "navigation_readiness": nav_ready,
            },
            now_text=now_text,
        )
    return runtime


def runtime_guard_lost_decision(
    active: Dict[str, Any],
    guard: Dict[str, Any],
    *,
    now_monotonic: float,
    timeout_s: float,
) -> Dict[str, Any]:
    if guard.get("ready"):
        return {
            "action": "clear",
            "reason": "runtime_guard_ready",
            "clear_keys": [
                "runtime_guard",
                "runtime_guard_lost_started_monotonic",
                "runtime_guard_lost_at",
                "runtime_guard_lost_age_s",
            ],
        }

    try:
        started = float(active.get("runtime_guard_lost_started_monotonic", 0.0) or 0.0)
    except (TypeError, ValueError):
        started = 0.0
    if started <= 0.0:
        started = float(now_monotonic)
    age_s = max(0.0, float(now_monotonic) - started)
    message = "任务执行中关键链路异常：%s；%.1f 秒内未恢复将停止任务" % (
        str(guard.get("message") or "未知异常"),
        float(timeout_s),
    )
    base = {
        "reason": "runtime_guard_lost",
        "guard": guard,
        "started_monotonic": started,
        "age_s": age_s,
        "timeout_s": float(timeout_s),
        "wait_code": "runtime_" + str(guard.get("code") or "guard_not_ready"),
        "message": message,
    }
    if age_s < float(timeout_s):
        return {"action": "wait", **base}
    return {
        "action": "fail",
        **base,
        "message": "任务执行中关键链路异常超过 %.1f 秒，已停止任务：%s"
        % (float(timeout_s), str(guard.get("message") or "未知异常")),
    }


def runtime_guard_failure_extra(guard: Dict[str, Any], decision: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "runtime_guard": guard,
        "runtime_guard_lost_age_s": decision.get("age_s"),
    }


def runtime_guard_waiting_event_payload(
    active: Dict[str, Any],
    guard: Dict[str, Any],
    decision: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "event": "runtime_guard_waiting",
        "message": str(
            active.get("status_message")
            or decision.get("message")
            or "任务执行中关键链路异常，等待恢复"
        ),
        "extra": {
            "guard": guard,
            "age_s": decision.get("age_s"),
            "timeout_s": decision.get("timeout_s"),
        },
    }


def apply_runtime_guard_wait_state(
    active: Dict[str, Any],
    guard: Dict[str, Any],
    decision: Dict[str, Any],
    *,
    now_text: str,
    fallback_monotonic: float,
) -> Dict[str, Any]:
    updated = dict(active)
    previous_started = updated.get("runtime_guard_lost_started_monotonic")
    previous_wait_code = updated.get("last_wait_code")
    previous_status = updated.get("status_message")
    wait_code = str(decision.get("wait_code") or "runtime_guard_not_ready")
    status_message = str(decision.get("message") or "任务执行中关键链路异常")
    updated["runtime_guard_lost_started_monotonic"] = float(
        decision.get("started_monotonic") or fallback_monotonic
    )
    if "runtime_guard_lost_at" not in updated:
        updated["runtime_guard_lost_at"] = now_text
    updated["runtime_guard"] = guard
    updated["runtime_guard_lost_age_s"] = decision.get("age_s")
    updated["last_wait_code"] = wait_code
    updated["last_wait_at"] = now_text
    updated["status_message"] = status_message
    should_record_event = (
        decision.get("action") == "fail"
        or previous_started in (None, 0, 0.0)
        or str(previous_wait_code or "") != wait_code
        or str(previous_status or "") != status_message
    )
    return {
        "active": updated,
        "changed": updated != active,
        "should_record_event": should_record_event,
    }


def apply_runtime_guard_clear_state(
    active: Dict[str, Any],
    decision: Dict[str, Any],
) -> Dict[str, Any]:
    updated = dict(active)
    changed = False
    for key in decision.get("clear_keys") or ():
        if key in updated:
            updated.pop(key, None)
            changed = True
    return {"active": updated, "changed": changed}


def map_relocalization_task_readiness_payload(
    map_relocalization_required: Dict[str, Any],
    *,
    task_id: Optional[str],
    task_map_id: Optional[str],
    selected_map_id: Optional[str],
    now_text: Optional[NowText] = None,
) -> Optional[Dict[str, Any]]:
    if not map_relocalization_required:
        return None
    return readiness_failure(
        "map_relocalization_required",
        "Nav2 已加载当前固定地图，请先按开发手册2101完成重定位，再开始标点或任务",
        {
            "task_id": task_id,
            "task_map_id": task_map_id,
            "selected_map_id": selected_map_id,
            "map_relocalization_required": dict(map_relocalization_required),
        },
        now_text=now_text,
    )


def pose_map_bounds_error(
    pose: Dict[str, Any],
    map_payload: Dict[str, Any],
    label: str,
) -> Optional[Dict[str, Any]]:
    if not isinstance(map_payload, dict) or not map_payload.get("available"):
        return contract_error(
            "当前地图不可用，不能开始任务",
            {"label": label, "map_message": map_payload.get("message") if isinstance(map_payload, dict) else None},
        )
    try:
        width = int(map_payload.get("width"))
        height = int(map_payload.get("height"))
        resolution = float(map_payload.get("resolution"))
        origin = map_payload.get("origin") or {}
        x = float(pose.get("x"))
        y = float(pose.get("y"))
        ox = float(origin.get("x", 0.0))
        oy = float(origin.get("y", 0.0))
    except (TypeError, ValueError):
        return contract_error("地图或位姿数据无效，不能开始任务", {"label": label})
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return contract_error("地图尺寸无效，不能开始任务", {"label": label})
    mx = (x - ox) / resolution
    my = (y - oy) / resolution
    if mx < 0.0 or my < 0.0 or mx >= float(width) or my >= float(height):
        return contract_error(
            f"{label}不在当前地图范围内，请确认地图和重定位结果",
            {
                "label": label,
                "x": x,
                "y": y,
                "map_width": width,
                "map_height": height,
                "map_resolution": resolution,
                "map_origin": origin,
            },
        )
    return None


def pose_map_occupancy_error(
    pose: Dict[str, Any],
    map_payload: Dict[str, Any],
    label: str,
) -> Optional[Dict[str, Any]]:
    try:
        width = int(map_payload.get("width"))
        height = int(map_payload.get("height"))
        resolution = float(map_payload.get("resolution"))
        origin = map_payload.get("origin") or {}
        data = map_payload.get("data")
        x = float(pose.get("x"))
        y = float(pose.get("y"))
        ox = float(origin.get("x", 0.0))
        oy = float(origin.get("y", 0.0))
    except (TypeError, ValueError):
        return contract_error("地图或位姿数据无效，不能检查栅格占用", {"label": label})
    if not isinstance(data, list) or len(data) < width * height:
        return None
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return None
    mx = int(math.floor((x - ox) / resolution))
    my = int(math.floor((y - oy) / resolution))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    value = int(data[my * width + mx])
    base = {
        "label": label,
        "x": x,
        "y": y,
        "map_x": mx,
        "map_y": my,
        "map_value": value,
        "map_width": width,
        "map_height": height,
        "map_resolution": resolution,
        "map_origin": origin,
    }
    if value >= 65:
        return contract_error(
            f"{label}落在障碍物栅格上，请重新标点",
            {**base, "code": "pose_on_occupied_cell"},
        )
    if value < 0:
        return contract_error(
            f"{label}落在未知栅格上，请确认地图或重新标点",
            {**base, "code": "pose_on_unknown_cell"},
        )
    return None


def map_metadata_mismatch_error(
    live_map: Dict[str, Any],
    selected_map: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    if not isinstance(live_map, dict) or not live_map.get("available"):
        return contract_error("Nav2 当前 /map 不可用，不能开始任务", {"code": "live_map_unavailable"})
    if not isinstance(selected_map, dict) or not selected_map.get("available"):
        return contract_error("当前选择的任务地图不可用，不能开始任务", {"code": "selected_map_unavailable"})
    try:
        live_origin = live_map.get("origin") or {}
        selected_origin = selected_map.get("origin") or {}
        checks = {
            "width": int(live_map.get("width")) == int(selected_map.get("width")),
            "height": int(live_map.get("height")) == int(selected_map.get("height")),
            "resolution": abs(float(live_map.get("resolution")) - float(selected_map.get("resolution"))) < 1e-6,
            "origin_x": abs(float(live_origin.get("x", 0.0)) - float(selected_origin.get("x", 0.0))) < 1e-4,
            "origin_y": abs(float(live_origin.get("y", 0.0)) - float(selected_origin.get("y", 0.0))) < 1e-4,
        }
    except (TypeError, ValueError):
        return contract_error("地图元数据无效，不能开始任务", {"code": "map_metadata_invalid"})
    if all(checks.values()):
        return None
    return contract_error(
        "网页选择地图与 Nav2 当前加载地图不一致，请先切换到正确地图并重定位",
        {
            "code": "map_metadata_mismatch",
            "checks": checks,
            "live_map": {
                "width": live_map.get("width"),
                "height": live_map.get("height"),
                "resolution": live_map.get("resolution"),
                "origin": live_map.get("origin"),
            },
            "selected_map": {
                "map_id": selected_map.get("map_id"),
                "name": selected_map.get("name"),
                "floor": selected_map.get("floor"),
                "width": selected_map.get("width"),
                "height": selected_map.get("height"),
                "resolution": selected_map.get("resolution"),
                "origin": selected_map.get("origin"),
            },
        },
    )


def task_start_runtime_readiness_payload(
    first_annotation: Optional[Dict[str, Any]],
    task_map_id: str,
    *,
    task_id: Optional[str],
    selected_map_id: Optional[str],
    runtime_readiness: Dict[str, Any],
    current_floor: Any,
    live_map: Dict[str, Any],
    robot_pose: Dict[str, Any],
    target_map_payload: Dict[str, Any],
    nav_readiness: Optional[Dict[str, Any]],
    success_navigation_readiness: Optional[Dict[str, Any]],
    require_current_floor_known: bool,
    require_current_floor_match: bool,
    require_pose_on_map: bool,
    require_nav_ready: bool,
    max_first_waypoint_distance_m: float,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    if first_annotation is None:
        return readiness_failure(
            "missing_waypoint",
            "任务首个点位不存在，请重新生成任务",
            {"task_id": task_id, "task_map_id": task_map_id, "selected_map_id": selected_map_id},
            now_text=now_text,
        )
    runtime = dict(runtime_readiness or {})
    if not runtime.get("ready"):
        return runtime

    target_floor = str(first_annotation.get("floor") or "").strip()
    if require_current_floor_known and target_floor and not current_floor:
        return readiness_failure(
            "floor_unknown",
            "尚未收到当前楼层，等待 /m20pro/current_floor 后再开始任务",
            {
                **runtime,
                "current_floor": current_floor,
                "target_floor": target_floor,
            },
            now_text=now_text,
        )
    if require_current_floor_match and current_floor and target_floor and current_floor != target_floor:
        return readiness_failure(
            "wrong_floor",
            "当前楼层与任务首点楼层不一致，请先切换/确认地图和楼层",
            {
                **runtime,
                "current_floor": current_floor,
                "target_floor": target_floor,
            },
            now_text=now_text,
        )
    if require_pose_on_map:
        pose_error = pose_map_bounds_error(robot_pose, live_map, "机器人当前位置")
        if pose_error:
            return readiness_failure(
                "current_pose_out_of_map",
                str(pose_error.get("message") or "机器人当前位置不在当前地图范围内"),
                {**runtime, "detail": pose_error},
                now_text=now_text,
            )
        target_pose = first_annotation.get("pose") if isinstance(first_annotation.get("pose"), dict) else {}
        target_error = pose_map_bounds_error(target_pose, target_map_payload, "任务首点")
        if target_error:
            return readiness_failure(
                "target_out_of_map",
                str(target_error.get("message") or "任务首点不在当前地图范围内"),
                {**runtime, "detail": target_error},
                now_text=now_text,
            )
        if current_floor and target_floor and current_floor == target_floor and task_map_id != "live_map":
            metadata_error = map_metadata_mismatch_error(live_map, target_map_payload)
            if metadata_error:
                return readiness_failure(
                    "map_metadata_mismatch",
                    str(metadata_error.get("message") or "网页选择地图与 Nav2 当前加载地图不一致"),
                    {**runtime, "detail": metadata_error},
                    now_text=now_text,
                )

    same_floor_first_waypoint = not (
        current_floor and target_floor and str(current_floor).strip() != target_floor
    )
    first_distance = runtime.get("first_waypoint_distance_m")
    max_first_distance = max(0.0, float(max_first_waypoint_distance_m))
    if (
        same_floor_first_waypoint
        and isinstance(first_distance, (int, float))
        and max_first_distance > 0.0
        and first_distance > max_first_distance
    ):
        return readiness_failure(
            "first_waypoint_too_far",
            "机器人当前位置距离任务首点 %.2f m，超过 %.2f m；请确认重定位和首点是否属于同一现场后再开始"
            % (float(first_distance), max_first_distance),
            {
                **runtime,
                "current_floor": current_floor,
                "target_floor": target_floor,
                "first_waypoint_distance_m": float(first_distance),
                "first_waypoint_distance_max_m": max_first_distance,
            },
            now_text=now_text,
        )
    if not same_floor_first_waypoint:
        runtime["first_waypoint_distance_skipped"] = "cross_floor"
        runtime["first_waypoint_cross_floor"] = True
    if require_nav_ready:
        nav_ready = dict(nav_readiness or {})
        if not nav_ready.get("ready"):
            return readiness_failure(
                "navigation_not_ready",
                str(nav_ready.get("message") or "导航链路尚未就绪，请等待 Nav2/costmap 恢复"),
                {
                    **runtime,
                    "navigation_readiness": nav_ready,
                },
                now_text=now_text,
            )
    return readiness_success(
        "定位、位姿、地图和任务首点检查通过，可以开始执行",
        {
            **runtime,
            "current_floor": current_floor,
            "target_floor": target_floor,
            "floors": runtime.get("floors"),
            "map_ids": runtime.get("map_ids"),
            "multi_floor": runtime.get("multi_floor"),
            "multi_map": runtime.get("multi_map"),
            "navigation_readiness": success_navigation_readiness if success_navigation_readiness is not None else nav_readiness,
        },
        now_text=now_text,
    )


def task_readiness_pre_runtime_payload(
    *,
    task_id: str,
    active_task: Dict[str, Any],
    static_context: Dict[str, Any],
    task_validation: Optional[Dict[str, Any]],
    selected_map_id: Optional[str],
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    task_id = str(task_id or "").strip()
    active = dict(active_task or {})
    if active.get("status") == "running":
        same_task = active.get("task_id") == task_id
        return {
            "proceed": False,
            "readiness": readiness_failure(
                "task_running",
                "当前任务正在执行中" if same_task else "已有任务正在执行，请先停止当前任务",
                {
                    "task_id": task_id,
                    "active_task_id": active.get("task_id"),
                    "running": True,
                },
                now_text=now_text,
            ),
        }

    context = dict(static_context or {})
    if not context.get("ok"):
        readiness = context.get("readiness")
        if isinstance(readiness, dict):
            return {"proceed": False, "readiness": readiness}
        error_payload = dict(context.get("error") or {})
        return {
            "proceed": False,
            "readiness": readiness_failure(
                str(error_payload.get("code") or "task_static_context_invalid"),
                str(error_payload.get("message") or "任务静态条件无效"),
                {key: value for key, value in error_payload.items() if key != "message"},
                now_text=now_text,
            ),
        }

    if task_validation:
        return {"proceed": False, "readiness": task_validation}

    task_map_id = str(context.get("task_map_id") or "").strip() or "live_map"
    selected = str(selected_map_id or context.get("selected_map_id") or "").strip() or "live_map"
    first_annotation = context.get("first_annotation")
    annotation_map_ids = {
        str(item.get("map_id") or "").strip()
        for item in (context.get("annotations") or [])
        if isinstance(item, dict) and str(item.get("map_id") or "").strip()
    }
    task_contains_selected_map = bool(selected and selected in annotation_map_ids)
    if task_map_id != selected and not task_contains_selected_map:
        return {
            "proceed": False,
            "readiness": readiness_failure(
                "selected_map_mismatch",
                "当前地图与任务地图不一致，请先切换到任务对应地图",
                {
                    "task_id": task_id,
                    "task_map_id": task_map_id,
                    "selected_map_id": selected,
                    "first_waypoint": readiness_waypoint_payload(first_annotation),
                },
                now_text=now_text,
            ),
        }

    return {
        "proceed": True,
        "task_id": task_id,
        "task_map_id": task_map_id,
        "selected_map_id": selected,
        "first_annotation": first_annotation,
        "annotations": list(context.get("annotations") or []),
    }


def apply_task_start_pre_runtime_failure_state(
    tasks: Iterable[Dict[str, Any]],
    *,
    task_id: str,
    static_context: Dict[str, Any],
    task_validation: Optional[Dict[str, Any]],
    readiness: Dict[str, Any],
    now_text_value: str,
) -> Dict[str, Any]:
    target_id = str(task_id or "").strip()
    should_mark_invalid = bool((static_context or {}).get("mark_task_invalid") or task_validation)
    updated_tasks = [dict(task) for task in tasks]
    if not target_id or not should_mark_invalid:
        return {"tasks": updated_tasks, "changed": False, "task": None}

    last_error = str(
        (static_context or {}).get("last_error")
        or (readiness or {}).get("message")
        or "任务点位无效"
    )
    for task in updated_tasks:
        if str(task.get("id") or "").strip() != target_id:
            continue
        task["status"] = "invalid"
        task["updated_at"] = now_text_value
        task["last_error"] = last_error
        return {"tasks": updated_tasks, "changed": True, "task": dict(task)}

    return {"tasks": updated_tasks, "changed": False, "task": None}


def validate_task_annotations_for_map(
    annotations: Iterable[Optional[Dict[str, Any]]],
    task_map_id: str,
    *,
    target_map_payload: Optional[Dict[str, Any]] = None,
    target_map_payloads: Optional[Dict[str, Dict[str, Any]]] = None,
    allow_multi_floor: bool = False,
    allow_multi_map: bool = False,
    now_text: Optional[NowText] = None,
) -> Optional[Dict[str, Any]]:
    items = list(annotations)
    if not items:
        return readiness_failure("no_waypoint", "任务没有点位，请先添加点位后重新生成任务", now_text=now_text)
    missing = [index for index, item in enumerate(items) if item is None]
    if missing:
        return readiness_failure(
            "missing_waypoint",
            "任务中存在已删除的点位，请重新生成任务",
            {"missing_indices": missing},
            now_text=now_text,
        )
    expected_map_id = str(task_map_id or "").strip() or "live_map"
    bad_maps = []
    bad_floors = []
    bad_poses = []
    out_of_map = []
    blocked_cells = []
    unknown_cells = []
    floors = set()
    for index, annotation in enumerate(items):
        assert annotation is not None
        annotation_id = annotation.get("id")
        annotation_map_id = str(annotation.get("map_id") or "").strip() or "live_map"
        if not allow_multi_map and annotation_map_id != expected_map_id:
            bad_maps.append(
                {
                    "index": index,
                    "annotation_id": annotation_id,
                    "label": annotation.get("label"),
                    "annotation_map_id": annotation_map_id,
                    "task_map_id": expected_map_id,
                }
            )
        floor = str(annotation.get("floor") or "").strip()
        if floor:
            floors.add(floor)
        else:
            bad_floors.append(
                {
                    "index": index,
                    "annotation_id": annotation_id,
                    "label": annotation.get("label"),
                }
            )
        pose = annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {}
        if not is_plausible_waypoint_pose_dict(pose):
            bad_poses.append(
                {
                    "index": index,
                    "annotation_id": annotation_id,
                    "label": annotation.get("label"),
                    "pose": pose,
                }
            )
            continue
        point_map_payload = target_map_payload
        if target_map_payloads is not None and annotation_map_id:
            point_map_payload = target_map_payloads.get(annotation_map_id)
        if point_map_payload is not None:
            pose_error = pose_map_bounds_error(pose, point_map_payload, "任务点位")
            if pose_error:
                out_of_map.append(
                    {
                        "index": index,
                        "annotation_id": annotation_id,
                        "label": annotation.get("label"),
                        "pose": pose,
                        "detail": pose_error,
                    }
                )
                continue
            occupancy_error = pose_map_occupancy_error(pose, point_map_payload, "任务点位")
            if occupancy_error:
                target = blocked_cells if occupancy_error.get("code") == "pose_on_occupied_cell" else unknown_cells
                target.append(
                    {
                        "index": index,
                        "annotation_id": annotation_id,
                        "label": annotation.get("label"),
                        "pose": pose,
                        "detail": occupancy_error,
                    }
                )
    base = {
        "task_map_id": expected_map_id,
        "waypoint_count": len(items),
        "floors": sorted(floors),
    }
    if bad_maps:
        return readiness_failure(
            "waypoint_map_mismatch",
            "任务中存在不属于当前任务地图的点位，请重新生成任务",
            {**base, "bad_waypoints": bad_maps[:10]},
            now_text=now_text,
        )
    if bad_floors:
        return readiness_failure(
            "waypoint_floor_missing",
            "任务中存在楼层为空的点位，请重新标点后重新生成任务",
            {**base, "bad_waypoints": bad_floors[:10]},
            now_text=now_text,
        )
    if len(floors) > 1 and not allow_multi_floor:
        return readiness_failure(
            "waypoint_floor_mixed",
            "当前任务包含多个楼层点位，请先拆分为单楼层任务",
            base,
            now_text=now_text,
        )
    if bad_poses:
        return readiness_failure(
            "waypoint_pose_invalid",
            "任务中存在坐标无效的点位，请重新标点后重新生成任务",
            {**base, "bad_waypoints": bad_poses[:10]},
            now_text=now_text,
        )
    if out_of_map:
        return readiness_failure(
            "waypoint_out_of_map",
            "任务中存在超出任务地图范围的点位，请检查地图和点位后重新生成任务",
            {**base, "bad_waypoints": out_of_map[:10]},
            now_text=now_text,
        )
    if blocked_cells:
        return readiness_failure(
            "waypoint_on_occupied_cell",
            "任务中存在落在障碍物栅格上的点位，请重新标点后重新生成任务",
            {**base, "bad_waypoints": blocked_cells[:10]},
            now_text=now_text,
        )
    if unknown_cells:
        return readiness_failure(
            "waypoint_on_unknown_cell",
            "任务中存在落在未知区域的点位，请确认地图或重新标点后重新生成任务",
            {**base, "bad_waypoints": unknown_cells[:10]},
            now_text=now_text,
        )
    return None


def validate_task_create_map_selection(
    task_map_id: str,
    selected_map_id: Optional[str],
    *,
    allow_live_map: bool = False,
    now_text: Optional[NowText] = None,
) -> Optional[Dict[str, Any]]:
    expected = str(task_map_id or "").strip() or "live_map"
    selected = str(selected_map_id or "").strip()
    base = {
        "task_map_id": expected,
        "selected_map_id": selected or None,
    }
    if not selected:
        return readiness_failure(
            "selected_map_missing",
            "当前没有选中固定地图，请先在地图页选择当前地图，再标点生成任务",
            base,
            now_text=now_text,
        )
    if expected == "live_map" and not allow_live_map:
        return readiness_failure(
            "live_map_task_disabled",
            "当前阶段不允许基于实时 /map 生成可执行任务，请先选择固定地图",
            base,
            now_text=now_text,
        )
    if expected != selected:
        return readiness_failure(
            "task_create_map_mismatch",
            "生成任务只能使用当前选中地图的点位，请切换到点位所在地图或重新标点",
            base,
            now_text=now_text,
        )
    return None


def task_create_map_metadata_mismatch_payload(
    *,
    task_map_id: str,
    selected_map_id: Optional[str],
    selected_map_status: Dict[str, Any],
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    message = str(selected_map_status.get("message") or "网页选择地图与 Nav2 当前加载地图不一致")
    readiness = readiness_failure(
        "task_create_map_metadata_mismatch",
        message,
        {
            "task_map_id": task_map_id,
            "selected_map_id": selected_map_id,
            "selected_map_status": selected_map_status,
        },
        now_text=now_text,
    )
    return {
        "message": message,
        "readiness": readiness,
        "error_extra": readiness_error_payload(readiness),
    }


def task_create_static_context(
    payload: Dict[str, Any],
    annotations_by_id: Dict[str, Dict[str, Any]],
    *,
    selected_map_id: Optional[str],
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    annotation_ids = [str(item) for item in (payload.get("annotation_ids") or []) if str(item).strip()]
    if not annotation_ids:
        return {
            "ok": False,
            "error": contract_error("任务至少需要一个点位", {"code": "task_create_no_waypoint"}),
            "readiness": readiness_failure(
                "task_create_no_waypoint",
                "任务至少需要一个点位",
                now_text=now_text,
            ),
        }

    missing = [item for item in annotation_ids if item not in annotations_by_id]
    if missing:
        return {
            "ok": False,
            "error": contract_error("任务中存在已删除的点位", {"code": "task_create_missing_waypoint", "missing": missing}),
            "readiness": readiness_failure(
                "task_create_missing_waypoint",
                "任务中存在已删除的点位",
                {"missing": missing},
                now_text=now_text,
            ),
        }

    annotations = [annotations_by_id[item] for item in annotation_ids]
    order_error = validate_task_annotation_order(annotations)
    if order_error:
        return {
            "ok": False,
            "error": order_error,
            "readiness": readiness_failure(
                str(order_error.get("code") or "waypoint_order_invalid"),
                str(order_error.get("message") or "任务点位顺序无效"),
                {key: value for key, value in order_error.items() if key not in ("ok", "message")},
                now_text=now_text,
            ),
        }

    task_map_id = str(payload.get("map_id") or "").strip() or str(selected_map_id or "").strip()
    map_selection_error = validate_task_create_map_selection(
        task_map_id or "",
        selected_map_id,
        now_text=now_text,
    )
    if map_selection_error:
        return {
            "ok": False,
            "error": contract_error(str(map_selection_error.get("message") or "任务地图无效"), readiness_error_payload(map_selection_error)),
            "readiness": map_selection_error,
        }

    return {
        "ok": True,
        "annotation_ids": annotation_ids,
        "annotations": annotations,
        "task_map_id": task_map_id,
        "selected_map_id": selected_map_id,
        "name": str(payload.get("name") or "巡检任务").strip(),
    }


def build_task_create_record(
    context: Dict[str, Any],
    *,
    task_id: str,
    now_text_value: str,
) -> Dict[str, Any]:
    return {
        "id": task_id,
        "name": str(context.get("name") or "巡检任务").strip() or "巡检任务",
        "map_id": str(context.get("task_map_id") or "").strip(),
        "annotation_ids": [str(item) for item in (context.get("annotation_ids") or [])],
        "status": "ready",
        "created_at": now_text_value,
    }


def task_start_static_context(
    task_id: str,
    task: Optional[Dict[str, Any]],
    annotations_by_id: Dict[str, Dict[str, Any]],
    *,
    selected_map_id: Optional[str],
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    task_id = str(task_id or "").strip()
    selected = str(selected_map_id or "").strip() or "live_map"
    if task is None:
        base = {"task_id": task_id}
        return {
            "ok": False,
            "error": contract_error("任务不存在", base),
            "readiness": readiness_failure("task_missing", "任务不存在", base, now_text=now_text),
            "mark_task_invalid": False,
        }

    task_status = str(task.get("status") or "ready").strip() or "ready"
    if not task_status_allows_start(task_status):
        if task_status == "invalid":
            message = "任务点位已失效，请重新生成任务"
            code = "task_invalid"
        elif task_status == "running":
            message = "任务正在执行中"
            code = "task_status_blocked"
        else:
            message = f"任务状态 {task_status} 不允许启动"
            code = "task_status_blocked"
        base = {"task_id": task_id, "task_status": task_status}
        return {
            "ok": False,
            "error": contract_error(message, base),
            "readiness": readiness_failure(code, message, base, now_text=now_text),
            "mark_task_invalid": False,
        }

    annotation_ids = [str(item) for item in (task.get("annotation_ids") or []) if str(item).strip()]
    if not annotation_ids:
        base = {"task_id": task_id}
        return {
            "ok": False,
            "error": contract_error("任务没有点位", base),
            "readiness": readiness_failure(
                "no_waypoint",
                "任务没有点位，请先添加点位后重新生成任务",
                base,
                now_text=now_text,
            ),
            "mark_task_invalid": False,
        }

    missing = [item for item in annotation_ids if item not in annotations_by_id]
    if missing:
        base = {"task_id": task_id, "missing": missing}
        return {
            "ok": False,
            "error": contract_error("任务中存在已删除的点位，请重新生成任务", base),
            "readiness": readiness_failure(
                "missing_waypoint",
                "任务中存在已删除的点位，请重新生成任务",
                base,
                now_text=now_text,
            ),
            "mark_task_invalid": True,
            "last_error": "任务中存在已删除的点位，请重新生成任务",
        }

    annotations = [annotations_by_id[item] for item in annotation_ids]
    order_error = validate_task_annotation_order(annotations)
    if order_error:
        readiness = readiness_failure(
            str(order_error.get("code") or "waypoint_order_invalid"),
            str(order_error.get("message") or "任务点位顺序无效"),
            {
                **{key: value for key, value in order_error.items() if key not in ("ok", "message")},
                "task_id": task_id,
            },
            now_text=now_text,
        )
        return {
            "ok": False,
            "error": order_error,
            "readiness": readiness,
            "mark_task_invalid": False,
        }

    task_map_id = str(task.get("map_id") or "").strip() or selected
    first_annotation = annotations[0]
    return {
        "ok": True,
        "task_id": task_id,
        "task_map_id": task_map_id,
        "selected_map_id": selected,
        "annotation_ids": annotation_ids,
        "annotations": annotations,
        "first_annotation": first_annotation,
        "task_status": task_status,
    }


def apply_deleted_annotation_to_tasks(
    tasks: Iterable[Dict[str, Any]],
    annotation_id: str,
    *,
    now_text_value: str,
) -> Dict[str, Any]:
    deleted_id = str(annotation_id or "").strip()
    updated_tasks = []
    affected_tasks = []
    changed = False
    for task in tasks:
        updated = dict(task)
        ids = [str(item) for item in (updated.get("annotation_ids") or [])]
        if deleted_id not in ids:
            updated_tasks.append(updated)
            continue
        kept_ids = [item for item in ids if item != deleted_id]
        updated["annotation_ids"] = kept_ids
        updated["updated_at"] = now_text_value
        if not kept_ids:
            updated["status"] = "invalid"
        elif updated.get("status") in ("ready", "stopped", "completed"):
            updated["status"] = "ready"
        affected_tasks.append(updated.get("id"))
        changed = True
        updated_tasks.append(updated)
    return {
        "tasks": updated_tasks,
        "affected_tasks": affected_tasks,
        "changed": changed,
    }


def apply_task_name_update(
    tasks: Iterable[Dict[str, Any]],
    settings: Dict[str, Any],
    *,
    task_id: str,
    name: str,
    now_text_value: str,
) -> Dict[str, Any]:
    target_id = str(task_id or "").strip()
    updated_tasks = [dict(task) for task in tasks]
    updated_settings = dict(settings)
    updated_task = None
    task_found = False
    settings_changed = False

    for task in updated_tasks:
        if str(task.get("id") or "").strip() != target_id:
            continue
        task_found = True
        task["name"] = str(name)
        task["updated_at"] = now_text_value
        updated_task = dict(task)
        break

    active = updated_settings.get("active_task")
    if isinstance(active, dict) and str(active.get("task_id") or "").strip() == target_id:
        active_updated = dict(active)
        active_updated["task_name"] = str(name)
        updated_settings["active_task"] = active_updated
        settings_changed = True

    return {
        "ok": task_found,
        "code": "task_updated" if task_found else "task_missing",
        "message": "任务名称已更新" if task_found else "任务不存在",
        "tasks": updated_tasks,
        "settings": updated_settings,
        "task": updated_task,
        "settings_changed": settings_changed,
        "updated_task_id": target_id if task_found else None,
    }


def apply_task_delete(
    tasks: Iterable[Dict[str, Any]],
    settings: Dict[str, Any],
    *,
    task_id: str,
) -> Dict[str, Any]:
    target_id = str(task_id or "").strip()
    updated_settings = dict(settings)
    active = updated_settings.get("active_task")
    if (
        isinstance(active, dict)
        and active.get("status") == "running"
        and str(active.get("task_id") or "").strip() == target_id
    ):
        return {
            "ok": False,
            "code": "task_running",
            "message": "任务正在执行，请先停止当前任务再删除",
            "tasks": [dict(task) for task in tasks],
            "settings": updated_settings,
            "settings_changed": False,
        }

    updated_tasks = []
    deleted = False
    for task in tasks:
        if str(task.get("id") or "").strip() == target_id:
            deleted = True
            continue
        updated_tasks.append(dict(task))

    settings_changed = False
    if deleted and isinstance(active, dict) and str(active.get("task_id") or "").strip() == target_id:
        updated_settings["active_task"] = None
        settings_changed = True

    return {
        "ok": deleted,
        "code": "deleted" if deleted else "task_missing",
        "message": "任务已删除" if deleted else "任务不存在",
        "tasks": updated_tasks,
        "settings": updated_settings,
        "settings_changed": settings_changed,
        "deleted_task_id": target_id if deleted else None,
    }


def stop_stale_running_tasks(
    tasks: Iterable[Dict[str, Any]],
    *,
    active_task_id: Optional[str],
    now_text_value: str,
) -> Dict[str, Any]:
    active_id = str(active_task_id or "").strip()
    updated_tasks = []
    stopped_task_ids = []
    changed = False
    for task in tasks:
        updated = dict(task)
        task_id = str(updated.get("id") or "").strip()
        if updated.get("status") == "running" and (not active_id or task_id != active_id):
            updated["status"] = "stopped"
            updated["updated_at"] = now_text_value
            stopped_task_ids.append(updated.get("id"))
            changed = True
        updated_tasks.append(updated)
    return {
        "tasks": updated_tasks,
        "stopped_task_ids": stopped_task_ids,
        "changed": changed,
    }


def task_list_filter_payload(
    tasks: Iterable[Dict[str, Any]],
    *,
    selected_map_id: Optional[str],
    include_all: bool,
    annotations_by_id: Optional[Dict[str, Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    all_tasks = [dict(task) for task in tasks]
    if include_all:
        visible_tasks = all_tasks
        hidden_task_count = 0
    else:
        selected = str(selected_map_id or "")
        visible_tasks = [
            task
            for task in all_tasks
            if selected_map_id
            and (
                str(task.get("map_id") or "") == selected
                or any(
                    str((annotations_by_id or {}).get(str(annotation_id), {}).get("map_id") or "") == selected
                    for annotation_id in (task.get("annotation_ids") or [])
                )
            )
        ]
        hidden_task_count = len(all_tasks) - len(visible_tasks)
    return {
        "tasks": visible_tasks,
        "include_all": bool(include_all),
        "hidden_task_count": hidden_task_count,
        "total_task_count": len(all_tasks),
    }


def normalize_startup_task_runtime_state(
    settings: Dict[str, Any],
    tasks: Iterable[Dict[str, Any]],
    *,
    now_text_value: str,
) -> Dict[str, Any]:
    updated_settings = dict(settings)
    updated_tasks = [dict(task) for task in tasks]
    active = updated_settings.get("active_task")
    active_task_id = None
    changed = False
    cleared_active_task = False
    stopped_task_ids = []

    if isinstance(active, dict) and active:
        active_task_id = str(active.get("task_id") or "").strip()
        if active.get("status") == "running" and active_task_id:
            for task in updated_tasks:
                if str(task.get("id") or "").strip() != active_task_id:
                    continue
                if task.get("status") == "running":
                    task["status"] = "stopped"
                    task["updated_at"] = now_text_value
                    stopped_task_ids.append(task.get("id"))
                    changed = True
                break
        updated_settings["active_task"] = None
        cleared_active_task = True
        changed = True

    stale_result = stop_stale_running_tasks(
        updated_tasks,
        active_task_id=None,
        now_text_value=now_text_value,
    )
    if stale_result.get("changed"):
        updated_tasks = list(stale_result["tasks"])
        stopped_task_ids.extend(stale_result.get("stopped_task_ids") or [])
        changed = True

    return {
        "settings": updated_settings,
        "tasks": updated_tasks,
        "changed": changed,
        "cleared_active_task": cleared_active_task,
        "stopped_task_ids": list(dict.fromkeys(stopped_task_ids)),
    }


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def validate_task_start_expectations(
    payload: Dict[str, Any],
    task: Dict[str, Any],
    first_annotation: Optional[Dict[str, Any]],
    task_map_id: str,
) -> Optional[Dict[str, Any]]:
    expected_ids = payload.get("expected_annotation_ids")
    if isinstance(expected_ids, list):
        normalized_expected = [str(item) for item in expected_ids if str(item).strip()]
        actual_ids = [str(item) for item in (task.get("annotation_ids") or [])]
        if normalized_expected != actual_ids:
            return contract_error(
                "任务点顺序已变化，请刷新任务列表后重新确认执行",
                {"expected_annotation_ids": normalized_expected, "actual_annotation_ids": actual_ids},
            )

    expected_first_id = str(payload.get("expected_first_annotation_id") or "").strip()
    if expected_first_id and first_annotation is not None and expected_first_id != str(first_annotation.get("id") or ""):
        return contract_error(
            "任务首点已变化，请刷新任务列表后重新确认执行",
            {
                "expected_first_annotation_id": expected_first_id,
                "actual_first_annotation_id": first_annotation.get("id"),
            },
        )

    expected_map_id = str(payload.get("expected_map_id") or "").strip()
    if expected_map_id and expected_map_id != str(task_map_id or "").strip():
        return contract_error(
            "任务地图已变化，请刷新任务列表后重新确认执行",
            {"expected_map_id": expected_map_id, "actual_map_id": task_map_id},
        )

    expected_updated_at = str(payload.get("expected_task_updated_at") or "").strip()
    actual_updated_at = str(task.get("updated_at") or task.get("created_at") or "").strip()
    if expected_updated_at and actual_updated_at and expected_updated_at != actual_updated_at:
        return contract_error(
            "任务已被更新，请刷新任务列表后重新确认执行",
            {"expected_task_updated_at": expected_updated_at, "actual_task_updated_at": actual_updated_at},
        )

    expected_pose = payload.get("expected_first_pose")
    if isinstance(expected_pose, dict) and first_annotation is not None:
        pose = first_annotation.get("pose") if isinstance(first_annotation.get("pose"), dict) else {}
        for key, tolerance in (("x", 0.05), ("y", 0.05), ("z", 0.10), ("yaw", 0.10)):
            if key not in expected_pose:
                continue
            try:
                expected = float(expected_pose.get(key))
                actual = float(pose.get(key, 0.0))
            except (TypeError, ValueError):
                return contract_error("任务首点确认坐标无效，请刷新任务列表后重试", {"field": key})
            error = abs(wrap_angle(expected - actual)) if key == "yaw" else abs(expected - actual)
            if error > tolerance:
                return contract_error(
                    "任务首点坐标已变化，请刷新任务列表后重新确认执行",
                    {
                        "field": key,
                        "expected": expected,
                        "actual": actual,
                        "error": error,
                        "tolerance": tolerance,
                    },
                )
    return None


def validate_task_annotation_order(annotations: Iterable[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    items = list(annotations)
    for index, annotation in enumerate(items):
        if annotation.get("manual_point_type") == "charge" and index != len(items) - 1:
            return contract_error(
                "充电点必须放在任务最后。开发手册说明充电点到达后会自动进入充电并保持，不能继续串后续点位。",
                {"annotation_id": annotation.get("id"), "label": annotation.get("label")},
            )
    return None
