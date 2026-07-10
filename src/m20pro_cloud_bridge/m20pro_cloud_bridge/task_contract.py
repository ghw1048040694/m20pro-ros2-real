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


def validation_error_payload(validation: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "code": validation.get("code"),
        "validation": validation,
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
    payload["result_file_prefix"] = annotation.get("result_file_prefix")
    payload["radar"] = annotation.get("radar") if isinstance(annotation.get("radar"), dict) else {}
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


def apply_task_start_pre_runtime_failure_state(
    tasks: Iterable[Dict[str, Any]],
    *,
    task_id: str,
    static_context: Dict[str, Any],
    task_validation: Optional[Dict[str, Any]],
    validation: Dict[str, Any],
    now_text_value: str,
) -> Dict[str, Any]:
    target_id = str(task_id or "").strip()
    should_mark_invalid = bool((static_context or {}).get("mark_task_invalid") or task_validation)
    updated_tasks = [dict(task) for task in tasks]
    if not target_id or not should_mark_invalid:
        return {"tasks": updated_tasks, "changed": False, "task": None}

    last_error = str(
        (static_context or {}).get("last_error")
        or (validation or {}).get("message")
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
    validation = readiness_failure(
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
        "validation": validation,
        "error_extra": validation_error_payload(validation),
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
            "validation": readiness_failure(
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
            "validation": readiness_failure(
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
            "validation": readiness_failure(
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
            "error": contract_error(str(map_selection_error.get("message") or "任务地图无效"), validation_error_payload(map_selection_error)),
            "validation": map_selection_error,
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
            "validation": readiness_failure("task_missing", "任务不存在", base, now_text=now_text),
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
            "validation": readiness_failure(code, message, base, now_text=now_text),
            "mark_task_invalid": False,
        }

    annotation_ids = [str(item) for item in (task.get("annotation_ids") or []) if str(item).strip()]
    if not annotation_ids:
        base = {"task_id": task_id}
        return {
            "ok": False,
            "error": contract_error("任务没有点位", base),
            "validation": readiness_failure(
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
            "validation": readiness_failure(
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
        validation = readiness_failure(
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
            "validation": validation,
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
