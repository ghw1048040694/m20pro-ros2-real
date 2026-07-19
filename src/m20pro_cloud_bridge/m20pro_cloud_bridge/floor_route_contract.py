from __future__ import annotations

import math
import re
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple


ResolveMapYaml = Callable[[Dict[str, Any]], str]

_POINT_FIELDS = (
    ("entry_annotation_id", "stair_entry", "起始层爬楼梯点"),
    ("source_platform_annotation_id", "stair_switch", "起始层楼层切换点"),
    ("target_platform_annotation_id", "stair_switch", "目标层楼层切换点"),
    ("post_exit_annotation_id", "stair_exit", "目标层出楼梯点"),
)


def _error(code: str, message: str, **extra: Any) -> Dict[str, Any]:
    return {"ok": False, "code": code, "message": message, **extra}


def _pose(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    try:
        pose = {
            "x": float(value["x"]),
            "y": float(value["y"]),
            "z": float(value.get("z", 0.0)),
            "yaw": float(value["yaw"]),
        }
    except (KeyError, TypeError, ValueError):
        return None
    return pose if all(math.isfinite(item) for item in pose.values()) else None


def _floor_level(value: Any) -> Optional[int]:
    text = str(value or "").strip().upper()
    match = re.fullmatch(r"F(\d+)", text)
    if match:
        return int(match.group(1))
    match = re.fullmatch(r"B(\d+)", text)
    if match:
        return -int(match.group(1))
    return None


def _direction(source_floor: str, target_floor: str, requested: Any) -> str:
    explicit = str(requested or "").strip().lower()
    if explicit in ("up", "down"):
        return explicit
    source_level = _floor_level(source_floor)
    target_level = _floor_level(target_floor)
    if source_level is not None and target_level is not None:
        return "up" if target_level > source_level else "down"
    return "up"


def _factory_path(record: Dict[str, Any]) -> str:
    return str(record.get("factory_apply_path") or record.get("source_path") or "").strip()


def validate_floor_route(
    payload: Dict[str, Any],
    *,
    annotations_by_id: Dict[str, Dict[str, Any]],
    maps_by_id: Dict[str, Dict[str, Any]],
    resolve_map_yaml: ResolveMapYaml,
    route_id: str,
    now_text: str,
) -> Dict[str, Any]:
    selected: Dict[str, Dict[str, Any]] = {}
    for field, expected_type, label in _POINT_FIELDS:
        annotation_id = str(payload.get(field) or "").strip()
        if not annotation_id:
            return _error("floor_route_point_missing", f"请选择{label}", field=field)
        annotation = annotations_by_id.get(annotation_id)
        if annotation is None:
            return _error("floor_route_point_unknown", f"{label}不存在或已删除", field=field)
        actual_type = str(annotation.get("type") or "").strip()
        if actual_type != expected_type:
            return _error(
                "floor_route_point_type_mismatch",
                f"{label}必须使用“{expected_type}”类型点位",
                field=field,
                expected_type=expected_type,
                actual_type=actual_type,
            )
        if _pose(annotation.get("pose")) is None:
            return _error("floor_route_pose_invalid", f"{label}坐标无效", field=field)
        selected[field] = annotation

    entry = selected["entry_annotation_id"]
    source_platform = selected["source_platform_annotation_id"]
    target_platform = selected["target_platform_annotation_id"]
    post_exit = selected["post_exit_annotation_id"]
    source_floor = str(entry.get("floor") or "").strip()
    target_floor = str(target_platform.get("floor") or "").strip()
    if not source_floor or not target_floor:
        return _error("floor_route_floor_missing", "楼梯路线点位缺少楼层")
    if source_floor == target_floor:
        return _error("floor_route_same_floor", "跨楼层路线的起始层和目标层不能相同")

    source_map_id = str(entry.get("map_id") or "").strip()
    target_map_id = str(target_platform.get("map_id") or "").strip()
    if not source_map_id or not target_map_id:
        return _error("floor_route_map_missing", "楼梯路线点位必须绑定固定地图")
    for field, annotation in (
        ("source_platform_annotation_id", source_platform),
        ("target_platform_annotation_id", target_platform),
        ("post_exit_annotation_id", post_exit),
    ):
        expected_floor = source_floor if field == "source_platform_annotation_id" else target_floor
        expected_map = source_map_id if field == "source_platform_annotation_id" else target_map_id
        if str(annotation.get("floor") or "").strip() != expected_floor:
            return _error("floor_route_floor_mismatch", "同一侧的楼梯点位楼层不一致", field=field)
        if str(annotation.get("map_id") or "").strip() != expected_map:
            return _error("floor_route_map_mismatch", "同一侧的楼梯点位来自不同地图", field=field)

    source_map = maps_by_id.get(source_map_id)
    target_map = maps_by_id.get(target_map_id)
    if source_map is None or target_map is None:
        return _error("floor_route_map_unknown", "楼梯路线引用的地图不存在")
    if str(source_map.get("floor") or "").strip() != source_floor:
        return _error("floor_route_source_map_floor_mismatch", "起始层地图楼层与点位不一致")
    if str(target_map.get("floor") or "").strip() != target_floor:
        return _error("floor_route_target_map_floor_mismatch", "目标层地图楼层与点位不一致")

    source_factory_path = _factory_path(source_map)
    target_factory_path = _factory_path(target_map)
    factory_root = "/var/opt/robot/data/maps/"
    if not source_factory_path.startswith(factory_root) or source_factory_path.endswith("/active"):
        return _error("floor_route_source_factory_map_missing", "起始层地图缺少可切换的 106 原厂地图包")
    if not target_factory_path.startswith(factory_root) or target_factory_path.endswith("/active"):
        return _error("floor_route_target_factory_map_missing", "目标层地图缺少可切换的 106 原厂地图包")

    source_map_yaml = str(resolve_map_yaml(source_map) or "").strip()
    target_map_yaml = str(resolve_map_yaml(target_map) or "").strip()
    if not source_map_yaml or not target_map_yaml:
        return _error("floor_route_nav2_map_missing", "楼梯路线地图缺少 104 Nav2 栅格 yaml")

    name = str(payload.get("name") or f"{source_floor}到{target_floor}").strip()
    direction = _direction(source_floor, target_floor, payload.get("direction"))
    return {
        "ok": True,
        "route": {
            "id": str(route_id),
            "name": name,
            "source_floor": source_floor,
            "target_floor": target_floor,
            "direction": direction,
            "source_map_id": source_map_id,
            "target_map_id": target_map_id,
            "source_map_yaml": source_map_yaml,
            "target_map_yaml": target_map_yaml,
            "source_factory_path": source_factory_path,
            "target_factory_path": target_factory_path,
            "entry_annotation_id": str(entry.get("id")),
            "source_platform_annotation_id": str(source_platform.get("id")),
            "target_platform_annotation_id": str(target_platform.get("id")),
            "post_exit_annotation_id": str(post_exit.get("id")),
            "entry": _pose(entry.get("pose")),
            "source_platform": _pose(source_platform.get("pose")),
            "target_platform": _pose(target_platform.get("pose")),
            "post_exit": _pose(post_exit.get("pose")),
            "updated_at": str(now_text),
        },
    }


def upsert_floor_route(routes: Iterable[Dict[str, Any]], route: Dict[str, Any]) -> List[Dict[str, Any]]:
    route_id = str(route.get("id") or "")
    source = str(route.get("source_floor") or "")
    target = str(route.get("target_floor") or "")
    result = [
        dict(item)
        for item in routes
        if str(item.get("id") or "") != route_id
        and not (
            str(item.get("source_floor") or "") == source
            and str(item.get("target_floor") or "") == target
        )
    ]
    result.append(dict(route))
    return result


def validate_floor_route_set(routes: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    floor_maps: Dict[str, str] = {}
    for route in routes:
        for floor_key, map_key in (("source_floor", "source_map_id"), ("target_floor", "target_map_id")):
            floor = str(route.get(floor_key) or "").strip()
            map_id = str(route.get(map_key) or "").strip()
            if not floor or not map_id:
                return _error("floor_route_incomplete", "跨楼层路线缺少楼层或地图身份")
            if floor in floor_maps and floor_maps[floor] != map_id:
                return _error(
                    "floor_route_map_conflict",
                    f"{floor} 的跨楼层路线引用了不同地图，请统一使用一张正式地图",
                    floor=floor,
                    map_ids=sorted({floor_maps[floor], map_id}),
                )
            floor_maps[floor] = map_id
    return {"ok": True, "floor_maps": floor_maps}


def remove_floor_route(routes: Iterable[Dict[str, Any]], route_id: Any) -> Tuple[List[Dict[str, Any]], bool]:
    target = str(route_id or "").strip()
    source = [dict(item) for item in routes]
    result = [item for item in source if str(item.get("id") or "") != target]
    return result, len(result) != len(source)


def resolve_floor_switch_request(
    request: Dict[str, Any],
    *,
    routes: Iterable[Dict[str, Any]],
    active_task: Dict[str, Any],
    selected_map_id: Any,
) -> Dict[str, Any]:
    request_id = str(request.get("request_id") or "").strip()
    if not request_id:
        return _error("floor_switch_request_id_missing", "切层请求缺少 request_id")
    if str(active_task.get("status") or "") != "running" or not bool(active_task.get("multi_floor")):
        return _error("floor_switch_no_active_task", "没有运行中的跨楼层任务，拒绝自动切层")

    route_id = str(request.get("route_id") or "").strip()
    if not route_id:
        return _error("floor_switch_route_id_missing", "切层请求缺少已保存路线 ID")
    route = next(
        (dict(item) for item in routes if str(item.get("id") or "") == route_id),
        None,
    )
    if route is None:
        return _error("floor_switch_route_missing", f"没有找到跨楼层路线 {route_id}")

    expected = {
        "source_floor": str(route.get("source_floor") or ""),
        "target_floor": str(route.get("target_floor") or ""),
        "target_map_id": str(route.get("target_map_id") or ""),
    }
    mismatches = {
        key: {"requested": str(request.get(key) or ""), "configured": value}
        for key, value in expected.items()
        if str(request.get(key) or "") != value
    }
    if mismatches:
        return _error(
            "floor_switch_request_route_mismatch",
            "切层请求与已保存路线不一致，已拒绝执行",
            mismatches=mismatches,
        )

    source_map_id = str(route.get("source_map_id") or "")
    current_map_id = str(selected_map_id or "")
    if not source_map_id or current_map_id != source_map_id:
        return _error(
            "floor_switch_source_map_mismatch",
            "当前地图不是路线起始地图，已拒绝自动切层",
            selected_map_id=current_map_id or None,
            expected_source_map_id=source_map_id or None,
        )

    task_source = str(active_task.get("last_floor_goal_source_floor") or "").strip()
    task_target = str(active_task.get("last_floor_goal_target_floor") or "").strip()
    if (task_source and task_source != expected["source_floor"]) or (
        task_target and task_target != expected["target_floor"]
    ):
        return _error(
            "floor_switch_task_route_mismatch",
            "当前任务楼层与切层路线不一致，已拒绝执行",
            task_source_floor=task_source or None,
            task_target_floor=task_target or None,
            route_source_floor=expected["source_floor"],
            route_target_floor=expected["target_floor"],
        )

    return {
        "ok": True,
        "request_id": request_id,
        "task_id": str(active_task.get("task_id") or ""),
        "route": route,
        "source_map_id": source_map_id,
        **expected,
    }


def runtime_floor_config(routes: Iterable[Dict[str, Any]], *, mission: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    config: Dict[str, Any] = {
        "mission": {
            "name": "m20pro_runtime_navigation",
            "frame_id": "map",
            "stair_transition_defaults": {
                "model": "shared_platform",
                "point_type": "transition",
                "terrain": "stairs",
                "nav_mode": "autonomous",
                "direction_mode": "forward",
                "speed": "low",
                "obstacle_policy": "stop_only",
                "entry_margin_m": 0.8,
            },
        },
        "floors": {},
    }
    if isinstance(mission, dict):
        config["mission"].update(mission)

    for route in routes:
        source_floor = str(route.get("source_floor") or "").strip()
        target_floor = str(route.get("target_floor") or "").strip()
        if not source_floor or not target_floor:
            continue
        source = config["floors"].setdefault(
            source_floor,
            {
                "level": _floor_level(source_floor),
                "map_id": route.get("source_map_id"),
                "map_yaml": route.get("source_map_yaml"),
                "factory_apply_path": route.get("source_factory_path"),
                "initial_pose": dict(route.get("entry") or {}),
                "stairs": {},
                "terrain_segments": {},
            },
        )
        config["floors"].setdefault(
            target_floor,
            {
                "level": _floor_level(target_floor),
                "map_id": route.get("target_map_id"),
                "map_yaml": route.get("target_map_yaml"),
                "factory_apply_path": route.get("target_factory_path"),
                "initial_pose": dict(route.get("target_platform") or {}),
                "stairs": {},
                "terrain_segments": {},
            },
        )
        source["stairs"][str(route.get("id") or f"{source_floor}_to_{target_floor}")] = {
            "route_id": route.get("id"),
            "name": route.get("name"),
            "direction": route.get("direction"),
            "target_floor": target_floor,
            "target_map_id": route.get("target_map_id"),
            "entry": dict(route.get("entry") or {}),
            "source_platform": dict(route.get("source_platform") or {}),
            "target_platform": dict(route.get("target_platform") or {}),
            "post_exit": dict(route.get("post_exit") or {}),
            "transition": dict(config["mission"]["stair_transition_defaults"]),
        }
    return config


def floor_route_public_payload(
    routes: Iterable[Dict[str, Any]],
    *,
    annotations: Iterable[Dict[str, Any]],
    maps: Iterable[Dict[str, Any]],
) -> Dict[str, Any]:
    point_types = {expected for _, expected, _ in _POINT_FIELDS}
    candidates = [
        {
            "id": str(item.get("id") or ""),
            "label": str(item.get("label") or item.get("id") or ""),
            "floor": str(item.get("floor") or ""),
            "map_id": str(item.get("map_id") or ""),
            "type": str(item.get("type") or ""),
        }
        for item in annotations
        if str(item.get("type") or "") in point_types and _pose(item.get("pose")) is not None
    ]
    map_items = [
        {
            "id": str(item.get("id") or ""),
            "name": str(item.get("name") or item.get("id") or ""),
            "floor": str(item.get("floor") or ""),
            "factory_ready": _factory_path(item).startswith("/var/opt/robot/data/maps/")
            and not _factory_path(item).endswith("/active"),
        }
        for item in maps
        if item.get("id")
    ]
    return {
        "ok": True,
        "routes": [dict(item) for item in routes],
        "candidates": candidates,
        "maps": map_items,
    }
