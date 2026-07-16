from __future__ import annotations

import math
import re
from collections import deque
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .mapping_contract import mapping_floor_steps


def floor_sort_key(value: Any) -> Tuple[int, int, str]:
    text = str(value or "").strip()
    match = re.search(r"-?\d+", text)
    return (0, int(match.group(0)), text) if match else (1, 0, text)


def _pose_ready(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    try:
        return all(math.isfinite(float(value[key])) for key in ("x", "y", "yaw"))
    except (KeyError, TypeError, ValueError):
        return False


def stair_routes_from_config(config: Dict[str, Any]) -> List[Dict[str, Any]]:
    routes: List[Dict[str, Any]] = []
    floors = config.get("floors") if isinstance(config.get("floors"), dict) else {}
    for source_floor, floor_data in floors.items():
        stairs = floor_data.get("stairs") if isinstance(floor_data, dict) else {}
        if not isinstance(stairs, dict):
            continue
        for name, stair in stairs.items():
            if not isinstance(stair, dict):
                continue
            target_floor = str(stair.get("target_floor") or "").strip()
            if not target_floor:
                continue
            transition = stair.get("transition") if isinstance(stair.get("transition"), dict) else {}
            poses = {
                "entry": stair.get("entry"),
                "source_platform": stair.get("source_platform") or stair.get("traverse_to"),
                "target_platform": stair.get("target_platform") or stair.get("target_exit"),
                "post_exit": stair.get("post_exit"),
            }
            missing = [key for key, pose in poses.items() if not _pose_ready(pose)]
            routes.append(
                {
                    "id": f"{source_floor}:{name}",
                    "name": str(name),
                    "source_floor": str(source_floor),
                    "target_floor": target_floor,
                    "direction": str(stair.get("direction") or ""),
                    "model": str(transition.get("model") or "shared_platform"),
                    "poses": poses,
                    "missing_poses": missing,
                    "configured": not missing,
                }
            )
    return sorted(
        routes,
        key=lambda item: (
            floor_sort_key(item["source_floor"]),
            floor_sort_key(item["target_floor"]),
            item["name"],
        ),
    )


def find_floor_path(routes: Iterable[Dict[str, Any]], source: str, target: str) -> Optional[List[str]]:
    source = str(source or "").strip()
    target = str(target or "").strip()
    if not source or not target:
        return None
    if source == target:
        return [source]
    graph: Dict[str, List[str]] = {}
    for route in routes:
        if not route.get("configured"):
            continue
        start = str(route.get("source_floor") or "").strip()
        end = str(route.get("target_floor") or "").strip()
        if start and end:
            graph.setdefault(start, []).append(end)
    queue = deque([(source, [source])])
    visited = {source}
    while queue:
        current, path = queue.popleft()
        for next_floor in sorted(graph.get(current, []), key=floor_sort_key):
            if next_floor == target:
                return path + [next_floor]
            if next_floor not in visited:
                visited.add(next_floor)
                queue.append((next_floor, path + [next_floor]))
    return None


def _map_summary(record: Dict[str, Any]) -> Dict[str, Any]:
    source_path = str(record.get("factory_apply_path") or record.get("source_path") or "")
    return {
        "id": str(record.get("id") or ""),
        "name": str(record.get("name") or record.get("id") or ""),
        "floor": str(record.get("floor") or ""),
        "source": str(record.get("source") or ""),
        "created_at": record.get("created_at"),
        "factory_ready": source_path.startswith("/var/opt/robot/data/maps/")
        and source_path != "/var/opt/robot/data/maps/active",
        "source_note": record.get("source_note"),
    }


def _preferred_map(records: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not records:
        return None
    ranked = sorted(
        records,
        key=lambda item: (
            1 if item.get("factory_ready") else 0,
            1 if item.get("source") != "project_builtin" else 0,
            str(item.get("created_at") or ""),
        ),
        reverse=True,
    )
    return ranked[0]


def build_multi_floor_workspace(
    *,
    floor_config: Dict[str, Any],
    maps: Iterable[Dict[str, Any]],
    annotations: Iterable[Dict[str, Any]],
    sessions: Iterable[Dict[str, Any]],
    current_floor: Any,
    selected_map_id: Any,
) -> Dict[str, Any]:
    configured_floors = floor_config.get("floors") if isinstance(floor_config.get("floors"), dict) else {}
    map_items = [_map_summary(item) for item in maps if isinstance(item, dict) and item.get("id")]
    annotation_items = [dict(item) for item in annotations if isinstance(item, dict) and item.get("id")]
    session_items = [dict(item) for item in sessions if isinstance(item, dict)]
    route_floor_ids = set(str(item).strip() for item in configured_floors if str(item).strip())
    map_floor_ids = {item["floor"] for item in map_items if item["floor"]}
    annotation_floor_ids = {
        str(item.get("floor") or "").strip()
        for item in annotation_items
        if str(item.get("floor") or "").strip()
    }
    session_floor_ids = {
        str(floor).strip()
        for session in session_items
        for floor in (session.get("floors") or [])
        if str(floor).strip()
    }
    # With no explicit route profile, map/point labels are ordinary runtime
    # identities and must remain visible in the workspace. A route profile
    # still acts as a strict registry for cross-floor data integrity.
    floor_ids = route_floor_ids | (
        map_floor_ids | annotation_floor_ids | session_floor_ids
        if not route_floor_ids
        else set()
    )
    unregistered_map_ids = [
        item["id"] for item in map_items
        if route_floor_ids and item["floor"] not in route_floor_ids
    ]
    unregistered_annotation_ids = [
        str(item.get("id"))
        for item in annotation_items
        if route_floor_ids and str(item.get("floor") or "").strip() not in route_floor_ids
    ]
    valid_sessions = []
    unregistered_session_ids = []
    for session in session_items:
        session_floors = {
            str(item).strip() for item in (session.get("floors") or []) if str(item).strip()
        }
        if session_floors and session_floors.issubset(floor_ids):
            valid_sessions.append(session)
        elif session_floors and route_floor_ids:
            unregistered_session_ids.append(str(session.get("id") or ""))
        elif session_floors:
            valid_sessions.append(session)

    routes = stair_routes_from_config(floor_config)
    latest_session = valid_sessions[-1] if valid_sessions else None
    latest_steps = mapping_floor_steps(latest_session or {})
    steps_by_floor = {str(item.get("floor") or ""): item for item in latest_steps}
    selected_map = next((item for item in map_items if item["id"] == str(selected_map_id or "")), None)
    floors: List[Dict[str, Any]] = []
    for floor_id in sorted(floor_ids, key=floor_sort_key):
        floor_maps = [item for item in map_items if item["floor"] == floor_id]
        preferred = (
            selected_map
            if selected_map and selected_map.get("floor") == floor_id
            else _preferred_map(floor_maps)
        )
        all_floor_annotations = [item for item in annotation_items if str(item.get("floor") or "") == floor_id]
        floor_annotations = [
            item
            for item in all_floor_annotations
            if preferred and str(item.get("map_id") or "") == str(preferred.get("id") or "")
        ]
        outgoing = [item for item in routes if item["source_floor"] == floor_id]
        incoming = [item for item in routes if item["target_floor"] == floor_id]
        configured = configured_floors.get(floor_id) if isinstance(configured_floors, dict) else None
        registry_source = str(configured.get("registry_source") or "route_config") if isinstance(configured, dict) else "map"
        route_configured = floor_id in route_floor_ids and registry_source != "project"
        initial_pose = configured.get("initial_pose") if isinstance(configured, dict) else None
        terrain_segments = configured.get("terrain_segments") if isinstance(configured, dict) else {}
        if isinstance(terrain_segments, dict):
            terrain_segment_count = len([item for item in terrain_segments.values() if isinstance(item, dict)])
        elif isinstance(terrain_segments, list):
            terrain_segment_count = len([item for item in terrain_segments if isinstance(item, dict)])
        else:
            terrain_segment_count = 0
        warnings: List[str] = []
        if not floor_maps:
            warnings.append("缺少地图")
        elif not any(item.get("factory_ready") for item in floor_maps):
            warnings.append("缺少106可切换地图包")
        if route_configured and not _pose_ready(initial_pose):
            warnings.append("缺少初始定位位姿")
        if route_configured and not outgoing and len(floor_ids) > 1:
            warnings.append("缺少离开本层的楼梯路线")
        floors.append(
            {
                "id": floor_id,
                "level": configured.get("level") if isinstance(configured, dict) else None,
                "registry_source": registry_source,
                "route_configured": route_configured,
                "current": floor_id == str(current_floor or ""),
                "maps": floor_maps,
                "preferred_map_id": preferred.get("id") if preferred else None,
                "preferred_map_name": preferred.get("name") if preferred else None,
                "selected": bool(preferred and preferred.get("id") == str(selected_map_id or "")),
                "annotation_count": len(floor_annotations),
                "historical_annotation_count": len(all_floor_annotations) - len(floor_annotations),
                "annotations": [
                    {
                        "id": str(item.get("id")),
                        "label": str(item.get("label") or item.get("id")),
                        "floor": floor_id,
                        "map_id": str(item.get("map_id") or ""),
                        "type": str(item.get("type") or ""),
                        "manual_point_type": str(item.get("manual_point_type") or ""),
                    }
                    for item in floor_annotations
                ],
                "route_out_count": len(outgoing),
                "route_in_count": len(incoming),
                "terrain_segment_count": terrain_segment_count,
                "mapping_step": dict(steps_by_floor.get(floor_id) or {}),
                "warnings": warnings,
                "ready": not warnings,
            }
        )

    configured_routes = [item for item in routes if item.get("configured")]
    route_summaries = [
        {key: value for key, value in item.items() if key != "poses"}
        for item in routes
    ]
    route_floor_count = sum(1 for item in floors if item["route_configured"])
    required_route_count = max(0, route_floor_count - 1) if route_floor_count > 1 else 0
    return {
        "ok": True,
        "current_floor": str(current_floor or ""),
        "selected_map_id": str(selected_map_id or ""),
        "floors": floors,
        "routes": route_summaries,
        "latest_mapping_session": latest_session,
        "floor_count": len(floors),
        "configured_route_count": len(configured_routes),
        "ready_floor_count": sum(1 for item in floors if item["ready"]),
        "identity_issues": {
            "unregistered_map_ids": unregistered_map_ids,
            "unregistered_annotation_ids": unregistered_annotation_ids,
            "unregistered_session_ids": unregistered_session_ids,
        },
        "identity_issue_count": len(unregistered_map_ids)
        + len(unregistered_annotation_ids)
        + len(unregistered_session_ids),
        "ready": bool(floors)
        and all(item["ready"] for item in floors)
        and len(configured_routes) >= required_route_count,
    }


def cross_floor_task_context(
    payload: Dict[str, Any],
    *,
    annotations_by_id: Dict[str, Dict[str, Any]],
    routes: Iterable[Dict[str, Any]],
) -> Dict[str, Any]:
    annotation_ids: List[str] = []
    for value in payload.get("annotation_ids") or []:
        annotation_id = str(value or "").strip()
        if annotation_id and annotation_id not in annotation_ids:
            annotation_ids.append(annotation_id)
    if not annotation_ids:
        return {"ok": False, "code": "cross_floor_no_waypoint", "message": "请先按顺序加入跨楼层任务点"}
    missing = [item for item in annotation_ids if item not in annotations_by_id]
    if missing:
        return {
            "ok": False,
            "code": "cross_floor_missing_waypoint",
            "message": "跨楼层任务中存在已删除点位",
            "missing": missing,
        }
    annotations = [annotations_by_id[item] for item in annotation_ids]
    waypoint_floors = [str(item.get("floor") or "").strip() for item in annotations]
    if any(not floor for floor in waypoint_floors):
        return {"ok": False, "code": "cross_floor_waypoint_floor_missing", "message": "任务点中存在未填写楼层的点位"}
    waypoint_map_ids = [str(item.get("map_id") or "").strip() for item in annotations]
    if any(not map_id for map_id in waypoint_map_ids):
        return {"ok": False, "code": "cross_floor_waypoint_map_missing", "message": "任务点中存在未绑定固定地图的点位"}
    maps_by_floor: Dict[str, set] = {}
    for floor, map_id in zip(waypoint_floors, waypoint_map_ids):
        maps_by_floor.setdefault(floor, set()).add(map_id)
    mixed_floor = next((floor for floor, map_ids in maps_by_floor.items() if len(map_ids) > 1), None)
    if mixed_floor:
        return {
            "ok": False,
            "code": "cross_floor_mixed_maps_on_floor",
            "message": f"{mixed_floor} 的任务点来自不同地图，不能混合坐标系",
            "floor": mixed_floor,
            "map_ids": sorted(maps_by_floor[mixed_floor]),
        }
    floor_sequence: List[str] = []
    for floor in waypoint_floors:
        if not floor_sequence or floor_sequence[-1] != floor:
            floor_sequence.append(floor)
    if len(set(floor_sequence)) < 2:
        return {"ok": False, "code": "cross_floor_single_floor", "message": "跨楼层任务至少需要两个不同楼层的点位"}
    route_plans = []
    route_items = list(routes)
    for source, target in zip(floor_sequence, floor_sequence[1:]):
        path = find_floor_path(route_items, source, target)
        if path is None:
            return {
                "ok": False,
                "code": "cross_floor_route_missing",
                "message": f"没有可用的楼梯路线 {source}->{target}",
                "source_floor": source,
                "target_floor": target,
            }
        route_plans.append({"source_floor": source, "target_floor": target, "floor_path": path})
    first_map_id = waypoint_map_ids[0]
    return {
        "ok": True,
        "name": str(payload.get("name") or "跨楼层巡检任务").strip() or "跨楼层巡检任务",
        "annotation_ids": annotation_ids,
        "annotations": annotations,
        "task_map_id": first_map_id,
        "floor_sequence": floor_sequence,
        "route_plans": route_plans,
    }
