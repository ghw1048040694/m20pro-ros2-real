"""Unified navigation-plan validation for one or more floor segments.

The number of floors is data, not a navigation mode.  A one-floor mission is
the same plan shape as a multi-floor mission with zero connector transitions.
This module is intentionally pure Python so it can be tested without ROS and
used by the web/API layer before any runtime movement is authorized.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional

from .connector_contract import connector_terrain_guard_profile
from .multi_floor_contract import find_floor_path


def _text(value: Any) -> str:
    return str(value or "").strip()


def _int_or(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _route_key(route: Dict[str, Any]) -> tuple[str, str]:
    return (_text(route.get("source_floor")), _text(route.get("target_floor")))


def _configured_routes(routes: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [
        dict(route)
        for route in routes
        if isinstance(route, dict)
        and route.get("configured")
        and _route_key(route)[0]
        and _route_key(route)[1]
    ]


def _route_for_edge(
    routes: Iterable[Dict[str, Any]], source_floor: str, target_floor: str
) -> Optional[Dict[str, Any]]:
    for route in routes:
        if _route_key(route) == (source_floor, target_floor) and route.get("configured"):
            return dict(route)
    return None


def _ordered_unique(values: Iterable[Any]) -> List[str]:
    result: List[str] = []
    for value in values:
        item = _text(value)
        if item and item not in result:
            result.append(item)
    return result


def build_unified_navigation_plan(
    annotation_ids: Iterable[Any],
    *,
    annotations_by_id: Dict[str, Dict[str, Any]],
    routes: Iterable[Dict[str, Any]],
) -> Dict[str, Any]:
    """Build one plan shape for single-floor and multi-floor missions.

    The returned plan contains ordered floor segments and connector edges.  A
    single-floor plan has one floor segment and an empty ``transitions`` list;
    it is not routed through a separate single-floor implementation.
    """

    ordered_ids = _ordered_unique(annotation_ids)
    if not ordered_ids:
        return {
            "ok": False,
            "code": "navigation_no_waypoint",
            "message": "请先按顺序加入任务点",
        }

    missing = [item for item in ordered_ids if item not in annotations_by_id]
    if missing:
        return {
            "ok": False,
            "code": "navigation_missing_waypoint",
            "message": "任务中存在已删除点位",
            "missing": missing,
        }

    annotations: List[Dict[str, Any]] = [dict(annotations_by_id[item]) for item in ordered_ids]
    floors = [_text(item.get("floor")) for item in annotations]
    map_ids = [_text(item.get("map_id")) for item in annotations]
    if any(not floor for floor in floors):
        return {
            "ok": False,
            "code": "navigation_waypoint_floor_missing",
            "message": "任务点中存在未绑定楼层的点位",
        }
    if any(not map_id for map_id in map_ids):
        return {
            "ok": False,
            "code": "navigation_waypoint_map_missing",
            "message": "任务点中存在未绑定地图的点位",
        }

    maps_by_floor: Dict[str, set[str]] = {}
    for floor, map_id in zip(floors, map_ids):
        maps_by_floor.setdefault(floor, set()).add(map_id)
    mixed_floor = next(
        (floor for floor, floor_maps in maps_by_floor.items() if len(floor_maps) > 1),
        None,
    )
    if mixed_floor:
        return {
            "ok": False,
            "code": "navigation_mixed_maps_on_floor",
            "message": f"{mixed_floor} 的任务点来自不同地图，不能混合坐标系",
            "floor": mixed_floor,
            "map_ids": sorted(maps_by_floor[mixed_floor]),
        }

    # A segment is a contiguous run on one floor.  Do not group all points by
    # floor: a valid mission may return to an earlier floor (F1 -> F2 -> F1),
    # and merging those points would destroy execution order.
    segments: List[Dict[str, Any]] = []
    for item, floor, map_id in zip(annotations, floors, map_ids):
        if not segments or segments[-1]["floor"] != floor:
            segments.append(
                {
                    "kind": "floor",
                    "index": len(segments),
                    "floor": floor,
                    "map_id": map_id,
                    "annotation_ids": [],
                }
            )
        segments[-1]["annotation_ids"].append(_text(item.get("id")))

    floor_sequence = [str(item["floor"]) for item in segments]
    transition_paths: List[Dict[str, Any]] = []

    configured_routes = _configured_routes(routes)
    transitions: List[Dict[str, Any]] = []
    for boundary_index, (source_floor, target_floor) in enumerate(
        zip(floor_sequence, floor_sequence[1:])
    ):
        if source_floor == target_floor:
            continue
        path = find_floor_path(configured_routes, source_floor, target_floor)
        if path is None:
            return {
                "ok": False,
                "code": "navigation_route_missing",
                "message": f"没有可用的导航连接 {source_floor}->{target_floor}",
                "source_floor": source_floor,
                "target_floor": target_floor,
            }
        transition_paths.append(
            {
                "source_floor": source_floor,
                "target_floor": target_floor,
                "floor_path": list(path),
            }
        )
        for edge_source, edge_target in zip(path, path[1:]):
            route = _route_for_edge(configured_routes, edge_source, edge_target)
            if route is None:
                return {
                    "ok": False,
                    "code": "navigation_route_edge_missing",
                    "message": f"连接图缺少有向边 {edge_source}->{edge_target}",
                    "source_floor": edge_source,
                    "target_floor": edge_target,
                }
            transitions.append(
                {
                    "kind": "connector",
                    "route_id": _text(route.get("id")) or f"{edge_source}->{edge_target}",
                    "source_floor": edge_source,
                    "target_floor": edge_target,
                    "source_segment_index": boundary_index,
                    "target_segment_index": boundary_index + 1,
                    "path_step_index": len(
                        [
                            item
                            for item in transitions
                            if item.get("source_segment_index") == boundary_index
                        ]
                    ),
                    "path_step_count": len(path) - 1,
                    "terrain_guard": connector_terrain_guard_profile(route),
                    "route": route,
                }
            )

    distinct_floors = _ordered_unique(floor_sequence)

    return {
        "ok": True,
        "kind": "unified_navigation_plan",
        "annotation_ids": ordered_ids,
        "annotations": annotations,
        "floor_sequence": floor_sequence,
        "floor_count": len(distinct_floors),
        "segment_count": len(segments),
        "single_floor": len(distinct_floors) == 1,
        "segments": segments,
        "transition_paths": transition_paths,
        "transitions": transitions,
        "task_map_id": map_ids[0],
        "route_count": len(transitions),
    }


def summarize_plan(plan: Dict[str, Any]) -> Dict[str, Any]:
    """Return a stable, small status payload for the frontend task panel."""

    if not isinstance(plan, dict) or not plan.get("ok"):
        return {
            "ok": False,
            "code": _text(plan.get("code")) if isinstance(plan, dict) else "navigation_plan_invalid",
            "message": _text(plan.get("message")) if isinstance(plan, dict) else "导航计划无效",
        }
    floors = [_text(item.get("floor")) for item in plan.get("segments", [])]
    distinct_floors = _ordered_unique(floors)
    return {
        "ok": True,
        "kind": "unified_navigation_plan",
        "floor_count": len(distinct_floors),
        "segment_count": len(floors),
        "floors": floors,
        "waypoint_count": len(plan.get("annotation_ids") or []),
        "transition_count": len(plan.get("transitions") or []),
        "single_floor": len(floors) == 1,
    }


def navigation_plan_record(plan: Dict[str, Any]) -> Dict[str, Any]:
    """Return the compact immutable task record for a validated plan.

    Point payloads and full route configuration stay in their owning stores.
    The task only needs ordered segment/map identities and directed connector
    identities, preventing a task snapshot from becoming a second map or route
    database.
    """
    if not isinstance(plan, dict) or not plan.get("ok"):
        return summarize_plan(plan)
    return {
        "ok": True,
        "kind": "unified_navigation_plan",
        "version": 1,
        "annotation_ids": list(plan.get("annotation_ids") or []),
        "task_map_id": _text(plan.get("task_map_id")),
        "floor_sequence": list(plan.get("floor_sequence") or []),
        "floor_count": _int_or(plan.get("floor_count"), 0),
        "segment_count": _int_or(
            plan.get("segment_count"), len(plan.get("segments") or [])
        ),
        "single_floor": bool(plan.get("single_floor")),
        "segments": [
            {
                "kind": "floor",
                "index": _int_or(item.get("index", index), index),
                "floor": _text(item.get("floor")),
                "map_id": _text(item.get("map_id")),
                "annotation_ids": list(item.get("annotation_ids") or []),
            }
            for index, item in enumerate(plan.get("segments") or [])
            if isinstance(item, dict)
        ],
        "transition_paths": [
            {
                "source_floor": _text(item.get("source_floor")),
                "target_floor": _text(item.get("target_floor")),
                "floor_path": list(item.get("floor_path") or []),
            }
            for item in plan.get("transition_paths") or []
            if isinstance(item, dict)
        ],
        "transitions": [
            {
                "kind": "connector",
                "route_id": _text(item.get("route_id")),
                "source_floor": _text(item.get("source_floor")),
                "target_floor": _text(item.get("target_floor")),
                "source_segment_index": _int_or(item.get("source_segment_index", 0), 0),
                "target_segment_index": _int_or(item.get("target_segment_index", 0), 0),
                "path_step_index": _int_or(item.get("path_step_index", 0), 0),
                "path_step_count": _int_or(item.get("path_step_count", 1), 1),
                "terrain_guard": connector_terrain_guard_profile(
                    {
                        "id": _text(item.get("route_id")),
                        "terrain_guard": item.get("terrain_guard"),
                    }
                ),
            }
            for item in plan.get("transitions") or []
            if isinstance(item, dict)
        ],
        "route_count": len(plan.get("transitions") or []),
    }


def task_navigation_plan_state(
    task: Dict[str, Any],
    *,
    annotations_by_id: Dict[str, Dict[str, Any]],
    routes: Iterable[Dict[str, Any]],
) -> Dict[str, Any]:
    """Validate or migrate the canonical plan stored on a task.

    The point store and current directed route registry are the inputs of
    truth.  The compact plan on a task is a display/cache projection, so every
    task start rebuilds it from those sources instead of turning an older
    projection into a second route database.
    """
    if not isinstance(task, dict):
        return {
            "ok": False,
            "code": "navigation_task_invalid",
            "message": "任务记录无效，无法建立统一导航计划",
        }

    plan = build_unified_navigation_plan(
        task.get("annotation_ids") or [],
        annotations_by_id=annotations_by_id,
        routes=routes,
    )
    if not plan.get("ok"):
        return dict(plan)

    task_map_id = _text(task.get("map_id"))
    plan_map_id = _text(plan.get("task_map_id"))
    if task_map_id and task_map_id != "live_map" and plan_map_id and task_map_id != plan_map_id:
        return {
            "ok": False,
            "code": "navigation_task_map_mismatch",
            "message": "任务绑定地图与任务点地图不一致，请重新生成任务",
            "task_map_id": task_map_id,
            "plan_map_id": plan_map_id,
        }

    record = navigation_plan_record(plan)
    existing = task.get("navigation_plan")
    return {
        "ok": True,
        "plan": plan,
        "record": record,
        "migrated": not (isinstance(existing, dict) and existing.get("ok")),
        "refreshed": (
            isinstance(existing, dict)
            and existing.get("ok")
            and navigation_plan_record(existing) != record
        ),
    }


def runtime_transition_for_annotation(
    plan: Dict[str, Any],
    annotation_id: Any,
    *,
    current_floor: Any,
) -> Dict[str, Any]:
    """Resolve the connector boundary for the active waypoint.

    The lookup is segment-based rather than set-based, so returning to a
    previous floor remains ordered.  A multi-hop connector is returned as its
    ordered edge list; callers must not collapse it into a direct floor jump.
    """
    if not isinstance(plan, dict) or not plan.get("ok"):
        return {
            "action": "invalid",
            "code": "navigation_plan_missing",
            "message": "活动任务缺少统一导航计划",
        }
    target_id = _text(annotation_id)
    segments = [item for item in plan.get("segments") or [] if isinstance(item, dict)]
    target_index = next(
        (
            index
            for index, segment in enumerate(segments)
            if target_id in [_text(value) for value in segment.get("annotation_ids") or []]
        ),
        None,
    )
    if target_index is None:
        return {
            "action": "invalid",
            "code": "navigation_plan_waypoint_missing",
            "message": "当前点位不在统一导航计划中",
            "annotation_id": target_id,
        }

    target_segment = segments[target_index]
    target_floor = _text(target_segment.get("floor"))
    source_floor = _text(current_floor)
    if not source_floor or source_floor == target_floor:
        return {
            "action": "same_floor",
            "source_floor": source_floor,
            "target_floor": target_floor,
            "target_segment_index": target_index,
        }

    boundary_edges = [
        dict(item)
        for item in plan.get("transitions") or []
        if isinstance(item, dict)
        and _int_or(item.get("target_segment_index", -1), -1) == target_index
    ]
    if not boundary_edges:
        return {
            "action": "invalid",
            "code": "navigation_plan_transition_missing",
            "message": "统一导航计划缺少当前点位前的有向连接",
            "source_floor": source_floor,
            "target_floor": target_floor,
            "target_segment_index": target_index,
        }

    grouped: Dict[int, List[Dict[str, Any]]] = {}
    for edge in boundary_edges:
        group_index = _int_or(edge.get("source_segment_index", -1), -1)
        grouped.setdefault(group_index, []).append(edge)
    selected_source_index: Optional[int] = None
    selected_edges: List[Dict[str, Any]] = []
    for group_index, group_edges in grouped.items():
        group_edges.sort(key=lambda item: _int_or(item.get("path_step_index", 0), 0))
        start_index = next(
            (
                index
                for index, edge in enumerate(group_edges)
                if _text(edge.get("source_floor")) == source_floor
            ),
            None,
        )
        if start_index is None:
            continue
        remaining = group_edges[start_index:]
        expected_floor = source_floor
        path_contiguous = True
        for edge in remaining:
            next_floor = _text(edge.get("target_floor"))
            if _text(edge.get("source_floor")) != expected_floor or not next_floor:
                path_contiguous = False
                break
            expected_floor = next_floor
        if not path_contiguous:
            continue
        if expected_floor != target_floor:
            continue
        selected_source_index = group_index
        selected_edges = remaining
        break
    if selected_source_index is None or not selected_edges:
        return {
            "action": "invalid",
            "code": "navigation_plan_source_path_missing",
            "message": "当前楼层不在通往当前点位的剩余有向路径上",
            "source_floor": source_floor,
            "target_floor": target_floor,
            "target_segment_index": target_index,
        }
    return {
        "action": "transition",
        "source_floor": source_floor,
        "target_floor": target_floor,
        "source_segment_index": selected_source_index,
        "target_segment_index": target_index,
        "edges": selected_edges,
    }
