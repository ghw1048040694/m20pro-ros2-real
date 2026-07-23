"""Unified navigation-plan validation for one or more floor segments.

The number of floors is data, not a navigation mode.  A one-floor mission is
the same plan shape as a multi-floor mission with zero connector transitions.
This module is intentionally pure Python so it can be tested without ROS and
used by the web/API layer before any runtime movement is authorized.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional

from .multi_floor_contract import find_floor_path


def _text(value: Any) -> str:
    return str(value or "").strip()


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

    floor_sequence: List[str] = []
    for floor in floors:
        if not floor_sequence or floor_sequence[-1] != floor:
            floor_sequence.append(floor)

    configured_routes = _configured_routes(routes)
    transitions: List[Dict[str, Any]] = []
    for source_floor, target_floor in zip(floor_sequence, floor_sequence[1:]):
        path = find_floor_path(configured_routes, source_floor, target_floor)
        if path is None:
            return {
                "ok": False,
                "code": "navigation_route_missing",
                "message": f"没有可用的导航连接 {source_floor}->{target_floor}",
                "source_floor": source_floor,
                "target_floor": target_floor,
            }
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
                    "route": route,
                }
            )

    segments: List[Dict[str, Any]] = []
    for floor in floor_sequence:
        segment_annotations = [
            item for item in annotations if _text(item.get("floor")) == floor
        ]
        segments.append(
            {
                "kind": "floor",
                "floor": floor,
                "map_id": next(iter(maps_by_floor[floor])),
                "annotation_ids": [
                    _text(item.get("id")) for item in segment_annotations if _text(item.get("id"))
                ],
            }
        )

    return {
        "ok": True,
        "kind": "unified_navigation_plan",
        "annotation_ids": ordered_ids,
        "annotations": annotations,
        "floor_sequence": floor_sequence,
        "floor_count": len(floor_sequence),
        "single_floor": len(floor_sequence) == 1,
        "segments": segments,
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
    return {
        "ok": True,
        "kind": "unified_navigation_plan",
        "floor_count": len(floors),
        "floors": floors,
        "waypoint_count": len(plan.get("annotation_ids") or []),
        "transition_count": len(plan.get("transitions") or []),
        "single_floor": len(floors) == 1,
    }
