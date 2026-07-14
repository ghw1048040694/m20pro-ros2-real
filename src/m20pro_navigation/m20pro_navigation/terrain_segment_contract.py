"""Pure helpers for same-floor ramps and stair segments."""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional


def _point(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    try:
        x = float(value.get("x"))
        y = float(value.get("y"))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    return {"x": x, "y": y}


def _polygon(value: Any) -> List[Dict[str, float]]:
    if not isinstance(value, list):
        return []
    points = [_point(item) for item in value]
    return [point for point in points if point is not None]


def _corridor_polygon(
    entry: Dict[str, float],
    exit_pose: Dict[str, float],
    width_m: float,
    end_margin_m: float,
) -> List[Dict[str, float]]:
    dx = exit_pose["x"] - entry["x"]
    dy = exit_pose["y"] - entry["y"]
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        return []
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    half_width = max(0.1, width_m * 0.5)
    start_x = entry["x"] - ux * max(0.0, end_margin_m)
    start_y = entry["y"] - uy * max(0.0, end_margin_m)
    end_x = exit_pose["x"] + ux * max(0.0, end_margin_m)
    end_y = exit_pose["y"] + uy * max(0.0, end_margin_m)
    return [
        {"x": start_x + nx * half_width, "y": start_y + ny * half_width},
        {"x": end_x + nx * half_width, "y": end_y + ny * half_width},
        {"x": end_x - nx * half_width, "y": end_y - ny * half_width},
        {"x": start_x - nx * half_width, "y": start_y - ny * half_width},
    ]


def _items(value: Any) -> Iterable[tuple[str, Dict[str, Any]]]:
    if isinstance(value, dict):
        for name, item in value.items():
            if isinstance(item, dict):
                yield str(name), item
    elif isinstance(value, list):
        for index, item in enumerate(value):
            if isinstance(item, dict):
                yield str(item.get("id") or item.get("name") or f"segment_{index + 1}"), item


def terrain_segments_from_config(config: Dict[str, Any]) -> Dict[str, List[Dict[str, Any]]]:
    floors = config.get("floors") if isinstance(config.get("floors"), dict) else {}
    result: Dict[str, List[Dict[str, Any]]] = {}
    for floor_id, floor_data in floors.items():
        if not isinstance(floor_data, dict):
            continue
        segments = []
        for name, raw in _items(floor_data.get("terrain_segments")):
            terrain = str(raw.get("terrain") or "ramp").strip().lower()
            entry = _point(raw.get("entry"))
            exit_pose = _point(raw.get("exit"))
            polygon = _polygon(raw.get("polygon"))
            try:
                width_m = max(0.2, float(raw.get("width_m", 1.0)))
                end_margin_m = max(0.0, float(raw.get("end_margin_m", 0.2)))
            except (TypeError, ValueError):
                width_m, end_margin_m = 1.0, 0.2
            if len(polygon) < 3 and entry is not None and exit_pose is not None:
                polygon = _corridor_polygon(entry, exit_pose, width_m, end_margin_m)
            default_forward = "stair_up" if terrain == "stairs" else "terrain"
            default_reverse = "stair_down" if terrain == "stairs" else default_forward
            configured = len(polygon) >= 3
            segments.append(
                {
                    "id": "%s:%s" % (str(floor_id), name),
                    "name": name,
                    "floor": str(floor_id),
                    "terrain": terrain,
                    "entry": entry,
                    "exit": exit_pose,
                    "polygon": polygon,
                    "gait_forward": str(raw.get("gait_forward") or raw.get("gait") or default_forward),
                    "gait_reverse": str(raw.get("gait_reverse") or default_reverse),
                    "exit_gait": str(raw.get("exit_gait") or "flat"),
                    "configured": configured,
                    "error": None if configured else "terrain segment requires a polygon or distinct entry/exit poses",
                }
            )
        result[str(floor_id)] = segments
    return result


def point_in_polygon(x: float, y: float, polygon: Any) -> bool:
    if not isinstance(polygon, list) or len(polygon) < 3:
        return False
    inside = False
    previous = polygon[-1]
    for current in polygon:
        try:
            xi, yi = float(current["x"]), float(current["y"])
            xj, yj = float(previous["x"]), float(previous["y"])
        except (KeyError, TypeError, ValueError):
            previous = current
            continue
        if ((yi > y) != (yj > y)) and x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi:
            inside = not inside
        previous = current
    return inside


def terrain_segment_at_pose(
    segments_by_floor: Dict[str, List[Dict[str, Any]]],
    floor: Any,
    x: float,
    y: float,
) -> Optional[Dict[str, Any]]:
    for segment in segments_by_floor.get(str(floor or ""), []):
        if segment.get("configured") and point_in_polygon(x, y, segment.get("polygon")):
            return segment
    return None


def terrain_entry_gait(segment: Dict[str, Any], x: float, y: float) -> tuple[str, str]:
    entry = segment.get("entry")
    exit_pose = segment.get("exit")
    direction = "forward"
    if isinstance(entry, dict) and isinstance(exit_pose, dict):
        entry_distance = math.hypot(x - float(entry["x"]), y - float(entry["y"]))
        exit_distance = math.hypot(x - float(exit_pose["x"]), y - float(exit_pose["y"]))
        if exit_distance < entry_distance:
            direction = "reverse"
    gait = str(segment.get("gait_reverse" if direction == "reverse" else "gait_forward") or "terrain")
    return gait, direction
