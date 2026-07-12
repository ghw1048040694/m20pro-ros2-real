import math
from typing import Any, Dict, List, Optional


def _finite_pose(pose: Any) -> bool:
    if not isinstance(pose, dict):
        return False
    try:
        return all(math.isfinite(float(pose[key])) for key in ("x", "y", "yaw"))
    except (KeyError, TypeError, ValueError):
        return False


def path_points_in_map_frame(
    points: List[Dict[str, Any]],
    *,
    frame_id: str,
    map_pose: Any,
    odom_pose: Any,
) -> Optional[List[Dict[str, float]]]:
    """Return path points in map coordinates using the live map/odom pose pair."""
    source_frame = str(frame_id or "").strip().lstrip("/")
    if source_frame == "map":
        yaw = 0.0
        tx = 0.0
        ty = 0.0
    elif source_frame == "odom":
        if not _finite_pose(map_pose) or not _finite_pose(odom_pose):
            return None
        yaw = float(map_pose["yaw"]) - float(odom_pose["yaw"])
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = float(map_pose["x"]) - (
            cos_yaw * float(odom_pose["x"]) - sin_yaw * float(odom_pose["y"])
        )
        ty = float(map_pose["y"]) - (
            sin_yaw * float(odom_pose["x"]) + cos_yaw * float(odom_pose["y"])
        )
    else:
        return None

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    transformed: List[Dict[str, float]] = []
    for point in points:
        try:
            x = float(point["x"])
            y = float(point["y"])
            z = float(point.get("z", 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        transformed.append(
            {
                "x": cos_yaw * x - sin_yaw * y + tx,
                "y": sin_yaw * x + cos_yaw * y + ty,
                "z": z,
            }
        )
    return transformed
