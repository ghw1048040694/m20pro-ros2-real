"""Pure SE(2) helpers for keeping odom continuous across map relocalization."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional


def _wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _pose(payload: Any) -> Dict[str, float]:
    if not isinstance(payload, dict):
        raise ValueError("pose must be a mapping")
    try:
        pose = {
            "x": float(payload["x"]),
            "y": float(payload["y"]),
            "yaw": _wrap_angle(float(payload["yaw"])),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("pose must contain finite x, y and yaw") from exc
    if not all(math.isfinite(value) for value in pose.values()):
        raise ValueError("pose must contain finite x, y and yaw")
    return pose


def compose_pose(parent: Dict[str, Any], child: Dict[str, Any]) -> Dict[str, float]:
    """Compose parent->middle and middle->child planar transforms."""
    left = _pose(parent)
    right = _pose(child)
    cos_yaw = math.cos(left["yaw"])
    sin_yaw = math.sin(left["yaw"])
    return {
        "x": left["x"] + cos_yaw * right["x"] - sin_yaw * right["y"],
        "y": left["y"] + sin_yaw * right["x"] + cos_yaw * right["y"],
        "yaw": _wrap_angle(left["yaw"] + right["yaw"]),
    }


def inverse_pose(pose: Dict[str, Any]) -> Dict[str, float]:
    value = _pose(pose)
    cos_yaw = math.cos(value["yaw"])
    sin_yaw = math.sin(value["yaw"])
    return {
        "x": -cos_yaw * value["x"] - sin_yaw * value["y"],
        "y": sin_yaw * value["x"] - cos_yaw * value["y"],
        "yaw": _wrap_angle(-value["yaw"]),
    }


def odom_alignment_update(
    *,
    map_pose: Dict[str, Any],
    previous_map_pose: Optional[Dict[str, Any]],
    previous_odom_pose: Optional[Dict[str, Any]],
    map_to_odom: Optional[Dict[str, Any]],
    force_rebase: bool,
    jump_threshold_m: float,
    yaw_threshold_rad: float,
) -> Dict[str, Any]:
    """Return a TF-consistent update without letting relocalization jump odom."""
    current_map = _pose(map_pose)
    if previous_map_pose is None or previous_odom_pose is None or map_to_odom is None:
        return {
            "map_pose": current_map,
            "odom_pose": dict(current_map),
            "map_to_odom": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "initialized": True,
            "rebased": False,
            "reason": "initialized_identity",
            "map_step_distance_m": 0.0,
            "map_step_yaw_rad": 0.0,
        }

    previous_map = _pose(previous_map_pose)
    previous_odom = _pose(previous_odom_pose)
    current_map_to_odom = _pose(map_to_odom)
    distance = math.hypot(
        current_map["x"] - previous_map["x"],
        current_map["y"] - previous_map["y"],
    )
    yaw_error = abs(_wrap_angle(current_map["yaw"] - previous_map["yaw"]))
    discontinuity = bool(
        distance > max(0.0, float(jump_threshold_m))
        or yaw_error > max(0.0, float(yaw_threshold_rad))
    )
    rebase = bool(force_rebase or discontinuity)
    if rebase:
        current_map_to_odom = compose_pose(current_map, inverse_pose(previous_odom))
        current_odom = previous_odom
        reason = "commanded_relocalization" if force_rebase else "accepted_map_discontinuity"
    else:
        current_odom = compose_pose(inverse_pose(current_map_to_odom), current_map)
        reason = "continuous_motion"

    reconstructed = compose_pose(current_map_to_odom, current_odom)
    closure_error = math.hypot(
        reconstructed["x"] - current_map["x"],
        reconstructed["y"] - current_map["y"],
    )
    return {
        "map_pose": current_map,
        "odom_pose": current_odom,
        "map_to_odom": current_map_to_odom,
        "initialized": False,
        "rebased": rebase,
        "reason": reason,
        "map_step_distance_m": distance,
        "map_step_yaw_rad": yaw_error,
        "closure_error_m": closure_error,
    }
