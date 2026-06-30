"""ROS message conversion helpers kept free of node state."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional


def stamp_to_float(stamp: Any) -> Optional[float]:
    if stamp is None:
        return None
    sec = float(getattr(stamp, "sec", 0))
    nanosec = float(getattr(stamp, "nanosec", 0))
    value = sec + nanosec * 1e-9
    return value if value > 0.0 else None


def yaw_from_pose(pose: Any) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_to_dict(pose: Any) -> Dict[str, float]:
    yaw = yaw_from_pose(pose)
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "yaw": yaw,
        "yaw_deg": math.degrees(yaw),
    }


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_orientation(msg: Any, yaw: float) -> None:
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.orientation.w = math.cos(yaw * 0.5)
