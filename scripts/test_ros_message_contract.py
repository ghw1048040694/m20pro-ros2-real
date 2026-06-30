#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.ros_message_contract."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.ros_message_contract import (  # noqa: E402
    pose_to_dict,
    stamp_to_float,
    wrap_angle,
    yaw_to_orientation,
)


def assert_close(actual: float, expected: float, message: str, tol: float = 1e-6) -> None:
    if abs(actual - expected) > tol:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def pose(x: float = 1.0, y: float = 2.0, z: float = 0.0, yaw: float = 0.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(yaw * 0.5),
            w=math.cos(yaw * 0.5),
        ),
    )


def test_stamp_to_float() -> None:
    assert_equal(stamp_to_float(None), None, "none stamp")
    assert_equal(stamp_to_float(SimpleNamespace(sec=0, nanosec=0)), None, "zero stamp")
    assert_close(stamp_to_float(SimpleNamespace(sec=12, nanosec=500000000)), 12.5, "stamp seconds")


def test_pose_to_dict() -> None:
    payload = pose_to_dict(pose(1.5, 2.5, 0.1, math.pi / 2.0))
    assert_close(payload["x"], 1.5, "x")
    assert_close(payload["y"], 2.5, "y")
    assert_close(payload["z"], 0.1, "z")
    assert_close(payload["yaw"], math.pi / 2.0, "yaw")
    assert_close(payload["yaw_deg"], 90.0, "yaw deg")


def test_wrap_and_write_orientation() -> None:
    assert_close(wrap_angle(3.0 * math.pi), math.pi, "positive wrap")
    assert_close(wrap_angle(-math.pi), math.pi, "negative boundary wraps like runtime")

    msg = SimpleNamespace(pose=SimpleNamespace(orientation=SimpleNamespace()))
    yaw_to_orientation(msg, math.pi / 2.0)
    assert_close(msg.pose.orientation.x, 0.0, "orientation x")
    assert_close(msg.pose.orientation.y, 0.0, "orientation y")
    assert_close(msg.pose.orientation.z, math.sin(math.pi / 4.0), "orientation z")
    assert_close(msg.pose.orientation.w, math.cos(math.pi / 4.0), "orientation w")


def main() -> int:
    for test in (
        test_stamp_to_float,
        test_pose_to_dict,
        test_wrap_and_write_orientation,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] ROS message contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
