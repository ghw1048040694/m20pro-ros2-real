#!/usr/bin/env python3
"""Offline tests for local-plan map-frame display conversion."""

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "m20pro_cloud_bridge"))

from m20pro_cloud_bridge.path_display_contract import path_points_in_map_frame  # noqa: E402


def assert_close(actual, expected, label, tolerance=1e-9):
    if not math.isclose(actual, expected, abs_tol=tolerance):
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def test_map_path_is_unchanged():
    result = path_points_in_map_frame(
        [{"x": 1, "y": 2, "z": 0}],
        frame_id="map",
        map_pose=None,
        odom_pose=None,
    )
    assert result == [{"x": 1.0, "y": 2.0, "z": 0.0}]


def test_odom_path_is_aligned_to_map_pose():
    result = path_points_in_map_frame(
        [{"x": 1, "y": 2}, {"x": 2, "y": 2}],
        frame_id="odom",
        map_pose={"x": 10, "y": 20, "yaw": math.pi / 2},
        odom_pose={"x": 1, "y": 2, "yaw": 0},
    )
    assert result is not None
    assert_close(result[0]["x"], 10, "robot x")
    assert_close(result[0]["y"], 20, "robot y")
    assert_close(result[1]["x"], 10, "forward x")
    assert_close(result[1]["y"], 21, "forward y")


def test_unknown_or_unaligned_frame_is_rejected():
    assert path_points_in_map_frame([], frame_id="base_link", map_pose={}, odom_pose={}) is None
    assert path_points_in_map_frame([], frame_id="odom", map_pose={}, odom_pose={}) is None


if __name__ == "__main__":
    test_map_path_is_unchanged()
    test_odom_path_is_aligned_to_map_pose()
    test_unknown_or_unaligned_frame_is_rejected()
    print("path display contract tests passed")
