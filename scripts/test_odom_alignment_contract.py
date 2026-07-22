#!/usr/bin/env python3
"""Offline tests for map/odom continuity across relocalization."""

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "m20pro_navigation"))

from m20pro_navigation.odom_alignment_contract import (  # noqa: E402
    compose_pose,
    odom_alignment_update,
    pose_source_alignment_update,
)


def assert_pose(actual, expected, tolerance=1e-9):
    for key in ("x", "y", "yaw"):
        if not math.isclose(actual[key], expected[key], abs_tol=tolerance):
            raise AssertionError(f"{key}: expected {expected[key]}, got {actual[key]}")


def update(map_pose, previous_map=None, previous_odom=None, map_to_odom=None, force=False):
    return odom_alignment_update(
        map_pose=map_pose,
        previous_map_pose=previous_map,
        previous_odom_pose=previous_odom,
        map_to_odom=map_to_odom,
        force_rebase=force,
        jump_threshold_m=0.6,
        yaw_threshold_rad=0.75,
    )


def test_initial_identity_and_continuous_motion():
    initial = update({"x": -0.9, "y": 1.4, "yaw": 0.2})
    assert initial["initialized"]
    assert_pose(initial["odom_pose"], initial["map_pose"])
    assert_pose(initial["map_to_odom"], {"x": 0.0, "y": 0.0, "yaw": 0.0})

    moved = update(
        {"x": -0.7, "y": 1.5, "yaw": 0.3},
        initial["map_pose"],
        initial["odom_pose"],
        initial["map_to_odom"],
    )
    assert not moved["rebased"]
    assert_pose(moved["odom_pose"], moved["map_pose"])


def test_commanded_relocalization_preserves_odom():
    previous_map = {"x": -0.9, "y": 1.4, "yaw": 0.0}
    previous_odom = dict(previous_map)
    jumped_map = {"x": -3.81, "y": -10.01, "yaw": -1.88}
    result = update(
        jumped_map,
        previous_map,
        previous_odom,
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        force=True,
    )
    assert result["rebased"]
    assert result["reason"] == "commanded_relocalization"
    assert_pose(result["odom_pose"], previous_odom)
    assert_pose(compose_pose(result["map_to_odom"], result["odom_pose"]), jumped_map)
    assert result["closure_error_m"] < 1e-9


def test_motion_after_relocalization_uses_new_map_to_odom():
    previous_map = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    previous_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    relocalized = update(
        {"x": 10.0, "y": 20.0, "yaw": math.pi / 2},
        previous_map,
        previous_odom,
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        force=True,
    )
    moved = update(
        {"x": 10.0, "y": 20.2, "yaw": math.pi / 2},
        relocalized["map_pose"],
        relocalized["odom_pose"],
        relocalized["map_to_odom"],
    )
    assert not moved["rebased"]
    assert_pose(moved["odom_pose"], {"x": 0.2, "y": 0.0, "yaw": 0.0})
    assert_pose(compose_pose(moved["map_to_odom"], moved["odom_pose"]), moved["map_pose"])


def test_uncommanded_accepted_jump_also_rebases():
    result = update(
        {"x": 2.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    assert result["rebased"]
    assert result["reason"] == "accepted_map_discontinuity"
    assert_pose(result["odom_pose"], {"x": 0.0, "y": 0.0, "yaw": 0.0})


def test_fallback_pose_source_is_aligned_without_switch_jump():
    reference = {"x": -11.12, "y": -3.32, "yaw": -1.65}
    raw_start = {"x": -10.96, "y": -3.57, "yaw": -1.51}
    started = pose_source_alignment_update(
        source_pose=raw_start,
        reference_pose=reference,
        alignment=None,
    )
    assert_pose(started["pose"], reference)

    raw_next = compose_pose(raw_start, {"x": 0.4, "y": 0.0, "yaw": 0.1})
    continued = pose_source_alignment_update(
        source_pose=raw_next,
        reference_pose=None,
        alignment=started["alignment"],
    )
    expected = compose_pose(reference, {"x": 0.4, "y": 0.0, "yaw": 0.1})
    assert_pose(continued["pose"], expected)


def main():
    for test in (
        test_initial_identity_and_continuous_motion,
        test_commanded_relocalization_preserves_odom,
        test_motion_after_relocalization_uses_new_map_to_odom,
        test_uncommanded_accepted_jump_also_rebases,
        test_fallback_pose_source_is_aligned_without_switch_jump,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] odom alignment contract tests passed")


if __name__ == "__main__":
    main()
