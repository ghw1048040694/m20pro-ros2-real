#!/usr/bin/env python3
"""Offline tests for same-floor terrain segments."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.terrain_segment_contract import (  # noqa: E402
    terrain_entry_gait,
    terrain_segment_at_pose,
    terrain_segments_from_config,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def test_same_floor_segment_direction_and_bounds() -> None:
    config = {
        "floors": {
            "F20": {
                "terrain_segments": {
                    "same_level_stairs": {
                        "terrain": "stairs",
                        "entry": {"x": 0.0, "y": 0.0},
                        "exit": {"x": 4.0, "y": 0.0},
                        "width_m": 1.0,
                    }
                }
            }
        }
    }
    segments = terrain_segments_from_config(config)
    segment = segments["F20"][0]
    assert_equal(segment["configured"], True, "segment configured")
    assert_equal(terrain_segment_at_pose(segments, "F20", 1.0, 0.0)["id"], "F20:same_level_stairs", "inside")
    assert_equal(terrain_segment_at_pose(segments, "F20", 1.0, 2.0), None, "outside")
    assert_equal(terrain_entry_gait(segment, 0.2, 0.0), ("stair_up", "forward"), "forward gait")
    assert_equal(terrain_entry_gait(segment, 3.8, 0.0), ("stair_down", "reverse"), "reverse gait")
    assert_equal(segment["floor"], "F20", "same floor retained")


def test_invalid_segment_is_not_active() -> None:
    segments = terrain_segments_from_config(
        {"floors": {"F20": {"terrain_segments": {"bad": {"terrain": "ramp"}}}}}
    )
    assert_equal(segments["F20"][0]["configured"], False, "invalid segment")
    assert_equal(terrain_segment_at_pose(segments, "F20", 0.0, 0.0), None, "invalid ignored")


def main() -> int:
    for test in (test_same_floor_segment_direction_and_bounds, test_invalid_segment_is_not_active):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] terrain segment contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
