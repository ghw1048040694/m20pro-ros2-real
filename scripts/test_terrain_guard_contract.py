#!/usr/bin/env python3
"""Offline contract tests for the 106-local stair terrain guard."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.terrain_guard_contract import (  # noqa: E402
    inspect_cloud,
    terrain_request_ownership_decision,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def request(direction: str = "forward") -> dict:
    return {
        "enabled": True,
        "request_id": "terrain-1",
        "route_id": "stairs-a-up",
        "plan_id": "task-1:run-1",
        "map_epoch": 7,
        "corridor_version": "corridor-v1",
        "direction": direction,
        "corridor": {
            "width_m": 1.0,
            "lookahead_m": 1.2,
            "bin_size_m": 0.2,
            "min_step_height_m": 0.05,
            "max_step_height_m": 0.24,
            "obstacle_height_m": 0.22,
            "min_points_per_bin": 4,
            "min_step_count": 2,
            "min_coverage": 0.55,
        },
    }


def cloud(levels: list[float], *, reverse: bool = False) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index, level in enumerate(levels):
        x = -(index * 0.2 + 0.05) if reverse else index * 0.2 + 0.05
        for lateral in (-0.30, -0.10, 0.10, 0.30):
            points.append((x, lateral, level))
    return points


def test_no_corridor_is_unknown() -> None:
    result = inspect_cloud(cloud([0.0] * 6), request={"direction": "forward"}, cloud_age_s=0.01)
    assert_equal(result["state"], "unknown", "missing corridor state")
    assert_equal(result["permit_motion"], False, "missing corridor permission")


def test_request_ownership_rejects_stale_release_and_malformed_replacement() -> None:
    active = request()
    stale_release = terrain_request_ownership_decision(
        active,
        {**active, "enabled": False, "map_epoch": 6},
    )
    assert_equal(stale_release["code"], "terrain_request_owner_mismatch", "old release rejected")
    assert_equal(stale_release["preserve_current"], True, "old release preserves active corridor")
    malformed = terrain_request_ownership_decision(active, {"enabled": True})
    assert_equal(malformed["code"], "terrain_request_identity_missing", "malformed request rejected")
    assert_equal(malformed["preserve_current"], True, "malformed request preserves active corridor")
    release = terrain_request_ownership_decision(active, {**active, "enabled": False})
    assert_equal(release["action"], "release", "matching owner can release corridor")


def test_stale_cloud_is_stale() -> None:
    result = inspect_cloud(cloud([0.0] * 6), request=request(), cloud_age_s=0.8)
    assert_equal(result["state"], "stale", "stale state")
    assert_equal(result["permit_motion"], False, "stale permission")


def test_empty_cloud_is_unknown() -> None:
    result = inspect_cloud([], request=request(), cloud_age_s=0.01)
    assert_equal(result["state"], "unknown", "empty cloud state")
    assert_equal(result["reason"], "corridor_no_points", "empty cloud reason")


def test_low_coverage_is_unknown() -> None:
    result = inspect_cloud(cloud([0.0, 0.0, 0.12]), request=request(), cloud_age_s=0.01)
    assert_equal(result["state"], "unknown", "low coverage state")
    assert_equal(result["reason"], "corridor_coverage_low", "low coverage reason")


def test_narrow_returns_are_not_a_full_tread() -> None:
    points: list[tuple[float, float, float]] = []
    for index, level in enumerate([0.0, 0.0, 0.12, 0.12, 0.24, 0.24]):
        x = index * 0.2 + 0.05
        for lateral in (-0.05, 0.05, 0.0, 0.02):
            points.append((x, lateral, level))
    result = inspect_cloud(points, request=request(), cloud_age_s=0.01)
    assert_equal(result["state"], "unknown", "narrow lateral state")
    assert_equal(result["reason"], "corridor_lateral_coverage_low", "narrow lateral reason")
    assert_equal(result["permit_motion"], False, "narrow lateral permission")


def test_profile_gap_is_unknown_even_with_two_steps() -> None:
    points = cloud([0.0, 0.0, 0.12, 0.12, 0.24, 0.24])
    points = [point for point in points if point[0] < 0.45 or point[0] > 0.65]
    result = inspect_cloud(points, request=request(), cloud_age_s=0.01)
    assert_equal(result["state"], "unknown", "profile gap state")
    assert_equal(result["reason"], "corridor_profile_gap", "profile gap reason")


def test_continuous_upstairs_is_traversable() -> None:
    result = inspect_cloud(
        cloud([0.0, 0.0, 0.12, 0.12, 0.24, 0.24]),
        request=request(),
        cloud_age_s=0.01,
    )
    assert_equal(result["state"], "traversable", "upstairs state")
    assert_equal(result["step_direction"], "up", "upstairs direction")
    assert_equal(result["min_lateral_span_m"], 0.4, "calibrated lateral threshold")
    assert_equal(result["permit_motion"], False, "shadow guard permission")


def test_continuous_downstairs_is_traversable() -> None:
    result = inspect_cloud(
        cloud([0.24, 0.24, 0.12, 0.12, 0.0, 0.0], reverse=True),
        request=request(direction="reverse"),
        cloud_age_s=0.01,
    )
    assert_equal(result["state"], "traversable", "downstairs state")
    assert_equal(result["step_direction"], "down", "downstairs direction")


def test_excessive_step_is_blocked() -> None:
    result = inspect_cloud(
        cloud([0.0, 0.0, 0.40, 0.40, 0.40, 0.40]),
        request=request(),
        cloud_age_s=0.01,
    )
    assert_equal(result["state"], "blocked", "excessive step state")
    assert_equal(result["reason"], "step_height_out_of_range", "excessive step reason")


def test_inconsistent_step_profile_is_blocked() -> None:
    result = inspect_cloud(
        cloud([0.0, 0.0, 0.12, 0.12, 0.34, 0.34]),
        request=request(),
        cloud_age_s=0.01,
    )
    assert_equal(result["state"], "blocked", "inconsistent step state")
    assert_equal(result["reason"], "step_profile_inconsistent", "inconsistent step reason")


def test_front_obstacle_is_blocked() -> None:
    result = inspect_cloud(
        cloud([0.0, 0.35, 0.35, 0.35, 0.35, 0.35]),
        request=request(),
        cloud_age_s=0.01,
    )
    assert_equal(result["state"], "blocked", "obstacle state")
    assert_equal(result["reason"], "step_height_out_of_range", "obstacle reason")
    assert_equal(result["permit_motion"], False, "obstacle permission")


def test_reverse_direction_uses_reverse_longitudinal_axis() -> None:
    result = inspect_cloud(
        cloud([0.0, 0.0, 0.12, 0.12, 0.24, 0.24], reverse=True),
        request=request(direction="reverse"),
        cloud_age_s=0.01,
    )
    assert_equal(result["state"], "traversable", "reverse state")
    assert_equal(result["step_direction"], "up", "reverse longitudinal direction")


def main() -> int:
    tests = [
        test_no_corridor_is_unknown,
        test_request_ownership_rejects_stale_release_and_malformed_replacement,
        test_stale_cloud_is_stale,
        test_empty_cloud_is_unknown,
        test_low_coverage_is_unknown,
        test_narrow_returns_are_not_a_full_tread,
        test_profile_gap_is_unknown_even_with_two_steps,
        test_continuous_upstairs_is_traversable,
        test_continuous_downstairs_is_traversable,
        test_excessive_step_is_blocked,
        test_inconsistent_step_profile_is_blocked,
        test_front_obstacle_is_blocked,
        test_reverse_direction_uses_reverse_longitudinal_axis,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] terrain guard contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
