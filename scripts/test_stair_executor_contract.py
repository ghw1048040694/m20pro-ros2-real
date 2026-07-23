#!/usr/bin/env python3
"""Offline tests for the certified stair connector reducer."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.stair_executor_contract import (  # noqa: E402
    create_connector_execution,
    step_connector_execution,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def route(certified: bool = True) -> dict:
    return {
        "id": "route_up",
        "source_floor": "F1",
        "target_floor": "F2",
        "source_map_id": "map_f1",
        "target_map_id": "map_f2",
        "direction": "up",
        "entry": {"x": 1.0, "y": 2.0, "yaw": 0.1},
        "source_platform": {"x": 2.0, "y": 2.0, "yaw": 0.1},
        "target_platform": {"x": 3.0, "y": 2.0, "yaw": 0.1},
        "post_exit": {"x": 4.0, "y": 2.0, "yaw": 0.1},
        "terrain_guard": {
            "profile_id": "route_up:terrain",
            "corridor_version": "field-v1",
            "motion_policy": "certified_connector" if certified else "stop_only",
            "certified_motion": certified,
            "corridor": {"width_m": 1.0, "lookahead_m": 2.4},
        },
    }


def status(**extra) -> dict:
    payload = {
        "profile_id": "route_up:terrain",
        "corridor_version": "field-v1",
        "state": "traversable",
        "cloud_age_s": 0.1,
        "certified_motion": True,
    }
    payload.update(extra)
    return payload


def test_shadow_route_is_rejected_before_motion() -> None:
    result = create_connector_execution(route(certified=False), request_id="r1", now_monotonic=0.0)
    assert_equal(result["ok"], False, "shadow route rejected")
    assert_equal(result["code"], "stair_execution_retired", "shadow route gate")
    assert_equal(result["actions"][0]["kind"], "stop", "shadow route stop")


def test_full_connector_sequence_is_ordered_and_uses_route_poses() -> None:
    created = create_connector_execution(route(), request_id="r1", now_monotonic=0.0)
    assert_equal(created["execution"]["state"], "PREPARING", "initial state")
    prepared = step_connector_execution(
        created["execution"],
        {"type": "terrain_status", "request_id": "r1", "status": status()},
        now_monotonic=1.0,
    )
    assert_equal(prepared["execution"]["state"], "ENTRY_NAVIGATION", "entry state")
    assert_equal(prepared["actions"][0]["pose"], route()["entry"], "entry pose is route-owned")

    traversing = step_connector_execution(
        prepared["execution"],
        {"type": "entry_reached", "request_id": "r1", "entry_pose": {"x": 999, "y": 999, "yaw": 9}},
        now_monotonic=2.0,
    )
    assert_equal(traversing["execution"]["state"], "TRAVERSING", "traversing state")
    assert_equal(traversing["actions"][0], {"kind": "set_gait", "gait": "stair_up"}, "up gait")

    holding = step_connector_execution(
        traversing["execution"],
        {"type": "platform_reached", "request_id": "r1", "terrain_status": status()},
        now_monotonic=3.0,
    )
    assert_equal(holding["execution"]["state"], "PLATFORM_HOLD", "platform hold")
    assert_equal(holding["actions"][0]["kind"], "stop", "platform stop first")
    assert_equal(holding["actions"][1]["kind"], "request_floor_switch", "platform requests map switch")

    exiting = step_connector_execution(
        holding["execution"],
        {"type": "floor_switch_result", "request_id": "r1", "ok": True, "target_floor": "F2", "target_map_id": "map_f2", "post_exit_pose": {"x": 999, "y": 999, "yaw": 9}},
        now_monotonic=4.0,
    )
    assert_equal(exiting["execution"]["state"], "EXIT_NAVIGATION", "exit state")
    assert_equal(exiting["actions"][0]["pose"], route()["post_exit"], "exit pose is route-owned")

    completed = step_connector_execution(
        exiting["execution"],
        {"type": "exit_reached", "request_id": "r1"},
        now_monotonic=5.0,
    )
    assert_equal(completed["execution"]["state"], "COMPLETED", "completed state")
    assert_equal(completed["actions"][-1]["kind"], "resume_flat_navigation", "resume flat navigation")


def test_blocked_terrain_stops_without_recovery() -> None:
    created = create_connector_execution(route(), request_id="r1", now_monotonic=0.0)
    blocked = step_connector_execution(
        created["execution"],
        {"type": "terrain_status", "request_id": "r1", "status": status(state="blocked")},
        now_monotonic=1.0,
    )
    assert_equal(blocked["ok"], False, "blocked terrain fails")
    assert_equal(blocked["execution"]["state"], "FAILED", "blocked terminal state")
    recovered = step_connector_execution(
        blocked["execution"],
        {"type": "terrain_status", "request_id": "r1", "status": status()},
        now_monotonic=2.0,
    )
    assert_equal(recovered["code"], "connector_terminal_ignored", "failed connector cannot auto-recover")
    assert_equal(recovered["actions"], [], "no recovery action")


def test_stale_event_and_wrong_floor_result_are_safe() -> None:
    created = create_connector_execution(route(), request_id="r1", now_monotonic=0.0)
    stale = step_connector_execution(
        created["execution"],
        {"type": "terrain_status", "request_id": "old", "status": status()},
        now_monotonic=1.0,
    )
    assert_equal(stale["code"], "connector_stale_event_ignored", "stale event ignored")
    prepared = step_connector_execution(
        created["execution"],
        {"type": "terrain_status", "request_id": "r1", "status": status()},
        now_monotonic=1.0,
    )
    traversing = step_connector_execution(prepared["execution"], {"type": "entry_reached", "request_id": "r1"}, now_monotonic=2.0)
    holding = step_connector_execution(
        traversing["execution"],
        {"type": "platform_reached", "request_id": "r1", "terrain_status": status()},
        now_monotonic=3.0,
    )
    mismatch = step_connector_execution(
        holding["execution"],
        {"type": "floor_switch_result", "request_id": "r1", "ok": True, "target_floor": "F3", "target_map_id": "map_f3"},
        now_monotonic=4.0,
    )
    assert_equal(mismatch["execution"]["state"], "FAILED", "wrong switch result fails")
    assert_equal(mismatch["actions"][0]["kind"], "stop", "wrong switch stops")


def test_operator_stop_and_timeout_are_terminal() -> None:
    created = create_connector_execution(route(), request_id="r1", now_monotonic=0.0, stage_timeout_s=2.0)
    stopped = step_connector_execution(created["execution"], {"type": "stop_requested", "request_id": "r1"}, now_monotonic=0.1)
    assert_equal(stopped["execution"]["state"], "STOPPED", "operator stop")
    created = create_connector_execution(route(), request_id="r2", now_monotonic=0.0, stage_timeout_s=2.0)
    timeout = step_connector_execution(created["execution"], {"type": "terrain_status", "request_id": "r2", "status": status()}, now_monotonic=3.0)
    assert_equal(timeout["execution"]["state"], "FAILED", "stage timeout")
    assert_equal(timeout["code"], "connector_stage_timeout", "timeout code")


def main() -> int:
    tests = [
        test_shadow_route_is_rejected_before_motion,
        test_full_connector_sequence_is_ordered_and_uses_route_poses,
        test_blocked_terrain_stops_without_recovery,
        test_stale_event_and_wrong_floor_result_are_safe,
        test_operator_stop_and_timeout_are_terminal,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] stair executor contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
