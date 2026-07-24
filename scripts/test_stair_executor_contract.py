#!/usr/bin/env python3
"""Offline tests for the minimal directed stair connector reducer."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.stair_executor_contract import (  # noqa: E402
    connector_motion_decision,
    create_connector_execution,
    step_connector_execution,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def route(direction: str = "up") -> dict:
    return {
        "id": "route_up",
        "source_floor": "F1",
        "target_floor": "F2",
        "source_map_id": "map_f1",
        "target_map_id": "map_f2",
        "direction": direction,
        "entry": {"x": 1.0, "y": 2.0, "yaw": 0.1},
        "source_platform": {"x": 2.0, "y": 2.0, "yaw": 0.1},
        "target_platform": {"x": 3.0, "y": 2.0, "yaw": 0.1},
        "post_exit": {"x": 4.0, "y": 2.0, "yaw": 0.1},
    }


def create(request_id: str = "r1", **extra) -> dict:
    return create_connector_execution(
        route(),
        request_id=request_id,
        plan_id="plan-1",
        map_epoch=4,
        now_monotonic=0.0,
        **extra,
    )


def event(event_type: str, request_id: str = "r1", **extra) -> dict:
    return {
        "type": event_type,
        "request_id": request_id,
        "route_id": "route_up",
        "plan_id": "plan-1",
        "map_epoch": 4,
        **extra,
    }


def test_valid_route_starts_with_entry_navigation() -> None:
    result = create()
    assert_equal(result["ok"], True, "configured route accepted")
    assert_equal(result["execution"]["state"], "ENTRY_NAVIGATION", "entry is first state")
    assert_equal(result["actions"][0], {"kind": "set_gait", "gait": "flat"}, "executor owns entry flat gait")
    assert_equal(result["actions"][1]["kind"], "dispatch_entry_goal", "entry goal follows flat gait")


def test_full_connector_sequence_is_ordered_and_uses_route_poses() -> None:
    created = create()
    assert_equal(created["actions"][1]["pose"], route()["entry"], "entry pose is route-owned")

    traversing = step_connector_execution(
        created["execution"],
        event("entry_reached", entry_pose={"x": 999, "y": 999, "yaw": 9}),
        now_monotonic=1.0,
    )
    assert_equal(traversing["execution"]["state"], "TRAVERSING", "traversing state")
    assert_equal(traversing["actions"][0], {"kind": "set_gait", "gait": "stair_up"}, "up gait")
    assert_equal(traversing["actions"][1]["target_pose"], route()["source_platform"], "motion owns source platform")

    holding = step_connector_execution(
        traversing["execution"],
        event("platform_reached"),
        now_monotonic=2.0,
    )
    assert_equal(holding["execution"]["state"], "PLATFORM_HOLD", "platform hold")
    assert_equal(holding["actions"][0]["kind"], "stop_motion", "platform stop first")
    assert_equal(holding["actions"][1]["kind"], "request_floor_switch", "platform requests map switch")

    exiting = step_connector_execution(
        holding["execution"],
        event("floor_switch_result", ok=True, target_floor="F2", target_map_id="map_f2", post_exit_pose={"x": 999, "y": 999, "yaw": 9}),
        now_monotonic=3.0,
    )
    assert_equal(exiting["execution"]["state"], "EXIT_NAVIGATION", "exit state")
    assert_equal(exiting["actions"][0]["pose"], route()["post_exit"], "exit pose is route-owned")

    completed = step_connector_execution(
        exiting["execution"],
        event("exit_reached"),
        now_monotonic=4.0,
    )
    assert_equal(completed["execution"]["state"], "COMPLETED", "completed state")
    assert_equal(completed["actions"][-1], {"kind": "set_gait", "gait": "flat"}, "resume flat gait")


def test_motion_decision_uses_platform_distance_and_reverse_downstairs() -> None:
    moving = connector_motion_decision(
        current_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        target_pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
        pose_age_s=0.1,
        pose_timeout_s=1.0,
        tolerance_m=0.5,
        speed_mps=0.12,
        direction="down",
    )
    assert_equal(moving["action"], "move", "fresh pose moves")
    assert_equal(moving["linear_x"], -0.12, "down stairs uses reverse motion")
    reached = connector_motion_decision(
        current_pose={"x": 1.7, "y": 0.0, "yaw": 0.0},
        target_pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
        pose_age_s=0.1,
        pose_timeout_s=1.0,
        tolerance_m=0.5,
        speed_mps=0.12,
        direction="up",
    )
    assert_equal(reached["action"], "reached", "platform distance stops motion")
    stale = connector_motion_decision(
        current_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        target_pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
        pose_age_s=2.0,
        pose_timeout_s=1.0,
        tolerance_m=0.5,
        speed_mps=0.12,
        direction="up",
    )
    assert_equal(stale["action"], "stop", "stale pose stops")


def test_event_without_request_identity_is_ignored() -> None:
    created = create()
    ignored = step_connector_execution(
        created["execution"],
        {"type": "entry_reached"},
        now_monotonic=0.2,
    )
    assert_equal(ignored["code"], "connector_event_identity_missing", "event identity required")
    assert_equal(ignored["execution"]["state"], "ENTRY_NAVIGATION", "missing identity cannot advance")


def test_stale_event_and_wrong_floor_result_are_safe() -> None:
    created = create()
    stale = step_connector_execution(
        created["execution"],
        event("entry_reached", request_id="old"),
        now_monotonic=1.0,
    )
    assert_equal(stale["code"], "connector_stale_event_ignored", "stale event ignored")
    traversing = step_connector_execution(created["execution"], event("entry_reached"), now_monotonic=2.0)
    holding = step_connector_execution(
        traversing["execution"],
        event("platform_reached"),
        now_monotonic=3.0,
    )
    mismatch = step_connector_execution(
        holding["execution"],
        event("floor_switch_result", ok=True, target_floor="F3", target_map_id="map_f3"),
        now_monotonic=4.0,
    )
    assert_equal(mismatch["execution"]["state"], "FAILED", "wrong switch result fails")
    assert_equal(mismatch["actions"][0]["kind"], "stop_motion", "wrong switch stops")


def test_operator_stop_and_timeout_are_terminal() -> None:
    created = create(stage_timeout_s=2.0)
    stopped = step_connector_execution(created["execution"], event("stop_requested"), now_monotonic=0.1)
    assert_equal(stopped["execution"]["state"], "STOPPED", "operator stop")
    assert_equal(stopped["actions"][-1]["kind"], "stop_motion", "operator stop emits zero-motion action")
    created = create("r2", stage_timeout_s=2.0)
    timeout = step_connector_execution(created["execution"], event("entry_reached", request_id="r2"), now_monotonic=3.0)
    assert_equal(timeout["execution"]["state"], "FAILED", "stage timeout")
    assert_equal(timeout["code"], "connector_stage_timeout", "timeout code")
    assert_equal(timeout["actions"][-1]["kind"], "stop_motion", "timeout stops connector motion")


def main() -> int:
    tests = [
        test_valid_route_starts_with_entry_navigation,
        test_full_connector_sequence_is_ordered_and_uses_route_poses,
        test_motion_decision_uses_platform_distance_and_reverse_downstairs,
        test_event_without_request_identity_is_ignored,
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
