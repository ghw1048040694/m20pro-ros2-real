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
        "request_id": "r1",
        "route_id": "route_up",
        "plan_id": "plan-1",
        "map_epoch": 4,
        "profile_id": "route_up:terrain",
        "corridor_version": "field-v1",
        "state": "traversable",
        "cloud_age_s": 0.1,
        "certified_motion": True,
    }
    payload.update(extra)
    return payload


def create(request_id: str = "r1", *, certified: bool = True, **extra) -> dict:
    return create_connector_execution(
        route(certified=certified),
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


def test_shadow_route_is_rejected_before_motion() -> None:
    result = create(certified=False)
    assert_equal(result["ok"], False, "shadow route rejected")
    assert_equal(result["code"], "stair_execution_retired", "shadow route gate")
    assert_equal(result["actions"][0]["kind"], "stop", "shadow route stop")


def test_full_connector_sequence_is_ordered_and_uses_route_poses() -> None:
    created = create()
    assert_equal(created["execution"]["state"], "PREPARING", "initial state")
    prepared = step_connector_execution(
        created["execution"],
        event("terrain_status", status=status()),
        now_monotonic=1.0,
    )
    assert_equal(prepared["execution"]["state"], "ENTRY_NAVIGATION", "entry state")
    assert_equal(prepared["actions"][0]["pose"], route()["entry"], "entry pose is route-owned")

    traversing = step_connector_execution(
        prepared["execution"],
        event("entry_reached", entry_pose={"x": 999, "y": 999, "yaw": 9}),
        now_monotonic=2.0,
    )
    assert_equal(traversing["execution"]["state"], "TRAVERSING", "traversing state")
    assert_equal(traversing["actions"][0], {"kind": "set_gait", "gait": "stair_up"}, "up gait")

    holding = step_connector_execution(
        traversing["execution"],
        event("platform_reached", terrain_status=status()),
        now_monotonic=3.0,
    )
    assert_equal(holding["execution"]["state"], "PLATFORM_HOLD", "platform hold")
    assert_equal(holding["actions"][0]["kind"], "stop", "platform stop first")
    assert_equal(holding["actions"][1]["kind"], "request_floor_switch", "platform requests map switch")

    exiting = step_connector_execution(
        holding["execution"],
        event("floor_switch_result", ok=True, target_floor="F2", target_map_id="map_f2", post_exit_pose={"x": 999, "y": 999, "yaw": 9}),
        now_monotonic=4.0,
    )
    assert_equal(exiting["execution"]["state"], "EXIT_NAVIGATION", "exit state")
    assert_equal(exiting["actions"][0]["pose"], route()["post_exit"], "exit pose is route-owned")

    completed = step_connector_execution(
        exiting["execution"],
        event("exit_reached"),
        now_monotonic=5.0,
    )
    assert_equal(completed["execution"]["state"], "COMPLETED", "completed state")
    assert_equal(completed["actions"][-2]["kind"], "release_terrain_guard", "release terrain before resume")
    assert_equal(completed["actions"][-1]["kind"], "resume_flat_navigation", "resume flat navigation")


def test_preparing_waits_for_initial_unknown_terrain() -> None:
    created = create()
    waiting = step_connector_execution(
        created["execution"],
        event(
            "terrain_status",
            status=status(state="unknown", reason="awaiting_pointcloud"),
        ),
        now_monotonic=0.2,
    )
    assert_equal(waiting["ok"], True, "initial unknown terrain waits")
    assert_equal(waiting["execution"]["state"], "PREPARING", "initial unknown keeps preparing")
    assert_equal(waiting["actions"], [], "initial unknown emits no motion action")


def test_event_without_request_identity_is_ignored() -> None:
    created = create()
    ignored = step_connector_execution(
        created["execution"],
        {"type": "entry_reached"},
        now_monotonic=0.2,
    )
    assert_equal(ignored["code"], "connector_event_identity_missing", "event identity required")
    assert_equal(ignored["execution"]["state"], "PREPARING", "missing identity cannot advance")


def test_blocked_terrain_stops_without_recovery() -> None:
    created = create()
    blocked = step_connector_execution(
        created["execution"],
        event("terrain_status", status=status(state="blocked")),
        now_monotonic=1.0,
    )
    assert_equal(blocked["ok"], False, "blocked terrain fails")
    assert_equal(blocked["execution"]["state"], "FAILED", "blocked terminal state")
    assert_equal(blocked["actions"][-1]["kind"], "release_terrain_guard", "failure releases terrain request")
    recovered = step_connector_execution(
        blocked["execution"],
        event("terrain_status", status=status()),
        now_monotonic=2.0,
    )
    assert_equal(recovered["code"], "connector_terminal_ignored", "failed connector cannot auto-recover")
    assert_equal(recovered["actions"], [], "no recovery action")


def test_stale_event_and_wrong_floor_result_are_safe() -> None:
    created = create()
    stale = step_connector_execution(
        created["execution"],
        event("terrain_status", request_id="old", status=status()),
        now_monotonic=1.0,
    )
    assert_equal(stale["code"], "connector_stale_event_ignored", "stale event ignored")
    prepared = step_connector_execution(
        created["execution"],
        event("terrain_status", status=status()),
        now_monotonic=1.0,
    )
    traversing = step_connector_execution(prepared["execution"], event("entry_reached"), now_monotonic=2.0)
    holding = step_connector_execution(
        traversing["execution"],
        event("platform_reached", terrain_status=status()),
        now_monotonic=3.0,
    )
    mismatch = step_connector_execution(
        holding["execution"],
        event("floor_switch_result", ok=True, target_floor="F3", target_map_id="map_f3"),
        now_monotonic=4.0,
    )
    assert_equal(mismatch["execution"]["state"], "FAILED", "wrong switch result fails")
    assert_equal(mismatch["actions"][0]["kind"], "stop", "wrong switch stops")


def test_operator_stop_and_timeout_are_terminal() -> None:
    created = create(stage_timeout_s=2.0)
    stopped = step_connector_execution(created["execution"], event("stop_requested"), now_monotonic=0.1)
    assert_equal(stopped["execution"]["state"], "STOPPED", "operator stop")
    assert_equal(stopped["actions"][-1]["kind"], "release_terrain_guard", "operator stop releases terrain request")
    created = create("r2", stage_timeout_s=2.0)
    timeout = step_connector_execution(created["execution"], event("terrain_status", request_id="r2", status=status(request_id="r2")), now_monotonic=3.0)
    assert_equal(timeout["execution"]["state"], "FAILED", "stage timeout")
    assert_equal(timeout["code"], "connector_stage_timeout", "timeout code")
    assert_equal(timeout["actions"][-1]["kind"], "release_terrain_guard", "timeout releases terrain request")


def main() -> int:
    tests = [
        test_shadow_route_is_rejected_before_motion,
        test_full_connector_sequence_is_ordered_and_uses_route_poses,
        test_preparing_waits_for_initial_unknown_terrain,
        test_event_without_request_identity_is_ignored,
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
