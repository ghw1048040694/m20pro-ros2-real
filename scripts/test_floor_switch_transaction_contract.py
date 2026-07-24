#!/usr/bin/env python3
"""Offline contract for the minimal map-switch and relocalization sequence."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.floor_switch_transaction_contract import (  # noqa: E402
    advance_transaction,
    begin_transaction,
    completion_decision,
    next_map_epoch,
    recover_interrupted_transaction,
    request_admission,
)


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def context() -> dict:
    return {
        "task_id": "task-1",
        "source_floor": "F1",
        "target_floor": "F2",
        "source_map_id": "map-f1",
        "target_map_id": "map-f2",
        "route": {"id": "route-up"},
    }


def transaction() -> dict:
    result = begin_transaction(
        request={"request_id": "switch-1", "plan_id": "task-1:run-1"},
        context=context(),
        map_epoch=1,
        now_text="now",
        now_unix_s=100.0,
    )
    check(result["ok"], "transaction starts")
    check(result["transaction"]["state"] == "SWITCHING_MAP", "map switch is first")
    return result["transaction"]


def activation(*, nav2: bool = True, factory: bool = True) -> dict:
    return {
        "ok": nav2 and factory,
        "nav2_load_map": {"ok": nav2},
        "factory_apply_map": {"ok": factory},
    }


def relocalization(*, confirmed: bool = True, pose: bool = True) -> dict:
    return {
        "confirmed": confirmed,
        "verification": {"factory_pose_accepted": pose},
    }


def relocalizing() -> dict:
    result = advance_transaction(
        transaction(),
        "RELOCALIZING",
        message="relocalize",
        now_text="t1",
        now_unix_s=101.0,
    )
    check(result["ok"], "map activation advances to relocalization")
    return result["transaction"]


def test_identity_and_single_inflight_admission() -> None:
    check(next_map_epoch({"floor_switch_map_epoch": 7}) == 8, "epoch increments")
    check(request_admission(None, "switch-1")["ok"], "empty runtime admits switch")
    busy = request_admission(transaction(), "switch-2")
    check(busy["code"] == "floor_switch_busy", "one in-flight switch owns map activation")
    missing_plan = begin_transaction(
        request={"request_id": "switch-1"},
        context=context(),
        map_epoch=1,
        now_text="now",
    )
    check(missing_plan["code"] == "floor_switch_plan_id_missing", "plan identity is required")


def test_completion_requires_only_maps_2101_and_target_pose() -> None:
    tx = relocalizing()
    check(
        completion_decision(
            tx,
            task_active=True,
            target_map_id="map-f2",
            map_activation=activation(),
            relocalization=relocalization(),
        )["ok"],
        "both maps plus 2101 target pose complete the switch",
    )
    nav2_failed = completion_decision(
        tx,
        task_active=True,
        target_map_id="map-f2",
        map_activation=activation(nav2=False),
        relocalization=relocalization(),
    )
    check(nav2_failed["code"] == "floor_switch_map_failed", "104 failure blocks completion")
    factory_failed = completion_decision(
        tx,
        task_active=True,
        target_map_id="map-f2",
        map_activation=activation(factory=False),
        relocalization=relocalization(),
    )
    check(factory_failed["code"] == "floor_switch_map_failed", "106 failure blocks completion")
    pose_missing = completion_decision(
        tx,
        task_active=True,
        target_map_id="map-f2",
        map_activation=activation(),
        relocalization=relocalization(pose=False),
    )
    check(
        pose_missing["code"] == "floor_switch_relocalization_failed",
        "2101 without target-map pose cannot continue",
    )


def test_failure_is_terminal_and_restart_never_resumes_motion() -> None:
    failed = advance_transaction(
        transaction(),
        "FAILED",
        code="floor_switch_map_failed",
        message="stop",
        now_text="t1",
        now_unix_s=102.0,
    )
    check(failed["ok"], "active switch can fail")
    check(failed["transaction"]["state"] == "FAILED", "failure is terminal")
    check(request_admission(failed["transaction"], "switch-2")["ok"], "failure does not create a permanent lock")
    recovered = recover_interrupted_transaction(
        relocalizing(), now_text="restart", now_unix_s=103.0
    )
    check(recovered["changed"], "restart closes in-flight switch")
    check(recovered["transaction"]["state"] == "FAILED", "restart cannot resume movement")


def test_phase_order_is_fixed() -> None:
    invalid = advance_transaction(
        transaction(),
        "COMMITTED",
        message="skip",
        now_text="t1",
    )
    check(invalid["code"] == "floor_switch_phase_invalid", "cannot skip relocalization")
    tx = relocalizing()
    done = advance_transaction(
        tx,
        "COMMITTED",
        message="done",
        now_text="t2",
        now_unix_s=104.0,
    )
    check(done["ok"], "verified relocalization commits")
    check(done["transaction"]["duration_s"] == 4.0, "duration is observable")


def main() -> int:
    test_identity_and_single_inflight_admission()
    test_completion_requires_only_maps_2101_and_target_pose()
    test_failure_is_terminal_and_restart_never_resumes_motion()
    test_phase_order_is_fixed()
    print("floor switch transaction contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
