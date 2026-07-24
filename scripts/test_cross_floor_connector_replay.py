#!/usr/bin/env python3
"""Replay the complete connector reducer and map-switch contract offline."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.floor_switch_transaction_contract import (  # noqa: E402
    advance_transaction,
    begin_transaction,
    completion_decision,
)
from m20pro_navigation.stair_executor_contract import (  # noqa: E402
    connector_motion_decision,
    create_connector_execution,
    step_connector_execution,
)


IDENTITY = {
    "request_id": "switch-replay-1",
    "route_id": "F1-F2-up",
    "plan_id": "task-1:run-1",
    "map_epoch": 7,
}
ROUTE = {
    "id": IDENTITY["route_id"],
    "source_floor": "F1",
    "target_floor": "F2",
    "source_map_id": "map-f1",
    "target_map_id": "map-f2",
    "direction": "up",
    "entry": {"x": 1.0, "y": 0.0, "yaw": 0.0},
    "source_platform": {"x": 3.0, "y": 0.0, "yaw": 0.0},
    "target_platform": {"x": 0.8, "y": 0.0, "yaw": 0.0},
    "post_exit": {"x": 2.0, "y": 0.0, "yaw": 0.0},
}


def event(event_type: str, **extra) -> dict:
    return {"type": event_type, **IDENTITY, **extra}


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    created = create_connector_execution(
        ROUTE,
        request_id=IDENTITY["request_id"],
        plan_id=IDENTITY["plan_id"],
        map_epoch=IDENTITY["map_epoch"],
        now_monotonic=0.0,
        stage_timeout_s=180.0,
    )
    check(created["actions"][0]["kind"] == "dispatch_entry_goal", "entry goal starts")
    traversing = step_connector_execution(
        created["execution"], event("entry_reached"), now_monotonic=10.0
    )
    check(traversing["execution"]["state"] == "TRAVERSING", "entry starts stairs")
    check(traversing["actions"][0] == {"kind": "set_gait", "gait": "stair_up"}, "stair gait")

    moving = connector_motion_decision(
        current_pose={"x": 1.2, "y": 0.0, "yaw": 0.0},
        target_pose=ROUTE["source_platform"],
        pose_age_s=0.1,
        pose_timeout_s=1.5,
        tolerance_m=0.5,
        speed_mps=0.12,
        direction="up",
    )
    check(moving["action"] == "move" and moving["linear_x"] > 0.0, "connector moves")
    reached = connector_motion_decision(
        current_pose={"x": 2.7, "y": 0.0, "yaw": 0.0},
        target_pose=ROUTE["source_platform"],
        pose_age_s=0.1,
        pose_timeout_s=1.5,
        tolerance_m=0.5,
        speed_mps=0.12,
        direction="up",
    )
    check(reached["action"] == "reached", "source platform stops connector")
    holding = step_connector_execution(
        traversing["execution"], event("platform_reached"), now_monotonic=20.0
    )
    check(holding["actions"][0]["kind"] == "stop_motion", "stop before switching")
    check(holding["actions"][1]["kind"] == "request_floor_switch", "request switch")

    context = {
        "task_id": "task-1",
        "source_floor": "F1",
        "target_floor": "F2",
        "source_map_id": "map-f1",
        "target_map_id": "map-f2",
        "route": ROUTE,
    }
    started = begin_transaction(
        request=IDENTITY,
        context=context,
        map_epoch=IDENTITY["map_epoch"],
        now_text="start",
        now_unix_s=100.0,
    )
    relocalizing = advance_transaction(
        started["transaction"],
        "RELOCALIZING",
        message="maps active",
        now_text="maps",
        now_unix_s=102.0,
    )
    activation = {
        "ok": True,
        "nav2_load_map": {"ok": True},
        "factory_apply_map": {"ok": True},
    }
    relocalization = {
        "confirmed": True,
        "verification": {"factory_pose_accepted": True},
    }
    ready = completion_decision(
        relocalizing["transaction"],
        task_active=True,
        target_map_id="map-f2",
        map_activation=activation,
        relocalization=relocalization,
    )
    check(ready["ok"], "target maps and 2101 pose complete switch")
    committed = advance_transaction(
        relocalizing["transaction"],
        "COMMITTED",
        message="ready",
        now_text="done",
        now_unix_s=105.0,
    )
    check(committed["ok"], "map switch commits")

    exiting = step_connector_execution(
        holding["execution"],
        event(
            "floor_switch_result",
            ok=True,
            target_floor="F2",
            target_map_id="map-f2",
        ),
        now_monotonic=25.0,
    )
    check(exiting["actions"][0]["kind"] == "dispatch_exit_goal", "exit follows switch")
    completed = step_connector_execution(
        exiting["execution"], event("exit_reached"), now_monotonic=35.0
    )
    check(completed["execution"]["state"] == "COMPLETED", "connector completes")
    check(completed["actions"] == [{"kind": "set_gait", "gait": "flat"}], "flat gait restored")
    print("cross-floor connector replay passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
