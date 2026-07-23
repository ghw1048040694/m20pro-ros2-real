#!/usr/bin/env python3
"""Offline tests for the single stair semantic-action translation boundary."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.stair_action_orchestrator_contract import (  # noqa: E402
    event_for_floor_switch_result,
    event_for_stair_status,
    translate_action_envelope,
)


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def envelope(actions: list[dict], sequence: int = 1) -> dict:
    return {
        "source": "m20pro_stair_executor",
        "request_id": "stair-1",
        "route_id": "route-f1-f2",
        "plan_id": "plan-1",
        "map_epoch": 4,
        "source_floor": "F1",
        "target_floor": "F2",
        "source_map_id": "map-f1",
        "target_map_id": "map-f2",
        "sequence": sequence,
        "actions": actions,
    }


def test_safe_actions_map_to_existing_topics() -> None:
    result = translate_action_envelope(
        envelope(
            [
                {"kind": "dispatch_entry_goal", "map_id": "map-f1", "pose": {"x": 1, "y": 2, "yaw": 0.1}},
                {
                    "kind": "request_floor_switch",
                    "request_id": "stair-1",
                    "source_floor": "F1",
                    "target_floor": "F2",
                    "target_map_id": "map-f2",
                },
                {"kind": "stop", "reason": "platform_reached"},
            ]
        )
    )
    check(result["ok"], "safe action translation succeeds")
    kinds = [item["kind"] for item in result["commands"]]
    check(kinds == ["publish_floor_goal", "publish_floor_switch_request", "publish_stop_task"], "safe topics are ordered")
    check(result["commands"][0]["topic"] == "/m20pro/floor_goal", "entry uses existing floor goal")
    check(result["commands"][1]["topic"] == "/m20pro/floor_switch_request", "switch uses web transaction topic")
    check(result["commands"][2]["topic"] == "/m20pro/stop_task", "stop uses existing stop topic")


def test_terrain_guard_has_one_identity_bound_action_owner() -> None:
    started = translate_action_envelope(
        envelope(
            [
                {
                    "kind": "request_terrain_guard",
                    "request_id": "stair-1",
                    "route_id": "route-f1-f2",
                    "profile_id": "route-f1-f2:terrain",
                    "corridor_version": "field-v1",
                    "direction": "up",
                    "corridor": {"width_m": 1.0, "lookahead_m": 2.0},
                }
            ]
        )
    )
    check(started["ok"], "terrain request is accepted")
    command = started["commands"][0]
    check(command["kind"] == "publish_terrain_guard_request", "orchestrator owns terrain request")
    check(command["payload"]["enabled"] is True, "terrain request enables matching profile")
    check(command["payload"]["request_id"] == "stair-1", "terrain request keeps identity")
    released = translate_action_envelope(
        envelope([{"kind": "release_terrain_guard"}], sequence=2),
        expected_identity=started["identity"],
        last_sequence=1,
    )
    check(released["ok"], "terrain release is accepted")
    check(released["commands"][0]["payload"]["enabled"] is False, "terrain release disables request")

    mismatch = envelope(
        [
            {
                "kind": "request_terrain_guard",
                "request_id": "other",
                "route_id": "route-f1-f2",
                "profile_id": "route-f1-f2:terrain",
                "corridor_version": "field-v1",
                "corridor": {"width_m": 1.0, "lookahead_m": 2.0},
            }
        ]
    )
    rejected = translate_action_envelope(mismatch)
    check(rejected["code"] == "stair_action_terrain_identity_mismatch", "terrain identity mismatch rejected")


def test_motion_actions_remain_non_dispatchable_intents() -> None:
    result = translate_action_envelope(
        envelope(
            [
                {"kind": "set_gait", "gait": "stair_up"},
                {"kind": "start_connector_motion", "route_id": "route-f1-f2"},
            ]
        )
    )
    check(result["ok"], "semantic actions are accepted as intents")
    check(len(result["commands"]) == 2, "each semantic action preserves reducer order")
    intent = result["commands"][0]
    check(intent["kind"] == "publish_semantic_intent", "motion becomes semantic intent")
    check(all(item["dispatchable"] is False for item in result["commands"]), "motion intents are never dispatchable")
    ordered = translate_action_envelope(
        envelope(
            [
                {"kind": "set_gait", "gait": "flat"},
                {"kind": "release_terrain_guard"},
                {"kind": "resume_flat_navigation"},
            ]
        )
    )
    check(
        [item["kind"] for item in ordered["commands"]]
        == ["publish_semantic_intent", "publish_terrain_guard_request", "publish_semantic_intent"],
        "completion command order is preserved",
    )
    check("cmd_vel" not in str(result), "translation never creates cmd_vel")
    check("gait_command" not in str(result), "translation never creates vendor gait topic")


def test_identity_and_sequence_barriers() -> None:
    first = translate_action_envelope(envelope([{ "kind": "stop" }]))
    check(first["ok"], "first sequence accepted")
    stale = translate_action_envelope(
        envelope([{ "kind": "stop" }], sequence=1),
        expected_identity=first["identity"],
        last_sequence=1,
    )
    check(stale.get("ignored") is True, "duplicate sequence ignored")
    mismatch = translate_action_envelope(
        envelope([{ "kind": "stop" }], sequence=2),
        expected_identity={**first["identity"], "request_id": "other"},
        last_sequence=1,
    )
    check(mismatch["code"] == "stair_action_identity_mismatch", "wrong request rejected")


def test_status_events_are_bound_to_current_stage() -> None:
    identity = {"request_id": "stair-1", "route_id": "route-f1-f2", "plan_id": "plan-1", "map_epoch": 4}
    entry = event_for_stair_status(
        "nav_goal_succeeded label=floor_goal goal_seq=2",
        identity=identity,
        expected_nav_label="floor_goal",
        expected_stage="stair_entry",
        expected_goal_seq=2,
    )
    check(entry and entry["type"] == "entry_reached", "entry success becomes event")
    exit_event = event_for_stair_status(
        "nav_goal_succeeded label=floor_goal goal_seq=4",
        identity=identity,
        expected_nav_label="floor_goal",
        expected_stage="stair_exit",
        expected_goal_seq=4,
    )
    check(exit_event and exit_event["type"] == "exit_reached", "exit stage uses the same floor_goal API")
    stale = event_for_stair_status(
        "nav_goal_succeeded label=floor_goal goal_seq=3",
        identity=identity,
        expected_nav_label="floor_goal",
        expected_stage="stair_entry",
        expected_goal_seq=2,
    )
    check(stale is None, "wrong stage cannot advance reducer")
    not_accepted = event_for_stair_status(
        "nav_goal_succeeded label=floor_goal goal_seq=2",
        identity=identity,
        expected_nav_label="floor_goal",
        expected_stage="stair_entry",
    )
    check(not_accepted is None, "success without current accepted goal is ignored")
    early_failure = event_for_stair_status(
        "error reason=unknown_goal_floor floor=F2 label=floor_goal",
        identity=identity,
        expected_nav_label="floor_goal",
        expected_stage="stair_entry",
    )
    check(early_failure and early_failure["type"] == "stop_requested", "identified early floor goal error stops connector")
    switch = event_for_floor_switch_result(
        {"request_id": "stair-1", "ok": True, "target_floor": "F2", "target_map_id": "map-f2"},
        identity=identity,
    )
    check(switch and switch["type"] == "floor_switch_result", "matching switch result becomes event")
    wrong = event_for_floor_switch_result(
        {"request_id": "other", "ok": True}, identity=identity
    )
    check(wrong is None, "late switch result is ignored")


if __name__ == "__main__":
    test_safe_actions_map_to_existing_topics()
    test_terrain_guard_has_one_identity_bound_action_owner()
    test_motion_actions_remain_non_dispatchable_intents()
    test_identity_and_sequence_barriers()
    test_status_events_are_bound_to_current_stage()
    print("stair action orchestrator contract tests passed")
