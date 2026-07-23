#!/usr/bin/env python3
"""Tests for the unified n=1/n>1 navigation plan contract."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.unified_navigation_contract import (  # noqa: E402
    build_unified_navigation_plan,
    navigation_plan_record,
    runtime_transition_for_annotation,
    summarize_plan,
    task_navigation_plan_state,
)


def pose(x: float) -> dict:
    return {"x": x, "y": 0.0, "yaw": 0.0}


def route(source: str, target: str, name: str) -> dict:
    return {
        "id": name,
        "source_floor": source,
        "target_floor": target,
        "configured": True,
        "poses": {"entry": pose(0), "source_platform": pose(1), "target_platform": pose(2), "post_exit": pose(3)},
    }


def test_single_floor_is_unified_plan() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F7", "map_id": "map7", "pose": pose(1)},
        "p2": {"id": "p2", "floor": "F7", "map_id": "map7", "pose": pose(2)},
    }
    plan = build_unified_navigation_plan(["p1", "p2"], annotations_by_id=annotations, routes=[])
    assert plan["ok"] is True
    assert plan["single_floor"] is True
    assert plan["floor_count"] == 1
    assert plan["transitions"] == []
    assert summarize_plan(plan)["kind"] == "unified_navigation_plan"


def test_multi_floor_uses_same_plan_shape() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F1", "map_id": "map1", "pose": pose(1)},
        "p2": {"id": "p2", "floor": "F2", "map_id": "map2", "pose": pose(2)},
        "p3": {"id": "p3", "floor": "F3", "map_id": "map3", "pose": pose(3)},
    }
    plan = build_unified_navigation_plan(
        ["p1", "p2", "p3"],
        annotations_by_id=annotations,
        routes=[route("F1", "F2", "r12"), route("F2", "F3", "r23")],
    )
    assert plan["ok"] is True
    assert plan["single_floor"] is False
    assert plan["floor_count"] == 3
    assert [item["route_id"] for item in plan["transitions"]] == ["r12", "r23"]
    assert [item["terrain_guard"]["profile_id"] for item in plan["transitions"]] == [
        "r12:terrain",
        "r23:terrain",
    ]
    assert all(not item["terrain_guard"]["certified_motion"] for item in plan["transitions"])
    assert [item["path_step_index"] for item in plan["transitions"]] == [0, 0]
    assert [item["path_step_count"] for item in plan["transitions"]] == [1, 1]
    assert summarize_plan(plan)["transition_count"] == 2


def test_multi_hop_connector_is_expanded() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F1", "map_id": "map1", "pose": pose(1)},
        "p3": {"id": "p3", "floor": "F3", "map_id": "map3", "pose": pose(3)},
    }
    plan = build_unified_navigation_plan(
        ["p1", "p3"],
        annotations_by_id=annotations,
        routes=[route("F1", "F2", "r12"), route("F2", "F3", "r23")],
    )
    assert plan["ok"] is True
    assert [item["source_floor"] for item in plan["transitions"]] == ["F1", "F2"]
    assert [item["target_floor"] for item in plan["transitions"]] == ["F2", "F3"]
    assert plan["transition_paths"] == [
        {"source_floor": "F1", "target_floor": "F3", "floor_path": ["F1", "F2", "F3"]}
    ]
    assert [item["path_step_index"] for item in plan["transitions"]] == [0, 1]
    assert [item["path_step_count"] for item in plan["transitions"]] == [2, 2]
    from_f1 = runtime_transition_for_annotation(plan, "p3", current_floor="F1")
    assert [item["route_id"] for item in from_f1["edges"]] == ["r12", "r23"]
    from_intermediate_f2 = runtime_transition_for_annotation(plan, "p3", current_floor="F2")
    assert from_intermediate_f2["action"] == "transition"
    assert [item["route_id"] for item in from_intermediate_f2["edges"]] == ["r23"]


def test_returning_to_previous_floor_keeps_contiguous_segments() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F1", "map_id": "map1", "pose": pose(1)},
        "p2": {"id": "p2", "floor": "F2", "map_id": "map2", "pose": pose(2)},
        "p3": {"id": "p3", "floor": "F1", "map_id": "map1", "pose": pose(3)},
    }
    plan = build_unified_navigation_plan(
        ["p1", "p2", "p3"],
        annotations_by_id=annotations,
        routes=[route("F1", "F2", "r12"), route("F2", "F1", "r21")],
    )
    assert plan["ok"] is True
    assert plan["single_floor"] is False
    assert plan["floor_sequence"] == ["F1", "F2", "F1"]
    assert [item["annotation_ids"] for item in plan["segments"]] == [["p1"], ["p2"], ["p3"]]
    assert [item["source_segment_index"] for item in plan["transitions"]] == [0, 1]
    assert [item["target_segment_index"] for item in plan["transitions"]] == [1, 2]
    record = navigation_plan_record(plan)
    assert record["kind"] == "unified_navigation_plan"
    assert "annotations" not in record
    assert "route" not in record["transitions"][0]
    assert record["segments"][2]["annotation_ids"] == ["p3"]
    assert record["transitions"][1]["terrain_guard"]["profile_id"] == "r21:terrain"

    back_to_f1 = runtime_transition_for_annotation(
        plan,
        "p3",
        current_floor="F2",
    )
    assert back_to_f1["action"] == "transition"
    assert [item["route_id"] for item in back_to_f1["edges"]] == ["r21"]
    assert back_to_f1["target_segment_index"] == 2

    forward_to_f2 = runtime_transition_for_annotation(
        plan,
        "p2",
        current_floor="F1",
    )
    assert [item["route_id"] for item in forward_to_f2["edges"]] == ["r12"]


def test_missing_connector_fails_before_execution() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F1", "map_id": "map1", "pose": pose(1)},
        "p2": {"id": "p2", "floor": "F2", "map_id": "map2", "pose": pose(2)},
    }
    plan = build_unified_navigation_plan(["p1", "p2"], annotations_by_id=annotations, routes=[])
    assert plan["ok"] is False
    assert plan["code"] == "navigation_route_missing"


def test_mixed_maps_on_one_floor_are_rejected() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F1", "map_id": "map1", "pose": pose(1)},
        "p2": {"id": "p2", "floor": "F1", "map_id": "map1_old", "pose": pose(2)},
    }
    plan = build_unified_navigation_plan(["p1", "p2"], annotations_by_id=annotations, routes=[])
    assert plan["ok"] is False
    assert plan["code"] == "navigation_mixed_maps_on_floor"


def test_legacy_task_is_migrated_to_compact_plan() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F1", "map_id": "map1", "pose": pose(1)},
        "p2": {"id": "p2", "floor": "F2", "map_id": "map2", "pose": pose(2)},
    }
    state = task_navigation_plan_state(
        {"id": "task1", "map_id": "map1", "annotation_ids": ["p1", "p2"]},
        annotations_by_id=annotations,
        routes=[route("F1", "F2", "r12")],
    )
    assert state["ok"] is True
    assert state["migrated"] is True
    assert state["record"]["annotation_ids"] == ["p1", "p2"]
    assert state["record"]["transitions"][0]["route_id"] == "r12"


def test_existing_plan_must_match_current_route_and_order() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F1", "map_id": "map1", "pose": pose(1)},
        "p2": {"id": "p2", "floor": "F2", "map_id": "map2", "pose": pose(2)},
    }
    routes = [route("F1", "F2", "r12")]
    plan = build_unified_navigation_plan(
        ["p1", "p2"], annotations_by_id=annotations, routes=routes
    )
    task = {
        "id": "task1",
        "map_id": "map1",
        "annotation_ids": ["p1", "p2"],
        "navigation_plan": navigation_plan_record(plan),
    }
    matched = task_navigation_plan_state(task, annotations_by_id=annotations, routes=routes)
    assert matched["ok"] is True
    assert matched["migrated"] is False

    stale_task = dict(task)
    stale_task["navigation_plan"] = navigation_plan_record(
        build_unified_navigation_plan(
            ["p1", "p2"],
            annotations_by_id=annotations,
            routes=[route("F1", "F2", "old-route")],
        )
    )
    stale = task_navigation_plan_state(stale_task, annotations_by_id=annotations, routes=routes)
    assert stale["ok"] is False
    assert stale["code"] == "navigation_task_plan_stale"


def test_task_map_binding_must_match_plan() -> None:
    annotations = {
        "p1": {"id": "p1", "floor": "F1", "map_id": "map1", "pose": pose(1)},
    }
    state = task_navigation_plan_state(
        {"id": "task1", "map_id": "wrong-map", "annotation_ids": ["p1"]},
        annotations_by_id=annotations,
        routes=[],
    )
    assert state["ok"] is False
    assert state["code"] == "navigation_task_map_mismatch"


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
    print("unified navigation contract tests passed")
