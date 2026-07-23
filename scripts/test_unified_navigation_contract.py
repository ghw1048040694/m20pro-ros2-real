#!/usr/bin/env python3
"""Tests for the unified n=1/n>1 navigation plan contract."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.unified_navigation_contract import (  # noqa: E402
    build_unified_navigation_plan,
    summarize_plan,
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


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
    print("unified navigation contract tests passed")

