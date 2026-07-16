#!/usr/bin/env python3
"""Offline tests for the cross-floor dashboard contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.multi_floor_contract import (  # noqa: E402
    build_multi_floor_workspace,
    cross_floor_task_context,
    find_floor_path,
    stair_routes_from_config,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def pose(x: float) -> dict:
    return {"x": x, "y": 0.0, "yaw": 0.0}


def floor_config() -> dict:
    return {
        "floors": {
            "F19": {
                "level": 19,
                "initial_pose": pose(0),
                "stairs": {
                    "up": {
                        "target_floor": "F20",
                        "entry": pose(1),
                        "source_platform": pose(2),
                        "target_platform": pose(3),
                        "post_exit": pose(4),
                    }
                },
            },
            "F20": {
                "level": 20,
                "initial_pose": pose(0),
                "terrain_segments": {"same_floor_ramp": {"entry": pose(0), "exit": pose(1)}},
                "stairs": {
                    "up": {
                        "target_floor": "F21",
                        "entry": pose(1),
                        "source_platform": pose(2),
                        "target_platform": pose(3),
                        "post_exit": pose(4),
                    }
                },
            },
            "F21": {"level": 21, "initial_pose": pose(0), "stairs": {}},
        }
    }


def annotations() -> list:
    return [
        {"id": "p19", "label": "19点", "floor": "F19", "map_id": "map19", "pose": pose(1)},
        {"id": "p20", "label": "20点", "floor": "F20", "map_id": "map20", "pose": pose(2)},
        {"id": "p21", "label": "21点", "floor": "F21", "map_id": "map21", "pose": pose(3)},
    ]


def maps() -> list:
    return [
        {
            "id": f"map{level}",
            "name": f"F{level}",
            "floor": f"F{level}",
            "factory_apply_path": f"/var/opt/robot/data/maps/map-{level}",
        }
        for level in (19, 20, 21)
    ]


def test_routes_and_multi_hop_path() -> None:
    routes = stair_routes_from_config(floor_config())
    assert_equal(len(routes), 2, "route count")
    assert_equal(all(item["configured"] for item in routes), True, "routes configured")
    assert_equal(find_floor_path(routes, "F19", "F21"), ["F19", "F20", "F21"], "multi-hop path")
    assert_equal(find_floor_path(routes, "F21", "F19"), None, "directed route")


def test_workspace_aggregation() -> None:
    annotation_items = annotations() + [
        {"id": "old20", "label": "旧图点", "floor": "F20", "map_id": "old20", "pose": pose(9)}
    ]
    map_items = maps() + [
        {
            "id": "old20",
            "name": "旧 F20",
            "floor": "F20",
            "factory_apply_path": "/var/opt/robot/data/maps/map-old20",
            "created_at": "2025-01-01",
        }
    ]
    workspace = build_multi_floor_workspace(
        floor_config=floor_config(),
        maps=map_items,
        annotations=annotation_items,
        sessions=[
            {
                "id": "session",
                "floors": ["F19", "F20", "F21"],
                "active_floor": "F20",
                "floor_steps": [
                    {"floor": "F19", "status": "imported", "map_name": "F19_site"},
                    {"floor": "F20", "status": "mapping", "map_name": "F20_site"},
                    {"floor": "F21", "status": "pending", "map_name": "F21_site"},
                ],
            }
        ],
        current_floor="F20",
        selected_map_id="map20",
    )
    assert_equal(workspace["floor_count"], 3, "floor count")
    assert_equal(workspace["configured_route_count"], 2, "configured route count")
    assert_equal(workspace["floors"][1]["current"], True, "current floor")
    assert_equal(workspace["floors"][1]["selected"], True, "selected floor map")
    assert_equal(workspace["floors"][1]["annotation_count"], 1, "annotation count")
    assert_equal(workspace["floors"][1]["historical_annotation_count"], 1, "historical point excluded")
    assert_equal(workspace["floors"][1]["annotations"][0]["id"], "p20", "selected map points only")
    assert_equal(workspace["floors"][1]["mapping_step"]["status"], "mapping", "mapping step")
    assert_equal(workspace["floors"][1]["terrain_segment_count"], 1, "same-floor terrain count")


def test_workspace_excludes_unregistered_floor_data() -> None:
    workspace = build_multi_floor_workspace(
        floor_config=floor_config(),
        maps=maps() + [{"id": "bad", "name": "F1", "floor": "F1"}],
        annotations=annotations() + [{"id": "bad_point", "floor": "F1", "map_id": "bad"}],
        sessions=[{"id": "bad_session", "floors": ["F1"], "active_floor": "F1"}],
        current_floor="F20",
        selected_map_id="map20",
    )
    assert_equal(workspace["floor_count"], 3, "only registered floors exposed")
    assert_equal(workspace["identity_issue_count"], 3, "unregistered records reported")
    assert_equal(workspace["latest_mapping_session"], None, "invalid session excluded")


def test_project_single_floor_does_not_require_cross_floor_route() -> None:
    workspace = build_multi_floor_workspace(
        floor_config={
            "floors": {
                "F7": {
                    "level": 7,
                    "registry_source": "project",
                    "stairs": {},
                    "terrain_segments": {},
                }
            }
        },
        maps=[
            {
                "id": "map7",
                "name": "7层现场图",
                "floor": "F7",
                "factory_apply_path": "/var/opt/robot/data/maps/map-7",
            }
        ],
        annotations=[],
        sessions=[],
        current_floor="F7",
        selected_map_id="map7",
    )
    floor = workspace["floors"][0]
    assert_equal(floor["route_configured"], False, "project floor is single-floor only")
    assert_equal(floor["warnings"], [], "single floor does not require route or switch pose")
    assert_equal(floor["ready"], True, "single floor ready with a factory map")


def test_runtime_map_library_has_no_implicit_routes() -> None:
    workspace = build_multi_floor_workspace(
        floor_config={"mission": {"frame_id": "map"}, "floors": {}},
        maps=[
            {"id": "builtin_F19", "name": "19楼地图", "floor": "F19", "factory_apply_path": "/var/opt/robot/data/maps/f19"},
            {"id": "builtin_F20", "name": "20楼地图", "floor": "F20", "factory_apply_path": "/var/opt/robot/data/maps/f20"},
            {"id": "builtin_F21", "name": "21楼地图", "floor": "F21", "factory_apply_path": "/var/opt/robot/data/maps/f21"},
        ],
        annotations=[],
        sessions=[],
        current_floor="F20",
        selected_map_id="builtin_F20",
    )
    assert_equal(workspace["floor_count"], 3, "ordinary maps remain visible")
    assert_equal(workspace["configured_route_count"], 0, "no implicit stair routes")
    assert_equal(all(not floor["route_configured"] for floor in workspace["floors"]), True, "maps are not route floors")
    assert_equal(workspace["ready"], True, "ordinary map library is ready")


def test_cross_floor_task_order_and_route() -> None:
    known = {item["id"]: item for item in annotations()}
    context = cross_floor_task_context(
        {"name": "巡检", "annotation_ids": ["p19", "p20", "p21"]},
        annotations_by_id=known,
        routes=stair_routes_from_config(floor_config()),
    )
    assert_equal(context["ok"], True, "context ready")
    assert_equal(context["annotation_ids"], ["p19", "p20", "p21"], "point order")
    assert_equal(context["floor_sequence"], ["F19", "F20", "F21"], "floor order")
    assert_equal(context["task_map_id"], "map19", "start map")

    single = cross_floor_task_context(
        {"annotation_ids": ["p19"]},
        annotations_by_id=known,
        routes=stair_routes_from_config(floor_config()),
    )
    assert_equal(single["code"], "cross_floor_single_floor", "single-floor rejection")

    reverse = cross_floor_task_context(
        {"annotation_ids": ["p21", "p19"]},
        annotations_by_id=known,
        routes=stair_routes_from_config(floor_config()),
    )
    assert_equal(reverse["code"], "cross_floor_route_missing", "missing route rejection")

    mixed = dict(known)
    mixed["p20_old"] = {**known["p20"], "id": "p20_old", "map_id": "old_map20"}
    mixed_result = cross_floor_task_context(
        {"annotation_ids": ["p20", "p20_old", "p21"]},
        annotations_by_id=mixed,
        routes=stair_routes_from_config(floor_config()),
    )
    assert_equal(mixed_result["code"], "cross_floor_mixed_maps_on_floor", "same-floor map mixing rejection")


def main() -> int:
    for test in (
        test_routes_and_multi_hop_path,
        test_workspace_aggregation,
        test_workspace_excludes_unregistered_floor_data,
        test_project_single_floor_does_not_require_cross_floor_route,
        test_runtime_map_library_has_no_implicit_routes,
        test_cross_floor_task_order_and_route,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] multi-floor contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
