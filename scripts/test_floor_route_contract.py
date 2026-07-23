#!/usr/bin/env python3
"""Offline tests for the persisted cross-floor route contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.floor_route_contract import (  # noqa: E402
    floor_route_public_payload,
    resolve_floor_switch_request,
    runtime_floor_config,
    upsert_floor_route,
    validate_floor_route,
    validate_floor_route_set,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def pose(x: float, y: float, yaw: float = 0.0) -> dict:
    return {"x": x, "y": y, "z": 0.0, "yaw": yaw}


def annotations() -> dict:
    return {
        "entry": {"id": "entry", "label": "1层楼梯入口", "type": "stair_entry", "floor": "F1", "map_id": "map_f1", "pose": pose(1.0, 2.0)},
        "source_switch": {"id": "source_switch", "label": "1层平台", "type": "stair_switch", "floor": "F1", "map_id": "map_f1", "pose": pose(3.0, 4.0, 0.2)},
        "target_switch": {"id": "target_switch", "label": "2层平台", "type": "stair_switch", "floor": "F2", "map_id": "map_f2", "pose": pose(5.0, 6.0, 0.3)},
        "exit": {"id": "exit", "label": "2层楼梯出口", "type": "stair_exit", "floor": "F2", "map_id": "map_f2", "pose": pose(7.0, 8.0, 0.4)},
    }


def maps() -> dict:
    return {
        "map_f1": {"id": "map_f1", "name": "现场1层", "floor": "F1", "yaml_path": "/maps/f1.yaml", "factory_apply_path": "/var/opt/robot/data/maps/site-f1-001"},
        "map_f2": {"id": "map_f2", "name": "现场2层", "floor": "F2", "yaml_path": "/maps/f2.yaml", "factory_apply_path": "/var/opt/robot/data/maps/site-f2-001"},
    }


def route_payload(**extra) -> dict:
    payload = {
        "name": "东侧楼梯上行",
        "entry_annotation_id": "entry",
        "source_platform_annotation_id": "source_switch",
        "target_platform_annotation_id": "target_switch",
        "post_exit_annotation_id": "exit",
    }
    payload.update(extra)
    return payload


def valid_route() -> dict:
    result = validate_floor_route(
        route_payload(),
        annotations_by_id=annotations(),
        maps_by_id=maps(),
        resolve_map_yaml=lambda record: record.get("yaml_path", ""),
        route_id="route_up",
        now_text="2026-07-19 16:00:00",
    )
    assert_equal(result["ok"], True, "valid route")
    return result["route"]


def terrain_status(route_id: str = "route_up", stamp: float = 100.0, **extra) -> dict:
    payload = {
        "route_id": route_id,
        "profile_id": f"{route_id}:terrain",
        "corridor_version": "shadow-v1",
        "state": "traversable",
        "reason": "step_profile_continuous",
        "stamp_unix_s": stamp,
        "cloud_age_s": 0.1,
    }
    payload.update(extra)
    return payload


def test_validate_route_and_runtime_config() -> None:
    route = valid_route()
    assert_equal(route["source_floor"], "F1", "source floor")
    assert_equal(route["target_floor"], "F2", "target floor")
    assert_equal(route["direction"], "up", "direction inferred")
    assert_equal(route["source_platform"], pose(3.0, 4.0, 0.2), "source platform pose")
    assert_equal(route["terrain_guard"]["profile_id"], "route_up:terrain", "terrain profile identity")
    assert_equal(route["terrain_guard"]["corridor_version"], "shadow-v1", "shadow corridor version")
    assert_equal(route["terrain_guard"]["certified_motion"], False, "route API cannot certify motion")
    config = runtime_floor_config([route])
    stair = config["floors"]["F1"]["stairs"]["route_up"]
    assert_equal(stair["target_floor"], "F2", "directed target")
    assert_equal(stair["target_map_id"], "map_f2", "target map")
    assert_equal(stair["post_exit"], pose(7.0, 8.0, 0.4), "post-exit pose")
    assert_equal(stair["terrain_guard"], route["terrain_guard"], "runtime keeps one terrain identity")
    assert_equal(config["floors"]["F2"]["stairs"], {}, "reverse route is not invented")


def test_route_validation_rejects_bad_assets() -> None:
    bad_annotations = annotations()
    bad_annotations["entry"] = {**bad_annotations["entry"], "type": "patrol"}
    bad_type = validate_floor_route(
        route_payload(), annotations_by_id=bad_annotations, maps_by_id=maps(),
        resolve_map_yaml=lambda record: record.get("yaml_path", ""), route_id="bad", now_text="now",
    )
    assert_equal(bad_type["code"], "floor_route_point_type_mismatch", "semantic type enforced")
    bad_maps = maps()
    bad_maps["map_f2"] = {**bad_maps["map_f2"], "factory_apply_path": ""}
    missing_factory = validate_floor_route(
        route_payload(), annotations_by_id=annotations(), maps_by_id=bad_maps,
        resolve_map_yaml=lambda record: record.get("yaml_path", ""), route_id="bad", now_text="now",
    )
    assert_equal(missing_factory["code"], "floor_route_target_factory_map_missing", "106 package required")


def test_route_set_and_upsert() -> None:
    route = valid_route()
    replacement = {**route, "id": "route_up_v2", "name": "新路线"}
    routes = upsert_floor_route([route], replacement)
    assert_equal(len(routes), 1, "one directed route per floor pair")
    assert_equal(routes[0]["id"], "route_up_v2", "new route replaces old pair")
    reverse = {**route, "id": "route_down", "source_floor": "F2", "target_floor": "F1", "source_map_id": "map_f2", "target_map_id": "map_f1"}
    assert_equal(validate_floor_route_set([route, reverse])["ok"], True, "two directions share floor maps")
    conflict = validate_floor_route_set([route, {**reverse, "source_map_id": "map_f2_other"}])
    assert_equal(conflict["code"], "floor_route_map_conflict", "floor map conflict rejected")


def test_route_edit_cannot_enable_certified_motion() -> None:
    route = valid_route()
    edited = validate_floor_route(
        route_payload(
            terrain_guard={
                "profile_id": "field-calibrated",
                "corridor_version": "v3",
                "motion_policy": "certified_connector",
                "certified_motion": True,
            }
        ),
        annotations_by_id=annotations(),
        maps_by_id=maps(),
        resolve_map_yaml=lambda record: record.get("yaml_path", ""),
        route_id="route_edit",
        now_text="now",
    )
    assert_equal(edited["ok"], True, "edited route remains valid")
    assert_equal(edited["route"]["terrain_guard"]["profile_id"], "field-calibrated", "profile id retained")
    assert_equal(edited["route"]["terrain_guard"]["corridor_version"], "v3", "corridor version retained")
    assert_equal(edited["route"]["terrain_guard"]["motion_policy"], "stop_only", "motion policy forced stop-only")
    assert_equal(edited["route"]["terrain_guard"]["certified_motion"], False, "certification forced false")


def test_floor_switch_request_contract() -> None:
    route = valid_route()
    request = {"request_id": "switch_1", "route_id": "route_up", "source_floor": "F1", "target_floor": "F2", "target_map_id": "map_f2"}
    active = {"task_id": "task_1", "status": "running", "multi_floor": True, "last_floor_goal_source_floor": "F1", "last_floor_goal_target_floor": "F2"}
    accepted = resolve_floor_switch_request(
        request,
        routes=[route],
        active_task=active,
        selected_map_id="map_f1",
        terrain_guard_status=terrain_status(),
        now_unix_s=100.2,
    )
    assert_equal(accepted["ok"], True, "matching request accepted")
    assert_equal(accepted["task_id"], "task_1", "transaction binds active task")
    wrong_map = resolve_floor_switch_request(
        request,
        routes=[route],
        active_task=active,
        selected_map_id="map_f2",
        terrain_guard_status=terrain_status(),
        now_unix_s=100.2,
    )
    assert_equal(wrong_map["code"], "floor_switch_source_map_mismatch", "source map enforced")
    wrong_route = resolve_floor_switch_request({**request, "target_floor": "F3"}, routes=[route], active_task=active, selected_map_id="map_f1")
    assert_equal(wrong_route["code"], "floor_switch_request_route_mismatch", "request cannot override route")
    ordinary = resolve_floor_switch_request(request, routes=[route], active_task={**active, "multi_floor": False}, selected_map_id="map_f1")
    assert_equal(ordinary["code"], "floor_switch_no_active_task", "ordinary task cannot switch")

    missing_status = resolve_floor_switch_request(
        request,
        routes=[route],
        active_task=active,
        selected_map_id="map_f1",
        now_unix_s=100.2,
    )
    assert_equal(missing_status["code"], "terrain_guard_status_missing", "terrain evidence required")
    blocked_status = resolve_floor_switch_request(
        request,
        routes=[route],
        active_task=active,
        selected_map_id="map_f1",
        terrain_guard_status=terrain_status(state="blocked"),
        now_unix_s=100.2,
    )
    assert_equal(blocked_status["code"], "terrain_guard_not_traversable", "blocked terrain rejected")
    stale_status = resolve_floor_switch_request(
        request,
        routes=[route],
        active_task=active,
        selected_map_id="map_f1",
        terrain_guard_status=terrain_status(stamp=95.0),
        now_unix_s=100.2,
    )
    assert_equal(stale_status["code"], "terrain_guard_status_stale", "stale terrain rejected")


def test_public_payload_only_exposes_semantic_candidates() -> None:
    items = list(annotations().values()) + [{"id": "patrol", "type": "patrol", "floor": "F1", "map_id": "map_f1", "pose": pose(0, 0)}]
    payload = floor_route_public_payload([valid_route()], annotations=items, maps=maps().values())
    assert_equal(len(payload["candidates"]), 4, "only stair semantic points")
    assert_equal(all(item["factory_ready"] for item in payload["maps"]), True, "factory readiness")


def main() -> int:
    for test in (
        test_validate_route_and_runtime_config,
        test_route_validation_rejects_bad_assets,
        test_route_set_and_upsert,
        test_route_edit_cannot_enable_certified_motion,
        test_floor_switch_request_contract,
        test_public_payload_only_exposes_semantic_candidates,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] floor route contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
