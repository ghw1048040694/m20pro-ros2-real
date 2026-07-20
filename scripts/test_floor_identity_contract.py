#!/usr/bin/env python3
"""Offline tests for the site floor identity registry."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.floor_identity_contract import (  # noqa: E402
    augment_floor_config,
    configured_floor_ids,
    floor_level_from_id,
    normalize_floor_id,
    project_floor_ids,
    resolve_operational_floor,
    validate_floor_matches_map,
    validate_mapping_session_identity,
    validate_registered_floor,
    validate_runtime_map_floor,
)


CONFIG = {"floors": {"F19": {}, "F20": {}, "F21": {}}}
RUNTIME_CONFIG = {"mission": {"frame_id": "map"}, "floors": {}}


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def test_registry_and_unknown_floor() -> None:
    assert_equal(configured_floor_ids(CONFIG), ["F19", "F20", "F21"], "registered floors")
    assert_equal(validate_registered_floor("F20", CONFIG)["ok"], True, "registered floor")
    assert_equal(validate_registered_floor("F1", CONFIG)["code"], "floor_identity_unknown", "unknown floor")


def test_operator_floor_normalization() -> None:
    assert_equal(normalize_floor_id("7"), "F7", "plain number")
    assert_equal(normalize_floor_id("F07"), "F7", "canonical above-ground floor")
    assert_equal(normalize_floor_id("7层"), "F7", "Chinese floor suffix")
    assert_equal(normalize_floor_id("-2"), "B2", "negative basement")
    assert_equal(normalize_floor_id("负2层"), "B2", "Chinese basement")
    assert_equal(floor_level_from_id("B2"), -2, "basement level")
    assert_equal(normalize_floor_id("七楼"), "", "invalid free text")


def test_project_registry_augmentation_has_no_routes() -> None:
    projects = [{"id": "site", "floors": ["F7", "B1", "F7"]}]
    assert_equal(project_floor_ids(projects), ["F7", "B1"], "deduplicated project floors")
    augmented = augment_floor_config(CONFIG, projects)
    assert_equal(configured_floor_ids(augmented), ["F19", "F20", "F21", "F7", "B1"], "merged registry")
    assert_equal(augmented["floors"]["F7"]["stairs"], {}, "registration does not invent routes")
    assert_equal(augmented["floors"]["B1"]["level"], -1, "registered basement level")


def test_custom_single_floor_becomes_operational_floor() -> None:
    routes = ["F19", "F20", "F21"]
    assert_equal(resolve_operational_floor("F20", {"floor": "F7"}, routes), "F7", "custom map floor")
    assert_equal(resolve_operational_floor("F20", {"floor": "F21"}, routes), "F20", "route floor stays reported")
    assert_equal(resolve_operational_floor("F20", {}, routes), "F20", "no selected map")


def test_mapping_identity() -> None:
    valid = validate_mapping_session_identity(
        {"mode": "multi", "floors": ["F19", "F20"], "active_floor": "F20"},
        CONFIG,
    )
    assert_equal(valid["ok"], True, "valid mapping floors")
    unknown = validate_mapping_session_identity(
        {"mode": "single", "floors": ["F1"], "active_floor": "F1"},
        CONFIG,
    )
    assert_equal(unknown["code"], "mapping_floor_unknown", "mapping unknown floor")
    registered = validate_mapping_session_identity(
        {"mode": "single", "floors": ["7层"], "active_floor": "7"},
        CONFIG,
        allow_floor_registration=True,
    )
    assert_equal(registered["floors"], ["F7"], "new actual floor normalized")
    assert_equal(registered["active_floor"], "F7", "active floor normalized")
    duplicate = validate_mapping_session_identity(
        {"mode": "multi", "floors": ["7", "F07"], "active_floor": "7"},
        CONFIG,
        allow_floor_registration=True,
    )
    assert_equal(duplicate["code"], "mapping_floor_duplicate", "canonical duplicate rejected")
    wrong_count = validate_mapping_session_identity(
        {"mode": "single", "floors": ["F19", "F20"], "active_floor": "F19"},
        CONFIG,
    )
    assert_equal(wrong_count["code"], "single_mapping_floor_count", "single floor count")
    invalid_mode = validate_mapping_session_identity(
        {"mode": "single_floor", "floors": ["F19"], "active_floor": "F19"},
        CONFIG,
    )
    assert_equal(invalid_mode["code"], "mapping_mode_invalid", "invalid mode rejected")


def test_map_binding() -> None:
    record = {"id": "map20", "floor": "F20"}
    assert_equal(validate_floor_matches_map("F20", record, CONFIG, subject="点位楼层")["ok"], True, "map match")
    mismatch = validate_floor_matches_map("F19", record, CONFIG, subject="点位楼层")
    assert_equal(mismatch["code"], "floor_map_identity_mismatch", "map mismatch")


def test_runtime_maps_are_not_a_floor_registry() -> None:
    assert_equal(validate_registered_floor("F19", RUNTIME_CONFIG)["ok"], True, "ordinary F19 map label")
    assert_equal(validate_registered_floor("F20", RUNTIME_CONFIG)["ok"], True, "ordinary F20 map label")
    assert_equal(validate_registered_floor("F21", RUNTIME_CONFIG)["ok"], True, "ordinary F21 map label")
    assert_equal(validate_registered_floor("??", RUNTIME_CONFIG)["ok"], False, "invalid map label")
    mapping = validate_mapping_session_identity(
        {"mode": "single", "floors": ["7"], "active_floor": "7"},
        RUNTIME_CONFIG,
    )
    assert_equal(mapping["ok"], True, "runtime map can define an actual floor label")


def test_project_registration_does_not_block_ordinary_map_selection() -> None:
    project_only = augment_floor_config(
        RUNTIME_CONFIG,
        [{"id": "demo", "floors": ["F1"]}],
    )
    ordinary = validate_runtime_map_floor("F20", project_only, subject="地图楼层")
    assert_equal(ordinary["ok"], True, "ordinary map is valid outside project floors")
    assert_equal(ordinary["registry_mode"], "runtime_map", "ordinary map uses runtime identity")
    record = {"id": "builtin_F20", "floor": "F20"}
    bound = validate_floor_matches_map(
        "F20",
        record,
        project_only,
        subject="点位楼层",
        allow_unregistered_map=True,
    )
    assert_equal(bound["ok"], True, "ordinary map point binding is valid")
    strict = validate_registered_floor("F20", project_only, subject="地图楼层")
    assert_equal(strict["code"], "floor_identity_unknown", "route/task registry remains strict")


def main() -> int:
    for test in (
        test_registry_and_unknown_floor,
        test_operator_floor_normalization,
        test_project_registry_augmentation_has_no_routes,
        test_custom_single_floor_becomes_operational_floor,
        test_mapping_identity,
        test_map_binding,
        test_runtime_maps_are_not_a_floor_registry,
        test_project_registration_does_not_block_ordinary_map_selection,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] floor identity contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
