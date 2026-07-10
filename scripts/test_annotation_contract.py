#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.annotation_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.annotation_contract import (  # noqa: E402
    annotation_list_filter_payload,
    annotation_create_static_context,
    annotation_create_readiness_payload,
    annotation_dwell_s,
    annotation_map_pose_error_payload,
    annotation_semantics_payload,
    build_annotation_record,
    manual_point_type_from_payload,
    normalize_annotation_semantics,
    resolve_annotation_dwell_s,
    string_list,
    vendor_navigation_from_payload,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def now_text() -> str:
    return "2026-06-27 02:32:00"


def ready_payload(**overrides) -> dict:
    payload = {
        "map_id": "map_a",
        "selected_map_id": "map_a",
        "selected_map_status": {"ready": True, "code": "ready", "message": "地图一致"},
        "map_relocalization_required": None,
        "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.1},
        "localization_ok": True,
        "pose_age_sec": 0.5,
        "pose_timeout_s": 3.0,
        "now_text": now_text,
    }
    payload.update(overrides)
    return annotation_create_readiness_payload(**payload)


def test_requires_fixed_map() -> None:
    payload = ready_payload(map_id="live_map")
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "annotation_fixed_map_required", "code")


def test_blocks_map_mismatch() -> None:
    payload = ready_payload(map_id="map_a", selected_map_id="map_b")
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "annotation_map_mismatch", "code")


def test_blocks_selected_map_metadata_mismatch() -> None:
    payload = ready_payload(
        selected_map_status={
            "ready": False,
            "code": "selected_map_metadata_mismatch",
            "message": "网页选择地图与 Nav2 当前加载地图不一致",
        }
    )
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "annotation_map_metadata_mismatch", "code")
    assert_true("Nav2 当前加载地图不一致" in payload["message"], "message")


def test_manual_map_click_does_not_require_live_pose() -> None:
    payload = ready_payload(
        selected_map_status={
            "ready": False,
            "code": "selected_map_metadata_mismatch",
            "message": "网页选择地图与 Nav2 当前加载地图不一致",
        },
        map_relocalization_required={"map_id": "map_a", "reason": "manual_select"},
        localization_ok=False,
        pose={},
        pose_age_sec=None,
        require_live_pose=False,
    )
    assert_equal(payload["ready"], True, "manual map click ready")
    assert_equal(payload["code"], "ready", "manual map click code")
    assert_equal(payload["require_live_pose"], False, "manual map click live pose flag")


def test_blocks_required_relocalization() -> None:
    payload = ready_payload(
        map_relocalization_required={
            "map_id": "map_a",
            "reason": "startup_sync",
        }
    )
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "annotation_map_relocalization_required", "code")
    assert_equal(payload["map_relocalization_required"]["reason"], "startup_sync", "reason")


def test_blocks_unconfirmed_or_stale_pose() -> None:
    unconfirmed = ready_payload(localization_ok=False)
    assert_equal(unconfirmed["code"], "annotation_localization_not_confirmed", "unconfirmed code")

    stale = ready_payload(pose_age_sec=9.0)
    assert_equal(stale["code"], "annotation_pose_invalid_or_stale", "stale code")


def test_ready() -> None:
    payload = ready_payload()
    assert_equal(payload["ready"], True, "ready")
    assert_equal(payload["code"], "ready", "code")
    assert_equal(payload["updated_at"], now_text(), "timestamp")


def test_annotation_create_static_context() -> None:
    context = annotation_create_static_context(
        {"pose": {"x": "1.2", "y": 3, "yaw": "0.5"}, "floor": "F20", "type": "patrol"},
        default_label_index=4,
    )
    assert_true(context["ok"], "valid context")
    assert_equal(context["pose"], {"x": 1.2, "y": 3.0, "z": 0.0, "yaw": 0.5}, "pose normalized")
    assert_equal(context["floor"], "F20", "floor")
    assert_equal(context["type"], "patrol", "type")
    assert_equal(context["label"], "F20_patrol_4", "default label")
    assert_equal(context["map_id"], None, "missing map id")

    labeled = annotation_create_static_context(
        {"pose": {"x": 1, "y": 2}, "floor": "F20", "label": "  工位点  ", "map_id": "live_map"},
        default_label_index=1,
    )
    assert_true(labeled["ok"], "labeled context")
    assert_equal(labeled["label"], "工位点", "label trimmed")
    assert_equal(labeled["map_id"], "live_map", "live map preserved")

    bad_pose = annotation_create_static_context(
        {"pose": {"x": "bad", "y": 2}, "floor": "F20"},
        default_label_index=1,
    )
    assert_equal(bad_pose["ok"], False, "bad pose rejected")
    assert_equal(bad_pose["code"], "annotation_pose_invalid", "bad pose code")

    missing_floor = annotation_create_static_context(
        {"pose": {"x": 1, "y": 2}, "floor": " "},
        default_label_index=1,
    )
    assert_equal(missing_floor["ok"], False, "missing floor rejected")
    assert_equal(missing_floor["code"], "annotation_floor_missing", "missing floor code")


def test_build_annotation_record() -> None:
    context = annotation_create_static_context(
        {
            "pose": {"x": 1.2, "y": 3.4, "z": 0.1, "yaw": 0.5},
            "floor": "F20",
            "type": "charging",
            "label": "充电点",
        },
        default_label_index=1,
    )
    record = build_annotation_record(
        {
            "area": "A区",
            "place": "工位",
            "scan_point": "P01",
            "result_file_prefix": "B03_U01_H2008_F20_工位_P01",
            "radar": {
                "enabled": True,
                "scans": [{"mode": "measuring", "label": "实测实量", "result_suffix": "measure"}],
            },
            "target_classes": "helmet, vest",
            "notes": "检查安全帽",
            "speed": "2",
        },
        context,
        annotation_id="point_1",
        map_id="map_a",
        dwell_s=6.0,
        now_text_value=now_text(),
    )
    assert_equal(record["id"], "point_1", "record id")
    assert_equal(record["map_id"], "map_a", "record map")
    assert_equal(record["floor"], "F20", "record floor")
    assert_equal(record["label"], "充电点", "record label")
    assert_equal(record["area"], "A区", "record area")
    assert_equal(record["room"], "工位", "record room")
    assert_equal(record["scan_point"], "P01", "record scan point")
    assert_equal(record["result_file_prefix"], "B03_U01_H2008_F20_工位_P01", "record result prefix")
    assert_equal(record["radar"]["scans"][0]["mode"], "measuring", "record radar plan")
    assert_equal(record["pose"], {"x": 1.2, "y": 3.4, "z": 0.1, "yaw": 0.5}, "record pose")
    assert_equal(record["manual_point_type"], "charge", "record manual point type")
    assert_equal(record["vendor_navigation"]["PointInfo"], 3, "record point info")
    assert_equal(record["vendor_navigation"]["Speed"], 2, "record speed")
    assert_equal(record["dwell_s"], 6.0, "record dwell")
    assert_equal(record["inspect_duration_s"], 6.0, "record inspect duration")
    assert_equal(record["target_classes"], ["helmet", "vest"], "record target classes")
    assert_equal(record["notes"], "检查安全帽", "record notes")
    assert_equal(record["created_at"], now_text(), "record created time")


def test_annotation_map_pose_error_payload() -> None:
    map_payload = {
        "available": True,
        "width": 3,
        "height": 2,
        "resolution": 1.0,
        "origin": {"x": 0.0, "y": 0.0},
        "data": [0, 100, -1, 0, 0, 0],
    }
    assert_equal(
        annotation_map_pose_error_payload({"x": 0.2, "y": 0.2}, map_payload),
        None,
        "free point accepted",
    )

    outside = annotation_map_pose_error_payload({"x": 5.0, "y": 0.2}, map_payload)
    assert_true(outside is not None, "outside rejected")
    assert_equal(outside["code"], "annotation_out_of_map", "outside code")
    assert_true("不在当前地图范围内" in outside["message"], "outside message")

    occupied = annotation_map_pose_error_payload({"x": 1.2, "y": 0.2}, map_payload)
    assert_true(occupied is not None, "occupied rejected")
    assert_equal(occupied["code"], "annotation_on_occupied_cell", "occupied code")
    assert_true("障碍物栅格" in occupied["message"], "occupied message")

    unknown = annotation_map_pose_error_payload({"x": 2.2, "y": 0.2}, map_payload)
    assert_true(unknown is not None, "unknown rejected")
    assert_equal(unknown["code"], "annotation_on_unknown_cell", "unknown code")
    assert_true("未知栅格" in unknown["message"], "unknown message")


def test_annotation_semantics_normalization() -> None:
    item = {
        "id": "p1",
        "type": "stair_entry",
        "floor": "F20",
        "region": "A区",
        "place": "门口",
        "name": "过渡",
        "pose": {"x": 1.2, "y": 3.4, "z": 0.0, "yaw": 1.57},
        "inspect_duration_s": "2.5",
        "vendor_navigation": {"Gait": "13", "bad": "ignored"},
        "target_classes": "helmet, vest",
        "station": "S02",
        "radar": "bad",
    }
    normalized = normalize_annotation_semantics(item)
    assert_equal(normalized["manual_point_type"], "transition", "manual point type")
    assert_equal(normalized["vendor_navigation"]["PointInfo"], 0, "transition point info")
    assert_equal(normalized["vendor_navigation"]["NavMode"], 0, "transition nav mode")
    assert_equal(normalized["vendor_navigation"]["Gait"], 13, "vendor gait")
    assert_equal(normalized["dwell_s"], 2.5, "dwell from legacy duration")
    assert_equal(normalized["area"], "A区", "area alias")
    assert_equal(normalized["room"], "门口", "room alias")
    assert_equal(normalized["scan_point"], "S02", "scan point alias")
    assert_equal(normalized["radar"], {}, "bad radar normalized")
    assert_equal(normalized["target_classes"], ["helmet", "vest"], "target classes")
    assert_true(normalized["result_file_prefix"].startswith("F20_A区_门口_S02_过渡"), "result prefix")


def test_payload_helpers() -> None:
    assert_equal(manual_point_type_from_payload({"type": "charging"}), "charge", "charging alias")
    assert_equal(
        resolve_annotation_dwell_s(
            {"type": "transition"},
            default_task_dwell_s=5.0,
            default_transition_dwell_s=1.5,
            default_charge_dwell_s=0.0,
        ),
        1.5,
        "transition default dwell",
    )
    assert_equal(
        resolve_annotation_dwell_s(
            {"manual_point_type": "task", "dwell_s": "7"},
            default_task_dwell_s=5.0,
            default_transition_dwell_s=0.0,
            default_charge_dwell_s=0.0,
        ),
        7.0,
        "explicit dwell",
    )
    vendor = vendor_navigation_from_payload({"manual_point_type": "charge", "speed": "2", "nav_mode": "0"})
    assert_equal(vendor["PointInfo"], 3, "charge point info")
    assert_equal(vendor["Speed"], 2, "speed alias")
    assert_equal(vendor["NavMode"], 0, "nav mode alias")
    assert_equal(string_list([" a ", "", 3]), ["a", "3"], "list strings")


def test_annotation_semantics_payload() -> None:
    annotation = {
        "id": "p2",
        "label": "任务点",
        "type": "patrol",
        "floor": "F20",
        "pose": {"x": 1.0, "y": 2.0, "z": 0.1, "yaw": 0.2},
        "dwell_s": "4",
        "room": "客厅",
        "scan_point": "P03",
        "radar": {"enabled": True, "scans": [{"mode": "modeling"}]},
    }
    payload = annotation_semantics_payload(annotation)
    assert_equal(payload["manual_point_type"], "task", "task type")
    assert_equal(payload["manual_point_type_label"], "任务点", "type label")
    assert_true("type" not in payload, "legacy type is omitted from semantic payload")
    assert_equal(payload["dwell_s"], 4.0, "payload dwell")
    assert_equal(payload["vendor_navigation"]["PosX"], 1.0, "vendor pos x")
    assert_equal(payload["vendor_navigation"]["AngleYaw"], 0.2, "vendor yaw")
    assert_equal(payload["room"], "客厅", "payload room")
    assert_equal(payload["scan_point"], "P03", "payload scan point")
    assert_equal(payload["radar"]["scans"][0]["mode"], "modeling", "payload radar")
    assert_equal(annotation_dwell_s(annotation), 4.0, "annotation dwell")


def test_annotation_list_filter_payload() -> None:
    annotations = [
        {"id": "p1", "map_id": "map_a", "label": "A"},
        {"id": "p2", "map_id": "map_b", "label": "B"},
        {"id": "p3", "map_id": "map_a", "label": "C"},
        "bad",
    ]
    all_items = annotation_list_filter_payload(annotations)
    assert_equal(all_items["ok"], True, "all ok")
    assert_equal([item["id"] for item in all_items["annotations"]], ["p1", "p2", "p3"], "all ids")
    assert_equal(all_items["hidden_annotation_count"], 0, "all hidden count")
    assert_equal(all_items["total_annotation_count"], 3, "all total count")

    filtered = annotation_list_filter_payload(annotations, map_id="map_a")
    assert_equal([item["id"] for item in filtered["annotations"]], ["p1", "p3"], "filtered ids")
    assert_equal(filtered["hidden_annotation_count"], 1, "filtered hidden count")
    assert_equal(filtered["total_annotation_count"], 3, "filtered total count")


def main() -> int:
    for test in (
        test_requires_fixed_map,
        test_blocks_map_mismatch,
        test_blocks_selected_map_metadata_mismatch,
        test_manual_map_click_does_not_require_live_pose,
        test_blocks_required_relocalization,
        test_blocks_unconfirmed_or_stale_pose,
        test_ready,
        test_annotation_create_static_context,
        test_build_annotation_record,
        test_annotation_map_pose_error_payload,
        test_annotation_semantics_normalization,
        test_payload_helpers,
        test_annotation_semantics_payload,
        test_annotation_list_filter_payload,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] annotation contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
