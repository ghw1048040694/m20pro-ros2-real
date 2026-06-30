#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.map_selection_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.map_selection_contract import (  # noqa: E402
    apply_selected_map_choice_state,
    map_relocalization_required_payload,
    selected_map_status_payload,
    selected_map_wait_timeout_payload,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def now_text() -> str:
    return "2026-06-27 01:52:00"


def map_payload(**extra) -> dict:
    payload = {
        "available": True,
        "map_id": "map_a",
        "name": "F20_TEST",
        "floor": "F20",
        "width": 100,
        "height": 80,
        "resolution": 0.1,
        "origin": {"x": -5.0, "y": -4.0, "z": 0.0, "yaw": 0.0},
    }
    payload.update(extra)
    return payload


def status(selected_map_id="map_a", live_map=None, selected_map=None) -> dict:
    return selected_map_status_payload(
        selected_map_id=selected_map_id,
        live_map=live_map if live_map is not None else map_payload(),
        selected_map=selected_map if selected_map is not None else map_payload(),
        now_text=now_text,
    )


def test_selected_map_missing() -> None:
    payload = status(selected_map_id="")
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "selected_map_missing", "missing code")
    assert_equal(payload["selected_map_id"], None, "missing id")
    assert_equal(payload["updated_at"], now_text(), "timestamp")


def test_selected_map_ready() -> None:
    payload = status()
    assert_equal(payload["ready"], True, "ready")
    assert_equal(payload["code"], "ready", "ready code")
    assert_equal(payload["selected_map_id"], "map_a", "selected id")
    assert_equal(payload["selected_map"]["name"], "F20_TEST", "selected name")
    assert_equal(payload["live_map"]["width"], 100, "live width")


def test_selected_map_metadata_mismatch() -> None:
    payload = status(live_map=map_payload(width=101))
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "selected_map_metadata_mismatch", "mismatch code")
    assert_true(payload["detail"]["checks"]["width"] is False, "width mismatch")
    assert_true("Nav2 当前加载地图不一致" in payload["message"], "message")


def test_live_map_unavailable() -> None:
    payload = status(live_map={"available": False, "message": "no map"})
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "selected_map_metadata_mismatch", "mismatch code")
    assert_equal(payload["detail"]["code"], "live_map_unavailable", "live map code")
    assert_equal(payload["live_map"]["available"], False, "live unavailable")


def test_map_relocalization_required_payload() -> None:
    manual = map_relocalization_required_payload(
        map_id="map_a",
        map_name="F20_TEST",
        yaml_path="/tmp/map.yaml",
        reason="manual_select",
        now_text=now_text,
    )
    assert_equal(manual["map_id"], "map_a", "map id")
    assert_equal(manual["map_name"], "F20_TEST", "map name")
    assert_equal(manual["yaml_path"], "/tmp/map.yaml", "yaml")
    assert_equal(manual["loaded_at"], now_text(), "timestamp")
    assert_equal(manual["reason"], "manual_select", "manual reason")
    assert_true("当前固定地图已选择并同步到 Nav2" in manual["message"], "manual message")

    startup = map_relocalization_required_payload(
        map_id="map_a",
        map_name="F20_TEST",
        yaml_path="/tmp/map.yaml",
        reason="startup_sync",
        now_text=now_text,
    )
    assert_equal(startup["reason"], "startup_sync", "startup reason")
    assert_true("启动后已把当前固定地图同步到 Nav2" in startup["message"], "startup message")


def test_selected_map_wait_timeout_payload() -> None:
    payload = selected_map_wait_timeout_payload(
        selected_map_id="map_a",
        now_text=now_text,
    )
    assert_equal(payload["ready"], False, "timeout not ready")
    assert_equal(payload["code"], "selected_map_metadata_mismatch", "timeout code")
    assert_equal(payload["message"], "等待 Nav2 /map 更新超时", "timeout message")
    assert_equal(payload["selected_map_id"], "map_a", "timeout selected map")
    assert_equal(payload["updated_at"], now_text(), "timeout timestamp")


def test_apply_selected_map_choice_state() -> None:
    selected = apply_selected_map_choice_state(
        {"selected_map_id": "old"},
        map_id="map_a",
        previous_map_id="old",
        record={"name": "F20_TEST"},
        nav2_load={"ok": True, "loaded": True, "yaml_path": "/tmp/map.yaml"},
        now_text=now_text,
    )
    assert_equal(selected["settings"]["selected_map_id"], "map_a", "selected map stored")
    assert_true(selected["selection_changed"], "selection changed")
    assert_true(selected["clear_pose"], "loaded map clears pose")
    assert_equal(selected["relocalization_required"]["map_id"], "map_a", "relocalization map id")
    assert_equal(selected["relocalization_required"]["map_name"], "F20_TEST", "relocalization map name")
    assert_equal(selected["relocalization_required"]["reason"], "manual_select", "manual select reason")

    unchanged = apply_selected_map_choice_state(
        {"selected_map_id": "map_a", "map_relocalization_required": {"old": True}},
        map_id="map_a",
        previous_map_id="map_a",
        record={"name": "F20_TEST"},
        nav2_load={"ok": True, "loaded": False},
        now_text=now_text,
    )
    assert_true(not unchanged["selection_changed"], "same map unchanged")
    assert_true(not unchanged["clear_pose"], "same unloaded map keeps pose")
    assert_equal(unchanged["relocalization_required"], {"old": True}, "old relocalization requirement retained")

    live = apply_selected_map_choice_state(
        {"selected_map_id": "map_a", "map_relocalization_required": {"old": True}},
        map_id=None,
        previous_map_id="map_a",
        record=None,
        nav2_load={"ok": True, "skipped": True},
        now_text=now_text,
    )
    assert_equal(live["settings"]["selected_map_id"], None, "live map selection stored")
    assert_true("map_relocalization_required" not in live["settings"], "live map clears relocalization requirement")
    assert_true(not live["clear_pose"], "live map does not clear pose through fixed-map gate")


def main() -> int:
    for test in (
        test_selected_map_missing,
        test_selected_map_ready,
        test_selected_map_metadata_mismatch,
        test_live_map_unavailable,
        test_map_relocalization_required_payload,
        test_selected_map_wait_timeout_payload,
        test_apply_selected_map_choice_state,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] map selection contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
