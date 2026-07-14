#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.mapping_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.mapping_contract import (  # noqa: E402
    apply_mapping_command_result,
    floor_map_name,
    mark_mapping_floor_imported,
    mapping_command_context,
    mapping_command_status,
    normalize_mapping_session_request,
    prepare_mapping_session_create,
    sanitize_mapping_name,
    select_mapping_floor,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


class IdFactory:
    def __init__(self) -> None:
        self.counts = {}

    def __call__(self, prefix: str) -> str:
        count = self.counts.get(prefix, 0) + 1
        self.counts[prefix] = count
        return f"{prefix}_{count}"


def test_sanitize_mapping_name() -> None:
    assert_equal(sanitize_mapping_name(" F20 / desk map ", "fallback"), "F20_desk_map", "sanitize")
    assert_equal(sanitize_mapping_name(" .. ", "fallback"), "fallback", "fallback")


def test_normalize_mapping_session_request() -> None:
    request = normalize_mapping_session_request(
        {
            "name": "  ",
            "building": "B1",
            "mode": "",
            "floors": " F20, F21 ,, ",
            "map_name": " desk/test ",
        },
        default_project_name="M20Pro 工地巡检",
        default_map_name="{active_floor}_20260629_101010",
    )
    assert_equal(request["project_name"], "M20Pro 工地巡检", "default project")
    assert_equal(request["building"], "B1", "building")
    assert_equal(request["mode"], "multi", "mode")
    assert_equal(request["floors"], ["F20", "F21"], "floors")
    assert_equal(request["active_floor"], "F20", "active floor")
    assert_equal(request["map_name"], "desk_test", "map name")


def test_prepare_mapping_session_create_reuses_project() -> None:
    id_factory = IdFactory()
    project = {"id": "project_existing", "name": "Project", "building": "B1"}
    prepared = prepare_mapping_session_create(
        {"project_name": "Project", "building": "B1", "floors": ["F20"]},
        projects=[project],
        id_factory=id_factory,
        now_text=lambda: "2026-06-29 10:20:00",
        default_project_name="M20Pro 工地巡检",
        default_map_name="{active_floor}_stamp",
    )
    assert_equal(prepared["created_project"], None, "no new project")
    assert_equal(prepared["updated_project"]["floors"], ["F20"], "existing project registers floor")
    assert_equal(prepared["session"]["project_id"], "project_existing", "project id")
    assert_equal(prepared["session"]["id"], "map_session_1", "session id")
    assert_equal(prepared["session"]["map_name"], "F20_stamp", "map name")
    assert_equal(prepared["session"]["status"], "created", "status")
    assert_equal(
        prepared["session"]["floor_steps"],
        [{"floor": "F20", "map_name": "F20_stamp", "status": "ready", "updated_at": "2026-06-29 10:20:00"}],
        "single-floor step",
    )


def test_multi_floor_steps_and_selection() -> None:
    prepared = prepare_mapping_session_create(
        {
            "project_name": "Project",
            "building": "B1",
            "floors": ["F19", "F20", "F21"],
            "active_floor": "F20",
            "map_name": "F20_site",
        },
        projects=[{"id": "project_existing", "name": "Project", "building": "B1"}],
        id_factory=IdFactory(),
        now_text=lambda: "created",
        default_project_name="default",
        default_map_name="{active_floor}_stamp",
    )
    session = prepared["session"]
    assert_equal(
        [item["map_name"] for item in session["floor_steps"]],
        ["F19_site", "F20_site", "F21_site"],
        "floor-specific names",
    )
    assert_equal(floor_map_name("site", "F20", "F21", True), "site_F21", "generic base name")

    selected = select_mapping_floor(session, "F21", updated_at="selected")
    assert_equal(selected["ok"], True, "select floor")
    assert_equal(selected["session"]["active_floor"], "F21", "active floor after selection")
    assert_equal(selected["session"]["map_name"], "F21_site", "active map name after selection")
    assert_equal(selected["session"]["status"], "pending", "selected step status")
    assert_equal(select_mapping_floor(session, "F22", updated_at="now")["code"], "mapping_floor_missing", "missing floor")

    busy = dict(session)
    busy["status"] = "mapping"
    assert_equal(select_mapping_floor(busy, "F21", updated_at="now")["code"], "mapping_floor_busy", "busy floor")

    legacy = {
        "id": "legacy",
        "floors": ["F20", "F21"],
        "active_floor": "F20",
        "map_name": "F20_legacy",
        "status": "saved",
        "updated_at": "old",
    }
    migrated = select_mapping_floor(legacy, "F21", updated_at="migrated")
    assert_equal(migrated["ok"], True, "legacy session selection")
    assert_equal(migrated["session"]["map_name"], "F21_legacy", "legacy step map name")
    assert_equal(len(migrated["session"]["floor_steps"]), 2, "legacy steps persisted")


def test_per_floor_mapping_lifecycle() -> None:
    session = {
        "id": "s1",
        "active_floor": "F20",
        "map_name": "F20_site",
        "status": "created",
        "floor_steps": [
            {"floor": "F20", "map_name": "F20_site", "status": "ready"},
            {"floor": "F21", "map_name": "F21_site", "status": "pending"},
        ],
    }
    mapping = apply_mapping_command_result(
        session,
        param_name="factory_mapping_start_command",
        result={"ok": True},
        updated_at="mapping",
    )
    assert_equal(mapping["status"], "mapping", "session mapping")
    assert_equal(mapping["floor_steps"][0]["status"], "mapping", "step mapping")
    saved = apply_mapping_command_result(
        mapping,
        param_name="factory_mapping_finish_command",
        result={"ok": True},
        updated_at="saved",
    )
    assert_equal(saved["floor_steps"][0]["status"], "saved", "step saved")
    imported = mark_mapping_floor_imported(
        saved,
        floor="F20",
        map_id="map_20",
        updated_at="imported",
    )
    assert_equal(imported["floor_steps"][0]["status"], "imported", "step imported")
    assert_equal(imported["floor_steps"][0]["map_id"], "map_20", "step imported map")


def test_prepare_mapping_session_create_adds_project() -> None:
    id_factory = IdFactory()
    prepared = prepare_mapping_session_create(
        {"project_name": "", "building": "", "floors": []},
        projects=[],
        id_factory=id_factory,
        now_text=lambda: "2026-06-29 10:21:00",
        default_project_name="M20Pro 工地巡检",
        default_map_name="{active_floor}_stamp",
    )
    assert_equal(prepared["created_project"]["id"], "project_1", "new project id")
    assert_equal(prepared["created_project"]["floors"], [], "new empty project floor registry")
    assert_equal(prepared["session"]["project_id"], "project_1", "session project id")
    assert_equal(prepared["session"]["map_name"], "map_stamp", "fallback map name")


def test_mapping_command_status() -> None:
    assert_equal(mapping_command_status("factory_mapping_start_command", "created", {"ok": True}), "mapping", "start")
    assert_equal(mapping_command_status("factory_mapping_finish_command", "mapping", {"ok": True}), "saved", "finish")
    assert_equal(mapping_command_status("factory_mapping_cancel_command", "mapping", {"ok": True}), "cancelled", "cancel")
    assert_equal(mapping_command_status("unknown", "created", {"ok": True}), "created", "unknown ok")
    assert_equal(mapping_command_status("unknown", "created", {"manual_required": True}), "waiting_manual", "manual")
    assert_equal(mapping_command_status("unknown", "created", {"ok": False}), "created", "failure keeps status")


def test_apply_mapping_command_result() -> None:
    session = {"id": "s1", "status": "created", "updated_at": "old"}
    updated = apply_mapping_command_result(
        session,
        param_name="factory_mapping_start_command",
        result={"ok": True},
        updated_at="new",
    )
    assert_equal(updated["status"], "mapping", "updated status")
    assert_equal(updated["updated_at"], "new", "updated at")
    assert_equal(session["status"], "created", "original unchanged")


def test_mapping_command_context() -> None:
    context = mapping_command_context(
        {
            "id": "session_a",
            "project_name": "Project",
            "building": "B1",
            "mode": "multi",
            "active_floor": "F20",
            "map_name": "desk/test",
            "floors": ["F20", "F21"],
        },
        factory_host="10.21.31.106",
        factory_user="user",
        factory_active_map="/home/user/active_map",
        map_archive_dir="/home/user/m20pro_maps",
    )
    assert_equal(context["session_id"], "session_a", "session id")
    assert_equal(context["map_name"], "desk_test", "sanitized map name")
    assert_equal(context["floors"], "F20,F21", "floors")
    assert_equal(context["factory_host"], "10.21.31.106", "factory host")


def main() -> int:
    for test in (
        test_sanitize_mapping_name,
        test_normalize_mapping_session_request,
        test_prepare_mapping_session_create_reuses_project,
        test_multi_floor_steps_and_selection,
        test_per_floor_mapping_lifecycle,
        test_prepare_mapping_session_create_adds_project,
        test_mapping_command_status,
        test_apply_mapping_command_result,
        test_mapping_command_context,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] mapping contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
