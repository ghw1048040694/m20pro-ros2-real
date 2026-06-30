#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.map_derived_contract."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.map_derived_contract import (  # noqa: E402
    builtin_map_derived_payload,
    read_json_object,
    resolve_map_asset_path,
    should_generate_builtin_stair_zones,
    stair_zones_available_payload,
    stair_zones_relative_path,
    stair_zones_unavailable_payload,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def test_builtin_map_derived_payload_pending_and_ready() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        yaml_path = root / "map.yaml"
        yaml_path.write_text("image: map.pgm\n", encoding="utf-8")

        pending = builtin_map_derived_payload(yaml_path, "/tmp/map.pcd")
        assert_equal(pending["status"], "pending", "pending status")
        assert_equal(pending["pcd_path"], "/tmp/map.pcd", "pending pcd")

        derived = root / "derived"
        derived.mkdir()
        (derived / "stair_zones.json").write_text("{}", encoding="utf-8")
        ready = builtin_map_derived_payload(yaml_path, "/tmp/map.pcd")
        assert_equal(ready["status"], "ready", "ready status")
        assert_equal(ready["stair_zones"], "derived/stair_zones.json", "relative zones")


def test_stair_zone_generation_decision_and_relative_path() -> None:
    record = {
        "source": "project_builtin",
        "derived": {"status": "pending", "stair_zones": "derived/stair_zones.json"},
    }
    assert_true(
        should_generate_builtin_stair_zones(record, enable_stair_zone_postprocess=True),
        "pending builtin generates",
    )
    assert_true(
        not should_generate_builtin_stair_zones(record, enable_stair_zone_postprocess=False),
        "disabled does not generate",
    )
    assert_equal(stair_zones_relative_path(record), "derived/stair_zones.json", "relative path")


def test_resolve_map_asset_path() -> None:
    record = {
        "directory": "pkg://maps/F20",
        "derived": {"base_dir": "pkg://cache/F20"},
    }
    resolver = lambda value: value.replace("pkg://", "/resolved/")
    assert_equal(
        resolve_map_asset_path(record, "derived/stair_zones.json", path_resolver=resolver),
        Path("/resolved/cache/F20/derived/stair_zones.json"),
        "derived base dir wins",
    )
    record["derived"] = {}
    assert_equal(
        resolve_map_asset_path(record, "derived/stair_zones.json", path_resolver=resolver),
        Path("/resolved/maps/F20/derived/stair_zones.json"),
        "map directory fallback",
    )
    assert_equal(
        resolve_map_asset_path(record, "/tmp/stair_zones.json", path_resolver=resolver),
        Path("/tmp/stair_zones.json"),
        "absolute path",
    )
    assert_equal(resolve_map_asset_path({"derived": {}}, "", path_resolver=resolver), None, "blank path")


def test_read_json_object() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "payload.json"
        path.write_text('{"zones":[]}', encoding="utf-8")
        assert_equal(read_json_object(path), {"zones": []}, "json object")
        path.write_text("[]", encoding="utf-8")
        try:
            read_json_object(path)
        except RuntimeError as exc:
            assert_equal(str(exc), "JSON payload is not an object", "non-object message")
        else:
            raise AssertionError("non-object JSON should fail")


def test_stair_zones_payloads() -> None:
    record = {"id": "map_a", "name": "Desk", "floor": "F20"}
    unavailable = stair_zones_unavailable_payload(record, "missing")
    assert_equal(unavailable["available"], False, "unavailable")
    assert_equal(unavailable["map_id"], "map_a", "unavailable map id")
    assert_equal(unavailable["zones"], [], "unavailable zones")
    assert_equal(
        stair_zones_unavailable_payload(None, "no map"),
        {"ok": True, "available": False, "message": "no map", "zones": []},
        "no-record unavailable",
    )

    payload = stair_zones_available_payload(
        record,
        {"status": "ready"},
        {"type": "stair_zones", "zones": [{"id": "zone_a"}]},
    )
    assert_equal(payload["available"], True, "available")
    assert_equal(payload["map"]["id"], "map_a", "available map id")
    assert_equal(payload["map"]["derived_status"], "ready", "derived status")
    assert_equal(payload["zones"][0]["id"], "zone_a", "zone preserved")


def main() -> int:
    for test in (
        test_builtin_map_derived_payload_pending_and_ready,
        test_stair_zone_generation_decision_and_relative_path,
        test_resolve_map_asset_path,
        test_read_json_object,
        test_stair_zones_payloads,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] map derived contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
