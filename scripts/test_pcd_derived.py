#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.pcd_derived."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.pcd_derived import process_imported_map  # noqa: E402


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def write_floor_config(path: Path) -> None:
    path.write_text(
        """
floors:
  F20:
    stairs:
      stair_A_up:
        target_floor: F21
        direction: up
        transition:
          entry: {x: 1.0, y: 2.0, z: 0.0, yaw: 0.1}
          source_platform: {x: 1.5, y: 2.4, z: 0.0, yaw: 0.2}
          target_platform: {x: 2.0, y: 2.8, z: 0.4, yaw: 0.3}
          post_exit: {x: 2.5, y: 3.1, z: 0.4, yaw: 0.4}
          entry_margin_m: 0.5
""".strip(),
        encoding="utf-8",
    )


def test_process_imported_map_generates_only_stair_zones() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        map_dir = root / "map"
        map_dir.mkdir()
        yaml_path = map_dir / "map.yaml"
        yaml_path.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
        floor_config = root / "inspection_waypoints.yaml"
        write_floor_config(floor_config)
        pcd_path = map_dir / "full_cloud.pcd"
        pcd_path.write_text("placeholder", encoding="utf-8")

        result = process_imported_map(
            map_dir,
            yaml_path,
            "F20",
            "map_F20",
            floor_config_path=floor_config,
            pcd_path_override=pcd_path,
            cell_size=0.05,
            stair_point_max=10,
        )

        assert_equal(result["status"], "ready", "result status")
        assert_equal(result["zone_count"], 1, "zone count")
        assert_true("stair_zones" in result, "stair zones path returned")
        derived = map_dir / "derived"
        assert_true((derived / "stair_zones.json").exists(), "stair zones file exists")
        assert_true(not (derived / "terrain_mesh.json").exists(), "terrain mesh is not generated")
        assert_true(not (derived / "height_grid.json").exists(), "height grid is not generated")
        assert_true(not (derived / "stairs").exists(), "local stair pointclouds are not generated")

        payload = json.loads((derived / "stair_zones.json").read_text(encoding="utf-8"))
        assert_equal(payload["type"], "stair_zones", "payload type")
        assert_equal(payload["map_id"], "map_F20", "payload map id")
        assert_equal(payload["floor"], "F20", "payload floor")
        assert_equal(payload["source_pcd"], str(pcd_path), "source pcd is recorded only as metadata")
        zone = payload["zones"][0]
        assert_equal(zone["target_floor"], "F21", "target floor")
        assert_equal(zone["source"], "configured_route", "zone source")
        assert_true("pointcloud" not in zone, "zone has no local pointcloud asset")


def main() -> int:
    test_process_imported_map_generates_only_stair_zones()
    print("[OK] test_process_imported_map_generates_only_stair_zones")
    print("[OK] pcd derived tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
