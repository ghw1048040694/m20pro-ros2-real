#!/usr/bin/env python3
"""Offline tests for normalized occupancy-map content identity."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.map_identity_contract import (  # noqa: E402
    map_content_match,
    occupancy_grid_content_digest,
)


def grid(data):
    return {
        "available": True,
        "width": 2,
        "height": 2,
        "resolution": 0.05,
        "origin": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.1},
        "data": list(data),
    }


def main() -> int:
    first = grid([0, 100, -1, 0])
    same = grid([0, 100, -1, 0])
    changed = grid([0, 100, 100, 0])
    digest = occupancy_grid_content_digest(first)
    assert digest and digest == occupancy_grid_content_digest(same)
    assert digest != occupancy_grid_content_digest(changed)
    assert map_content_match(first, same)["ok"]
    assert map_content_match(first, changed)["code"] == "map_content_mismatch"
    web_source = (
        ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py"
    ).read_text(encoding="utf-8")
    for retired_factory_gate in (
        "sha256sum",
        "source_factory_identity",
        "_parse_factory_identity_output",
        "_factory_map_identity",
        "_capture_factory_active_identity",
        "_restore_factory_active_identity",
        "_confirm_factory_active_map",
    ):
        assert retired_factory_gate not in web_source
    print("map identity contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
