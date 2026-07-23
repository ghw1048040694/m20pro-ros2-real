#!/usr/bin/env python3
"""Offline tests for normalized occupancy-map content identity."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.map_identity_contract import (  # noqa: E402
    factory_identity_match,
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
    expected = {"resolved_path": "/maps/source", "content_digest": "a" * 64}
    active_same_path_changed = {"resolved_path": "/maps/source", "content_digest": "b" * 64}
    assert not factory_identity_match(expected, active_same_path_changed)["ok"]
    active_copy = {"resolved_path": "/maps/active-copy", "content_digest": "a" * 64}
    assert factory_identity_match(expected, active_copy)["identity_mode"] == "content"
    assert not factory_identity_match(
        expected,
        {"resolved_path": "/maps/active-copy", "content_digest": "c" * 64},
    )["ok"]
    print("map identity contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
