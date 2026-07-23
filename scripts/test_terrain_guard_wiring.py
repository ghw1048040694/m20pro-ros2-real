#!/usr/bin/env python3
"""Static runtime contract checks for the 106-local terrain guard node."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "src/m20pro_navigation/m20pro_navigation/terrain_guard_106_node.py"
SETUP = ROOT / "src/m20pro_navigation/setup.py"


def _method(tree: ast.AST, name: str) -> ast.FunctionDef:
    for item in ast.walk(tree):
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)) and item.name == name:
            return item  # type: ignore[return-value]
    raise AssertionError(f"missing method: {name}")


def _called_attributes(method: ast.FunctionDef) -> set[str]:
    return {
        node.func.attr
        for node in ast.walk(method)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }


def main() -> int:
    source = NODE.read_text(encoding="utf-8")
    setup_source = SETUP.read_text(encoding="utf-8")
    tree = ast.parse(source)
    cloud_callback = _method(tree, "_on_cloud")

    assert "terrain_guard_106 = m20pro_navigation.terrain_guard_106_node:main" in setup_source
    assert "terrain_guard_replay = m20pro_navigation.terrain_guard_replay:main" in setup_source
    assert "from .terrain_guard_contract import (" in source
    assert "inspect_cloud," in source
    assert "normalize_corridor," in source
    assert "sys.path.insert" not in source
    assert "uvs=uvs" in source
    assert "max_points" in source
    assert "terrain_request_identity_missing" in source
    assert "self._request_id = request_id" in source
    assert '"request_id": self._request_id' in source
    assert '"plan_id": str((self._request or {}).get("plan_id") or "")' in source
    assert '"map_epoch": (self._request or {}).get("map_epoch")' in source
    assert "terrain_request_ownership_decision" in source
    assert '"profile_id": self._request_profile_id' in source
    assert '"permit_motion": False' in source
    assert '"certified_motion": False' in source
    assert 'create_subscription(PointCloud2, cloud_topic, self._on_cloud' in source
    assert 'create_subscription(String, request_topic, self._on_request' in source

    # The point-cloud callback is deliberately an ingest boundary. Evaluation
    # belongs to the bounded timer so a vendor callback cannot run analysis at
    # an unbounded rate or duplicate timer work.
    called = _called_attributes(cloud_callback)
    assert "_evaluate" not in called
    assert "_publish_status" not in called

    # This package must remain a read-only 106 adapter. These checks guard the
    # actual control/network surfaces rather than prose in the module docstring.
    assert '"/cmd_vel"' not in source
    assert "Twist" not in source
    assert "socket" not in source
    assert "paramiko" not in source
    assert "requests" not in source
    assert "10.21.31.104" not in source
    print("[OK] terrain guard ROS wiring contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
