#!/usr/bin/env python3
"""Regression tests for connected battery-slot detection."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.battery_contract import battery_pack_present  # noqa: E402


def test_connected_pack_is_present() -> None:
    pack = SimpleNamespace(
        voltage=7645,
        current=-155,
        remaining_capacity=261,
        nominal_capacity=450,
        battery_level=62,
        cycles=43,
        battery_temperature=[29.3],
    )
    assert battery_pack_present(pack) is True


def test_unplugged_vendor_slot_is_not_a_pack() -> None:
    empty = SimpleNamespace(
        voltage=0,
        current=0,
        remaining_capacity=0,
        nominal_capacity=0,
        battery_level=0,
        cycles=0,
        mos_state=0,
        protected_state=0,
        battery_quantity=0,
        battery_ntc=0,
        battery_temperature=[0.0],
        battery_serialnum=[0] * 32,
    )
    assert battery_pack_present(empty) is False


def main() -> int:
    test_connected_pack_is_present()
    test_unplugged_vendor_slot_is_not_a_pack()
    print("battery contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
