#!/usr/bin/env python3
"""Static contract checks for the end-to-end one-key charging path."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TCP_BRIDGE = (
    ROOT
    / "src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py"
).read_text(encoding="utf-8")
WEB = (
    ROOT
    / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py"
).read_text(encoding="utf-8")
RECORD = (
    ROOT / "src/m20pro_bringup/scripts/m20pro_record_real.sh"
).read_text(encoding="utf-8")


def assert_contains(text: str, needle: str, message: str) -> None:
    if needle not in text:
        raise AssertionError(f"{message}: missing {needle!r}")


def main() -> int:
    assert_contains(TCP_BRIDGE, "import json", "TCP bridge parses charge JSON")
    assert_contains(TCP_BRIDGE, "def _on_charge_command", "TCP bridge charge subscriber")
    assert_contains(TCP_BRIDGE, "self.client.request(\n                1003,\n                1,", "native charge command")
    assert_contains(TCP_BRIDGE, '"PointInfo": 3', "native charge point semantics")
    assert_contains(TCP_BRIDGE, "charge_response_timeout_s", "native response timeout")
    assert_contains(TCP_BRIDGE, "vendor_status", "native status evidence")
    assert_contains(TCP_BRIDGE, '"status": status', "charge result status")

    assert_contains(WEB, 'if active.get("phase") == "charging"', "charging phase tick")
    assert_contains(WEB, '"phase": "charging"', "charge phase transition")
    assert_contains(WEB, 'status == "accepted"', "accepted result handling")
    assert_contains(WEB, 'status == "failed"', "failed result handling")
    assert_contains(WEB, 'manual_point_type") or "").strip().lower() == "charge"', "charge-only trigger")
    assert_contains(WEB, "charge_command_discovery_timeout_s", "DDS discovery wait")
    assert_contains(WEB, "charge_result_topic", "dedicated charge result topic")

    assert_contains(RECORD, "/m20pro/charge_command", "charge request recording")
    assert_contains(RECORD, "/m20pro_tcp_bridge/charge_result", "charge result recording")
    print("[OK] charge command contract checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
