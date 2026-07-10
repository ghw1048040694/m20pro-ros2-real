#!/usr/bin/env python3
"""Offline tests for the production edge-scan perception contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.perception_contract import perception_status_payload  # noqa: E402


def fixed_now_text() -> str:
    return "2026-07-10 14:30:00"


def test_fresh_scan_is_ready() -> None:
    payload = perception_status_payload(
        {"scan": {"last_update": 99.5, "finite_ranges": 190, "frame_id": "m20pro_base_link"}},
        now=100.0,
        now_text=fixed_now_text,
    )
    assert payload["ready"] is True
    assert payload["code"] == "perception_ready"
    assert payload["mode"] == "edge_scan"
    assert payload["scan"]["finite_ranges"] == 190


def test_missing_or_empty_scan_fails() -> None:
    for scan in (None, {"last_update": 99.5, "finite_ranges": 0}):
        payload = perception_status_payload({"scan": scan}, now=100.0, now_text=fixed_now_text)
        assert payload["ready"] is False
        assert payload["code"] == "scan_unavailable"


def test_stale_scan_fails() -> None:
    payload = perception_status_payload(
        {"scan": {"last_update": 90.0, "finite_ranges": 190}},
        now=100.0,
        scan_timeout_s=2.0,
        now_text=fixed_now_text,
    )
    assert payload["ready"] is False
    assert payload["scan"]["age_sec"] == 10.0


def main() -> int:
    for test in (test_fresh_scan_is_ready, test_missing_or_empty_scan_fails, test_stale_scan_fails):
        test()
        print(f"[OK] {test.__name__}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
