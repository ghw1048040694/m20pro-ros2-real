#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.startup_map_sync_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.startup_map_sync_contract import (  # noqa: E402
    startup_map_sync_missing_record_payload,
    startup_map_sync_retry_decision,
    startup_map_sync_result_payload,
    startup_map_sync_skipped_payload,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def now_text() -> str:
    return "2026-06-27 02:18:00"


def test_skipped_payload() -> None:
    payload = startup_map_sync_skipped_payload(
        reason="selected_map_missing",
        attempt=1,
        max_attempts=5,
        now_text=now_text,
    )
    assert_equal(payload["ok"], True, "ok")
    assert_equal(payload["skipped"], True, "skipped")
    assert_equal(payload["reason"], "selected_map_missing", "reason")
    assert_equal(payload["attempt"], 1, "attempt")
    assert_equal(payload["max_attempts"], 5, "max attempts")
    assert_equal(payload["updated_at"], now_text(), "timestamp")


def test_missing_record_payload() -> None:
    payload = startup_map_sync_missing_record_payload(
        selected_map_id="map_missing",
        attempt=2,
        max_attempts=5,
        now_text=now_text,
    )
    assert_equal(payload["ok"], False, "not ok")
    assert_equal(payload["code"], "selected_map_missing", "code")
    assert_equal(payload["selected_map_id"], "map_missing", "selected map id")
    assert_equal(payload["attempt"], 2, "attempt")


def test_result_payload() -> None:
    nav2_load = {
        "ok": True,
        "loaded": True,
        "yaml_path": "/tmp/map.yaml",
        "result": 0,
    }
    payload = startup_map_sync_result_payload(
        selected_map_id="map_a",
        map_name="F20_TEST",
        nav2_load_map=nav2_load,
        attempt=1,
        max_attempts=5,
        now_text=now_text,
    )
    assert_equal(payload["ok"], True, "ok")
    assert_equal(payload["selected_map_id"], "map_a", "selected map id")
    assert_equal(payload["map_name"], "F20_TEST", "map name")
    assert_equal(payload["nav2_load_map"], nav2_load, "nav2 load")
    assert_equal(payload["updated_at"], now_text(), "timestamp")


def test_retry_decision_for_delayed_nav2_load_map_service() -> None:
    decision = startup_map_sync_retry_decision(
        {"ok": False, "code": "load_map_service_unavailable"},
        attempt=3,
        max_attempts=12,
    )
    assert_equal(decision["retryable"], True, "retryable service unavailable")
    assert_equal(decision["retry"], True, "retry service unavailable")
    assert_equal(decision["attempts_left"], 9, "attempts left")
    assert_equal(decision["next_attempt"], 4, "next attempt")

    timeout = startup_map_sync_retry_decision(
        {"ok": False, "code": "load_map_timeout"},
        attempt=11,
        max_attempts=12,
    )
    assert_equal(timeout["retry"], True, "retry timeout before final attempt")

    exhausted = startup_map_sync_retry_decision(
        {"ok": False, "code": "load_map_service_unavailable"},
        attempt=12,
        max_attempts=12,
    )
    assert_equal(exhausted["retryable"], True, "still retryable type")
    assert_equal(exhausted["retry"], False, "no retry after max attempts")
    assert_equal(exhausted["next_attempt"], None, "no next attempt")

    hard_failure = startup_map_sync_retry_decision(
        {"ok": False, "code": "map_yaml_missing"},
        attempt=1,
        max_attempts=12,
    )
    assert_equal(hard_failure["retryable"], False, "hard failure not retryable")
    assert_equal(hard_failure["retry"], False, "hard failure no retry")


def main() -> int:
    for test in (
        test_skipped_payload,
        test_missing_record_payload,
        test_result_payload,
        test_retry_decision_for_delayed_nav2_load_map_service,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] startup map sync contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
