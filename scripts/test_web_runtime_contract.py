#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.web_runtime_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.web_runtime_contract import (  # noqa: E402
    api_error_payload,
    as_bool,
    fmt_age_text,
    payload_with_age,
    random_suffix,
    sanitize_name,
    parse_json_text,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def test_parse_json_text() -> None:
    assert_equal(parse_json_text('{"ok": true}'), {"ok": True}, "valid json")
    assert_equal(parse_json_text("{bad"), None, "invalid json returns none")


def test_age_helpers() -> None:
    assert_equal(fmt_age_text(None), "无时间", "none age")
    assert_equal(fmt_age_text(0.4), "<1s", "subsecond age")
    assert_equal(fmt_age_text(12.4), "12s前", "rounded age")
    assert_equal(payload_with_age(None), None, "empty payload")
    aged = payload_with_age({"timestamp": 95.0, "value": 1}, now=100.0)
    assert_equal(aged["age_sec"], 5.0, "age injected")
    assert_equal(aged["value"], 1, "payload value retained")


def test_name_bool_error_helpers() -> None:
    assert_equal(sanitize_name(" F20 / 配电室:入口 ", "fallback"), "F20_配电室_入口", "name sanitized")
    assert_equal(sanitize_name("///", "fallback"), "fallback", "fallback used")
    assert_true(as_bool(True), "bool true")
    assert_true(as_bool(" yes "), "string yes")
    assert_true(not as_bool("0"), "string zero")
    assert_equal(api_error_payload("失败"), {"ok": False, "message": "失败"}, "simple error")
    assert_equal(
        api_error_payload("失败", {"code": "bad"}),
        {"ok": False, "message": "失败", "code": "bad"},
        "error extra",
    )
    assert_equal(len(random_suffix(6)), 6, "short suffix length")
    assert_true(random_suffix(0), "suffix has minimum length")


def main() -> int:
    for test in (
        test_parse_json_text,
        test_age_helpers,
        test_name_bool_error_helpers,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] web runtime contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
