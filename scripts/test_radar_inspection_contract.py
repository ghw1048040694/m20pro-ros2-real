#!/usr/bin/env python3
"""Offline contract tests for the U360 radar integration."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_radar_inspection"))

from m20pro_radar_inspection.radar_inspection_node import (  # noqa: E402
    extract_state,
    generate_u360_task_id,
    is_timeout_error,
    parse_device_data,
    scan_plan_from_waypoint,
    summarize_measurement_result,
    waypoint_key,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def test_waypoint_identity_and_scan_plan() -> None:
    active = {
        "task_id": "task_001",
        "index": 2,
        "waypoint": {
            "id": "point_003",
            "radar": {
                "enabled": True,
                "scans": [
                    {"mode": "modeling", "order": 2},
                    {"mode": "measuring", "order": 1, "density": "high"},
                ],
            },
        },
    }
    assert_equal(waypoint_key(active), "task_001:2:point_003", "stable waypoint key")
    plan = scan_plan_from_waypoint(active, "measuring", "low")
    assert_equal([item["mode"] for item in plan["scans"]], ["measuring", "modeling"], "ordered scans")
    assert_equal(plan["scans"][0]["density"], "high", "per-scan density")
    assert_equal(plan["scans"][1]["manual_measure_required"], True, "modeling manual result")


def test_disabled_and_explicit_scan_plan() -> None:
    disabled = {"waypoint": {"radar": {"enabled": "false"}}}
    assert_equal(scan_plan_from_waypoint(disabled, "measuring", "low"), {"enabled": False, "scans": []}, "disabled plan")
    fallback = scan_plan_from_waypoint({"waypoint": {}}, "invalid", "normal")
    assert_equal(fallback, {"enabled": False, "scans": []}, "missing radar is navigation only")


def test_nested_device_state_and_timeout_detection() -> None:
    response = {"data": '{"state":"analyzing","progress":"87"}'}
    parsed = parse_device_data(response["data"])
    assert_equal(parsed["state"], "analyzing", "nested device JSON")
    assert_equal(extract_state(response), ("analyzing", 87), "state and progress")
    assert_true(is_timeout_error(RuntimeError("request timed out")), "English timeout")
    assert_true(is_timeout_error(RuntimeError("请求超时")), "Chinese timeout")
    assert_equal(is_timeout_error(RuntimeError("connection refused")), False, "non-timeout error")


def test_measurement_summary_and_task_id() -> None:
    response = {
        "parsedData": {
            "status": "finished",
            "walls": [{"measurements": [{"measurementItemId": 3, "value": "1.25 mm"}]}],
        }
    }
    request = {"taskId": "radar_001", "mode": "measuring", "room": "2008"}
    summary = summarize_measurement_result(response, request)
    assert_equal(summary["metricCount"], 1, "measurement count")
    assert_equal(summary["metrics"][0]["measurementItemId"], 3, "measurement id")
    assert_equal(summary["metrics"][0]["numericValue"], 1.25, "numeric value")
    task_id = generate_u360_task_id("3", "1", "F20", "2008", 1, "fallback", "measure")
    assert_true(task_id.startswith("B03_U01_F20_R2008_P01_measure_"), "U360 task id contract")


def main() -> None:
    tests = [
        test_waypoint_identity_and_scan_plan,
        test_disabled_and_explicit_scan_plan,
        test_nested_device_state_and_timeout_detection,
        test_measurement_summary_and_task_id,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] radar inspection contract tests passed")


if __name__ == "__main__":
    main()
