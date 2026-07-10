#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.perception_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.perception_contract import perception_status_payload  # noqa: E402


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def fixed_now_text() -> str:
    return "2026-06-27 01:40:00"


def relay(**extra) -> dict:
    payload = {
        "last_update": 99.0,
        "input_topic": "/LIDAR/POINTS",
        "input_publisher_count": 1,
        "messages": 10,
        "messages_published": 10,
        "output_width": 6000,
        "output_height": 1,
        "cloud_reliability": "auto",
        "subscription_modes": ["best_effort", "reliable"],
        "last_subscription_mode": "best_effort",
        "input_rate_hz": 8.0,
        "publish_rate_hz": 5.0,
        "downsample_method": "numpy_stride",
    }
    payload.update(extra)
    return payload


def lidar(**extra) -> dict:
    payload = {
        "last_update": 99.2,
        "width": 6000,
        "height": 1,
        "source": "/m20pro/lidar_points_relay",
        "frame_id": "m20pro_base_link",
    }
    payload.update(extra)
    return payload


def scan(**extra) -> dict:
    payload = {
        "last_update": 99.4,
        "finite_ranges": 360,
        "frame_id": "m20pro_base_link",
    }
    payload.update(extra)
    return payload


def status(runtime_state: dict, now: float = 100.0, perception_mode: str = "local_fusion") -> dict:
    return perception_status_payload(
        runtime_state,
        now=now,
        now_text=fixed_now_text,
        perception_mode=perception_mode,
    )


def test_factory_lidar_publisher_missing() -> None:
    payload = status(
        {
            "lidar_relay_status": relay(input_publisher_count=0, messages=0, messages_published=0, output_width=0),
            "lidar_points": None,
            "scan": None,
        }
    )
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "factory_lidar_points_publisher_missing", "factory publisher code")
    assert_equal(payload["relay"]["input_publisher_count"], 0, "input publisher count")
    assert_equal(payload["relay"]["messages"], 0, "relay messages")
    assert_equal(payload["updated_at"], fixed_now_text(), "timestamp")


def test_relay_no_samples_when_publishers_are_unknown_or_stale() -> None:
    payload = status(
        {
            "lidar_relay_status": relay(last_update=80.0, input_publisher_count=1, messages=10, messages_published=10),
            "lidar_points": lidar(),
            "scan": scan(),
        }
    )
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "lidar_relay_no_samples", "relay stale code")
    assert_true(payload["relay"]["age_sec"] > 4.0, "relay age")


def test_relay_output_unavailable() -> None:
    payload = status(
        {
            "lidar_relay_status": relay(),
            "lidar_points": lidar(width=0, height=1),
            "scan": scan(),
        }
    )
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "lidar_relay_output_unavailable", "lidar output code")


def test_scan_unavailable() -> None:
    payload = status(
        {
            "lidar_relay_status": relay(),
            "lidar_points": lidar(),
            "scan": scan(finite_ranges=0),
        }
    )
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "scan_unavailable", "scan code")
    assert_equal(payload["scan"]["finite_ranges"], 0, "finite ranges")


def test_perception_ready() -> None:
    payload = status(
        {
            "lidar_relay_status": relay(),
            "lidar_points": lidar(),
            "scan": scan(),
        }
    )
    assert_equal(payload["ready"], True, "ready")
    assert_equal(payload["code"], "perception_ready", "ready code")
    assert_equal(payload["relay"]["output_points"], 6000, "output points")
    assert_equal(payload["lidar_points"]["points"], 6000, "lidar points")
    assert_equal(payload["scan"]["finite_ranges"], 360, "scan ranges")


def test_default_mode_still_requires_relay() -> None:
    payload = status(
        {
            "lidar_relay_status": {},
            "lidar_points": {},
            "scan": scan(),
        }
    )
    assert_equal(payload["ready"], False, "default mode not ready without relay")
    assert_equal(payload["code"], "lidar_relay_no_samples", "default mode relay code")
    assert_equal(payload["mode"], "local_fusion", "default mode")


def test_edge_scan_mode_uses_scan_as_hard_condition() -> None:
    payload = status(
        {
            "lidar_relay_status": {},
            "lidar_points": {},
            "scan": scan(finite_ranges=240),
        },
        perception_mode="edge_scan",
    )
    assert_equal(payload["ready"], True, "edge scan ready with fresh scan")
    assert_equal(payload["code"], "perception_ready", "edge scan ready code")
    assert_equal(payload["mode"], "edge_scan", "edge scan mode")
    assert_equal(payload["relay"]["not_used"], True, "relay not used")
    assert_equal(payload["relay"]["ok"], False, "relay not required")
    assert_equal(payload["lidar_points"]["ok"], False, "relay pointcloud not required")
    assert_equal(payload["scan"]["finite_ranges"], 240, "edge scan ranges")


def test_edge_scan_mode_fails_when_scan_is_missing() -> None:
    payload = status(
        {
            "lidar_relay_status": relay(),
            "lidar_points": lidar(),
            "scan": scan(finite_ranges=0),
        },
        perception_mode="edge_scan",
    )
    assert_equal(payload["ready"], False, "edge scan not ready without scan")
    assert_equal(payload["code"], "scan_unavailable", "edge scan missing scan code")
    assert_equal(payload["relay"]["not_used"], True, "relay ignored in edge scan mode")


def main() -> int:
    for test in (
        test_factory_lidar_publisher_missing,
        test_relay_no_samples_when_publishers_are_unknown_or_stale,
        test_relay_output_unavailable,
        test_scan_unavailable,
        test_perception_ready,
        test_default_mode_still_requires_relay,
        test_edge_scan_mode_uses_scan_as_hard_condition,
        test_edge_scan_mode_fails_when_scan_is_missing,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] perception contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
