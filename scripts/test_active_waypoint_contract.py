#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.active_waypoint_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.active_waypoint_contract import (  # noqa: E402
    build_active_waypoint_payload,
    build_idle_waypoint_payload,
    pose_age_sec,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def active_payload() -> dict:
    return {
        "task_id": "task_1",
        "task_name": "单层测试",
        "index": 1,
        "phase": "navigating",
        "status_message": "已下发当前点位",
        "last_nav_goal_status": "accepted",
        "last_robot_pose": {"x": 0.5, "y": 0.6, "yaw": 0.1},
        "last_distance_m": 1.8,
    }


def waypoint_payload() -> dict:
    return {
        "id": "p2",
        "label": "客厅P01",
        "floor": "F20",
        "room": "客厅",
        "scan_point": "P01",
        "result_file_prefix": "B03_U01_H2008_F20_客厅_P01",
        "radar": {
            "enabled": True,
            "scans": [
                {"mode": "measuring", "label": "实测实量", "result_suffix": "measure"},
                {"mode": "modeling", "label": "点云建模", "result_suffix": "cloud"},
            ],
        },
    }


def test_pose_age_sec() -> None:
    assert_equal(pose_age_sec({}, now=100.0), None, "missing pose age")
    assert_equal(pose_age_sec({"last_update": 97.5}, now=100.0), 2.5, "pose age")
    assert_equal(pose_age_sec({"last_update": "bad"}, now=100.0), None, "bad pose age")


def test_active_waypoint_payload() -> None:
    active = active_payload()
    active.update(
        {
            "phase": "dwelling",
            "dwell_until": 105.0,
            "waypoint_started_monotonic": 80.0,
        }
    )
    payload = build_active_waypoint_payload(
        active,
        {
            "id": "p2",
            "label": "客厅P01",
            "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.3},
        },
        phase="dwelling",
        now_text="now",
        now_time=100.0,
        now_monotonic=100.0,
        waypoint=waypoint_payload(),
    )
    assert_equal(payload["task_id"], "task_1", "task id")
    assert_equal(payload["phase"], "dwelling", "phase")
    assert_equal(payload["remaining_dwell_s"], 5.0, "remaining dwell")
    assert_equal(payload["elapsed_s"], 20.0, "elapsed")
    assert_equal(payload["goal_pose"], {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.3}, "goal pose")
    assert_equal(payload["nav_goal_status"], "accepted", "nav goal status")
    assert_equal(payload["robot_pose"], {"x": 0.5, "y": 0.6, "yaw": 0.1}, "robot pose")
    assert_equal(payload["distance_m"], 1.8, "distance")
    assert_equal(payload["waypoint"]["room"], "客厅", "room semantics retained")
    assert_equal(payload["waypoint"]["scan_point"], "P01", "scan point retained")
    assert_equal(payload["waypoint"]["radar"]["scans"][0]["mode"], "measuring", "radar scan mode retained")
    assert_equal(payload["updated_at"], "now", "updated time")


def test_idle_waypoint_payload() -> None:
    payload = build_idle_waypoint_payload(reason="task_completed", now_text="now")
    assert_equal(payload["phase"], "idle", "idle phase")
    assert_equal(payload["reason"], "task_completed", "idle reason")
    assert_equal(payload["updated_at"], "now", "idle updated time")

    fallback = build_idle_waypoint_payload(reason="", now_text="now")
    assert_equal(fallback["reason"], "idle", "blank idle reason falls back")


def main() -> int:
    for test in (
        test_pose_age_sec,
        test_active_waypoint_payload,
        test_idle_waypoint_payload,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] active-waypoint contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
