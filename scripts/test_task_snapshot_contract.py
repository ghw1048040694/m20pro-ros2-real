#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.task_snapshot_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.task_snapshot_contract import (  # noqa: E402
    apply_task_result_persistence,
    apply_task_result_to_tasks,
    build_active_waypoint_payload,
    build_idle_waypoint_payload,
    build_task_result_snapshot,
    build_task_runtime_snapshot,
    last_task_result_payload,
    pose_age_sec,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def active_payload() -> dict:
    return {
        "task_id": "task_1",
        "task_name": "单层测试",
        "map_id": "builtin_F20",
        "index": 1,
        "phase": "navigating",
        "status_message": "已下发当前点位",
        "last_goal_annotation_id": "p2",
        "last_goal_label": "点2",
        "last_goal_pose": {"floor": "F20", "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.3},
        "last_goal_attempt_id": "goal_1",
        "last_floor_goal_published_at": "2026-06-27 09:29:00",
        "last_floor_goal_annotation_id": "p2",
        "last_floor_goal_label": "点2",
        "last_floor_goal_pose": {"floor": "F20", "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.3},
        "floor_goal_publish_count": 1,
        "goal_sent_path_version": 8,
        "plan_goal_verified": True,
        "plan_goal_error_m": 0.12,
        "plan_path_version": 9,
        "last_nav_goal_status": "accepted",
        "last_nav_status": "nav_goal_accepted label=floor_goal goal_seq=4",
        "last_nav_feedback": {"distance_remaining": 1.2, "recoveries": 0},
        "last_nav_goal_match": {"matches": True, "nav_goal_seq": 4},
        "last_robot_pose": {"x": 0.5, "y": 0.6, "yaw": 0.1},
        "last_distance_m": 1.8,
        "runtime_guard": {"ready": True, "code": "ready"},
        "timeline": [{"event": f"e{i}", "message": str(i)} for i in range(14)],
    }


def state_payload() -> dict:
    return {
        "floor": "F20",
        "localization_ok": True,
        "navigation_status": "nav_goal_feedback label=floor_goal",
        "pose": {"x": 0.5, "y": 0.6, "yaw": 0.1, "last_update": 95.0},
        "path": {"version": 9, "point_count": 4, "raw_point_count": 9, "last_point": {"x": 1.0, "y": 2.0}},
        "scan": {"finite_ranges": 211, "frame_id": "laser", "last_update": 98.0},
        "lidar_points": {"width": 3000, "height": 1, "source": "relay", "frame_id": "lidar", "last_update": 97.0},
        "lidar_relay_status": {
            "output_width": 3000,
            "output_height": 1,
            "output_stride": 2,
            "downsample_method": "numpy_stride",
            "input_rate_hz": 10.0,
            "publish_rate_hz": 5.0,
            "skip_ratio": 0.5,
            "max_output_points": 6000,
            "min_publish_interval_s": 0.2,
            "last_update": 99.0,
        },
        "topics": {
            "scan": {"last_update": 98.0},
            "unused_topic": {"last_update": 1.0},
        },
    }


def test_pose_age_sec() -> None:
    assert_equal(pose_age_sec({}, now=100.0), None, "missing pose age")
    assert_equal(pose_age_sec({"last_update": 97.5}, now=100.0), 2.5, "pose age")
    assert_equal(pose_age_sec({"last_update": "bad"}, now=100.0), None, "bad pose age")


def test_runtime_snapshot() -> None:
    snapshot = build_task_runtime_snapshot(
        active_payload(),
        state_payload(),
        camera_proxy_status={"enabled": False},
        now=100.0,
    )
    assert_equal(snapshot["floor"], "F20", "floor")
    assert_equal(snapshot["pose_age_sec"], 5.0, "pose age")
    assert_equal(snapshot["scan"]["age_sec"], 2.0, "scan age")
    assert_equal(snapshot["lidar_points"]["source"], "relay", "lidar source")
    assert_equal(snapshot["lidar_relay_status"]["downsample_method"], "numpy_stride", "relay method")
    assert_equal(snapshot["active_index"], 1, "active index")
    assert_equal(snapshot["last_goal_attempt_id"], "goal_1", "goal attempt")
    assert_equal(snapshot["last_floor_goal_published_at"], "2026-06-27 09:29:00", "floor goal published time")
    assert_equal(snapshot["floor_goal_publish_count"], 1, "floor goal publish count")
    assert_equal(snapshot["last_nav_feedback"]["distance_remaining"], 1.2, "nav feedback")
    assert_true("unused_topic" not in snapshot["topics"], "unused topics are omitted")


def test_active_waypoint_payload() -> None:
    active = active_payload()
    active.update(
        {
            "phase": "dwelling",
            "dwell_until": 105.0,
            "waypoint_started_monotonic": 80.0,
            "last_nav_feedback_monotonic": 97.5,
            "waypoint_goal_send_count": 2,
            "total_goal_send_count": 3,
            "resend_goal_count": 1,
            "last_progress_moved_m": 0.4,
            "last_progress_yaw_delta_rad": 0.1,
            "last_progress_distance_delta_m": -0.3,
            "runtime_guard_lost_age_s": 0.0,
        }
    )
    payload = build_active_waypoint_payload(
        active,
        {
            "id": "p2",
            "label": "点2",
            "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.3},
        },
        state_payload(),
        phase="dwelling",
        now_text="now",
        now_time=100.0,
        now_monotonic=100.0,
        waypoint={"id": "p2", "label": "点2"},
    )
    assert_equal(payload["task_id"], "task_1", "task id")
    assert_equal(payload["phase"], "dwelling", "phase")
    assert_equal(payload["remaining_dwell_s"], 5.0, "remaining dwell")
    assert_equal(payload["elapsed_s"], 20.0, "elapsed")
    assert_equal(payload["path_goal_error_m"], 0.0, "path goal error")
    assert_equal(payload["path_point_count"], 4, "path point count")
    assert_equal(payload["goal_attempt_id"], "goal_1", "goal attempt")
    assert_equal(payload["goal_send_count"], 2, "goal send count")
    assert_equal(payload["last_floor_goal_published_at"], "2026-06-27 09:29:00", "floor goal published time")
    assert_equal(payload["floor_goal_publish_count"], 1, "floor goal publish count")
    assert_equal(payload["nav_feedback_age_s"], 2.5, "feedback age")
    assert_equal(payload["state_pose_age_s"], 5.0, "state pose age")
    assert_equal(payload["state_pose"]["x"], 0.5, "state pose retained")
    assert_equal(payload["waypoint"]["id"], "p2", "waypoint semantics")
    assert_equal(payload["updated_at"], "now", "updated time")

    invalid_pose = state_payload()
    invalid_pose["pose"] = {"x": "bad", "y": 0.0, "last_update": 99.0}
    invalid_payload = build_active_waypoint_payload(
        active,
        {"pose": {"x": 1.0, "y": 2.0}},
        invalid_pose,
        phase="navigating",
        now_text="now",
        now_time=100.0,
        now_monotonic=100.0,
        waypoint={},
    )
    assert_equal(invalid_payload["state_pose"], None, "invalid state pose omitted")


def test_idle_waypoint_payload() -> None:
    payload = build_idle_waypoint_payload(reason="task_completed", now_text="now")
    assert_equal(payload["phase"], "idle", "idle phase")
    assert_equal(payload["reason"], "task_completed", "idle reason")
    assert_equal(payload["updated_at"], "now", "idle updated time")

    fallback = build_idle_waypoint_payload(reason="", now_text="now")
    assert_equal(fallback["reason"], "idle", "blank idle reason falls back")


def test_result_snapshot() -> None:
    runtime = build_task_runtime_snapshot(active_payload(), state_payload(), now=100.0)
    result = build_task_result_snapshot(
        active_payload(),
        status="error",
        waypoint={"id": "p2", "label": "点2"},
        runtime_snapshot=runtime,
        now_text="now",
        message="路径终点不匹配",
        extra={
            "reason": "path_goal_mismatch",
            "nav_status": "error nav2 action unavailable",
            "path_goal_error_m": 3.2,
        },
    )
    assert_equal(result["task_id"], "task_1", "task id")
    assert_equal(result["status"], "error", "status")
    assert_equal(result["message"], "路径终点不匹配", "message")
    assert_equal(result["last_goal_attempt_id"], "goal_1", "goal attempt")
    assert_equal(result["last_floor_goal_published_at"], "2026-06-27 09:29:00", "floor goal published time")
    assert_equal(result["floor_goal_publish_count"], 1, "floor goal publish count")
    assert_equal(result["plan_goal_verified"], True, "plan verified")
    assert_equal(result["runtime_snapshot"]["last_nav_feedback"]["distance_remaining"], 1.2, "runtime snapshot")
    assert_equal(result["reason"], "path_goal_mismatch", "extra reason")
    assert_equal(result["nav_status"], "error nav2 action unavailable", "raw Nav2 status promoted")
    assert_equal(result["extra"]["nav_status"], "error nav2 action unavailable", "raw Nav2 status retained in extra")
    assert_equal(len(result["timeline_tail"]), 12, "timeline tail is capped")


def test_last_task_result_payload() -> None:
    assert_equal(last_task_result_payload([]), None, "empty tasks have no last result")
    assert_equal(last_task_result_payload("bad"), None, "non-list tasks have no last result")

    payload = last_task_result_payload(
        [
            {"id": "old", "name": "旧任务", "status": "completed", "updated_at": "2026-01-01"},
            {
                "id": "task_1",
                "name": "单层测试",
                "status": "error",
                "created_at": "2026-01-02",
                "updated_at": "2026-01-03",
                "last_result": {"status": "error", "message": "路径终点不匹配"},
                "last_timeline": [{"event": "start"}, {"event": "fail"}],
            },
            {
                "id": "task_2",
                "name": "较早错误",
                "status": "error",
                "updated_at": "2026-01-02",
                "last_error": "旧错误",
            },
        ]
    )
    assert_equal(payload["task_id"], "task_1", "latest task id")
    assert_equal(payload["task_name"], "单层测试", "latest task name")
    assert_equal(payload["last_result"]["message"], "路径终点不匹配", "last result retained")
    assert_equal(payload["last_event"], {"event": "fail"}, "last timeline event")

    error_only = last_task_result_payload(
        [{"id": "task_error", "status": "error", "created_at": "2026-01-01", "last_error": "失败"}]
    )
    assert_equal(error_only["task_id"], "task_error", "error-only task selected")
    assert_equal(error_only["last_error"], "失败", "error retained")


def test_apply_task_result_persistence() -> None:
    active = {
        "last_error": "active error",
        "timeline": [{"event": "start"}, {"event": "fail"}],
    }
    failed = apply_task_result_persistence(
        {"id": "task_1", "status": "error", "last_error": "old"},
        active,
        status="error",
        result={"status": "error"},
        message="路径终点不匹配",
        now_text="now",
    )
    assert_equal(failed["status"], "error", "status stored with result")
    assert_equal(failed["last_result"], {"status": "error"}, "result stored")
    assert_equal(failed["last_timeline"], [{"event": "start"}, {"event": "fail"}], "timeline stored")
    assert_equal(failed["last_error"], "路径终点不匹配", "message becomes last error")
    assert_equal(failed["updated_at"], "now", "updated time stored")

    fallback_error = apply_task_result_persistence(
        {"id": "task_1"},
        active,
        status="error",
        result={"status": "error"},
        message=None,
        now_text="now",
    )
    assert_equal(fallback_error["last_error"], "active error", "active last error fallback")

    completed = apply_task_result_persistence(
        {"id": "task_1", "last_error": "old"},
        active,
        status="completed",
        result={"status": "completed"},
        message="任务已完成",
        now_text="done",
    )
    assert_equal(completed["status"], "completed", "completed status stored")
    assert_equal(completed["last_error"], None, "completed task clears last error")
    assert_equal(completed["updated_at"], "done", "completed updated time")


def test_apply_task_result_to_tasks() -> None:
    tasks = [
        {"id": "task_1", "status": "running", "updated_at": "old"},
        {"id": "task_2", "status": "ready", "updated_at": "old"},
    ]
    active = {
        "last_error": "active error",
        "timeline": [{"event": "fail"}],
    }
    updated = apply_task_result_to_tasks(
        tasks,
        task_id="task_1",
        active=active,
        status="error",
        result={"status": "error", "task_id": "task_1"},
        message="路径终点不匹配",
        now_text="now",
    )
    assert_true(updated["ok"], "task result list update finds task")
    assert_true(updated["changed"], "task result list update changes state")
    by_id = {task["id"]: task for task in updated["tasks"]}
    assert_equal(by_id["task_1"]["status"], "error", "target status updated")
    assert_equal(by_id["task_1"]["last_error"], "路径终点不匹配", "target error updated")
    assert_equal(by_id["task_1"]["last_result"]["task_id"], "task_1", "target result stored")
    assert_equal(by_id["task_2"]["status"], "ready", "other task retained")
    assert_equal(updated["task"]["id"], "task_1", "updated task returned")
    assert_equal(tasks[0]["status"], "running", "original task list retained")

    missing = apply_task_result_to_tasks(
        tasks,
        task_id="missing",
        active=active,
        status="error",
        result={"status": "error"},
        message="失败",
        now_text="now",
    )
    assert_true(not missing["ok"], "missing task result update fails")
    assert_true(not missing["changed"], "missing task result update unchanged")
    assert_equal(missing["tasks"], tasks, "missing task keeps list value")

    bad_tasks = apply_task_result_to_tasks(
        "bad",
        task_id="task_1",
        active=active,
        status="error",
        result={"status": "error"},
        message="失败",
        now_text="now",
    )
    assert_true(not bad_tasks["ok"], "non-list tasks fail")
    assert_equal(bad_tasks["tasks"], [], "non-list tasks return empty list")


def main() -> int:
    for test in (
        test_pose_age_sec,
        test_runtime_snapshot,
        test_active_waypoint_payload,
        test_idle_waypoint_payload,
        test_result_snapshot,
        test_last_task_result_payload,
        test_apply_task_result_persistence,
        test_apply_task_result_to_tasks,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] task snapshot contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
