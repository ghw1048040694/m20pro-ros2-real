#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.task_contract."""

from __future__ import annotations

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.task_contract import (  # noqa: E402
    apply_deleted_annotation_to_tasks,
    apply_task_start_pre_runtime_failure_state,
    apply_task_delete,
    apply_task_name_update,
    battery_readiness_payload,
    build_task_create_record,
    current_task_readiness_payload,
    is_plausible_pose_dict,
    map_metadata_mismatch_error,
    map_relocalization_task_readiness_payload,
    normalize_startup_task_runtime_state,
    perception_readiness_payload,
    pose_distance_m,
    pose_map_bounds_error,
    pose_map_occupancy_error,
    readiness_error_payload,
    readiness_failure,
    readiness_success,
    readiness_waypoint_payload,
    apply_runtime_guard_clear_state,
    apply_runtime_guard_wait_state,
    runtime_guard_failure_extra,
    runtime_guard_lost_decision,
    runtime_guard_readiness_payload,
    runtime_guard_waiting_event_payload,
    stop_stale_running_tasks,
    task_list_filter_payload,
    task_create_static_context,
    task_readiness_pre_runtime_payload,
    task_runtime_readiness_payload,
    task_start_runtime_readiness_payload,
    task_start_static_context,
    task_status_allows_start,
    task_create_map_metadata_mismatch_payload,
    task_pose_readiness_payload,
    task_waypoint_payload,
    validate_task_annotation_order,
    validate_task_annotations_for_map,
    validate_task_create_map_selection,
    validate_task_start_expectations,
)


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def sample_task() -> dict:
    return {
        "id": "task_1",
        "annotation_ids": ["p1", "p2"],
        "map_id": "builtin_F20",
        "created_at": "2026-06-26 10:00:00",
        "updated_at": "2026-06-26 10:05:00",
    }


def sample_annotation(annotation_id: str = "p1") -> dict:
    return {
        "id": annotation_id,
        "label": "工位点",
        "floor": "F20",
        "map_id": "builtin_F20",
        "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.2},
        "manual_point_type": "task",
        "dwell_s": 5.0,
        "area": "A",
        "room": "R1",
    }


def sample_robot_pose() -> dict:
    return {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "yaw_deg": 0.0}


def sample_map(data=None) -> dict:
    return {
        "available": True,
        "width": 4,
        "height": 3,
        "resolution": 0.5,
        "origin": {"x": 1.0, "y": 2.0},
        "data": data if data is not None else [0] * 12,
    }


def test_pose_helpers_and_readiness() -> None:
    now_text = lambda: "fixed-time"
    assert_true(is_plausible_pose_dict(sample_robot_pose()), "sample robot pose is plausible")
    assert_true(not is_plausible_pose_dict({"x": 0.0}), "incomplete robot pose is invalid")
    assert_equal(pose_distance_m({"x": 0, "y": 0}, {"x": 3, "y": 4}), 5.0, "pose distance")

    ready = task_pose_readiness_payload(
        sample_robot_pose(),
        sample_annotation(),
        task_id="task_1",
        task_map_id="builtin_F20",
        selected_map_id="builtin_F20",
        localization_ok=True,
        current_floor="F20",
        navigation_status="ok",
        pose_age_sec=1.0,
        pose_timeout_s=5.0,
        require_localization_ok=True,
        warn_first_waypoint_distance_m=8.0,
        max_first_waypoint_distance_m=25.0,
        now_text=now_text,
    )
    assert_equal(ready["ready"], True, "fresh localized pose passes")
    assert_equal(ready["first_waypoint_distance_m"], math.hypot(1.0, 2.0), "first waypoint distance recorded")
    assert_equal(ready["updated_at"], "fixed-time", "pose readiness timestamp")

    unlocalized = task_pose_readiness_payload(
        sample_robot_pose(),
        sample_annotation(),
        task_id="task_1",
        task_map_id="builtin_F20",
        selected_map_id="builtin_F20",
        localization_ok=False,
        current_floor="F20",
        navigation_status="ok",
        pose_age_sec=1.0,
        pose_timeout_s=5.0,
        require_localization_ok=True,
        warn_first_waypoint_distance_m=8.0,
        max_first_waypoint_distance_m=25.0,
        now_text=now_text,
    )
    assert_equal(unlocalized["code"], "localization_not_confirmed", "unlocalized pose fails")

    stale = task_pose_readiness_payload(
        sample_robot_pose(),
        sample_annotation(),
        task_id="task_1",
        task_map_id="builtin_F20",
        selected_map_id="builtin_F20",
        localization_ok=True,
        current_floor="F20",
        navigation_status="ok",
        pose_age_sec=9.0,
        pose_timeout_s=5.0,
        require_localization_ok=True,
        warn_first_waypoint_distance_m=8.0,
        max_first_waypoint_distance_m=25.0,
        now_text=now_text,
    )
    assert_equal(stale["code"], "pose_invalid_or_stale", "stale pose fails")


def test_battery_readiness_payload() -> None:
    now_text = lambda: "fixed-time"
    disabled = battery_readiness_payload(
        {},
        required=False,
        min_level=25,
        timeout_s=10.0,
        now=100.0,
        success_message="电池可用",
        now_text=now_text,
    )
    assert_equal(disabled["ready"], True, "disabled battery check passes")
    assert_equal(disabled["required"], False, "disabled battery required flag")

    ok = battery_readiness_payload(
        {"last_update": 95.0, "primary": {"level": 80}},
        required=True,
        min_level=25,
        timeout_s=10.0,
        now=100.0,
        success_message="电池可用",
        now_text=now_text,
    )
    assert_equal(ok["ready"], True, "fresh battery passes")
    assert_equal(ok["level"], 80, "battery level copied")

    missing = battery_readiness_payload(
        {"last_update": 99.0},
        required=True,
        min_level=25,
        timeout_s=10.0,
        now=100.0,
        success_message="电池可用",
        now_text=now_text,
    )
    assert_equal(missing["code"], "battery_missing", "missing battery fails")

    stale = battery_readiness_payload(
        {"last_update": 80.0, "primary": {"level": 80}},
        required=True,
        min_level=25,
        timeout_s=10.0,
        now=100.0,
        success_message="电池可用",
        now_text=now_text,
    )
    assert_equal(stale["code"], "battery_stale", "stale battery fails")

    low = battery_readiness_payload(
        {"last_update": 99.0, "primary": {"level": 10}},
        required=True,
        min_level=25,
        timeout_s=10.0,
        now=100.0,
        success_message="电池可用",
        now_text=now_text,
    )
    assert_equal(low["code"], "battery_low", "low battery fails")


def test_perception_readiness_payload() -> None:
    now_text = lambda: "fixed-time"
    disabled = perception_readiness_payload(
        {},
        {},
        require_scan=False,
        require_lidar=False,
        timeout_s=2.0,
        min_scan_ranges=20,
        min_lidar_points=1,
        now=100.0,
        success_message="感知可用",
        now_text=now_text,
    )
    assert_equal(disabled["ready"], True, "disabled perception check passes")
    assert_equal(disabled["required"], False, "disabled perception required flag")

    ok = perception_readiness_payload(
        {"last_update": 99.0, "finite_ranges": 50, "frame_id": "scan"},
        {"last_update": 99.0, "width": 10, "height": 2, "source": "relay", "frame_id": "lidar"},
        require_scan=True,
        require_lidar=True,
        timeout_s=2.0,
        min_scan_ranges=20,
        min_lidar_points=1,
        now=100.0,
        success_message="感知可用",
        now_text=now_text,
    )
    assert_equal(ok["ready"], True, "fresh scan and lidar pass")
    assert_equal(ok["checks"]["lidar_points"]["points"], 20, "lidar point count")

    scan_bad = perception_readiness_payload(
        {"last_update": 99.0, "finite_ranges": 5},
        {"last_update": 99.0, "width": 10, "height": 1},
        require_scan=True,
        require_lidar=True,
        timeout_s=2.0,
        min_scan_ranges=20,
        min_lidar_points=1,
        now=100.0,
        success_message="感知可用",
        now_text=now_text,
    )
    assert_equal(scan_bad["code"], "perception_scan_unavailable", "bad scan fails first")

    lidar_bad = perception_readiness_payload(
        {"last_update": 99.0, "finite_ranges": 50},
        {"last_update": 80.0, "width": 0, "height": 1},
        require_scan=True,
        require_lidar=True,
        timeout_s=2.0,
        min_scan_ranges=20,
        min_lidar_points=1,
        now=100.0,
        success_message="感知可用",
        now_text=now_text,
    )
    assert_equal(lidar_bad["code"], "perception_lidar_unavailable", "bad lidar fails")

    disabled = perception_readiness_payload(
        {},
        {},
        require_scan=False,
        require_lidar=False,
        timeout_s=2.0,
        min_scan_ranges=20,
        min_lidar_points=1,
        now=100.0,
        success_message="ignored",
        now_text=now_text,
    )
    assert_equal(disabled["ready"], True, "disabled perception is ready")
    assert_equal(disabled["required"], False, "disabled perception required flag")
    assert_equal(disabled["message"], "任务感知检查已关闭", "disabled perception message")


def test_runtime_guard_readiness_payload() -> None:
    now_text = lambda: "fixed-time"
    battery_ok = readiness_success("电池可用", {"level": 80}, now_text=now_text)
    perception_ok = readiness_success("感知可用", {"checks": {}}, now_text=now_text)
    ready = runtime_guard_readiness_payload(
        battery_readiness=battery_ok,
        perception_readiness=perception_ok,
        now_text=now_text,
    )
    assert_equal(ready["ready"], True, "runtime guard ready")
    assert_equal(ready["battery_readiness"]["level"], 80, "battery evidence preserved")
    assert_equal(ready["updated_at"], "fixed-time", "timestamp")

    battery_low = readiness_failure("battery_low", "电量低", {"level": 10}, now_text=now_text)
    blocked_by_battery = runtime_guard_readiness_payload(
        battery_readiness=battery_low,
        perception_readiness=perception_ok,
        now_text=now_text,
    )
    assert_equal(blocked_by_battery["ready"], False, "battery blocks runtime guard")
    assert_equal(blocked_by_battery["code"], "battery_low", "battery code preserved")
    assert_true("perception_readiness" not in blocked_by_battery, "battery failure remains primary")

    perception_lost = readiness_failure("perception_scan_unavailable", "scan 丢失", {}, now_text=now_text)
    blocked_by_perception = runtime_guard_readiness_payload(
        battery_readiness=battery_ok,
        perception_readiness=perception_lost,
        now_text=now_text,
    )
    assert_equal(blocked_by_perception["ready"], False, "perception blocks runtime guard")
    assert_equal(blocked_by_perception["code"], "perception_scan_unavailable", "perception code preserved")
    assert_equal(blocked_by_perception["battery_readiness"]["level"], 80, "battery evidence preserved")


def test_task_runtime_readiness_payload() -> None:
    now_text = lambda: "fixed-time"
    pose_ok = readiness_success(
        "pose ok",
        {
            "task_id": "task_1",
            "task_map_id": "builtin_F20",
            "selected_map_id": "builtin_F20",
            "first_waypoint_distance_m": 1.2,
        },
        now_text=now_text,
    )
    battery_ok = readiness_success("电池可用", {"level": 80}, now_text=now_text)
    perception_ok = readiness_success("感知可用", {"checks": {}}, now_text=now_text)

    map_blocked = task_runtime_readiness_payload(
        map_relocalization_readiness=readiness_failure(
            "map_relocalization_required",
            "需要按开发手册2101重定位",
            now_text=now_text,
        ),
        pose_readiness=pose_ok,
        battery_readiness=battery_ok,
        perception_readiness=perception_ok,
        success_message="runtime ok",
        now_text=now_text,
    )
    assert_equal(map_blocked["code"], "map_relocalization_required", "map relocalization failure is primary")

    pose_blocked = task_runtime_readiness_payload(
        map_relocalization_readiness=None,
        pose_readiness=readiness_failure("pose_invalid_or_stale", "pose bad", now_text=now_text),
        battery_readiness=battery_ok,
        perception_readiness=perception_ok,
        success_message="runtime ok",
        now_text=now_text,
    )
    assert_equal(pose_blocked["code"], "pose_invalid_or_stale", "pose failure is primary")

    battery_blocked = task_runtime_readiness_payload(
        map_relocalization_readiness=None,
        pose_readiness=pose_ok,
        battery_readiness=readiness_failure("battery_low", "battery low", {"level": 10}, now_text=now_text),
        perception_readiness=perception_ok,
        success_message="runtime ok",
        now_text=now_text,
    )
    assert_equal(battery_blocked["code"], "battery_low", "battery failure blocks runtime")
    assert_equal(battery_blocked["first_waypoint_distance_m"], 1.2, "pose evidence preserved on battery failure")
    assert_true("perception_readiness" not in battery_blocked, "perception is not reported when battery is primary")

    perception_blocked = task_runtime_readiness_payload(
        map_relocalization_readiness=None,
        pose_readiness=pose_ok,
        battery_readiness=battery_ok,
        perception_readiness=readiness_failure("perception_lidar_unavailable", "lidar lost", now_text=now_text),
        success_message="runtime ok",
        now_text=now_text,
    )
    assert_equal(perception_blocked["code"], "perception_lidar_unavailable", "perception failure blocks runtime")
    assert_equal(perception_blocked["battery_readiness"]["level"], 80, "battery evidence preserved")

    ready = task_runtime_readiness_payload(
        map_relocalization_readiness=None,
        pose_readiness=pose_ok,
        battery_readiness=battery_ok,
        perception_readiness=perception_ok,
        success_message="runtime ok",
        now_text=now_text,
    )
    assert_equal(ready["ready"], True, "runtime readiness passes")
    assert_equal(ready["message"], "runtime ok", "runtime success message")
    assert_equal(ready["perception_readiness"]["ready"], True, "perception evidence preserved")


def test_current_task_readiness_payload() -> None:
    now_text = lambda: "fixed-time"
    runtime_ready = readiness_success(
        "定位和地图位姿已就绪；具体任务点位请看任务列表的执行条件",
        {"selected_map_id": "builtin_F20", "pose_age_sec": 0.5},
        now_text=now_text,
    )
    nav_ready = readiness_success("nav ok", {"checks": {}}, now_text=now_text)

    running = current_task_readiness_payload(
        active_task={
            "status": "running",
            "task_id": "task_1",
            "task_name": "巡检",
            "index": 2,
            "phase": "navigating",
            "last_distance_m": 1.5,
            "last_nav_goal_status": "accepted",
            "last_nav_status": "running",
            "status_message": "正在去第 3 个点",
        },
        runtime_readiness=runtime_ready,
        nav_readiness=nav_ready,
        require_nav_ready=True,
        now_text=now_text,
    )
    assert_equal(running["code"], "task_running", "running active task blocks current readiness")
    assert_equal(running["task_id"], "task_1", "running task id preserved")
    assert_equal(running["running"], True, "running flag preserved")

    runtime_blocked = current_task_readiness_payload(
        active_task={},
        runtime_readiness=readiness_failure("battery_low", "电量低", {"level": 10}, now_text=now_text),
        nav_readiness=nav_ready,
        require_nav_ready=True,
        now_text=now_text,
    )
    assert_equal(runtime_blocked["code"], "battery_low", "runtime failure passes through")

    nav_blocked = current_task_readiness_payload(
        active_task={},
        runtime_readiness=runtime_ready,
        nav_readiness=readiness_failure("costmap_stale", "costmap stale", now_text=now_text),
        require_nav_ready=True,
        now_text=now_text,
    )
    assert_equal(nav_blocked["code"], "navigation_not_ready", "nav failure blocks current readiness")
    assert_equal(nav_blocked["navigation_readiness"]["code"], "costmap_stale", "nav evidence preserved")

    ready = current_task_readiness_payload(
        active_task={},
        runtime_readiness=runtime_ready,
        nav_readiness=nav_ready,
        require_nav_ready=True,
        now_text=now_text,
    )
    assert_equal(ready["ready"], True, "current readiness passes")
    assert_true("导航链路已就绪" in ready["message"], "ready message includes nav")
    assert_equal(ready["navigation_readiness"]["ready"], True, "ready nav evidence preserved")

    no_nav_required = current_task_readiness_payload(
        active_task={},
        runtime_readiness=runtime_ready,
        nav_readiness=None,
        require_nav_ready=False,
        now_text=now_text,
    )
    assert_equal(no_nav_required["message"], runtime_ready["message"], "runtime message preserved without nav check")


def test_runtime_guard_lost_decision() -> None:
    ready = runtime_guard_lost_decision(
        {"task_id": "task_1", "runtime_guard_lost_started_monotonic": 10.0},
        {"ready": True},
        now_monotonic=20.0,
        timeout_s=5.0,
    )
    assert_equal(ready["action"], "clear", "ready guard clears stale runtime loss state")
    assert_true("runtime_guard_lost_age_s" in ready["clear_keys"], "clear keys include age")

    guard = {"ready": False, "code": "perception_scan_unavailable", "message": "scan 丢失"}
    wait = runtime_guard_lost_decision(
        {"task_id": "task_1"},
        guard,
        now_monotonic=20.0,
        timeout_s=5.0,
    )
    assert_equal(wait["action"], "wait", "first runtime guard loss waits")
    assert_equal(wait["started_monotonic"], 20.0, "first runtime guard loss starts timer")
    assert_equal(wait["wait_code"], "runtime_perception_scan_unavailable", "wait code includes guard code")

    still_wait = runtime_guard_lost_decision(
        {"task_id": "task_1", "runtime_guard_lost_started_monotonic": 20.0},
        guard,
        now_monotonic=23.0,
        timeout_s=5.0,
    )
    assert_equal(still_wait["action"], "wait", "runtime guard waits under timeout")
    assert_equal(still_wait["age_s"], 3.0, "runtime guard age")

    failed = runtime_guard_lost_decision(
        {"task_id": "task_1", "runtime_guard_lost_started_monotonic": 20.0},
        guard,
        now_monotonic=26.0,
        timeout_s=5.0,
    )
    assert_equal(failed["action"], "fail", "runtime guard fails after timeout")
    assert_equal(failed["reason"], "runtime_guard_lost", "runtime guard fail reason")
    assert_true("scan 丢失" in failed["message"], "runtime guard fail message keeps guard message")

    extra = runtime_guard_failure_extra(guard, failed)
    assert_equal(extra["runtime_guard"], guard, "runtime guard failure extra stores guard")
    assert_equal(extra["runtime_guard_lost_age_s"], 6.0, "runtime guard failure extra stores age")


def test_apply_runtime_guard_wait_state() -> None:
    guard = {"ready": False, "code": "perception_scan_unavailable", "message": "scan 丢失"}
    decision = {
        "started_monotonic": 20.0,
        "age_s": 3.0,
        "timeout_s": 5.0,
        "wait_code": "runtime_perception_scan_unavailable",
        "message": "等待 scan 恢复",
    }
    result = apply_runtime_guard_wait_state(
        {"task_id": "task_1", "runtime_guard_lost_at": "old"},
        guard,
        decision,
        now_text="now",
        fallback_monotonic=99.0,
    )
    updated = result["active"]
    assert_true(result["changed"], "runtime wait state changed")
    assert_true(result["should_record_event"], "first runtime wait records event")
    assert_equal(updated["runtime_guard_lost_started_monotonic"], 20.0, "started from decision")
    assert_equal(updated["runtime_guard_lost_at"], "old", "first lost time preserved")
    assert_equal(updated["runtime_guard"], guard, "guard evidence stored")
    assert_equal(updated["runtime_guard_lost_age_s"], 3.0, "age stored")
    assert_equal(updated["last_wait_code"], "runtime_perception_scan_unavailable", "wait code stored")
    assert_equal(updated["last_wait_at"], "now", "wait timestamp stored")
    assert_equal(updated["status_message"], "等待 scan 恢复", "status message stored")

    repeated = apply_runtime_guard_wait_state(
        updated,
        guard,
        decision,
        now_text="later",
        fallback_monotonic=99.0,
    )
    assert_true(not repeated["should_record_event"], "same runtime wait does not spam timeline")
    assert_equal(repeated["active"]["last_wait_at"], "later", "same runtime wait still refreshes timestamp")

    changed_code = apply_runtime_guard_wait_state(
        updated,
        {"ready": False, "code": "battery_low", "message": "电量低"},
        {**decision, "wait_code": "runtime_battery_low", "message": "等待电量恢复"},
        now_text="later",
        fallback_monotonic=99.0,
    )
    assert_true(changed_code["should_record_event"], "runtime wait records changed reason")

    failed_event = apply_runtime_guard_wait_state(
        updated,
        guard,
        {**decision, "action": "fail"},
        now_text="later",
        fallback_monotonic=99.0,
    )
    assert_true(failed_event["should_record_event"], "runtime wait records failure event")

    fallback = apply_runtime_guard_wait_state(
        {"task_id": "task_1"},
        guard,
        {},
        now_text="now",
        fallback_monotonic=99.0,
    )["active"]
    assert_equal(fallback["runtime_guard_lost_started_monotonic"], 99.0, "fallback monotonic used")
    assert_equal(fallback["runtime_guard_lost_at"], "now", "lost time initialized")
    assert_equal(fallback["last_wait_code"], "runtime_guard_not_ready", "default wait code")


def test_runtime_guard_waiting_event_payload() -> None:
    guard = {"ready": False, "code": "perception_scan_unavailable", "message": "scan 丢失"}
    decision = {
        "age_s": 2.5,
        "timeout_s": 5.0,
        "message": "等待 scan 恢复",
    }
    payload = runtime_guard_waiting_event_payload(
        {"status_message": "任务执行中关键链路异常：scan 丢失"},
        guard,
        decision,
    )
    assert_equal(payload["event"], "runtime_guard_waiting", "runtime waiting event name")
    assert_true("scan 丢失" in payload["message"], "runtime waiting event message from active status")
    assert_equal(payload["extra"]["guard"], guard, "runtime waiting event stores guard evidence")
    assert_equal(payload["extra"]["age_s"], 2.5, "runtime waiting event stores age")
    assert_equal(payload["extra"]["timeout_s"], 5.0, "runtime waiting event stores timeout")

    fallback = runtime_guard_waiting_event_payload({}, guard, decision)
    assert_equal(fallback["message"], "等待 scan 恢复", "runtime waiting event falls back to decision message")


def test_apply_runtime_guard_clear_state() -> None:
    active = {
        "task_id": "task_1",
        "runtime_guard": {"ready": False},
        "runtime_guard_lost_started_monotonic": 10.0,
        "runtime_guard_lost_at": "old",
        "runtime_guard_lost_age_s": 3.0,
        "status_message": "old",
    }
    cleared = apply_runtime_guard_clear_state(
        active,
        {
            "clear_keys": [
                "runtime_guard",
                "runtime_guard_lost_started_monotonic",
                "runtime_guard_lost_at",
                "runtime_guard_lost_age_s",
            ],
        },
    )
    assert_true(cleared["changed"], "runtime guard clear reports changed")
    updated = cleared["active"]
    assert_true("runtime_guard" not in updated, "runtime guard evidence cleared")
    assert_true("runtime_guard_lost_started_monotonic" not in updated, "runtime guard timer cleared")
    assert_true("runtime_guard_lost_at" not in updated, "runtime guard lost time cleared")
    assert_true("runtime_guard_lost_age_s" not in updated, "runtime guard age cleared")
    assert_equal(updated["status_message"], "old", "unlisted status message retained")

    unchanged = apply_runtime_guard_clear_state({"task_id": "task_1"}, {"clear_keys": ["missing"]})
    assert_true(not unchanged["changed"], "missing keys do not report changed")
    assert_equal(unchanged["active"], {"task_id": "task_1"}, "unchanged active preserved")


def test_map_relocalization_task_readiness_payload() -> None:
    payload = map_relocalization_task_readiness_payload(
        {
            "map_id": "map_a",
            "map_name": "F20_TEST",
            "yaml_path": "/tmp/map.yaml",
            "reason": "startup_sync",
        },
        task_id="task_1",
        task_map_id="map_a",
        selected_map_id="map_a",
        now_text=lambda: "fixed-time",
    )
    assert_true(payload is not None, "map relocalization payload exists")
    assert_equal(payload["ready"], False, "not ready")
    assert_equal(payload["code"], "map_relocalization_required", "code")
    assert_equal(payload["task_id"], "task_1", "task id")
    assert_equal(payload["map_relocalization_required"]["reason"], "startup_sync", "reason")
    assert_equal(payload["updated_at"], "fixed-time", "timestamp")

    assert_equal(
        map_relocalization_task_readiness_payload(
            {},
            task_id=None,
            task_map_id="map_a",
            selected_map_id="map_a",
            now_text=lambda: "fixed-time",
        ),
        None,
        "empty relocalization payload is ignored",
    )


def test_readiness_payloads() -> None:
    now = lambda: "fixed-time"
    ok = readiness_success("ready", {"x": 1}, now_text=now)
    assert_equal(ok["ready"], True, "success ready")
    assert_equal(ok["code"], "ready", "success code")
    assert_equal(ok["updated_at"], "fixed-time", "success timestamp")
    assert_equal(ok["x"], 1, "success extra")

    fail = readiness_failure("bad", "broken", {"y": 2}, now_text=now)
    assert_equal(fail["ready"], False, "failure ready")
    assert_equal(fail["code"], "bad", "failure code")
    assert_equal(readiness_error_payload(fail)["task_readiness"], fail, "error payload embeds readiness")


def test_task_status_rules() -> None:
    for status in ("", None, "ready", "stopped", "completed", "error"):
        assert_true(task_status_allows_start(status), f"{status!r} should start")
    for status in ("running", "invalid", "paused"):
        assert_true(not task_status_allows_start(status), f"{status!r} should not start")


def test_waypoint_payloads() -> None:
    annotation = sample_annotation()
    readiness = readiness_waypoint_payload(annotation)
    assert_equal(readiness["id"], "p1", "readiness waypoint id")
    assert_equal(readiness["pose"]["yaw"], 0.2, "readiness yaw")

    missing = task_waypoint_payload("missing", None, 3)
    assert_equal(missing, {"id": "missing", "index": 3, "missing": True}, "missing waypoint")

    payload = task_waypoint_payload("p1", annotation, 0)
    assert_equal(payload["manual_point_type"], "task", "manual point type")
    assert_equal(payload["dwell_s"], 5.0, "dwell")
    assert_equal(payload["area"], "A", "area")


def test_pose_map_bounds_error() -> None:
    assert_equal(
        pose_map_bounds_error({"x": 1.5, "y": 2.5}, sample_map(), "任务点位"),
        None,
        "pose inside map passes",
    )
    out = pose_map_bounds_error({"x": 3.5, "y": 2.5}, sample_map(), "任务点位")
    assert_true(out is not None, "pose outside map fails")
    assert_true("不在当前地图范围内" in out["message"], "outside map message")

    unavailable = pose_map_bounds_error({"x": 1.5, "y": 2.5}, {"available": False, "message": "missing"}, "任务点位")
    assert_true(unavailable is not None, "unavailable map fails")
    assert_equal(unavailable["map_message"], "missing", "unavailable map message")


def test_pose_map_occupancy_error() -> None:
    occupied_data = [0] * 12
    occupied_data[1 * 4 + 1] = 80
    occupied = pose_map_occupancy_error({"x": 1.6, "y": 2.6}, sample_map(occupied_data), "任务点位")
    assert_true(occupied is not None, "occupied cell fails")
    assert_equal(occupied["code"], "pose_on_occupied_cell", "occupied code")

    unknown_data = [0] * 12
    unknown_data[1 * 4 + 1] = -1
    unknown = pose_map_occupancy_error({"x": 1.6, "y": 2.6}, sample_map(unknown_data), "任务点位")
    assert_true(unknown is not None, "unknown cell fails")
    assert_equal(unknown["code"], "pose_on_unknown_cell", "unknown code")

    assert_equal(
        pose_map_occupancy_error({"x": 1.1, "y": 2.1}, sample_map(), "任务点位"),
        None,
        "free cell passes",
    )


def test_map_metadata_mismatch_error() -> None:
    live = sample_map()
    selected = {**sample_map(), "map_id": "builtin_F20", "name": "F20", "floor": "F20"}
    assert_equal(map_metadata_mismatch_error(live, selected), None, "matching maps pass")

    width_mismatch = {**selected, "width": 5}
    mismatch = map_metadata_mismatch_error(live, width_mismatch)
    assert_true(mismatch is not None, "width mismatch fails")
    assert_equal(mismatch["checks"]["width"], False, "width check false")
    assert_equal(mismatch["selected_map"]["map_id"], "builtin_F20", "selected map id preserved")

    origin_mismatch = {**selected, "origin": {"x": 1.0, "y": 2.1}}
    mismatch = map_metadata_mismatch_error(live, origin_mismatch)
    assert_true(mismatch is not None, "origin mismatch fails")
    assert_equal(mismatch["checks"]["origin_y"], False, "origin y check false")

    unavailable = map_metadata_mismatch_error({"available": False}, selected)
    assert_true(unavailable is not None, "unavailable live map fails")
    assert_true("Nav2 当前 /map 不可用" in unavailable["message"], "live unavailable message")


def test_validate_task_annotations_for_map() -> None:
    now_text = lambda: "fixed-time"
    ok = validate_task_annotations_for_map(
        [sample_annotation("p1"), sample_annotation("p2")],
        "builtin_F20",
        target_map_payload=sample_map(),
        now_text=now_text,
    )
    assert_equal(ok, None, "valid same-map annotations pass")

    empty = validate_task_annotations_for_map([], "builtin_F20", now_text=now_text)
    assert_equal(empty["code"], "no_waypoint", "empty task fails")

    missing = validate_task_annotations_for_map([sample_annotation(), None], "builtin_F20", now_text=now_text)
    assert_equal(missing["code"], "missing_waypoint", "missing annotation fails")
    assert_equal(missing["missing_indices"], [1], "missing index")

    bad_map = validate_task_annotations_for_map(
        [{**sample_annotation(), "map_id": "builtin_F21"}],
        "builtin_F20",
        now_text=now_text,
    )
    assert_equal(bad_map["code"], "waypoint_map_mismatch", "map mismatch fails")

    missing_floor = validate_task_annotations_for_map(
        [{**sample_annotation(), "floor": ""}],
        "builtin_F20",
        now_text=now_text,
    )
    assert_equal(missing_floor["code"], "waypoint_floor_missing", "missing floor fails")

    mixed_floor = validate_task_annotations_for_map(
        [sample_annotation("p1"), {**sample_annotation("p2"), "floor": "F21"}],
        "builtin_F20",
        now_text=now_text,
    )
    assert_equal(mixed_floor["code"], "waypoint_floor_mixed", "mixed floor fails")

    bad_pose = validate_task_annotations_for_map(
        [{**sample_annotation(), "pose": {"x": 1.0}}],
        "builtin_F20",
        now_text=now_text,
    )
    assert_equal(bad_pose["code"], "waypoint_pose_invalid", "bad pose fails")

    out_of_map = validate_task_annotations_for_map(
        [{**sample_annotation(), "pose": {"x": 3.5, "y": 2.5, "z": 0.0, "yaw": 0.0}}],
        "builtin_F20",
        target_map_payload=sample_map(),
        now_text=now_text,
    )
    assert_equal(out_of_map["code"], "waypoint_out_of_map", "out-of-map waypoint fails")

    occupied_data = [0] * 12
    occupied_data[0] = 80
    occupied = validate_task_annotations_for_map(
        [sample_annotation()],
        "builtin_F20",
        target_map_payload=sample_map(occupied_data),
        now_text=now_text,
    )
    assert_equal(occupied["code"], "waypoint_on_occupied_cell", "occupied waypoint fails")

    unknown_data = [0] * 12
    unknown_data[0] = -1
    unknown = validate_task_annotations_for_map(
        [sample_annotation()],
        "builtin_F20",
        target_map_payload=sample_map(unknown_data),
        now_text=now_text,
    )
    assert_equal(unknown["code"], "waypoint_on_unknown_cell", "unknown waypoint fails")


def test_task_start_runtime_readiness_payload() -> None:
    now_text = lambda: "fixed-time"
    live_map = sample_map()
    robot_pose = {"x": 1.2, "y": 2.2, "z": 0.0, "yaw": 0.0, "yaw_deg": 0.0}
    runtime = readiness_success(
        "runtime ok",
        {
            "task_id": "task_1",
            "task_map_id": "builtin_F20",
            "selected_map_id": "builtin_F20",
            "first_waypoint_distance_m": 1.0,
        },
        now_text=now_text,
    )
    nav_ready = readiness_success("nav ok", {"source": "test"}, now_text=now_text)

    def call(**overrides):
        payload = {
            "first_annotation": sample_annotation(),
            "task_map_id": "builtin_F20",
            "task_id": "task_1",
            "selected_map_id": "builtin_F20",
            "runtime_readiness": runtime,
            "current_floor": "F20",
            "live_map": live_map,
            "robot_pose": robot_pose,
            "target_map_payload": live_map,
            "nav_readiness": nav_ready,
            "success_navigation_readiness": nav_ready,
            "require_current_floor_known": True,
            "require_current_floor_match": True,
            "require_pose_on_map": True,
            "require_nav_ready": True,
            "max_first_waypoint_distance_m": 25.0,
            "now_text": now_text,
        }
        payload.update(overrides)
        first_annotation = payload.pop("first_annotation")
        task_map_id = payload.pop("task_map_id")
        return task_start_runtime_readiness_payload(first_annotation, task_map_id, **payload)

    ready = call()
    assert_equal(ready["ready"], True, "ready task start runtime passes")
    assert_equal(ready["current_floor"], "F20", "ready current floor")
    assert_equal(ready["target_floor"], "F20", "ready target floor")
    assert_equal(ready["navigation_readiness"]["ready"], True, "ready nav payload")

    missing = call(first_annotation=None)
    assert_equal(missing["code"], "missing_waypoint", "missing first waypoint fails")

    runtime_failed = call(
        runtime_readiness=readiness_failure("localization_not_confirmed", "定位未确认", now_text=now_text)
    )
    assert_equal(runtime_failed["code"], "localization_not_confirmed", "runtime failure passes through")

    floor_unknown = call(current_floor=None)
    assert_equal(floor_unknown["code"], "floor_unknown", "unknown current floor fails")

    wrong_floor = call(current_floor="F21")
    assert_equal(wrong_floor["code"], "wrong_floor", "wrong floor fails")

    robot_out = call(robot_pose={"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "yaw_deg": 0.0})
    assert_equal(robot_out["code"], "current_pose_out_of_map", "robot out of map fails")

    target_out = call(
        first_annotation={**sample_annotation(), "pose": {"x": 9.0, "y": 9.0, "z": 0.0, "yaw": 0.0}}
    )
    assert_equal(target_out["code"], "target_out_of_map", "target out of map fails")

    metadata_mismatch = call(target_map_payload={**live_map, "width": 5})
    assert_equal(metadata_mismatch["code"], "map_metadata_mismatch", "metadata mismatch fails")

    too_far = call(
        runtime_readiness=readiness_success(
            "runtime ok",
            {"first_waypoint_distance_m": 30.0},
            now_text=now_text,
        )
    )
    assert_equal(too_far["code"], "first_waypoint_too_far", "far first waypoint fails")

    nav_failed = call(nav_readiness=readiness_failure("costmap_stale", "costmap stale", now_text=now_text))
    assert_equal(nav_failed["code"], "navigation_not_ready", "nav not ready fails")


def test_validate_task_create_map_selection() -> None:
    now_text = lambda: "fixed-time"
    assert_equal(
        validate_task_create_map_selection("builtin_F20", "builtin_F20", now_text=now_text),
        None,
        "current selected map can create a task",
    )

    missing = validate_task_create_map_selection("builtin_F20", None, now_text=now_text)
    assert_equal(missing["code"], "selected_map_missing", "missing selected map fails")
    assert_equal(missing["updated_at"], "fixed-time", "create map timestamp")

    live = validate_task_create_map_selection("live_map", "builtin_F20", now_text=now_text)
    assert_equal(live["code"], "live_map_task_disabled", "live-map task creation is disabled")

    mismatch = validate_task_create_map_selection("builtin_F19", "builtin_F20", now_text=now_text)
    assert_equal(mismatch["code"], "task_create_map_mismatch", "wrong selected map fails")


def test_task_create_map_metadata_mismatch_payload() -> None:
    selected_map_status = {
        "ready": False,
        "message": "网页选择地图与 Nav2 当前加载地图不一致",
        "code": "selected_map_metadata_mismatch",
    }
    result = task_create_map_metadata_mismatch_payload(
        task_map_id="builtin_F20",
        selected_map_id="builtin_F20",
        selected_map_status=selected_map_status,
        now_text=lambda: "fixed-time",
    )
    assert_equal(result["message"], "网页选择地图与 Nav2 当前加载地图不一致", "mismatch message")
    readiness = result["readiness"]
    assert_equal(readiness["code"], "task_create_map_metadata_mismatch", "mismatch code")
    assert_equal(readiness["task_map_id"], "builtin_F20", "mismatch task map")
    assert_equal(readiness["selected_map_status"], selected_map_status, "mismatch selected map status")
    assert_equal(readiness["updated_at"], "fixed-time", "mismatch timestamp")
    assert_equal(result["error_extra"]["code"], "task_create_map_metadata_mismatch", "error extra code")
    assert_equal(result["error_extra"]["task_readiness"], readiness, "error extra wraps readiness")


def test_task_create_static_context() -> None:
    now_text = lambda: "fixed-time"
    annotations = {"p1": sample_annotation("p1"), "p2": sample_annotation("p2")}
    ok = task_create_static_context(
        {"annotation_ids": ["p1", "p2"], "name": "巡检 A", "map_id": "builtin_F20"},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_true(ok["ok"], "valid task create context passes")
    assert_equal(ok["annotation_ids"], ["p1", "p2"], "create annotation ids")
    assert_equal([item["id"] for item in ok["annotations"]], ["p1", "p2"], "create annotations")
    assert_equal(ok["task_map_id"], "builtin_F20", "create task map")
    assert_equal(ok["name"], "巡检 A", "create task name")

    empty = task_create_static_context(
        {"annotation_ids": []},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_true(not empty["ok"], "empty task create fails")
    assert_equal(empty["readiness"]["code"], "task_create_no_waypoint", "empty create code")

    missing = task_create_static_context(
        {"annotation_ids": ["p1", "missing"]},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_true(not missing["ok"], "missing waypoint create fails")
    assert_equal(missing["readiness"]["code"], "task_create_missing_waypoint", "missing create code")
    assert_equal(missing["readiness"]["missing"], ["missing"], "missing create ids")

    bad_order = task_create_static_context(
        {"annotation_ids": ["charge", "p1"], "map_id": "builtin_F20"},
        {"charge": {**sample_annotation("charge"), "manual_point_type": "charge"}, "p1": sample_annotation("p1")},
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_true(not bad_order["ok"], "bad waypoint order create fails")
    assert_equal(bad_order["readiness"]["code"], "waypoint_order_invalid", "bad order create code")

    wrong_map = task_create_static_context(
        {"annotation_ids": ["p1"], "map_id": "builtin_F21"},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_true(not wrong_map["ok"], "wrong selected map create fails")
    assert_equal(wrong_map["readiness"]["code"], "task_create_map_mismatch", "wrong map create code")


def test_build_task_create_record() -> None:
    context = {
        "name": "巡检 A",
        "task_map_id": "builtin_F20",
        "annotation_ids": ["p1", "p2"],
    }
    task = build_task_create_record(context, task_id="task_new", now_text_value="fixed-time")
    assert_equal(task["id"], "task_new", "created task id")
    assert_equal(task["name"], "巡检 A", "created task name")
    assert_equal(task["map_id"], "builtin_F20", "created task map")
    assert_equal(task["annotation_ids"], ["p1", "p2"], "created annotation ids")
    assert_equal(task["status"], "ready", "created task status")
    assert_equal(task["created_at"], "fixed-time", "created timestamp")

    fallback = build_task_create_record(
        {"name": "", "task_map_id": "builtin_F20", "annotation_ids": ["p1"]},
        task_id="task_fallback",
        now_text_value="fixed-time",
    )
    assert_equal(fallback["name"], "巡检任务", "empty task name falls back")


def test_task_start_static_context() -> None:
    now_text = lambda: "fixed-time"
    annotations = {"p1": sample_annotation("p1"), "p2": sample_annotation("p2")}
    ok = task_start_static_context(
        "task_1",
        sample_task(),
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(ok["ok"], True, "valid task context passes")
    assert_equal(ok["task_map_id"], "builtin_F20", "task map id")
    assert_equal(ok["selected_map_id"], "builtin_F20", "selected map id")
    assert_equal(ok["first_annotation"]["id"], "p1", "first annotation")

    missing_task = task_start_static_context(
        "missing",
        None,
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(missing_task["ok"], False, "missing task fails")
    assert_true("任务不存在" in missing_task["error"]["message"], "missing task message")
    assert_equal(missing_task["readiness"]["code"], "task_missing", "missing task readiness")

    running = task_start_static_context(
        "task_1",
        {**sample_task(), "status": "running"},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(running["ok"], False, "running task fails")
    assert_true("任务正在执行中" in running["error"]["message"], "running message")
    assert_equal(running["readiness"]["code"], "task_status_blocked", "running readiness")

    empty = task_start_static_context(
        "task_1",
        {**sample_task(), "annotation_ids": []},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(empty["ok"], False, "empty task fails")
    assert_true("任务没有点位" in empty["error"]["message"], "empty message")
    assert_equal(empty["readiness"]["code"], "no_waypoint", "empty readiness")

    missing_waypoint = task_start_static_context(
        "task_1",
        sample_task(),
        {"p1": sample_annotation("p1")},
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(missing_waypoint["ok"], False, "missing waypoint fails")
    assert_equal(missing_waypoint["mark_task_invalid"], True, "missing waypoint invalidates task")
    assert_equal(missing_waypoint["error"]["missing"], ["p2"], "missing waypoint ids")
    assert_equal(missing_waypoint["readiness"]["code"], "missing_waypoint", "missing waypoint readiness")

    bad_order = task_start_static_context(
        "task_1",
        sample_task(),
        {
            "p1": {**sample_annotation("p1"), "manual_point_type": "charge"},
            "p2": sample_annotation("p2"),
        },
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(bad_order["ok"], False, "charge before end fails")
    assert_true("充电点必须放在任务最后" in bad_order["error"]["message"], "bad order message")
    assert_equal(bad_order["readiness"]["code"], "waypoint_order_invalid", "bad order readiness")


def test_task_readiness_pre_runtime_payload() -> None:
    now_text = lambda: "fixed-time"
    annotations = [sample_annotation("p1"), sample_annotation("p2")]
    static_context = {
        "ok": True,
        "task_id": "task_1",
        "task_map_id": "builtin_F20",
        "selected_map_id": "builtin_F20",
        "annotations": annotations,
        "first_annotation": annotations[0],
    }

    running_same = task_readiness_pre_runtime_payload(
        task_id="task_1",
        active_task={"status": "running", "task_id": "task_1"},
        static_context=static_context,
        task_validation=None,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(running_same["proceed"], False, "same running task blocks")
    assert_equal(running_same["readiness"]["code"], "task_running", "same running readiness")
    assert_true("当前任务正在执行中" in running_same["readiness"]["message"], "same running message")

    running_other = task_readiness_pre_runtime_payload(
        task_id="task_1",
        active_task={"status": "running", "task_id": "task_2"},
        static_context=static_context,
        task_validation=None,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(running_other["proceed"], False, "other running task blocks")
    assert_true("已有任务正在执行" in running_other["readiness"]["message"], "other running message")

    static_readiness = readiness_failure("task_missing", "任务不存在", {"task_id": "missing"}, now_text=now_text)
    static_failed = task_readiness_pre_runtime_payload(
        task_id="missing",
        active_task={},
        static_context={"ok": False, "readiness": static_readiness},
        task_validation=None,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(static_failed["proceed"], False, "static readiness blocks")
    assert_equal(static_failed["readiness"], static_readiness, "static readiness passes through")

    static_fallback = task_readiness_pre_runtime_payload(
        task_id="task_1",
        active_task={},
        static_context={"ok": False, "error": {"code": "bad_static", "message": "坏任务", "task_id": "task_1"}},
        task_validation=None,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(static_fallback["proceed"], False, "static fallback blocks")
    assert_equal(static_fallback["readiness"]["code"], "bad_static", "static fallback code")
    assert_equal(static_fallback["readiness"]["task_id"], "task_1", "static fallback keeps evidence")

    validation_failed = task_readiness_pre_runtime_payload(
        task_id="task_1",
        active_task={},
        static_context=static_context,
        task_validation=readiness_failure("waypoint_pose_invalid", "点位 pose 无效", now_text=now_text),
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(validation_failed["proceed"], False, "task validation blocks")
    assert_equal(validation_failed["readiness"]["code"], "waypoint_pose_invalid", "validation passes through")

    mismatch = task_readiness_pre_runtime_payload(
        task_id="task_1",
        active_task={},
        static_context={**static_context, "selected_map_id": "builtin_F20"},
        task_validation=None,
        selected_map_id="builtin_F19",
        now_text=now_text,
    )
    assert_equal(mismatch["proceed"], False, "selected map mismatch blocks")
    assert_equal(mismatch["readiness"]["code"], "selected_map_mismatch", "selected mismatch code")
    assert_equal(mismatch["readiness"]["first_waypoint"]["id"], "p1", "selected mismatch keeps first waypoint")

    ready = task_readiness_pre_runtime_payload(
        task_id="task_1",
        active_task={},
        static_context=static_context,
        task_validation=None,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(ready["proceed"], True, "pre-runtime passes")
    assert_equal(ready["task_map_id"], "builtin_F20", "pre-runtime task map")
    assert_equal(ready["selected_map_id"], "builtin_F20", "pre-runtime selected map")
    assert_equal(ready["first_annotation"]["id"], "p1", "pre-runtime first annotation")


def test_apply_task_start_pre_runtime_failure_state() -> None:
    tasks = [
        {"id": "task_1", "status": "ready", "annotation_ids": ["p1", "p2"]},
        {"id": "task_2", "status": "ready", "annotation_ids": ["p3"]},
    ]
    missing = apply_task_start_pre_runtime_failure_state(
        tasks,
        task_id="task_1",
        static_context={"mark_task_invalid": True, "last_error": "任务中存在已删除的点位，请重新生成任务"},
        task_validation=None,
        readiness=readiness_failure("missing_waypoint", "点位缺失", now_text=lambda: "fixed-time"),
        now_text_value="fixed-time",
    )
    assert_true(missing["changed"], "missing waypoint invalidates task")
    assert_equal(missing["tasks"][0]["status"], "invalid", "task status invalid")
    assert_equal(missing["tasks"][0]["updated_at"], "fixed-time", "invalid task timestamp")
    assert_equal(
        missing["tasks"][0]["last_error"],
        "任务中存在已删除的点位，请重新生成任务",
        "static context last_error wins",
    )
    assert_equal(tasks[0]["status"], "ready", "input task list is not mutated")

    validation = apply_task_start_pre_runtime_failure_state(
        tasks,
        task_id="task_2",
        static_context={"ok": True},
        task_validation=readiness_failure("waypoint_pose_invalid", "点位 pose 无效", now_text=lambda: "fixed-time"),
        readiness=readiness_failure("waypoint_pose_invalid", "点位 pose 无效", now_text=lambda: "fixed-time"),
        now_text_value="fixed-time",
    )
    assert_true(validation["changed"], "task validation failure invalidates task")
    assert_equal(validation["tasks"][1]["status"], "invalid", "validation invalid task status")
    assert_equal(validation["tasks"][1]["last_error"], "点位 pose 无效", "readiness message used as fallback")

    unchanged = apply_task_start_pre_runtime_failure_state(
        tasks,
        task_id="task_1",
        static_context={"ok": False},
        task_validation=None,
        readiness=readiness_failure("task_missing", "任务不存在", now_text=lambda: "fixed-time"),
        now_text_value="fixed-time",
    )
    assert_true(not unchanged["changed"], "non-invalidating failure does not edit tasks")
    assert_equal(unchanged["tasks"][0]["status"], "ready", "non-invalidating task retained")


def test_apply_deleted_annotation_to_tasks() -> None:
    tasks = [
        {"id": "task_a", "annotation_ids": ["p1", "p2"], "status": "ready", "updated_at": "old"},
        {"id": "task_b", "annotation_ids": ["p1"], "status": "completed", "updated_at": "old"},
        {"id": "task_c", "annotation_ids": ["p3"], "status": "running", "updated_at": "old"},
        {"id": "task_d", "annotation_ids": ["p1", "p4"], "status": "error", "updated_at": "old"},
    ]
    result = apply_deleted_annotation_to_tasks(tasks, "p1", now_text_value="fixed-time")
    assert_equal(result["changed"], True, "delete annotation changes tasks")
    assert_equal(result["affected_tasks"], ["task_a", "task_b", "task_d"], "affected task ids")
    updated = {task["id"]: task for task in result["tasks"]}
    assert_equal(updated["task_a"]["annotation_ids"], ["p2"], "removed deleted point")
    assert_equal(updated["task_a"]["status"], "ready", "ready task remains ready")
    assert_equal(updated["task_a"]["updated_at"], "fixed-time", "updated time set")
    assert_equal(updated["task_b"]["annotation_ids"], [], "single-point task emptied")
    assert_equal(updated["task_b"]["status"], "invalid", "empty task becomes invalid")
    assert_equal(updated["task_c"]["status"], "running", "unaffected running task untouched")
    assert_equal(updated["task_c"]["updated_at"], "old", "unaffected task time untouched")
    assert_equal(updated["task_d"]["annotation_ids"], ["p4"], "deleted point removed from error task")
    assert_equal(updated["task_d"]["status"], "error", "error task remains error for diagnostics")

    unchanged = apply_deleted_annotation_to_tasks(tasks, "missing", now_text_value="fixed-time")
    assert_equal(unchanged["changed"], False, "missing annotation changes nothing")
    assert_equal(unchanged["affected_tasks"], [], "missing annotation affects no tasks")
    assert_equal(unchanged["tasks"], tasks, "unchanged task list preserved by value")


def test_apply_task_name_update() -> None:
    tasks = [
        {"id": "task_a", "name": "旧名称", "updated_at": "old"},
        {"id": "task_b", "name": "其他", "updated_at": "old"},
    ]
    settings = {"active_task": {"task_id": "task_a", "task_name": "旧名称", "status": "running"}}
    result = apply_task_name_update(
        tasks,
        settings,
        task_id="task_a",
        name="新名称",
        now_text_value="fixed-time",
    )
    assert_true(result["ok"], "existing task is renamed")
    updated = {task["id"]: task for task in result["tasks"]}
    assert_equal(updated["task_a"]["name"], "新名称", "task name updated")
    assert_equal(updated["task_a"]["updated_at"], "fixed-time", "task timestamp updated")
    assert_equal(updated["task_b"]["name"], "其他", "other task unchanged")
    assert_true(result["settings_changed"], "active task name changes settings")
    assert_equal(result["code"], "task_updated", "rename success code")
    assert_equal(result["message"], "任务名称已更新", "rename success message")
    assert_equal(result["settings"]["active_task"]["task_name"], "新名称", "active task name synced")
    assert_equal(result["task"]["name"], "新名称", "updated task returned")
    assert_equal(result["updated_task_id"], "task_a", "updated task id returned")
    assert_equal(tasks[0]["name"], "旧名称", "original tasks preserved by value")

    inactive = apply_task_name_update(
        tasks,
        {"active_task": {"task_id": "task_x", "task_name": "x"}},
        task_id="task_b",
        name="B",
        now_text_value="fixed-time",
    )
    assert_true(inactive["ok"], "inactive task rename succeeds")
    assert_true(not inactive["settings_changed"], "inactive rename leaves settings unchanged")

    missing = apply_task_name_update(
        tasks,
        settings,
        task_id="missing",
        name="无",
        now_text_value="fixed-time",
    )
    assert_true(not missing["ok"], "missing task rename fails")
    assert_equal(missing["code"], "task_missing", "missing rename code")
    assert_equal(missing["message"], "任务不存在", "missing rename message")
    assert_equal(missing["task"], None, "missing task returns no task")


def test_apply_task_delete() -> None:
    tasks = [
        {"id": "task_a", "status": "ready"},
        {"id": "task_b", "status": "stopped"},
    ]
    running_same = apply_task_delete(
        tasks,
        {"active_task": {"task_id": "task_a", "status": "running"}},
        task_id="task_a",
    )
    assert_true(not running_same["ok"], "running active task cannot be deleted")
    assert_equal(running_same["code"], "task_running", "running delete code")
    assert_equal(len(running_same["tasks"]), 2, "running delete keeps tasks")

    historical_active = apply_task_delete(
        tasks,
        {"active_task": {"task_id": "task_b", "status": "stopped"}},
        task_id="task_b",
    )
    assert_true(historical_active["ok"], "historical active task can be deleted")
    assert_equal([task["id"] for task in historical_active["tasks"]], ["task_a"], "deleted task removed")
    assert_true(historical_active["settings_changed"], "deleted historical active clears settings")
    assert_equal(historical_active["settings"]["active_task"], None, "active task cleared")
    assert_equal(historical_active["deleted_task_id"], "task_b", "deleted id returned")

    normal = apply_task_delete(tasks, {"active_task": None}, task_id="task_a")
    assert_true(normal["ok"], "normal task delete succeeds")
    assert_equal([task["id"] for task in normal["tasks"]], ["task_b"], "normal delete removes only target")
    assert_true(not normal["settings_changed"], "normal delete leaves settings unchanged")

    missing = apply_task_delete(tasks, {"active_task": None}, task_id="missing")
    assert_true(not missing["ok"], "missing task delete fails")
    assert_equal(missing["code"], "task_missing", "missing delete code")
    assert_equal(missing["message"], "任务不存在", "missing delete message")


def test_stop_stale_running_tasks() -> None:
    tasks = [
        {"id": "task_active", "status": "running", "updated_at": "old"},
        {"id": "task_stale", "status": "running", "updated_at": "old"},
        {"id": "task_ready", "status": "ready", "updated_at": "old"},
    ]
    result = stop_stale_running_tasks(
        tasks,
        active_task_id="task_active",
        now_text_value="fixed-time",
    )
    assert_equal(result["changed"], True, "stale running task changes")
    assert_equal(result["stopped_task_ids"], ["task_stale"], "only stale task stopped")
    updated = {task["id"]: task for task in result["tasks"]}
    assert_equal(updated["task_active"]["status"], "running", "active running task preserved")
    assert_equal(updated["task_active"]["updated_at"], "old", "active running timestamp preserved")
    assert_equal(updated["task_stale"]["status"], "stopped", "stale running task stopped")
    assert_equal(updated["task_stale"]["updated_at"], "fixed-time", "stale running timestamp updated")
    assert_equal(updated["task_ready"]["status"], "ready", "ready task untouched")

    no_active = stop_stale_running_tasks(
        tasks,
        active_task_id=None,
        now_text_value="fixed-time",
    )
    assert_equal(no_active["stopped_task_ids"], ["task_active", "task_stale"], "all running tasks stale without active task")

    unchanged = stop_stale_running_tasks(
        [{"id": "task_ready", "status": "ready", "updated_at": "old"}],
        active_task_id="task_active",
        now_text_value="fixed-time",
    )
    assert_equal(unchanged["changed"], False, "no running tasks changes nothing")
    assert_equal(unchanged["tasks"], [{"id": "task_ready", "status": "ready", "updated_at": "old"}], "unchanged tasks")


def test_task_list_filter_payload() -> None:
    tasks = [
        {"id": "task_a", "map_id": "map_a"},
        {"id": "task_b", "map_id": "map_b"},
        {"id": "task_missing_map"},
    ]
    include_all = task_list_filter_payload(tasks, selected_map_id="map_a", include_all=True)
    assert_equal([task["id"] for task in include_all["tasks"]], ["task_a", "task_b", "task_missing_map"], "all tasks visible")
    assert_equal(include_all["include_all"], True, "include_all flag preserved")
    assert_equal(include_all["hidden_task_count"], 0, "history mode hides nothing")
    assert_equal(include_all["total_task_count"], 3, "total task count")

    current_map = task_list_filter_payload(tasks, selected_map_id="map_a", include_all=False)
    assert_equal([task["id"] for task in current_map["tasks"]], ["task_a"], "current map task visible")
    assert_equal(current_map["hidden_task_count"], 2, "non-current and missing-map tasks hidden")
    assert_equal(current_map["total_task_count"], 3, "current map total task count")

    no_selected_map = task_list_filter_payload(tasks, selected_map_id=None, include_all=False)
    assert_equal(no_selected_map["tasks"], [], "no selected map shows no current-map tasks")
    assert_equal(no_selected_map["hidden_task_count"], 3, "no selected map hides all tasks")


def test_normalize_startup_task_runtime_state() -> None:
    settings = {"active_task": {"task_id": "task_active", "status": "running"}, "selected_map_id": "map_a"}
    tasks = [
        {"id": "task_active", "status": "running", "updated_at": "old"},
        {"id": "task_stale", "status": "running", "updated_at": "old"},
        {"id": "task_ready", "status": "ready", "updated_at": "old"},
    ]
    result = normalize_startup_task_runtime_state(settings, tasks, now_text_value="fixed-time")
    assert_equal(result["changed"], True, "startup runtime state changes")
    assert_equal(result["cleared_active_task"], True, "active task cleared")
    assert_equal(result["settings"]["active_task"], None, "active task is none")
    assert_equal(result["settings"]["selected_map_id"], "map_a", "unrelated settings preserved")
    assert_equal(result["stopped_task_ids"], ["task_active", "task_stale"], "running task ids stopped")
    updated = {task["id"]: task for task in result["tasks"]}
    assert_equal(updated["task_active"]["status"], "stopped", "active running task stopped on startup")
    assert_equal(updated["task_stale"]["status"], "stopped", "stale running task stopped on startup")
    assert_equal(updated["task_active"]["updated_at"], "fixed-time", "active task timestamp updated")
    assert_equal(updated["task_ready"]["status"], "ready", "ready task preserved")

    empty_active = normalize_startup_task_runtime_state(
        {"active_task": None},
        [{"id": "task_ready", "status": "ready", "updated_at": "old"}],
        now_text_value="fixed-time",
    )
    assert_equal(empty_active["changed"], False, "no runtime residue changes nothing")
    assert_equal(empty_active["cleared_active_task"], False, "no active task cleared")
    assert_equal(empty_active["tasks"], [{"id": "task_ready", "status": "ready", "updated_at": "old"}], "tasks unchanged")


def test_start_expectations_pass() -> None:
    payload = {
        "expected_annotation_ids": ["p1", "p2"],
        "expected_first_annotation_id": "p1",
        "expected_map_id": "builtin_F20",
        "expected_task_updated_at": "2026-06-26 10:05:00",
        "expected_first_pose": {"x": 1.01, "y": 1.99, "z": 0.0, "yaw": 0.22},
    }
    assert_equal(
        validate_task_start_expectations(payload, sample_task(), sample_annotation(), "builtin_F20"),
        None,
        "matching expectations pass",
    )


def test_start_expectation_failures() -> None:
    cases = [
        (
            {"expected_annotation_ids": ["p2", "p1"]},
            "任务点顺序已变化",
            "annotation order mismatch",
        ),
        (
            {"expected_first_annotation_id": "p9"},
            "任务首点已变化",
            "first waypoint mismatch",
        ),
        (
            {"expected_map_id": "builtin_F21"},
            "任务地图已变化",
            "map mismatch",
        ),
        (
            {"expected_task_updated_at": "old"},
            "任务已被更新",
            "timestamp mismatch",
        ),
        (
            {"expected_first_pose": {"x": 1.2}},
            "任务首点坐标已变化",
            "pose mismatch",
        ),
        (
            {"expected_first_pose": {"yaw": 0.2 + math.pi * 2 + 0.2}},
            "任务首点坐标已变化",
            "yaw wraps and still applies tolerance",
        ),
    ]
    for payload, expected_message, message in cases:
        result = validate_task_start_expectations(payload, sample_task(), sample_annotation(), "builtin_F20")
        assert_true(result is not None, message)
        assert_true(expected_message in result["message"], message)


def test_charge_must_be_last() -> None:
    valid = [
        {"id": "p1", "manual_point_type": "task"},
        {"id": "charge", "manual_point_type": "charge"},
    ]
    assert_equal(validate_task_annotation_order(valid), None, "charge last is valid")

    invalid = [
        {"id": "charge", "label": "充电", "manual_point_type": "charge"},
        {"id": "p1", "manual_point_type": "task"},
    ]
    result = validate_task_annotation_order(invalid)
    assert_true(result is not None, "charge before end fails")
    assert_true("充电点必须放在任务最后" in result["message"], "charge message")


def main() -> int:
    tests = [
        test_pose_helpers_and_readiness,
        test_battery_readiness_payload,
        test_perception_readiness_payload,
        test_runtime_guard_readiness_payload,
        test_task_runtime_readiness_payload,
        test_current_task_readiness_payload,
        test_runtime_guard_lost_decision,
        test_apply_runtime_guard_wait_state,
        test_runtime_guard_waiting_event_payload,
        test_apply_runtime_guard_clear_state,
        test_map_relocalization_task_readiness_payload,
        test_readiness_payloads,
        test_task_status_rules,
        test_waypoint_payloads,
        test_pose_map_bounds_error,
        test_pose_map_occupancy_error,
        test_map_metadata_mismatch_error,
        test_validate_task_annotations_for_map,
        test_task_start_runtime_readiness_payload,
        test_validate_task_create_map_selection,
        test_task_create_map_metadata_mismatch_payload,
        test_task_create_static_context,
        test_build_task_create_record,
        test_task_start_static_context,
        test_task_readiness_pre_runtime_payload,
        test_apply_task_start_pre_runtime_failure_state,
        test_apply_deleted_annotation_to_tasks,
        test_apply_task_name_update,
        test_apply_task_delete,
        test_stop_stale_running_tasks,
        test_task_list_filter_payload,
        test_normalize_startup_task_runtime_state,
        test_start_expectations_pass,
        test_start_expectation_failures,
        test_charge_must_be_last,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] task contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
