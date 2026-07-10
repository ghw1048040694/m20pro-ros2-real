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
    build_task_create_record,
    is_plausible_pose_dict,
    map_metadata_mismatch_error,
    normalize_startup_task_runtime_state,
    pose_distance_m,
    pose_map_bounds_error,
    pose_map_occupancy_error,
    readiness_failure,
    readiness_success,
    readiness_waypoint_payload,
    stop_stale_running_tasks,
    task_list_filter_payload,
    task_create_static_context,
    task_start_static_context,
    task_status_allows_start,
    task_create_map_metadata_mismatch_payload,
    task_waypoint_payload,
    validation_error_payload,
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
        "scan_point": "P01",
        "result_file_prefix": "B03_U01_H2008_F20_R1_P01",
        "radar": {
            "enabled": True,
            "scans": [
                {"mode": "measuring", "label": "实测实量", "result_suffix": "measure"},
                {"mode": "modeling", "label": "点云建模", "result_suffix": "cloud"},
            ],
        },
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


def test_pose_helpers() -> None:
    assert_true(is_plausible_pose_dict(sample_robot_pose()), "sample robot pose is plausible")
    assert_true(not is_plausible_pose_dict({"x": 0.0}), "incomplete robot pose is invalid")
    assert_equal(pose_distance_m({"x": 0, "y": 0}, {"x": 3, "y": 4}), 5.0, "pose distance")


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
    assert_equal(validation_error_payload(fail)["validation"], fail, "error payload embeds validation")


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
    assert_equal(payload["room"], "R1", "room")
    assert_equal(payload["scan_point"], "P01", "scan point")
    assert_equal(payload["result_file_prefix"], "B03_U01_H2008_F20_R1_P01", "result prefix")
    assert_equal(payload["radar"]["enabled"], True, "radar plan")


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

    cross_floor = validate_task_annotations_for_map(
        [
            sample_annotation("p1"),
            {**sample_annotation("p2"), "floor": "F21", "map_id": "builtin_F21"},
        ],
        "builtin_F20",
        target_map_payloads={"builtin_F20": sample_map(), "builtin_F21": sample_map()},
        allow_multi_floor=True,
        allow_multi_map=True,
        now_text=now_text,
    )
    assert_equal(cross_floor, None, "explicit cross-floor annotations pass")

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
    validation = result["validation"]
    assert_equal(validation["code"], "task_create_map_metadata_mismatch", "mismatch code")
    assert_equal(validation["task_map_id"], "builtin_F20", "mismatch task map")
    assert_equal(validation["selected_map_status"], selected_map_status, "mismatch selected map status")
    assert_equal(validation["updated_at"], "fixed-time", "mismatch timestamp")
    assert_equal(result["error_extra"]["code"], "task_create_map_metadata_mismatch", "error extra code")
    assert_equal(result["error_extra"]["validation"], validation, "error extra wraps validation")


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
    assert_equal(empty["validation"]["code"], "task_create_no_waypoint", "empty create code")

    missing = task_create_static_context(
        {"annotation_ids": ["p1", "missing"]},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_true(not missing["ok"], "missing waypoint create fails")
    assert_equal(missing["validation"]["code"], "task_create_missing_waypoint", "missing create code")
    assert_equal(missing["validation"]["missing"], ["missing"], "missing create ids")

    bad_order = task_create_static_context(
        {"annotation_ids": ["charge", "p1"], "map_id": "builtin_F20"},
        {"charge": {**sample_annotation("charge"), "manual_point_type": "charge"}, "p1": sample_annotation("p1")},
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_true(not bad_order["ok"], "bad waypoint order create fails")
    assert_equal(bad_order["validation"]["code"], "waypoint_order_invalid", "bad order create code")

    wrong_map = task_create_static_context(
        {"annotation_ids": ["p1"], "map_id": "builtin_F21"},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_true(not wrong_map["ok"], "wrong selected map create fails")
    assert_equal(wrong_map["validation"]["code"], "task_create_map_mismatch", "wrong map create code")


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
    assert_equal(missing_task["validation"]["code"], "task_missing", "missing task validation")

    running = task_start_static_context(
        "task_1",
        {**sample_task(), "status": "running"},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(running["ok"], False, "running task fails")
    assert_true("任务正在执行中" in running["error"]["message"], "running message")
    assert_equal(running["validation"]["code"], "task_status_blocked", "running validation")

    empty = task_start_static_context(
        "task_1",
        {**sample_task(), "annotation_ids": []},
        annotations,
        selected_map_id="builtin_F20",
        now_text=now_text,
    )
    assert_equal(empty["ok"], False, "empty task fails")
    assert_true("任务没有点位" in empty["error"]["message"], "empty message")
    assert_equal(empty["validation"]["code"], "no_waypoint", "empty validation")

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
    assert_equal(missing_waypoint["validation"]["code"], "missing_waypoint", "missing waypoint validation")

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
    assert_equal(bad_order["validation"]["code"], "waypoint_order_invalid", "bad order validation")


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
        validation=readiness_failure("missing_waypoint", "点位缺失", now_text=lambda: "fixed-time"),
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
        validation=readiness_failure("waypoint_pose_invalid", "点位 pose 无效", now_text=lambda: "fixed-time"),
        now_text_value="fixed-time",
    )
    assert_true(validation["changed"], "task validation failure invalidates task")
    assert_equal(validation["tasks"][1]["status"], "invalid", "validation invalid task status")
    assert_equal(validation["tasks"][1]["last_error"], "点位 pose 无效", "validation message used as fallback")

    unchanged = apply_task_start_pre_runtime_failure_state(
        tasks,
        task_id="task_1",
        static_context={"ok": False},
        task_validation=None,
        validation=readiness_failure("task_missing", "任务不存在", now_text=lambda: "fixed-time"),
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
        {"id": "task_cross", "map_id": "map_a", "annotation_ids": ["p_a", "p_b"]},
        {"id": "task_missing_map"},
    ]
    annotations_by_id = {
        "p_a": {"id": "p_a", "map_id": "map_a"},
        "p_b": {"id": "p_b", "map_id": "map_b"},
    }
    include_all = task_list_filter_payload(tasks, selected_map_id="map_a", include_all=True)
    assert_equal([task["id"] for task in include_all["tasks"]], ["task_a", "task_b", "task_cross", "task_missing_map"], "all tasks visible")
    assert_equal(include_all["include_all"], True, "include_all flag preserved")
    assert_equal(include_all["hidden_task_count"], 0, "history mode hides nothing")
    assert_equal(include_all["total_task_count"], 4, "total task count")

    current_map = task_list_filter_payload(tasks, selected_map_id="map_a", include_all=False)
    assert_equal([task["id"] for task in current_map["tasks"]], ["task_a", "task_cross"], "current map task visible")
    assert_equal(current_map["hidden_task_count"], 2, "non-current and missing-map tasks hidden")
    assert_equal(current_map["total_task_count"], 4, "current map total task count")

    annotation_map = task_list_filter_payload(
        tasks,
        selected_map_id="map_b",
        include_all=False,
        annotations_by_id=annotations_by_id,
    )
    assert_equal([task["id"] for task in annotation_map["tasks"]], ["task_b", "task_cross"], "annotation map task visible")
    assert_equal(annotation_map["hidden_task_count"], 2, "annotation map hidden count")

    no_selected_map = task_list_filter_payload(tasks, selected_map_id=None, include_all=False)
    assert_equal(no_selected_map["tasks"], [], "no selected map shows no current-map tasks")
    assert_equal(no_selected_map["hidden_task_count"], 4, "no selected map hides all tasks")


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
        test_pose_helpers,
        test_readiness_payloads,
        test_task_status_rules,
        test_waypoint_payloads,
        test_pose_map_bounds_error,
        test_pose_map_occupancy_error,
        test_map_metadata_mismatch_error,
        test_validate_task_annotations_for_map,
        test_validate_task_create_map_selection,
        test_task_create_map_metadata_mismatch_payload,
        test_task_create_static_context,
        test_build_task_create_record,
        test_task_start_static_context,
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
