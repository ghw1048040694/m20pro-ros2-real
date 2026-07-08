#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.active_task_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.active_task_contract import (  # noqa: E402
    GOAL_SENT_RESET_KEYS,
    NEXT_WAYPOINT_RESET_KEYS,
    advance_active_task_state,
    active_annotation_from_list,
    active_annotation_missing_failure,
    active_annotation_resolution,
    active_task_failure_payload,
    active_waypoint_elapsed_s,
    append_active_task_timeline_event_state,
    begin_waypoint_dwell_state,
    create_active_task_state,
    dwell_tick_decision,
    fail_active_task_state,
    goal_dispatch_decision,
    idle_stop_task_response,
    mark_floor_goal_published_state,
    mark_active_task_failed_state,
    mark_active_task_stopped_state,
    mark_active_task_waiting_state,
    mark_goal_sent,
    normalize_stop_task_request,
    prepare_goal_send_state,
    remaining_dwell_s,
    stale_goal_dispatch_payload,
    stop_task_state,
    stop_task_operator_event_payload,
    task_terminal_event_payload,
    waypoint_goal_failure_extra,
    waypoint_goal_payload,
)


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def annotation(annotation_id: str = "p1") -> dict:
    return {
        "id": annotation_id,
        "label": f"点{annotation_id}",
        "floor": "F20",
        "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.3},
    }


def active(**extra) -> dict:
    payload = {
        "task_id": "task_1",
        "status": "running",
        "index": 0,
        "annotation_ids": ["p1", "p2"],
        "phase": "navigating",
        "last_nav_goal_status": "idle",
        "total_goal_send_count": 0,
        "waypoint_goal_send_count": 0,
        "resend_goal_count": 0,
    }
    payload.update(extra)
    return payload


def test_goal_payload() -> None:
    payload = waypoint_goal_payload(annotation())
    assert_equal(payload["ok"], True, "valid goal")
    assert_equal(payload["floor"], "F20", "floor")
    assert_equal(payload["x"], 1.0, "x")

    bad_floor = waypoint_goal_payload({**annotation(), "floor": ""})
    assert_equal(bad_floor["ok"], False, "bad floor")
    assert_equal(bad_floor["reason"], "bad_waypoint_floor", "bad floor reason")

    bad_pose = annotation()
    bad_pose["pose"] = {"x": "nan", "y": 2.0, "z": 0.0, "yaw": 0.0}
    result = waypoint_goal_payload(bad_pose)
    assert_equal(result["ok"], False, "bad pose")
    assert_equal(result["reason"], "bad_waypoint_pose", "bad pose reason")

    extra = waypoint_goal_failure_extra(annotation())
    assert_equal(extra["annotation_id"], "p1", "bad waypoint extra annotation id")
    assert_equal(extra["label"], "点p1", "bad waypoint extra label")
    assert_equal(extra["pose"], {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.3}, "bad waypoint extra pose")


def test_dispatch_decision() -> None:
    ann = annotation()
    assert_equal(
        goal_dispatch_decision(active(status="completed"), ann, force=True, now_monotonic=10.0, resend_interval_s=2.0)["action"],
        "idle",
        "completed task is idle",
    )
    assert_equal(
        goal_dispatch_decision(active(last_goal_annotation_id="p1", last_nav_goal_status="accepted"), ann, force=False, now_monotonic=10.0, resend_interval_s=2.0)["action"],
        "publish_status",
        "accepted goal only republishes status",
    )
    assert_equal(
        goal_dispatch_decision(active(last_goal_annotation_id="p1", last_nav_goal_status="sent", last_goal_sent_monotonic=9.5), ann, force=False, now_monotonic=10.0, resend_interval_s=2.0)["action"],
        "publish_status",
        "sent goal before resend interval only republishes status",
    )
    resend = goal_dispatch_decision(
        active(last_goal_annotation_id="p1", last_nav_goal_status="sent", last_goal_sent_monotonic=1.0),
        ann,
        force=False,
        now_monotonic=10.0,
        resend_interval_s=2.0,
    )
    assert_equal(resend["action"], "send_goal", "old sent goal resends")
    assert_equal(resend["resend"], True, "resend flag")
    assert_equal(resend["operator_event"], "补发当前任务点", "resend operator event")
    assert_equal(resend["operator_payload"]["annotation_id"], "p1", "resend payload annotation id")
    assert_equal(resend["operator_payload"]["age_s"], 9.0, "resend payload age")
    assert_equal(resend["resend_event"], resend["operator_payload"], "legacy resend payload alias")

    stale = stale_goal_dispatch_payload(
        active(index=1),
        annotation("p1"),
        annotation("p2"),
    )
    assert_equal(stale["event"], "goal_dispatch_ignored", "stale dispatch event")
    assert_equal(stale["message"], "任务点已切换，忽略过期目标下发", "stale dispatch message")
    assert_equal(stale["event_extra"]["requested_annotation_id"], "p1", "stale requested id")
    assert_equal(stale["event_extra"]["current_annotation_id"], "p2", "stale current id")
    assert_equal(stale["event_extra"]["index"], 1, "stale active index")


def test_mark_goal_sent_new_and_resend() -> None:
    ann = annotation()
    goal = waypoint_goal_payload(ann)
    initial = active(
        has_nav_feedback=True,
        last_nav_status="old_status",
        last_nav_status_at="old_status_at",
        last_nav_feedback={"stale": True},
        last_nav_feedback_monotonic=3.0,
        last_nav_goal_seq=99,
        last_ignored_nav_goal_match={"stale": True},
        last_floor_goal_published_at="old_publish",
        last_floor_goal_pose={"x": 99.0},
        stall_warned=True,
    )
    marked_result = mark_goal_sent(
        initial,
        ann,
        goal,
        now_text="now",
        now_monotonic=12.0,
        path_version=5,
        goal_attempt_id="goal_1",
        goal_semantics={"id": "p1"},
    )
    marked = marked_result["active"]
    assert_equal(marked_result["event"], "waypoint_goal_sent", "goal sent event")
    assert_equal(marked_result["message"], "已下发当前点位，等待 Nav2 接收", "goal sent message")
    assert_equal(marked_result["event_extra"]["goal_attempt_id"], "goal_1", "goal sent event attempt")
    assert_equal(marked_result["event_extra"]["goal_sent_path_version"], 5, "goal sent event path version")
    assert_equal(marked["last_goal_annotation_id"], "p1", "goal annotation")
    assert_equal(marked["waypoint_started_monotonic"], 12.0, "waypoint start")
    assert_equal(marked["total_goal_send_count"], 1, "total sends")
    assert_equal(marked["waypoint_goal_send_count"], 1, "waypoint sends")
    assert_equal(marked["resend_goal_count"], 0, "resends")
    for key in GOAL_SENT_RESET_KEYS:
        assert_true(key not in marked, f"{key} reset on new goal")

    resent = mark_goal_sent(
        marked,
        ann,
        goal,
        now_text="later",
        now_monotonic=20.0,
        path_version=6,
        goal_attempt_id="goal_2",
        goal_semantics={"id": "p1"},
    )["active"]
    assert_equal(resent["waypoint_started_monotonic"], 12.0, "resend keeps waypoint start")
    assert_equal(resent["total_goal_send_count"], 2, "total increment")
    assert_equal(resent["waypoint_goal_send_count"], 2, "waypoint increment")
    assert_equal(resent["resend_goal_count"], 1, "resend increment")


def test_prepare_goal_send_state() -> None:
    ann = annotation()
    goal = waypoint_goal_payload(ann)

    idle = prepare_goal_send_state(
        active(status="completed"),
        ann,
        ann,
        goal,
        now_text="now",
        now_monotonic=10.0,
        path_version=1,
        goal_attempt_id="goal_1",
        goal_semantics={"id": "p1"},
    )
    assert_equal(idle["action"], "idle", "non-running task is idle")
    assert_equal(idle["reason"], "task_not_running", "idle reason")

    missing = prepare_goal_send_state(
        active(index=2, annotation_ids=["p1"]),
        ann,
        None,
        goal,
        now_text="now",
        now_monotonic=10.0,
        path_version=1,
        goal_attempt_id="goal_1",
        goal_semantics={"id": "p1"},
    )
    assert_equal(missing["action"], "fail", "missing current annotation fails")
    assert_equal(missing["failure"]["reason"], "active_waypoint_missing", "missing failure reason")

    stale = prepare_goal_send_state(
        active(index=1),
        annotation("p1"),
        annotation("p2"),
        goal,
        now_text="now",
        now_monotonic=10.0,
        path_version=1,
        goal_attempt_id="goal_1",
        goal_semantics={"id": "p1"},
    )
    assert_equal(stale["action"], "record_stale", "stale switched waypoint is recorded")
    assert_equal(stale["event"], "goal_dispatch_ignored", "stale event")
    assert_equal(stale["event_extra"]["requested_annotation_id"], "p1", "stale requested id")
    assert_equal(stale["event_extra"]["current_annotation_id"], "p2", "stale current id")

    prepared = prepare_goal_send_state(
        active(),
        ann,
        ann,
        goal,
        now_text="now",
        now_monotonic=10.0,
        path_version=7,
        goal_attempt_id="goal_1",
        goal_semantics={"id": "p1"},
    )
    assert_equal(prepared["action"], "send_goal", "valid state sends goal")
    assert_equal(prepared["event"], "waypoint_goal_sent", "prepared event")
    assert_equal(prepared["active"]["last_goal_attempt_id"], "goal_1", "prepared active attempt")
    assert_equal(prepared["event_extra"]["goal_sent_path_version"], 7, "prepared path version")


def test_mark_floor_goal_published_state() -> None:
    ann = annotation()
    goal = waypoint_goal_payload(ann)
    result = mark_floor_goal_published_state(
        active(last_goal_attempt_id="goal_1", waypoint_goal_send_count=1, total_goal_send_count=1),
        ann,
        goal,
        now_text="now",
        now_monotonic=14.5,
    )
    updated = result["active"]
    assert_true(result["changed"], "floor goal publish updates active task")
    assert_equal(result["event"], "floor_goal_published", "floor goal published event")
    assert_equal(updated["last_floor_goal_published_at"], "now", "publish time")
    assert_equal(updated["last_floor_goal_published_monotonic"], 14.5, "publish monotonic")
    assert_equal(updated["last_floor_goal_annotation_id"], "p1", "publish annotation id")
    assert_equal(updated["floor_goal_publish_count"], 1, "publish count")
    assert_equal(updated["last_floor_goal_pose"]["floor"], "F20", "publish floor")
    assert_equal(updated["last_floor_goal_source_floor"], None, "missing source floor stored as none")
    assert_equal(updated["last_floor_goal_target_floor"], "F20", "target floor stored")
    assert_equal(updated["last_floor_goal_cross_floor"], False, "same or unknown source floor is not cross-floor")
    assert_equal(updated["status_message"], "已发布 /m20pro/floor_goal，等待 floor_goal_bridge/Nav2 接收", "publish message")
    assert_equal(result["event_extra"]["goal_attempt_id"], "goal_1", "publish attempt id")
    assert_equal(result["event_extra"]["goal"]["x"], 1.0, "publish goal x")
    assert_equal(result["event_extra"]["cross_floor"], False, "event records same-floor/cross-floor state")

    cross_goal = {**goal, "floor": "F21"}
    cross = mark_floor_goal_published_state(
        active(last_goal_attempt_id="goal_2", waypoint_goal_send_count=1, total_goal_send_count=1),
        {**ann, "floor": "F21"},
        cross_goal,
        now_text="now",
        now_monotonic=15.0,
        source_floor="F20",
    )
    assert_equal(cross["active"]["last_floor_goal_source_floor"], "F20", "cross source floor stored")
    assert_equal(cross["active"]["last_floor_goal_target_floor"], "F21", "cross target floor stored")
    assert_equal(cross["active"]["last_floor_goal_cross_floor"], True, "cross-floor publish marked")
    assert_equal(cross["event_extra"]["cross_floor"], True, "cross-floor event marked")

    idle = mark_floor_goal_published_state(
        active(status="completed"),
        ann,
        goal,
        now_text="now",
        now_monotonic=14.5,
    )
    assert_true(not idle["changed"], "non-running task does not change")
    assert_equal(idle["reason"], "task_not_running", "non-running reason")


def test_create_active_task_state() -> None:
    task = {"id": "task_1", "name": "巡检", "annotation_ids": ["p1", "p2"]}
    result = create_active_task_state(
        task,
        task_map_id="map_a",
        now_text="now",
    )
    created = result["active"]
    assert_equal(result["event"], "task_started", "created timeline event")
    assert_equal(result["message"], "任务已创建，准备下发第一个点位", "created timeline message")
    assert_equal(result["event_extra"], {"task_id": "task_1", "waypoints": 2}, "created timeline extra")
    assert_equal(result["operator_event"], "启动前端任务", "created operator event")
    assert_equal(result["operator_payload"], {"task_id": "task_1"}, "created operator payload")
    assert_equal(created["task_id"], "task_1", "created task id")
    assert_equal(created["task_name"], "巡检", "created task name")
    assert_equal(created["map_id"], "map_a", "created map id")
    assert_equal(created["status"], "running", "created status")
    assert_equal(created["index"], 0, "created index")
    assert_equal(created["annotation_ids"], ["p1", "p2"], "created annotation ids")
    assert_equal(created["started_at"], "now", "created start time")
    assert_equal(created["last_nav_goal_status"], "idle", "created nav status")
    assert_equal(created["status_message"], "任务已创建，准备下发第一个点位", "created message")


def test_mark_active_task_terminal_states() -> None:
    stopped = mark_active_task_stopped_state(active(status_message="old"))
    assert_equal(stopped["status_message"], "任务已手动停止/复位", "stopped message")
    assert_equal(stopped["task_id"], "task_1", "stopped retains task")

    failed = mark_active_task_failed_state(active(status_message="old"), message="失败原因")
    assert_equal(failed["last_error"], "失败原因", "failed last error")
    assert_equal(failed["status_message"], "失败原因", "failed message")
    assert_equal(failed["task_id"], "task_1", "failed retains task")


def test_fail_active_task_state() -> None:
    result = fail_active_task_state(
        active(status_message="old"),
        message="失败原因",
        event_extra={"reason": "bad_plan"},
    )
    assert_equal(result["task_id"], "task_1", "failed task id")
    assert_equal(result["event"], "task_failed", "failed event")
    assert_equal(result["message"], "失败原因", "failed message")
    assert_equal(result["result_status"], "error", "failed result status")
    assert_equal(result["event_extra"], {"reason": "bad_plan"}, "failed event extra")
    assert_equal(result["active"]["last_error"], "失败原因", "failed last error")
    assert_equal(result["active"]["status_message"], "失败原因", "failed active message")
    assert_equal(result["operator_event"], "前端任务停止", "failed operator event")
    assert_equal(
        result["operator_payload"],
        {"task_id": "task_1", "message": "失败原因", "reason": "bad_plan"},
        "failed operator payload",
    )

    empty_extra = fail_active_task_state(active(), message="失败原因")
    assert_equal(empty_extra["event_extra"], {}, "missing extra defaults to empty dict")

    nav_failed = fail_active_task_state(
        active(),
        message="导航失败",
        event_extra={"nav_status": "Status=error"},
        terminal_event="nav_failed",
        terminal_status_text="Status=error",
    )
    assert_equal(nav_failed["operator_event"], "前端任务因导航状态停止", "nav failed operator event")
    assert_equal(nav_failed["operator_payload"], {"task_id": "task_1", "status": "Status=error"}, "nav failed operator payload")


def test_stop_task_state() -> None:
    default_request = normalize_stop_task_request({})
    assert_equal(default_request["reason"], "web_manual_stop", "default stop reason")
    assert_equal(default_request["is_reset"], False, "default stop is not reset")

    reset_request = normalize_stop_task_request({"reason": " web_manual_reset "})
    assert_equal(reset_request["reason"], "web_manual_reset", "reset reason normalized")
    assert_equal(reset_request["is_reset"], True, "reset flag")

    blank_request = normalize_stop_task_request({"reason": "   "})
    assert_equal(blank_request["reason"], "web_manual_stop", "blank reason falls back")

    stopped = stop_task_state(active(status_message="old"), reason="web_manual_stop")
    assert_equal(stopped["task_id"], "task_1", "stopped task id")
    assert_equal(stopped["result_status"], "stopped", "result status")
    assert_equal(stopped["event"], "task_stopped", "event")
    assert_equal(stopped["message"], "任务已手动停止/复位", "message")
    assert_equal(stopped["event_extra"], {"reason": "web_manual_stop"}, "reason evidence")
    assert_equal(stopped["operator_event"], "停止前端任务", "stopped operator event")
    assert_equal(stopped["operator_payload"], {"task_id": "task_1", "reason": "web_manual_stop"}, "stopped operator payload")
    assert_equal(stopped["active"]["status_message"], "任务已手动停止/复位", "active message")
    assert_equal(stopped["active"]["task_id"], "task_1", "active retained")

    reset_operator = stop_task_operator_event_payload(task_id=None, reason="web_manual_reset")
    assert_equal(reset_operator["operator_event"], "停止前端任务", "reset operator event")
    assert_equal(reset_operator["operator_payload"], {"task_id": None, "reason": "web_manual_reset"}, "reset operator payload")

    idle = idle_stop_task_response()
    assert_true(idle["ok"], "idle stop is successful noop")
    assert_equal(idle["active_task"], None, "idle stop has no active task")
    assert_equal(idle["stopped_task_id"], None, "idle stop has no stopped task")
    assert_equal(idle["reset_navigation"], False, "idle stop does not reset navigation")
    assert_equal(idle["message"], "当前没有前端任务在执行，无需停止", "idle stop message")


def test_task_terminal_event_payload() -> None:
    completed = task_terminal_event_payload(event="completed", task_id="task_1")
    assert_equal(completed["event"], "前端任务完成", "completed event name")
    assert_equal(completed["payload"], {"task_id": "task_1"}, "completed payload")

    stopped = task_terminal_event_payload(event="stopped", task_id="task_1", reason="web_manual_stop")
    assert_equal(stopped["event"], "停止前端任务", "stopped event name")
    assert_equal(stopped["payload"]["reason"], "web_manual_stop", "stopped reason")

    failed = task_terminal_event_payload(
        event="failed",
        task_id="task_1",
        message="失败",
        extra={"code": "bad_plan"},
    )
    assert_equal(failed["event"], "前端任务停止", "failed event name")
    assert_equal(failed["payload"]["message"], "失败", "failed message")
    assert_equal(failed["payload"]["code"], "bad_plan", "failed extra")

    nav_failed = task_terminal_event_payload(event="nav_failed", task_id="task_1", status_text="Status=error")
    assert_equal(nav_failed["event"], "前端任务因导航状态停止", "nav failure event name")
    assert_equal(nav_failed["payload"]["status"], "Status=error", "nav failure status")


def test_dwell_state() -> None:
    current = active()
    result = begin_waypoint_dwell_state(
        current,
        annotation("p1"),
        dwell_s=5.0,
        now_text="now",
        now_time=100.0,
        reason="nav2_goal_succeeded",
    )
    assert_true(result["changed"], "dwell changed")
    updated = result["active"]
    assert_equal(updated["phase"], "dwelling", "dwelling phase")
    assert_equal(updated["dwell_s"], 5.0, "dwell seconds")
    assert_equal(updated["dwell_until"], 105.0, "dwell until")
    assert_equal(updated["last_reached_annotation_id"], "p1", "reached id")
    assert_equal(result["event"], "waypoint_dwell_started", "dwell event")
    assert_equal(result["operator_event"], "到达点位并开始停留", "dwell operator event")
    assert_equal(result["operator_payload"]["annotation_id"], "p1", "dwell operator annotation id")
    assert_equal(result["operator_payload"]["dwell_s"], 5.0, "dwell operator seconds")
    assert_equal(dwell_tick_decision(updated, now_time=102.0)["action"], "wait", "dwell waits")
    assert_equal(dwell_tick_decision(updated, now_time=105.0)["action"], "advance", "dwell advances")
    assert_equal(remaining_dwell_s(updated, now_time=102.0), 3.0, "remaining dwell")
    assert_equal(remaining_dwell_s(updated, now_time=106.0), 0.0, "remaining dwell clamps to zero")

    zero = begin_waypoint_dwell_state(
        current,
        annotation("p1"),
        dwell_s=0.0,
        now_text="now",
        now_time=100.0,
        reason="nav2_goal_succeeded",
    )
    assert_true(not zero["changed"], "zero dwell unchanged")


def test_active_waypoint_elapsed_s() -> None:
    assert_equal(active_waypoint_elapsed_s(active(), now_monotonic=10.0), None, "missing start")
    assert_equal(
        active_waypoint_elapsed_s(active(waypoint_started_monotonic=7.5), now_monotonic=10.0),
        2.5,
        "elapsed seconds",
    )
    assert_equal(
        active_waypoint_elapsed_s(active(waypoint_started_monotonic="bad"), now_monotonic=10.0),
        None,
        "bad start",
    )


def test_append_active_task_timeline_event_state() -> None:
    current = active(
        index=2,
        phase="navigating",
        last_nav_goal_status="accepted",
        waypoint_started_monotonic=10.0,
        timeline=[
            {"event": "old_1"},
            {"event": "old_2"},
        ],
    )
    updated = append_active_task_timeline_event_state(
        current,
        event="nav_feedback",
        message="正在导航",
        now_text="now",
        now_monotonic=13.5,
        max_events=2,
        extra={"distance_m": 1.2},
    )
    assert_equal([item["event"] for item in updated["timeline"]], ["old_2", "nav_feedback"], "timeline trimmed")
    item = updated["last_timeline_event"]
    assert_equal(item["event"], "nav_feedback", "event stored")
    assert_equal(item["message"], "正在导航", "message stored")
    assert_equal(item["time"], "now", "time stored")
    assert_equal(item["index"], 2, "index stored")
    assert_equal(item["phase"], "navigating", "phase stored")
    assert_equal(item["nav_goal_status"], "accepted", "nav status stored")
    assert_equal(item["elapsed_s"], 3.5, "elapsed stored")
    assert_equal(item["distance_m"], 1.2, "extra merged")
    assert_true(current["timeline"][-1]["event"] == "old_2", "original active retained")

    no_elapsed = append_active_task_timeline_event_state(
        active(),
        event="wait",
        message="waiting",
        now_text="now",
        now_monotonic=20.0,
        max_events=0,
    )
    assert_true("elapsed_s" not in no_elapsed["last_timeline_event"], "missing start omits elapsed")
    assert_equal(len(no_elapsed["timeline"]), 1, "minimum timeline limit")


def test_active_annotation_missing_failure() -> None:
    missing = active_annotation_missing_failure(active(index=1, annotation_ids=["p1"]))
    assert_equal(missing["action"], "fail", "missing active annotation fails")
    assert_equal(missing["reason"], "active_waypoint_missing", "missing active annotation reason")
    assert_equal(missing["annotation_id"], None, "out-of-range annotation id")
    assert_equal(missing["index"], 1, "out-of-range index")

    deleted = active_annotation_missing_failure(active(index=0, annotation_ids=["p_deleted"]))
    assert_equal(deleted["annotation_id"], "p_deleted", "deleted annotation id retained")
    assert_equal(deleted["task_id"], "task_1", "task id retained")


def test_active_annotation_resolution() -> None:
    resolved = active_annotation_resolution(active(index=1, annotation_ids=["p1", "p2"]))
    assert_equal(resolved["ok"], True, "active annotation resolves")
    assert_equal(resolved["index"], 1, "active annotation index")
    assert_equal(resolved["annotation_id"], "p2", "active annotation id")
    assert_equal(resolved["annotation_ids"], ["p1", "p2"], "active annotation ids")

    out_of_range = active_annotation_resolution(active(index=5, annotation_ids=["p1"]))
    assert_equal(out_of_range["ok"], False, "out-of-range annotation fails")
    assert_equal(out_of_range["annotation_id"], None, "out-of-range id")
    assert_equal(out_of_range["index"], 5, "out-of-range index retained")

    bad_index = active_annotation_resolution(active(index="bad", annotation_ids=["p1"]))
    assert_equal(bad_index["ok"], True, "bad index falls back to first waypoint")
    assert_equal(bad_index["index"], 0, "bad index fallback")
    assert_equal(bad_index["annotation_id"], "p1", "bad index annotation")


def test_active_annotation_from_list() -> None:
    items = [annotation("p1"), annotation("p2")]
    resolved = active_annotation_from_list(active(index=1, annotation_ids=["p1", "p2"]), items)
    assert_equal(resolved["id"], "p2", "list lookup resolves current annotation")
    resolved["label"] = "modified"
    assert_true(items[1]["label"] != "modified", "list lookup returns a copy")

    deleted = active_annotation_from_list(active(index=1, annotation_ids=["p1", "p_deleted"]), items)
    assert_equal(deleted, None, "missing annotation returns none")

    out_of_range = active_annotation_from_list(active(index=3, annotation_ids=["p1"]), items)
    assert_equal(out_of_range, None, "out-of-range annotation returns none")


def test_active_task_failure_payload() -> None:
    failure = {
        "action": "fail",
        "task_id": "task_1",
        "message": "stop now",
        "reason": "bad_state",
        "value": 3,
    }
    payload = active_task_failure_payload(
        failure,
        default_message="fallback",
        task_id="task_override",
        extra={"annotation_id": "p1"},
    )
    assert_equal(payload["task_id"], "task_override", "explicit task id wins")
    assert_equal(payload["message"], "stop now", "message preserved")
    assert_true("action" not in payload["extra"], "action removed from extra")
    assert_equal(payload["extra"]["reason"], "bad_state", "reason retained")
    assert_equal(payload["extra"]["annotation_id"], "p1", "extra merged")

    fallback = active_task_failure_payload({"task_id": "task_2"}, default_message="fallback")
    assert_equal(fallback["task_id"], "task_2", "failure task id used")
    assert_equal(fallback["message"], "fallback", "default message used")

    implicit_default = active_task_failure_payload({"task_id": "task_3"})
    assert_equal(implicit_default["task_id"], "task_3", "implicit default task id")
    assert_equal(implicit_default["message"], "任务执行失败，已停止任务", "implicit default message used")

    missing_waypoint = active_task_failure_payload(active_annotation_missing_failure(active(index=4)))
    assert_equal(
        missing_waypoint["message"],
        "当前任务点位已删除或索引越界，已停止任务；请重新生成任务",
        "missing waypoint contract message is enough for web helper",
    )


def test_mark_active_task_waiting_state() -> None:
    result = mark_active_task_waiting_state(
        active(status_message="old"),
        code="pose_stale",
        message="等待定位恢复",
        now_text="now",
    )
    updated = result["active"]
    assert_true(result["changed"], "wait state changed")
    assert_true(result["should_record_event"], "first wait records event")
    assert_equal(result["event"], "task_waiting", "wait event name")
    assert_equal(result["event_extra"], {"code": "pose_stale"}, "wait event extra")
    assert_equal(updated["last_wait_code"], "pose_stale", "wait code")
    assert_equal(updated["status_message"], "等待定位恢复", "wait message")
    assert_equal(updated["last_wait_at"], "now", "wait timestamp")
    assert_equal(updated["task_id"], "task_1", "active fields retained")

    repeated = mark_active_task_waiting_state(
        updated,
        code="pose_stale",
        message="等待定位恢复",
        now_text="later",
    )
    assert_true(not repeated["should_record_event"], "same wait does not spam timeline")
    assert_equal(repeated["active"]["last_wait_at"], "later", "same wait refreshes timestamp")

    changed_reason = mark_active_task_waiting_state(
        updated,
        code="wrong_floor",
        message="等待楼层切换",
        now_text="later",
    )
    assert_true(changed_reason["should_record_event"], "changed wait reason records event")
    assert_equal(changed_reason["event_extra"], {"code": "wrong_floor"}, "changed wait event extra")


def test_advance_active_task_state() -> None:
    current = active(
        last_goal_annotation_id="p1",
        waypoint_goal_send_count=3,
        has_nav_feedback=True,
        last_nav_status="old_status",
        last_nav_feedback={"stale": True},
        last_nav_feedback_monotonic=3.0,
        last_nav_goal_seq=99,
        stall_warned=True,
        goal_sent_path_version=8,
    )
    result = advance_active_task_state(current, annotation("p1"), now_text="now")
    assert_true(result["changed"], "advance changed")
    assert_true(not result["completed"], "not completed")
    updated = result["active"]
    assert_equal(updated["index"], 1, "next index")
    assert_equal(updated["last_nav_goal_status"], "idle", "nav reset")
    assert_equal(updated["waypoint_goal_send_count"], 0, "send count reset")
    for key in NEXT_WAYPOINT_RESET_KEYS:
        assert_true(key not in updated, f"{key} reset on advance")

    completed = advance_active_task_state(updated, annotation("p2"), now_text="done")
    assert_true(completed["completed"], "completed")
    assert_equal(completed["active"]["status"], "completed", "status completed")
    assert_equal(completed["task_id"], "task_1", "completed task id")
    assert_equal(completed["event"], "task_completed", "completed event")
    assert_equal(completed["message"], "任务已完成", "completed message")
    assert_equal(completed["result_status"], "completed", "completed result status")
    assert_equal(completed["event_extra"]["last_annotation_id"], "p2", "completed last annotation")
    assert_equal(completed["operator_event"], "前端任务完成", "completed operator event")
    assert_equal(completed["operator_payload"], {"task_id": "task_1"}, "completed operator payload")


def main() -> int:
    tests = [
        test_goal_payload,
        test_dispatch_decision,
        test_mark_goal_sent_new_and_resend,
        test_prepare_goal_send_state,
        test_mark_floor_goal_published_state,
        test_create_active_task_state,
        test_mark_active_task_terminal_states,
        test_fail_active_task_state,
        test_stop_task_state,
        test_task_terminal_event_payload,
        test_dwell_state,
        test_active_waypoint_elapsed_s,
        test_append_active_task_timeline_event_state,
        test_active_annotation_missing_failure,
        test_active_annotation_resolution,
        test_active_annotation_from_list,
        test_active_task_failure_payload,
        test_mark_active_task_waiting_state,
        test_advance_active_task_state,
    ]
    for test in tests:
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] active task contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
