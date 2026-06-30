#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.task_progress_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.task_progress_contract import (  # noqa: E402
    active_task_distance_decision,
    active_task_pre_dispatch_decision,
    active_task_tick_gate_decision,
    apply_localization_lost_start_state,
    apply_near_goal_wait_state,
    apply_stall_warning_state,
    goal_accept_timeout_decision,
    localization_lost_failure_extra,
    localization_lost_start_event_payload,
    localization_lost_timeout_decision,
    near_goal_wait_decision,
    near_goal_timeout_decision,
    prepare_near_goal_wait_update,
    stall_failure_extra,
    stall_warning_event_payload,
    task_stall_decision,
    timeout_failure_extra,
    update_active_task_progress_state,
    waypoint_timeout_decision,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def annotation() -> dict:
    return {"id": "p1", "label": "点1", "pose": {"x": 5.0, "y": 0.0, "yaw": 0.0}}


def active(**extra) -> dict:
    payload = {
        "task_id": "task_1",
        "status": "running",
        "last_goal_annotation_id": "p1",
        "last_nav_goal_status": "accepted",
        "last_nav_status": "nav_goal_feedback label=floor_goal",
        "last_nav_feedback": {"distance_remaining": 3.0},
    }
    payload.update(extra)
    return payload


def test_progress_initializes_reference() -> None:
    result = update_active_task_progress_state(
        active(last_goal_annotation_id="p0"),
        annotation(),
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        distance=5.0,
        navigation_status="nav",
        now_monotonic=10.0,
        now_text="now",
        goal_tolerance_m=0.3,
        min_pose_movement_m=0.08,
        min_distance_delta_m=0.12,
    )
    updated = result["active"]
    assert_true(result["made_progress"], "first sample is progress")
    assert_equal(updated["last_progress_monotonic"], 10.0, "progress time")
    assert_equal(updated["status_message"], "准备下发当前点位", "pre-dispatch status")


def test_tick_gate_decisions() -> None:
    missing_pose = active_task_tick_gate_decision(
        pose={},
        annotation=annotation(),
        current_floor="F20",
        localization_ok=True,
        pose_age=None,
        pose_timeout_s=2.0,
    )
    assert_equal(missing_pose["action"], "wait_and_monitor_localization", "missing pose waits")
    assert_equal(missing_pose["reason"], "no_pose", "missing pose reason")

    lost = active_task_tick_gate_decision(
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        annotation=annotation(),
        current_floor="F20",
        localization_ok=False,
        pose_age=0.2,
        pose_timeout_s=2.0,
    )
    assert_equal(lost["code"], "localization_lost", "localization loss code")

    stale = active_task_tick_gate_decision(
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        annotation=annotation(),
        current_floor="F20",
        localization_ok=True,
        pose_age=3.0,
        pose_timeout_s=2.0,
    )
    assert_equal(stale["reason"], "pose_stale", "stale pose reason")

    wrong_floor = active_task_tick_gate_decision(
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        annotation={**annotation(), "floor": "F21"},
        current_floor="F20",
        localization_ok=True,
        pose_age=0.2,
        pose_timeout_s=2.0,
    )
    assert_equal(wrong_floor["action"], "wait", "wrong floor waits without localization timeout")
    assert_equal(wrong_floor["target_floor"], "F21", "wrong floor target")

    ready = active_task_tick_gate_decision(
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        annotation={**annotation(), "floor": "F20"},
        current_floor="F20",
        localization_ok=True,
        pose_age=0.2,
        pose_timeout_s=2.0,
    )
    assert_equal(ready["action"], "pass", "ready tick gate passes")


def test_distance_decision() -> None:
    ready = active_task_distance_decision(
        pose={"x": 1.0, "y": 2.0, "yaw": 0.0},
        annotation={"id": "p1", "label": "点1", "pose": {"x": 4.0, "y": 6.0}},
    )
    assert_equal(ready["action"], "pass", "distance ready action")
    assert_equal(ready["reason"], "distance_ready", "distance ready reason")
    assert_equal(ready["distance_m"], 5.0, "distance calculated")

    bad_robot_pose = active_task_distance_decision(
        pose={"x": "bad", "y": 2.0},
        annotation={"id": "p1", "label": "点1", "pose": {"x": 4.0, "y": 6.0}},
    )
    assert_equal(bad_robot_pose["action"], "wait_and_monitor_localization", "bad robot pose waits")
    assert_equal(bad_robot_pose["reason"], "pose_invalid", "bad robot pose reason")

    bad_goal_pose = active_task_distance_decision(
        pose={"x": 1.0, "y": 2.0},
        annotation={"id": "p1", "label": "点1", "pose": {"x": "bad", "y": 6.0}},
    )
    assert_equal(bad_goal_pose["action"], "fail", "bad goal pose fails")
    assert_equal(bad_goal_pose["reason"], "active_waypoint_pose_invalid", "bad goal pose reason")
    assert_equal(bad_goal_pose["annotation_id"], "p1", "bad goal annotation id")

    non_finite = active_task_distance_decision(
        pose={"x": float("nan"), "y": 2.0},
        annotation={"id": "p1", "label": "点1", "pose": {"x": 4.0, "y": 6.0}},
    )
    assert_equal(non_finite["action"], "wait_and_monitor_localization", "non-finite pose waits")


def test_pre_dispatch_decision() -> None:
    lost = active_task_pre_dispatch_decision(
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        annotation={**annotation(), "floor": "F20"},
        current_floor="F20",
        localization_ok=False,
        pose_age=0.1,
        pose_timeout_s=2.0,
    )
    assert_equal(lost["action"], "wait_and_monitor_localization", "lost localization waits")
    assert_equal(lost["stage"], "tick_gate", "lost localization stage")

    wrong_floor = active_task_pre_dispatch_decision(
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        annotation={**annotation(), "floor": "F21"},
        current_floor="F20",
        localization_ok=True,
        pose_age=0.1,
        pose_timeout_s=2.0,
    )
    assert_equal(wrong_floor["action"], "wait", "wrong floor waits")
    assert_equal(wrong_floor["stage"], "tick_gate", "wrong floor stage")

    bad_goal = active_task_pre_dispatch_decision(
        pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        annotation={"id": "p1", "label": "点1", "pose": {"x": "bad", "y": 0.0}},
        current_floor="F20",
        localization_ok=True,
        pose_age=0.1,
        pose_timeout_s=2.0,
    )
    assert_equal(bad_goal["action"], "fail", "invalid active waypoint fails")
    assert_equal(bad_goal["stage"], "distance", "invalid active waypoint stage")
    assert_equal(bad_goal["reason"], "active_waypoint_pose_invalid", "invalid active waypoint reason")

    ready = active_task_pre_dispatch_decision(
        pose={"x": 1.0, "y": 2.0, "yaw": 0.0},
        annotation={"id": "p1", "label": "点1", "floor": "F20", "pose": {"x": 4.0, "y": 6.0}},
        current_floor="F20",
        localization_ok=True,
        pose_age=0.1,
        pose_timeout_s=2.0,
    )
    assert_equal(ready["action"], "pass", "pre-dispatch ready passes")
    assert_equal(ready["reason"], "pre_dispatch_ready", "pre-dispatch ready reason")
    assert_equal(ready["stage"], "ready", "pre-dispatch ready stage")
    assert_equal(ready["distance_m"], 5.0, "pre-dispatch distance")


def test_progress_detects_stall_and_recovery() -> None:
    stalled = update_active_task_progress_state(
        active(last_progress_monotonic=10.0, last_progress_pose={"x": 0.0, "y": 0.0, "yaw": 0.0}, last_progress_distance_m=5.0),
        annotation(),
        {"x": 0.01, "y": 0.0, "yaw": 0.01},
        distance=4.95,
        navigation_status="nav",
        now_monotonic=30.0,
        now_text="later",
        goal_tolerance_m=0.3,
        min_pose_movement_m=0.08,
        min_distance_delta_m=0.12,
    )["active"]
    assert_equal(stalled["stall_started_monotonic"], 10.0, "stall starts from last progress")
    assert_equal(stalled["stall_age_s"], 20.0, "stall age")

    recovered = update_active_task_progress_state(
        stalled,
        annotation(),
        {"x": 0.2, "y": 0.0, "yaw": 0.01},
        distance=4.7,
        navigation_status="nav",
        now_monotonic=35.0,
        now_text="recovered",
        goal_tolerance_m=0.3,
        min_pose_movement_m=0.08,
        min_distance_delta_m=0.12,
    )["active"]
    assert_equal(recovered["stall_started_monotonic"], 0.0, "stall reset after progress")
    assert_true("stall_age_s" not in recovered, "stall age cleared")


def test_stall_decision() -> None:
    assert_equal(task_stall_decision(active(), distance=2.0, now_monotonic=20.0, warn_timeout_s=5.0, stop_timeout_s=10.0)["reason"], "not_stalled", "not stalled")
    warn = task_stall_decision(active(stall_started_monotonic=10.0), distance=2.0, now_monotonic=17.0, warn_timeout_s=5.0, stop_timeout_s=10.0)
    assert_equal(warn["action"], "warn", "stall warning")
    fail = task_stall_decision(active(stall_started_monotonic=10.0), distance=2.0, now_monotonic=25.0, warn_timeout_s=5.0, stop_timeout_s=10.0)
    assert_equal(fail["action"], "fail", "stall fail")
    assert_equal(fail["reason"], "waypoint_stalled", "stall reason")


def test_apply_stall_warning_state() -> None:
    updated = apply_stall_warning_state(
        active(status_message="old"),
        {"message": "当前点位 7 秒内进展很小，继续观察；若持续到 10 秒会停止任务"},
    )
    assert_true(updated["stall_warned"], "stall warning flag")
    assert_equal(
        updated["status_message"],
        "当前点位 7 秒内进展很小，继续观察；若持续到 10 秒会停止任务",
        "stall warning message",
    )

    fallback = apply_stall_warning_state(active(), {})
    assert_equal(fallback["status_message"], "当前点位进展过慢，继续观察", "fallback stall warning message")


def test_stall_warning_event_payload() -> None:
    updated = active(
        task_id="task_1",
        status_message="当前点位 7 秒内进展很小，继续观察；若持续到 10 秒会停止任务",
        last_nav_status="nav_goal_feedback label=floor_goal",
        last_nav_feedback={"distance_remaining": 2.0},
    )
    event = stall_warning_event_payload(
        updated,
        annotation(),
        {"distance_m": 2.5, "stall_age_s": 7.0},
    )
    assert_equal(event["timeline_event"], "waypoint_stall_warning", "timeline event name")
    assert_equal(event["timeline_message"], updated["status_message"], "timeline message")
    assert_equal(event["timeline_extra"]["annotation_id"], "p1", "timeline annotation id")
    assert_equal(event["timeline_extra"]["distance_m"], 2.5, "timeline distance")
    assert_equal(event["timeline_extra"]["last_nav_feedback"], {"distance_remaining": 2.0}, "timeline feedback")
    assert_equal(event["operator_event"], "任务点位进展过慢", "operator event name")
    assert_equal(event["operator_payload"]["task_id"], "task_1", "operator task id")
    assert_equal(event["operator_payload"]["stall_age_s"], 7.0, "operator stall age")

    extra = stall_failure_extra(annotation())
    assert_equal(extra["annotation_id"], "p1", "stall failure extra annotation id")
    assert_equal(extra["label"], "点1", "stall failure extra label")


def test_localization_lost_timeout_decision() -> None:
    start = localization_lost_timeout_decision(
        active(),
        reason="pose_stale",
        now_monotonic=10.0,
        timeout_s=3.0,
    )
    assert_equal(start["action"], "start_timer", "localization loss starts timer")
    assert_equal(start["started_monotonic"], 10.0, "timer start time")

    wait = localization_lost_timeout_decision(
        active(localization_lost_started_monotonic=10.0),
        reason="pose_stale",
        now_monotonic=12.0,
        timeout_s=3.0,
    )
    assert_equal(wait["action"], "wait", "localization loss waits under timeout")

    fail = localization_lost_timeout_decision(
        active(localization_lost_started_monotonic=10.0),
        reason="pose_stale",
        now_monotonic=14.0,
        timeout_s=3.0,
    )
    assert_equal(fail["action"], "fail", "localization loss fails after timeout")
    assert_equal(fail["reason"], "pose_stale", "localization fail reason")

    extra = localization_lost_failure_extra(fail)
    assert_equal(extra["localization_lost_age_s"], 4.0, "localization loss failure extra age")


def test_apply_localization_lost_start_state() -> None:
    result = apply_localization_lost_start_state(
        active(status_message="old"),
        {
            "reason": "pose_stale",
            "started_monotonic": 10.0,
            "timeout_s": 3.0,
            "message": "定位/位姿暂时丢失，3.0 秒内未恢复将停止任务",
        },
        fallback_monotonic=99.0,
    )
    updated = result["active"]
    assert_true(result["changed"], "localization loss start reports changed")
    assert_equal(updated["localization_lost_started_monotonic"], 10.0, "decision start time")
    assert_equal(updated["status_message"], "定位/位姿暂时丢失，3.0 秒内未恢复将停止任务", "decision message")

    repeated = apply_localization_lost_start_state(
        updated,
        {
            "reason": "pose_stale",
            "started_monotonic": 10.0,
            "timeout_s": 3.0,
            "message": "定位/位姿暂时丢失，3.0 秒内未恢复将停止任务",
        },
        fallback_monotonic=99.0,
    )
    assert_true(not repeated["changed"], "same localization loss start does not spam timeline")

    fallback = apply_localization_lost_start_state(active(), {}, fallback_monotonic=99.0)["active"]
    assert_equal(fallback["localization_lost_started_monotonic"], 99.0, "fallback start time")
    assert_equal(fallback["status_message"], "定位/位姿暂时丢失", "fallback message")


def test_localization_lost_start_event_payload() -> None:
    decision = {
        "reason": "pose_stale",
        "timeout_s": 3.0,
        "message": "定位/位姿暂时丢失，3.0 秒内未恢复将停止任务",
    }
    event = localization_lost_start_event_payload(
        active(
            status_message="定位/位姿暂时丢失，3.0 秒内未恢复将停止任务",
            localization_lost_started_monotonic=10.0,
        ),
        decision,
    )
    assert_equal(event["event"], "localization_lost_waiting", "localization loss timeline event")
    assert_true("定位/位姿暂时丢失" in event["message"], "localization loss event message")
    assert_equal(event["extra"]["reason"], "pose_stale", "localization loss event reason")
    assert_equal(event["extra"]["timeout_s"], 3.0, "localization loss event timeout")
    assert_equal(event["extra"]["started_monotonic"], 10.0, "localization loss event start time")

    fallback = localization_lost_start_event_payload({}, decision)
    assert_equal(fallback["message"], decision["message"], "localization loss event falls back to decision message")


def test_goal_accept_timeout_decision() -> None:
    different_goal = goal_accept_timeout_decision(
        active(last_goal_annotation_id="p0"),
        annotation(),
        now_monotonic=50.0,
        timeout_s=10.0,
    )
    assert_equal(different_goal["reason"], "different_goal", "different goal ignored")

    accepted = goal_accept_timeout_decision(
        active(last_nav_goal_status="accepted", waypoint_started_monotonic=10.0),
        annotation(),
        now_monotonic=50.0,
        timeout_s=10.0,
    )
    assert_equal(accepted["reason"], "nav_goal_not_sent", "accepted goal does not trigger accept timeout")

    wait = goal_accept_timeout_decision(
        active(last_nav_goal_status="sent", waypoint_started_monotonic=10.0),
        annotation(),
        now_monotonic=15.0,
        timeout_s=10.0,
    )
    assert_equal(wait["action"], "pass", "goal accept waits under timeout")

    fail = goal_accept_timeout_decision(
        active(last_nav_goal_status="sent", waypoint_started_monotonic=10.0),
        annotation(),
        now_monotonic=25.0,
        timeout_s=10.0,
    )
    assert_equal(fail["action"], "fail", "goal accept timeout fails")
    assert_equal(fail["reason"], "goal_accept_timeout", "goal accept timeout reason")
    assert_equal(fail["annotation_id"], "p1", "goal accept timeout annotation id")


def test_near_goal_wait_decision() -> None:
    not_near = near_goal_wait_decision(
        active(),
        annotation(),
        distance=1.0,
        goal_tolerance_m=0.3,
        now_monotonic=20.0,
        now_text="now",
    )
    assert_equal(not_near["action"], "dispatch_goal", "not near dispatches")
    assert_equal(not_near["reason"], "not_near_goal", "not near reason")

    unsent = near_goal_wait_decision(
        active(last_goal_annotation_id="p0"),
        annotation(),
        distance=0.2,
        goal_tolerance_m=0.3,
        now_monotonic=20.0,
        now_text="now",
    )
    assert_equal(unsent["reason"], "current_goal_not_sent", "unsent current goal dispatches")

    inactive_nav = near_goal_wait_decision(
        active(last_nav_goal_status="idle"),
        annotation(),
        distance=0.2,
        goal_tolerance_m=0.3,
        now_monotonic=20.0,
        now_text="now",
    )
    assert_equal(inactive_nav["reason"], "nav_goal_not_active", "inactive nav dispatches")

    start_wait = near_goal_wait_decision(
        active(near_goal_started_monotonic=0.0),
        annotation(),
        distance=0.2,
        goal_tolerance_m=0.3,
        now_monotonic=20.0,
        now_text="now",
    )
    assert_equal(start_wait["action"], "wait_for_nav2", "near goal waits")
    assert_true(start_wait["changed"], "near goal timer starts")
    assert_equal(start_wait["active"]["near_goal_started_monotonic"], 20.0, "near goal timer time")
    assert_equal(start_wait["active"]["near_goal_started_at"], "now", "near goal timer text")

    keep_wait = near_goal_wait_decision(
        active(near_goal_started_monotonic=10.0, near_goal_started_at="old"),
        annotation(),
        distance=0.2,
        goal_tolerance_m=0.3,
        now_monotonic=20.0,
        now_text="now",
    )
    assert_equal(keep_wait["action"], "wait_for_nav2", "near goal keeps waiting")
    assert_true(not keep_wait["changed"], "near goal timer is not reset")
    assert_equal(keep_wait["active"]["near_goal_started_monotonic"], 10.0, "near goal start kept")


def test_apply_near_goal_wait_state() -> None:
    decision = {
        "action": "wait_for_nav2",
        "active": {
            "near_goal_started_monotonic": 20.0,
            "near_goal_started_at": "now",
            "status_message": "ignored",
        },
    }
    result = apply_near_goal_wait_state({"task_id": "task_1", "status_message": "old"}, decision)
    assert_true(result["changed"], "near goal wait state reports changed")
    updated = result["active"]
    assert_equal(updated["near_goal_started_monotonic"], 20.0, "near goal monotonic copied")
    assert_equal(updated["near_goal_started_at"], "now", "near goal timestamp copied")
    assert_equal(updated["status_message"], "old", "unowned fields are retained")

    unchanged = apply_near_goal_wait_state(
        {"task_id": "task_1", "near_goal_started_monotonic": 20.0, "near_goal_started_at": "now"},
        decision,
    )
    assert_true(not unchanged["changed"], "matching near goal state is unchanged")

    ignored = apply_near_goal_wait_state({"task_id": "task_1"}, {"action": "dispatch_goal"})
    assert_true(not ignored["changed"], "non-wait decisions are ignored")
    assert_equal(ignored["active"], {"task_id": "task_1"}, "non-wait active preserved")


def test_prepare_near_goal_wait_update() -> None:
    decision = {
        "action": "wait_for_nav2",
        "active": {
            "near_goal_started_monotonic": 20.0,
            "near_goal_started_at": "now",
        },
    }
    not_running = prepare_near_goal_wait_update(
        {"task_id": "task_1", "status": "completed"},
        active(),
        decision,
    )
    assert_equal(not_running["action"], "ignore", "non-running near-goal update ignored")
    assert_equal(not_running["reason"], "task_not_running", "non-running reason")

    changed_task = prepare_near_goal_wait_update(
        {"task_id": "task_2", "status": "running"},
        active(task_id="task_1"),
        decision,
    )
    assert_equal(changed_task["action"], "ignore", "changed task ignored")
    assert_equal(changed_task["reason"], "task_changed", "changed task reason")

    update = prepare_near_goal_wait_update(
        {"task_id": "task_1", "status": "running"},
        active(task_id="task_1"),
        decision,
    )
    assert_equal(update["action"], "update", "near-goal wait update applies")
    assert_equal(update["active"]["near_goal_started_monotonic"], 20.0, "near-goal wait monotonic saved")

    no_change = prepare_near_goal_wait_update(
        {"task_id": "task_1", "status": "running", "near_goal_started_monotonic": 20.0, "near_goal_started_at": "now"},
        active(task_id="task_1"),
        decision,
    )
    assert_equal(no_change["action"], "no_change", "matching near-goal wait unchanged")


def test_timeout_decisions() -> None:
    waypoint_wait = waypoint_timeout_decision(active(waypoint_started_monotonic=10.0), distance=2.0, now_monotonic=20.0, timeout_s=30.0)
    assert_equal(waypoint_wait["action"], "pass", "waypoint waits")
    waypoint_fail = waypoint_timeout_decision(active(waypoint_started_monotonic=10.0), distance=2.0, now_monotonic=50.0, timeout_s=30.0)
    assert_equal(waypoint_fail["reason"], "waypoint_timeout", "waypoint timeout")

    near_wait = near_goal_timeout_decision(active(near_goal_started_monotonic=10.0), distance=0.2, now_monotonic=15.0, goal_tolerance_m=0.3, timeout_s=10.0)
    assert_equal(near_wait["action"], "pass", "near goal waits")
    near_fail = near_goal_timeout_decision(active(near_goal_started_monotonic=10.0), distance=0.2, now_monotonic=25.0, goal_tolerance_m=0.3, timeout_s=10.0)
    assert_equal(near_fail["reason"], "near_goal_no_nav2_result", "near goal timeout")
    not_near = near_goal_timeout_decision(active(near_goal_started_monotonic=10.0), distance=1.0, now_monotonic=25.0, goal_tolerance_m=0.3, timeout_s=10.0)
    assert_equal(not_near["reason"], "not_near_goal", "not near")


def test_timeout_failure_extra() -> None:
    extra = timeout_failure_extra(
        annotation(),
        {
            "reason": "waypoint_timeout",
            "distance_m": 2.5,
            "age_s": 31.0,
            "timeout_s": 30.0,
        },
    )
    assert_equal(extra["annotation_id"], "p1", "timeout extra annotation id")
    assert_equal(extra["label"], "点1", "timeout extra label")
    assert_equal(extra["reason"], "waypoint_timeout", "timeout extra reason")
    assert_equal(extra["distance_m"], 2.5, "timeout extra distance")
    assert_equal(extra["age_s"], 31.0, "timeout extra age")
    assert_equal(extra["timeout_s"], 30.0, "timeout extra threshold")


def main() -> int:
    for test in (
        test_progress_initializes_reference,
        test_tick_gate_decisions,
        test_distance_decision,
        test_pre_dispatch_decision,
        test_progress_detects_stall_and_recovery,
        test_stall_decision,
        test_apply_stall_warning_state,
        test_stall_warning_event_payload,
        test_localization_lost_timeout_decision,
        test_apply_localization_lost_start_state,
        test_localization_lost_start_event_payload,
        test_goal_accept_timeout_decision,
        test_near_goal_wait_decision,
        test_apply_near_goal_wait_state,
        test_prepare_near_goal_wait_update,
        test_timeout_decisions,
        test_timeout_failure_extra,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] task progress contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
