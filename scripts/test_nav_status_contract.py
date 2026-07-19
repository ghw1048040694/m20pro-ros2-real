#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.nav_status_contract."""

from __future__ import annotations

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.nav_status_contract import (  # noqa: E402
    apply_ignored_nav_status_state,
    apply_nav_failure_state,
    apply_nav_feedback_state,
    apply_nav_goal_status_state,
    apply_nav_status_message_state,
    apply_transition_nav_status_state,
    classify_navigation_status,
    friendly_nav_status,
    ignored_nav_status_event_payload,
    nav_feedback_dispatch_payload,
    nav_feedback_event_payload,
    nav_goal_status_event_payload,
    nav_status_message_event_payload,
    nav_status_matches_active_goal,
    nav_success_completion_decision,
    parse_key_value_status,
    should_record_nav_feedback_event,
    transition_nav_status_event_payload,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def active(**extra) -> dict:
    payload = {
        "last_goal_annotation_id": "p1",
        "last_goal_attempt_id": "goal_1",
        "last_nav_goal_seq": 7,
    }
    payload.update(extra)
    return payload


def annotation(**extra) -> dict:
    payload = {
        "id": "p1",
        "label": "点1",
        "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 3.12},
    }
    payload.update(extra)
    return payload


def test_parse_key_value_status() -> None:
    parsed = parse_key_value_status(
        "nav_goal_feedback label=floor_goal goal_seq=7 goal_x=1.0 distance_remaining=2.5 recoveries=0 note=ok none=null"
    )
    assert_equal(parsed["label"], "floor_goal", "label")
    assert_equal(parsed["goal_seq"], 7, "int")
    assert_equal(parsed["goal_x"], 1, "integer float normalized")
    assert_equal(parsed["distance_remaining"], 2.5, "float")
    assert_equal(parsed["none"], None, "null")
    factory = parse_key_value_status("location=1 obstacle=0 usage_mode=None ooa=None")
    assert_equal(factory["location"], 1, "factory navigation location")
    assert_equal(factory["obstacle"], 0, "factory navigation obstacle")
    assert_equal(factory["usage_mode"], None, "factory usage mode")
    assert_equal(factory["ooa"], None, "factory ooa")


def test_classify_navigation_status() -> None:
    assert_equal(
        classify_navigation_status("")["action"],
        "ignore",
        "empty status ignored",
    )
    assert_equal(
        classify_navigation_status("nav_goal_accepted label=floor_goal")["goal_status"],
        "accepted",
        "accepted classified",
    )
    assert_equal(
        classify_navigation_status("nav_goal_feedback label=floor_goal")["action"],
        "update_feedback",
        "feedback classified",
    )
    assert_equal(
        classify_navigation_status("nav_goal_succeeded label=floor_goal")["action"],
        "complete_waypoint",
        "floor goal success completes waypoint",
    )
    assert_equal(
        classify_navigation_status("nav_goal_succeeded label=other")["goal_status"],
        "succeeded",
        "non-floor success only updates status",
    )
    transition = classify_navigation_status("nav_goal_accepted label=stair_entry goal_seq=3")
    assert_equal(transition["action"], "update_transition_status", "stair entry accepted is transition")
    assert_equal(transition["label"], "stair_entry", "transition label")
    transition_feedback = classify_navigation_status("nav_goal_feedback label=stair_traverse distance_remaining=1.0")
    assert_equal(transition_feedback["action"], "update_transition_feedback", "stair traverse feedback is transition")
    assert_equal(
        classify_navigation_status("ignored reason=duplicate_floor_goal")["goal_status"],
        "accepted",
        "duplicate goal keeps accepted",
    )
    assert_equal(
        classify_navigation_status("nav_goal_cancelled label=floor_goal")["goal_status"],
        "interrupted",
        "cancelled interrupts task",
    )
    assert_equal(
        classify_navigation_status("blocked reason=not_at_entry distance=1.2 tolerance=0.8")["goal_status"],
        "blocked",
        "blocked stair status fails task",
    )
    assert_equal(
        classify_navigation_status("error nav2 action unavailable")["goal_status"],
        "error",
        "error classified",
    )
    assert_equal(
        classify_navigation_status("same_floor_goal x=1")["action"],
        "update_message",
        "other status updates message",
    )


def test_matching_goal() -> None:
    status = parse_key_value_status(
        "label=floor_goal goal_seq=7 goal_x=1.04 goal_y=2.05 goal_z=0.01 goal_yaw=-3.12"
    )
    match = nav_status_matches_active_goal(active(), annotation(), status)
    assert_equal(match["matches"], True, "matches")
    assert_equal(match["nav_goal_seq"], 7, "sequence")
    assert_true(match["goal_x_error"] < 0.15, "x tolerance")
    assert_true(match["goal_yaw_error"] < 0.25, "yaw wrap tolerance")


def test_mismatch_reasons() -> None:
    assert_equal(
        nav_status_matches_active_goal(active(), None, {})["reason"],
        "no_active_annotation",
        "missing annotation",
    )
    assert_equal(
        nav_status_matches_active_goal(active(), annotation(), {"label": "other"})["reason"],
        "label_mismatch",
        "label mismatch",
    )
    assert_equal(
        nav_status_matches_active_goal(active(last_goal_annotation_id="p2"), annotation(), {})["reason"],
        "annotation_mismatch",
        "annotation mismatch",
    )
    assert_equal(
        nav_status_matches_active_goal(active(), annotation(), {"goal_seq": 8})["reason"],
        "goal_seq_mismatch",
        "sequence mismatch",
    )
    assert_equal(
        nav_status_matches_active_goal(active(), annotation(), {"goal_seq": "bad"})["reason"],
        "goal_seq_invalid",
        "sequence invalid",
    )
    assert_equal(
        nav_status_matches_active_goal(active(), annotation(), {"goal_x": 1.3})["reason"],
        "goal_x_mismatch",
        "x mismatch",
    )
    assert_equal(
        nav_status_matches_active_goal(active(), annotation(), {"goal_yaw": math.pi / 2})["reason"],
        "goal_yaw_mismatch",
        "yaw mismatch",
    )


def test_nav_success_completion_decision() -> None:
    status_text = "nav_goal_succeeded label=floor_goal goal_seq=7 goal_x=1.0 goal_y=2.0 goal_yaw=3.12"
    complete = nav_success_completion_decision(
        active(last_nav_goal_status="accepted", last_nav_distance_remaining_m=0.1),
        annotation(),
        status_text,
    )
    assert_equal(complete["action"], "complete", "matching success completes current waypoint")
    assert_equal(complete["reason"], "current_goal_succeeded", "completion reason")
    assert_true(complete["match"]["matches"], "completion match evidence")
    assert_equal(complete["event_extra"]["nav_status"], status_text, "completion event keeps raw status")

    fresh_pose_wins = nav_success_completion_decision(
        active(last_nav_goal_status="accepted", last_nav_distance_remaining_m=0.93),
        annotation(),
        status_text,
        goal_tolerance_m=0.3,
        fresh_pose_distance_m=0.06,
        fresh_pose_age_s=0.04,
    )
    assert_equal(fresh_pose_wins["action"], "complete", "fresh pose overrides stale far feedback")
    assert_equal(
        fresh_pose_wins["event_extra"]["completion_distance_source"],
        "fresh_map_pose",
        "completion records fresh pose evidence",
    )

    fresh_pose_far = nav_success_completion_decision(
        active(last_nav_goal_status="accepted", last_nav_distance_remaining_m=0.05),
        annotation(),
        status_text,
        goal_tolerance_m=0.3,
        fresh_pose_distance_m=1.2,
        fresh_pose_age_s=0.04,
    )
    assert_equal(fresh_pose_far["action"], "fail", "fresh far pose overrides stale near feedback")
    assert_equal(fresh_pose_far["reason"], "premature_nav_success", "fresh far pose fails safely")

    not_sent = nav_success_completion_decision(
        active(last_nav_goal_status="idle"),
        annotation(),
        status_text,
    )
    assert_equal(not_sent["action"], "ignore", "inactive nav status ignored")
    assert_equal(not_sent["reason"], "nav_goal_not_active", "inactive nav reason")

    wrong_annotation = nav_success_completion_decision(
        active(last_goal_annotation_id="p2", last_nav_goal_status="accepted"),
        annotation(),
        status_text,
    )
    assert_equal(wrong_annotation["action"], "ignore", "wrong active annotation ignored")
    assert_equal(wrong_annotation["reason"], "annotation_not_last_goal", "wrong active annotation reason")

    wrong_seq = nav_success_completion_decision(
        active(last_nav_goal_status="accepted", last_nav_distance_remaining_m=0.1),
        annotation(),
        "nav_goal_succeeded label=floor_goal goal_seq=8 goal_x=1.0 goal_y=2.0",
    )
    assert_equal(wrong_seq["action"], "ignore", "wrong nav sequence ignored")
    assert_equal(wrong_seq["reason"], "goal_seq_mismatch", "wrong nav sequence reason")
    assert_equal(wrong_seq["event_extra"]["nav_goal_match"]["matches"], False, "ignored event keeps mismatch")

    premature = nav_success_completion_decision(
        active(last_nav_goal_status="accepted", last_nav_distance_remaining_m=4.58),
        annotation(),
        status_text,
        goal_tolerance_m=0.3,
    )
    assert_equal(premature["action"], "fail", "far success fails safely")
    assert_equal(premature["reason"], "premature_nav_success", "far success reason")
    assert_equal(
        premature["event_extra"]["completion_distance_source"],
        "nav_feedback",
        "far success keeps distance evidence",
    )

    no_distance = nav_success_completion_decision(
        active(last_nav_goal_status="accepted"),
        annotation(),
        status_text,
    )
    assert_equal(no_distance["action"], "fail", "success without distance evidence fails safely")
    assert_equal(
        no_distance["reason"],
        "nav_success_without_distance_evidence",
        "missing distance reason",
    )


def test_friendly_nav_status() -> None:
    assert_equal(
        friendly_nav_status("nav_goal_accepted label=floor_goal goal_seq=7"),
        "Nav2 已接收当前点位，正在导航",
        "accepted text",
    )
    assert_equal(
        friendly_nav_status("nav_goal_feedback distance_remaining=1.25 navigation_time=8 recoveries=2"),
        "Nav2 正在执行当前点位，剩余 1.25 m，已导航 8 秒，恢复 2 次",
        "feedback text",
    )
    assert_equal(
        friendly_nav_status("nav_goal_succeeded label=floor_goal"),
        "Nav2 已确认到达当前点位",
        "succeeded text",
    )
    assert_equal(
        friendly_nav_status("nav_goal_feedback label=stair_traverse distance_remaining=1.2"),
        "正在通过楼梯平台，剩余 1.20 m",
        "stair traverse feedback text",
    )
    assert_equal(
        friendly_nav_status("switching_map_at_platform source_floor=F20 target_floor=F21"),
        "跨楼层：楼梯平台到位，正在切换楼层地图",
        "switching map text",
    )
    assert_equal(
        friendly_nav_status("requesting_coordinated_floor_switch source_floor=F20 target_floor=F21"),
        "跨楼层：已到楼梯平台，准备同步切换 106 与 104 地图",
        "coordinated switch preparation text",
    )
    assert_equal(
        friendly_nav_status("floor_switch_request_sent request_id=req source_floor=F20 target_floor=F21"),
        "跨楼层：正在同步切换 106 与 104 地图并确认定位",
        "coordinated switch transaction text",
    )
    assert_equal(
        friendly_nav_status("coordinated_floor_switch_confirmed target_floor=F21 map_id=map21"),
        "跨楼层：地图与目标层定位已确认",
        "coordinated switch confirmed text",
    )
    assert_equal(
        friendly_nav_status("blocked reason=not_at_entry distance=1.2 tolerance=0.8"),
        "跨楼层被阻塞：距离楼梯入口 1.20 m，超过 0.80 m",
        "blocked stair entry text",
    )
    assert_equal(
        friendly_nav_status("error nav2 action unavailable"),
        "导航链路报错，任务已停止",
        "error text",
    )
    assert_equal(
        friendly_nav_status("custom status"),
        "custom status",
        "unknown passthrough",
    )


def test_apply_nav_failure_state() -> None:
    updated = apply_nav_failure_state(
        active(status_message="old"),
        goal_status="error",
        status_text="error nav2 action unavailable",
    )
    assert_equal(updated["last_nav_goal_status"], "error", "failure goal status")
    assert_equal(updated["last_nav_status"], "error nav2 action unavailable", "failure raw status")
    assert_equal(updated["last_error"], "error nav2 action unavailable", "failure last error")
    assert_equal(updated["status_message"], "导航链路报错，任务已停止", "failure friendly status")


def test_apply_nav_goal_status_state() -> None:
    match = {"matches": True, "nav_goal_seq": 9}
    updated = apply_nav_goal_status_state(
        active(last_nav_goal_seq=None),
        goal_status="accepted",
        status_text="nav_goal_accepted label=floor_goal goal_seq=9",
        match=match,
        now_monotonic=42.5,
        now_text="now",
    )
    assert_equal(updated["last_nav_goal_status"], "accepted", "goal status")
    assert_equal(updated["last_nav_status_at"], "now", "status time")
    assert_equal(updated["last_nav_accepted_monotonic"], 42.5, "accepted monotonic")
    assert_equal(updated["last_nav_accepted_at"], "now", "accepted text time")
    assert_equal(updated["last_nav_goal_seq"], 9, "accepted sequence")
    assert_equal(updated["last_nav_goal_match"], match, "accepted match")
    assert_equal(updated["status_message"], "Nav2 已接收当前点位，正在导航", "accepted friendly status")

    event = nav_goal_status_event_payload(
        updated,
        goal_status="accepted",
        status_text="nav_goal_accepted label=floor_goal goal_seq=9",
        status_payload={"label": "floor_goal", "goal_seq": 9},
        match=match,
    )
    assert_equal(event["event"], "nav_accepted", "goal status event name")
    assert_equal(event["message"], "Nav2 已接收当前点位，正在导航", "goal status event message")
    assert_equal(event["extra"]["nav_status"], "nav_goal_accepted label=floor_goal goal_seq=9", "goal status raw text")
    assert_equal(event["extra"]["nav_status_payload"]["goal_seq"], 9, "goal status parsed payload")
    assert_equal(event["extra"]["nav_goal_match"], match, "goal status match payload")


def test_apply_nav_status_message_state() -> None:
    updated = apply_nav_status_message_state(
        active(status_message="old"),
        status_text="same_floor_goal x=1",
        now_text="now",
    )
    assert_equal(updated["last_nav_status"], "same_floor_goal x=1", "message raw status")
    assert_equal(updated["last_nav_status_at"], "now", "message status time")
    assert_equal(updated["status_message"], "同楼层目标已转交 Nav2", "message friendly status")

    event = nav_status_message_event_payload(updated, status_text="same_floor_goal x=1")
    assert_equal(event["event"], "nav_status", "message event name")
    assert_equal(event["message"], "同楼层目标已转交 Nav2", "message event text")
    assert_equal(event["extra"]["nav_status"], "same_floor_goal x=1", "message event raw status")


def test_apply_nav_feedback_state() -> None:
    dispatch = nav_feedback_dispatch_payload(
        "nav_goal_feedback label=floor_goal goal_seq=7 distance_remaining=1.25 navigation_time=8 recoveries=2"
    )
    assert_equal(dispatch["action"], "update_feedback", "floor-goal feedback dispatch action")
    assert_equal(dispatch["feedback"]["label"], "floor_goal", "floor-goal feedback parsed label")
    assert_equal(dispatch["feedback"]["goal_seq"], 7, "floor-goal feedback parsed sequence")

    other = nav_feedback_dispatch_payload("nav_goal_feedback label=other distance_remaining=1.0")
    assert_equal(other["action"], "update_message", "non-floor-goal feedback dispatch action")
    assert_equal(other["reason"], "not_floor_goal_feedback", "non-floor-goal feedback reason")
    assert_equal(other["feedback"]["label"], "other", "non-floor-goal feedback parsed label")

    feedback = parse_key_value_status(
        "nav_goal_feedback label=floor_goal goal_seq=7 distance_remaining=1.25 navigation_time=8 recoveries=2"
    )
    match = {"matches": True, "nav_goal_seq": 7}
    assert_true(should_record_nav_feedback_event(active(), feedback), "first feedback records")
    assert_true(
        should_record_nav_feedback_event(active(has_nav_feedback=True, last_nav_feedback_recoveries=1), feedback),
        "recovery count change records",
    )
    assert_true(
        not should_record_nav_feedback_event(active(has_nav_feedback=True, last_nav_feedback_recoveries=2), feedback),
        "same recovery count does not record",
    )
    updated = apply_nav_feedback_state(
        active(last_nav_goal_seq=None),
        status_text="nav_goal_feedback label=floor_goal distance_remaining=1.25 navigation_time=8 recoveries=2",
        feedback=feedback,
        match=match,
        now_monotonic=50.0,
        now_text="now",
    )
    assert_equal(updated["last_nav_feedback"], feedback, "feedback stored")
    assert_equal(updated["last_nav_feedback_at"], "now", "feedback time")
    assert_equal(updated["last_nav_feedback_monotonic"], 50.0, "feedback monotonic")
    assert_equal(updated["last_nav_goal_seq"], 7, "feedback sequence")
    assert_equal(updated["last_nav_distance_remaining_m"], 1.25, "feedback distance")
    assert_true(updated["has_nav_feedback"], "has feedback")
    assert_equal(updated["last_nav_feedback_recoveries"], 2, "feedback recoveries")
    assert_equal(
        updated["status_message"],
        "Nav2 正在执行当前点位，剩余 1.25 m，已导航 8 秒，恢复 2 次",
        "feedback friendly status",
    )

    event = nav_feedback_event_payload(
        updated,
        status_text="nav_goal_feedback label=floor_goal distance_remaining=1.25 navigation_time=8 recoveries=2",
        feedback=feedback,
        match=match,
    )
    assert_equal(event["event"], "nav_feedback", "feedback event name")
    assert_equal(event["message"], "Nav2 正在执行当前点位，剩余 1.25 m，已导航 8 秒，恢复 2 次", "feedback event message")
    assert_equal(event["extra"]["nav_feedback"], feedback, "feedback event payload")
    assert_equal(event["extra"]["nav_goal_match"], match, "feedback event match")


def test_apply_ignored_nav_status_state() -> None:
    match = {"matches": False, "reason": "goal_seq_mismatch"}
    updated = apply_ignored_nav_status_state(
        active(task_id="task_1"),
        status_text="nav_goal_accepted label=floor_goal goal_seq=8",
        match=match,
    )
    assert_equal(updated["last_ignored_nav_status"], "nav_goal_accepted label=floor_goal goal_seq=8", "ignored status")
    assert_equal(updated["last_ignored_nav_goal_match"], match, "ignored match")

    event = ignored_nav_status_event_payload(
        updated,
        message="忽略与当前任务点不匹配的 Nav2 状态",
        status_text="nav_goal_accepted label=floor_goal goal_seq=8",
        match={**match, "annotation_id": "p1", "label": "点1"},
    )
    assert_equal(event["timeline_event"], "nav_status_ignored", "ignored timeline event")
    assert_equal(event["timeline_extra"]["nav_status"], "nav_goal_accepted label=floor_goal goal_seq=8", "ignored timeline raw status")
    assert_equal(event["operator_event"], "忽略与当前任务点不匹配的 Nav2 状态", "ignored operator event")
    assert_equal(event["operator_payload"]["task_id"], "task_1", "ignored operator task id")
    assert_equal(event["operator_payload"]["annotation_id"], "p1", "ignored operator annotation id")


def test_transition_nav_status_state() -> None:
    payload = parse_key_value_status("nav_goal_feedback label=stair_entry goal_seq=4 distance_remaining=0.8")
    updated = apply_transition_nav_status_state(
        active(task_id="task_1"),
        goal_status=None,
        status_text="nav_goal_feedback label=stair_entry goal_seq=4 distance_remaining=0.8",
        status_payload=payload,
        now_text="now",
    )
    assert_equal(updated["last_transition_nav_status"], "nav_goal_feedback label=stair_entry goal_seq=4 distance_remaining=0.8", "transition status")
    assert_equal(updated["last_transition_nav_label"], "stair_entry", "transition label stored")
    assert_equal(updated["last_transition_nav_payload"]["distance_remaining"], 0.8, "transition payload stored")
    assert_equal(updated["status_message"], "正在前往楼梯入口，剩余 0.80 m", "transition status friendly")

    event = transition_nav_status_event_payload(
        updated,
        status_text="nav_goal_feedback label=stair_entry goal_seq=4 distance_remaining=0.8",
        status_payload=payload,
    )
    assert_equal(event["event"], "cross_floor_nav_status", "transition event")
    assert_equal(event["extra"]["nav_status_payload"]["label"], "stair_entry", "transition event payload")


def main() -> int:
    for test in (
        test_parse_key_value_status,
        test_classify_navigation_status,
        test_matching_goal,
        test_mismatch_reasons,
        test_nav_success_completion_decision,
        test_friendly_nav_status,
        test_apply_nav_failure_state,
        test_apply_nav_goal_status_state,
        test_apply_nav_status_message_state,
        test_apply_nav_feedback_state,
        test_apply_ignored_nav_status_state,
        test_transition_nav_status_state,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] nav status contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
