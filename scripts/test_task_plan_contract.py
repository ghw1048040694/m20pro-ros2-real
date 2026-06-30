#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.task_plan_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.task_plan_contract import (  # noqa: E402
    apply_plan_goal_verified_state,
    path_goal_error_m,
    plan_goal_verified_event_payload,
    task_plan_match_decision,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def active(**extra) -> dict:
    payload = {
        "task_id": "task_1",
        "last_goal_annotation_id": "p1",
        "last_goal_pose": {"floor": "F20", "x": 1.0, "y": 2.0, "yaw": 0.0},
        "last_nav_goal_status": "accepted",
        "goal_sent_path_version": 3,
        "last_nav_accepted_monotonic": 10.0,
        "last_nav_status": "nav_goal_accepted label=floor_goal",
        "last_nav_feedback": {"distance_remaining": 1.0},
    }
    payload.update(extra)
    return payload


def annotation(**extra) -> dict:
    payload = {"id": "p1", "label": "点1", "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0}}
    payload.update(extra)
    return payload


def path(**extra) -> dict:
    payload = {"version": 4, "last_point": {"x": 1.02, "y": 2.03, "z": 0.0}}
    payload.update(extra)
    return payload


def decision(active_payload=None, annotation_payload=None, path_payload=None, **kwargs) -> dict:
    params = {
        "required": True,
        "now_monotonic": 20.0,
        "timeout_s": 8.0,
        "tolerance_m": 0.2,
    }
    params.update(kwargs)
    return task_plan_match_decision(
        active_payload or active(),
        annotation_payload or annotation(),
        path_payload or path(),
        **params,
    )


def test_path_goal_error() -> None:
    assert_true(path_goal_error_m({"x": 1.0, "y": 2.0}, annotation()) == 0.0, "zero error")
    assert_equal(path_goal_error_m({"x": "bad"}, annotation()), None, "bad path endpoint")


def test_plan_verified() -> None:
    result = decision()
    assert_equal(result["action"], "verify", "verified action")
    assert_equal(result["reason"], "path_goal_verified", "verified reason")
    assert_true(result["path_goal_error_m"] < 0.2, "verified tolerance")


def test_apply_plan_goal_verified_state() -> None:
    result = decision()
    updated = apply_plan_goal_verified_state(active(status_message="old"), result)
    assert_true(updated["plan_goal_verified"], "plan verified flag")
    assert_equal(updated["plan_goal_error_m"], result["path_goal_error_m"], "plan verified error")
    assert_equal(updated["plan_path_version"], result["path_version"], "plan verified version")
    assert_equal(updated["status_message"], "规划路径已匹配当前点位，继续执行", "plan verified message")


def test_plan_goal_verified_event_payload() -> None:
    result = decision()
    updated = apply_plan_goal_verified_state(active(status_message="old"), result)
    event = plan_goal_verified_event_payload(updated, annotation(), result)
    assert_equal(event["event"], "plan_goal_verified", "event name")
    assert_equal(event["message"], "规划路径已匹配当前点位，继续执行", "event message")
    assert_equal(event["extra"]["annotation_id"], "p1", "annotation id")
    assert_equal(event["extra"]["label"], "点1", "label")
    assert_equal(event["extra"]["path_version"], 4, "path version")
    assert_equal(event["extra"]["path_last_point"], {"x": 1.02, "y": 2.03, "z": 0.0}, "path endpoint")
    assert_equal(event["extra"]["path_goal_error_m"], result["path_goal_error_m"], "path error")


def test_plan_mismatch() -> None:
    result = decision(path_payload=path(last_point={"x": 5.0, "y": 8.0, "z": 0.0}))
    assert_equal(result["action"], "fail", "mismatch fails")
    assert_equal(result["reason"], "path_goal_mismatch", "mismatch reason")
    assert_true(result["path_goal_error_m"] > 0.2, "mismatch error")


def test_plan_timeout_and_wait() -> None:
    waiting = decision(path_payload=path(version=3), now_monotonic=12.0)
    assert_equal(waiting["action"], "wait", "waits before timeout")
    assert_equal(waiting["reason"], "waiting_for_new_plan", "wait reason")

    timed_out = decision(path_payload=path(version=3), now_monotonic=25.0)
    assert_equal(timed_out["action"], "fail", "timeout fails")
    assert_equal(timed_out["reason"], "plan_update_timeout", "timeout reason")


def test_preconditions_pass() -> None:
    assert_equal(decision(required=False)["reason"], "not_required", "disabled")
    assert_equal(
        decision(active_payload=active(last_goal_annotation_id="other"))["reason"],
        "annotation_not_current",
        "annotation not current",
    )
    assert_equal(
        decision(active_payload=active(last_nav_goal_status="sent"))["reason"],
        "nav_goal_not_accepted",
        "nav not accepted",
    )
    assert_equal(
        decision(active_payload=active(goal_sent_path_version=None))["reason"],
        "missing_sent_path_version",
        "missing sent version",
    )


def main() -> int:
    for test in (
        test_path_goal_error,
        test_plan_verified,
        test_apply_plan_goal_verified_state,
        test_plan_goal_verified_event_payload,
        test_plan_mismatch,
        test_plan_timeout_and_wait,
        test_preconditions_pass,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] task plan contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
