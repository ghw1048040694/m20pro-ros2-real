#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.localization_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.localization_contract import (  # noqa: E402
    factory_localization_ok_from_sources,
    initialpose_api_response_payload,
    localization_status_payload,
    map_relocalization_clearance_payload,
    manual_relocalization_verification_payload,
    pose_tcp_2101_consistency_payload,
    parse_tcp_2101_success_pose,
    parse_initialpose_request,
    relocalization_sample_evidence,
    relocalization_response_payload,
    relocalization_stability_step,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def sample_pose() -> dict:
    return {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.1}


def test_parse_initialpose_request() -> None:
    parsed = parse_initialpose_request(
        {"x": "1.5", "y": 2, "z": "", "yaw": "0.3", "frame_id": " map ", "floor": " F20 "}
    )
    assert_equal(parsed["ok"], False, "blank z is invalid")
    assert_equal(parsed["code"], "initialpose_pose_invalid", "blank z invalid code")
    assert_equal(parsed["message"], "重定位坐标无效，请先在地图上拖箭头", "blank z invalid message")

    defaulted = parse_initialpose_request({"x": "1.5", "y": 2, "yaw": "0.3"})
    assert_equal(defaulted["ok"], True, "valid request passes")
    assert_equal(defaulted["pose"], {"x": 1.5, "y": 2.0, "z": 0.0, "yaw": 0.3}, "pose normalized")
    assert_equal(defaulted["frame_id"], "map", "frame defaults to map")
    assert_equal(defaulted["floor"], "", "floor defaults to empty")

    trimmed = parse_initialpose_request(
        {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.1, "frame_id": " odom ", "floor": " F20 "}
    )
    assert_equal(trimmed["frame_id"], "odom", "frame is trimmed")
    assert_equal(trimmed["floor"], "F20", "floor is trimmed")

    infinite = parse_initialpose_request({"x": "inf", "y": 2.0, "yaw": 0.0})
    assert_equal(infinite["ok"], False, "infinite pose fails")
    assert_equal(infinite["code"], "initialpose_pose_invalid", "infinite invalid code")
    assert_equal(infinite["message"], "重定位坐标无效，请先在地图上拖箭头", "infinite invalid message")


def test_localization_status_requires_fresh_map_pose() -> None:
    status = localization_status_payload(
        localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=0.4,
        pose_timeout_s=3.0,
        now_text=lambda: "fixed-time",
    )
    assert_equal(status["confirmed"], True, "fresh localized pose is confirmed")
    assert_true("task_ready" not in status, "localization status does not include task readiness")
    assert_equal(status["code"], "localized_confirmed", "ready code")
    assert_equal(status["updated_at"], "fixed-time", "timestamp")

    stale = localization_status_payload(
        localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=9.0,
        pose_timeout_s=3.0,
    )
    assert_equal(stale["confirmed"], False, "stale pose is not confirmed")
    assert_equal(stale["code"], "pose_stale", "stale code")

    missing_pose = localization_status_payload(
        localization_ok=True,
        pose={},
        pose_age_sec=0.1,
        pose_timeout_s=3.0,
    )
    assert_equal(missing_pose["confirmed"], False, "missing pose is not confirmed")
    assert_equal(missing_pose["code"], "pose_missing_or_invalid", "missing pose code")

    failed_attempt = localization_status_payload(
        localization_ok=True,
        factory_localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=0.1,
        pose_timeout_s=3.0,
        relocalization_attempt={
            "status": "failed",
            "message": "本次重定位失败，旧位姿不会作为本次成功结果",
        },
    )
    assert_equal(failed_attempt["confirmed"], False, "failed attempt clears old success")
    assert_equal(failed_attempt["code"], "relocalization_attempt_failed", "failed attempt code")


def test_factory_localization_source_precedence() -> None:
    assert_equal(
        factory_localization_ok_from_sources(False, {"location": 0}),
        False,
        "explicit bridge failure cannot be resurrected by stale factory status",
    )
    assert_equal(
        factory_localization_ok_from_sources(True, {"location": 1}),
        True,
        "explicit bridge success remains authoritative",
    )
    assert_equal(
        factory_localization_ok_from_sources(None, {"location": 0}),
        True,
        "navigation status is a startup fallback before the Bool topic speaks",
    )


def test_localization_status_reports_motion_away_from_tcp_pose_as_confirmed() -> None:
    status = localization_status_payload(
        localization_ok=True,
        pose={"x": -0.011, "y": -0.003, "z": 0.0, "yaw": -1.55},
        pose_age_sec=0.1,
        pose_timeout_s=3.0,
        relocalization_result={
            "last_update": 100.0,
            "raw": "success: x=-10.923 y=-3.264 z=0.000 yaw=-1.549",
        },
        now_time=101.0,
    )
    assert_equal(status["confirmed"], True, "moving away from the relocalization pose stays confirmed")
    assert_equal(status["code"], "localized_confirmed", "localization remains confirmed")
    assert_equal(status["pose_near_2101"], False, "distance from the last 2101 pose remains diagnostic")
    assert_true(status["pose_error_m"] > 10.0, "movement distance from 2101 pose is visible")

    consistent = pose_tcp_2101_consistency_payload(
        {"x": -10.9, "y": -3.3, "z": 0.0, "yaw": -1.55},
        "success: x=-10.923 y=-3.264 z=0.000 yaw=-1.549",
    )
    assert_equal(consistent["pose_near_2101"], True, "meter-scale pose matches 2101")


def test_localization_status_uses_factory_status_when_localization_topic_disagrees() -> None:
    status = localization_status_payload(
        localization_ok=False,
        factory_localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=0.2,
        pose_timeout_s=3.0,
        relocalization_result={
            "last_update": 100.0,
            "raw": "success: x=1.000 y=2.000 z=0.000 yaw=0.100",
        },
        now_time=101.0,
    )
    assert_equal(status["confirmed"], True, "recent 2101 plus factory status keeps relocalization confirmed")
    assert_equal(status["code"], "localized_confirmed", "topic disagreement does not make relocalization fail")
    assert_true("状态源不一致" in status["message"], "operator sees the source disagreement")


def test_relocalization_sample_evidence() -> None:
    evidence = relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 3.13},
        relocalization={"last_update": 10.2, "raw": "success: ErrorCode=0"},
        pose={"last_update": 10.3, "x": 1.2, "y": 2.1, "z": 0.0, "yaw": -3.13},
        localization_ok=True,
        scan={"last_update": 10.1, "finite_ranges": 25},
        local_costmap={"last_update": 10.1, "width": 10, "height": 10},
        global_costmap={"last_update": 10.1, "width": 20, "height": 20},
        pose_tolerance_m=0.5,
    )
    assert_equal(evidence["tcp_2101_accepted"], True, "fresh success 2101 is accepted")
    assert_equal(evidence["tcp_2101_fresh"], True, "fresh 2101 result")
    assert_equal(evidence["pose_ok"], True, "fresh plausible pose accepted")
    assert_equal(evidence["pose_near_request"], True, "pose near request")
    assert_equal(evidence["scan_ok"], True, "fresh scan accepted")
    assert_equal(evidence["local_costmap_ok"], True, "fresh local costmap accepted")
    assert_equal(evidence["global_costmap_ok"], True, "fresh global costmap accepted")
    assert_equal(evidence["ready_to_finish_wait"], True, "ready to finish wait")
    assert_true(evidence["yaw_error_rad"] < 0.04, "yaw wraps across pi boundary")
    assert_equal(evidence["tcp_pose_near_request"], None, "coordinate-free 2101 reply remains compatible")

    contradictory_reply = relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        relocalization={
            "last_update": 10.2,
            "raw": "success: x=8.000 y=2.000 z=0.000 yaw=0.000",
        },
        pose={"last_update": 10.3, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        localization_ok=True,
        scan={"last_update": 10.1, "finite_ranges": 25},
        local_costmap={"last_update": 10.1, "width": 10, "height": 10},
        global_costmap={"last_update": 10.1, "width": 20, "height": 20},
        pose_tolerance_m=0.5,
    )
    assert_equal(contradictory_reply["tcp_pose_near_request"], False, "contradictory 2101 pose rejected")
    assert_equal(contradictory_reply["ready_to_finish_wait"], False, "contradictory 2101 reply cannot confirm")

    stale = relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        relocalization={"last_update": 9.9, "raw": "success: stale"},
        pose={"last_update": 9.9, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        localization_ok=True,
        scan={"last_update": 9.9, "finite_ranges": 25},
        local_costmap={"last_update": 9.9, "width": 10, "height": 10},
        global_costmap={"last_update": 9.9, "width": 20, "height": 20},
        pose_tolerance_m=0.5,
    )
    assert_equal(stale["tcp_2101_accepted"], False, "stale 2101 result ignored")
    assert_equal(stale["pose_ok"], False, "stale pose ignored")
    assert_equal(stale["scan_ok"], False, "stale scan ignored")
    assert_equal(stale["ready_to_finish_wait"], False, "stale sample keeps waiting")

    no_2101 = relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        relocalization={"last_update": 10.2, "raw": ""},
        pose={"last_update": 10.3, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        localization_ok=True,
        scan={"last_update": 10.1, "finite_ranges": 25},
        local_costmap={"last_update": 10.1, "width": 10, "height": 10},
        global_costmap={"last_update": 10.1, "width": 20, "height": 20},
        pose_tolerance_m=0.5,
    )
    assert_equal(no_2101["pose_near_request"], True, "pose can be ready before 2101")
    assert_equal(no_2101["tcp_2101_accepted"], False, "blank 2101 is not accepted")
    assert_equal(no_2101["ready_to_finish_wait"], False, "wait does not finish before 2101 success")

    ambiguous = relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        relocalization={
            "last_update": 10.2,
            "raw": "pending_verification: ErrorCode=0xFFFF firmware reply ambiguous",
        },
        pose={"last_update": 10.3, "x": 1.1, "y": 2.0, "z": 0.0, "yaw": 0.0},
        localization_ok=True,
        scan={"last_update": 10.1, "finite_ranges": 25},
        local_costmap={"last_update": 10.1, "width": 10, "height": 10},
        global_costmap={"last_update": 10.1, "width": 20, "height": 20},
        pose_tolerance_m=0.5,
    )
    assert_equal(ambiguous["tcp_2101_accepted"], False, "0xFFFF is not a success reply")
    assert_equal(ambiguous["tcp_2101_ambiguous"], True, "0xFFFF enters verification")
    assert_equal(ambiguous["ready_to_finish_wait"], True, "target pose verifies ambiguous reply")


def _ready_relocalization_evidence() -> dict:
    return relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        relocalization={
            "last_update": 10.2,
            "raw": "success: x=1.000 y=2.000 z=0.000 yaw=0.000",
        },
        pose={"last_update": 10.3, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        localization_ok=True,
        scan={"last_update": 10.1, "finite_ranges": 25},
        local_costmap={"last_update": 10.1, "width": 10, "height": 10},
        global_costmap={"last_update": 10.1, "width": 20, "height": 20},
        pose_tolerance_m=0.5,
        yaw_tolerance_rad=0.45,
    )


def test_cross_floor_platform_yaw_tolerance_is_strict() -> None:
    """Cross-floor platform acceptance must check heading as well as XY."""

    outside = relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        relocalization={
            "last_update": 10.2,
            "raw": "success: x=1.000 y=2.000 z=0.000 yaw=0.460",
        },
        pose={"last_update": 10.3, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.460},
        localization_ok=True,
        scan={"last_update": 10.1, "finite_ranges": 25},
        local_costmap={"last_update": 10.1, "width": 10, "height": 10},
        global_costmap={"last_update": 10.1, "width": 20, "height": 20},
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(outside["pose_near_request"], False, "platform yaw outside tolerance is rejected")
    assert_equal(outside["tcp_pose_near_request"], False, "2101 platform yaw outside tolerance is rejected")
    assert_equal(outside["ready_to_finish_wait"], False, "wrong platform heading cannot confirm")
    assert_equal(outside["yaw_error_rad"], 0.46, "platform yaw error is exposed")

    source_outside = relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 4.0, "y": -1.0, "z": 0.0, "yaw": -1.0},
        relocalization={
            "last_update": 10.2,
            "raw": "success: x=4.000 y=-1.000 z=0.000 yaw=-0.540",
        },
        pose={"last_update": 10.3, "x": 4.0, "y": -1.0, "z": 0.0, "yaw": -0.540},
        localization_ok=True,
        scan={"last_update": 10.1, "finite_ranges": 25},
        local_costmap={"last_update": 10.1, "width": 10, "height": 10},
        global_costmap={"last_update": 10.1, "width": 20, "height": 20},
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(source_outside["pose_near_request"], False, "source platform yaw outside tolerance is rejected")
    assert_equal(source_outside["tcp_pose_near_request"], False, "source 2101 yaw outside tolerance is rejected")
    assert_equal(source_outside["ready_to_finish_wait"], False, "source platform cannot confirm with wrong heading")

    boundary = relocalization_sample_evidence(
        request_started_at=10.0,
        requested_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 3.13},
        relocalization={
            "last_update": 10.2,
            "raw": "success: x=1.000 y=2.000 z=0.000 yaw=-3.000000000",
        },
        pose={"last_update": 10.3, "x": 1.0, "y": 2.0, "z": 0.0, "yaw": -3.000000000},
        localization_ok=True,
        scan={"last_update": 10.1, "finite_ranges": 25},
        local_costmap={"last_update": 10.1, "width": 10, "height": 10},
        global_costmap={"last_update": 10.1, "width": 20, "height": 20},
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_true(boundary["yaw_error_rad"] < 0.2, "platform yaw comparison wraps across +/-pi")
    assert_equal(boundary["pose_near_request"], True, "wrapped platform yaw inside tolerance passes")
    assert_equal(boundary["tcp_pose_near_request"], True, "wrapped 2101 platform yaw inside tolerance passes")
    assert_equal(boundary["ready_to_finish_wait"], True, "valid platform heading confirms")


def test_relocalization_stability_window_is_continuous() -> None:
    evidence = _ready_relocalization_evidence()
    first = relocalization_stability_step(
        evidence=evidence,
        current_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        previous_pose=None,
        stable_since=None,
        now_time=20.0,
        stability_window_s=2.0,
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(first["stable"], False, "first stable sample starts but does not finish window")
    assert_equal(first["code"], "stability_window_waiting", "window waiting code")
    assert_equal(first["window_elapsed_s"], 0.0, "window starts at first sample")

    before_deadline = relocalization_stability_step(
        evidence=evidence,
        current_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        previous_pose=first["previous_stable_pose"],
        stable_since=first["stable_since"],
        now_time=21.99,
        stability_window_s=2.0,
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(before_deadline["stable"], False, "window not yet elapsed cannot confirm")
    assert_true(before_deadline["window_elapsed_s"] < 2.0, "elapsed stability is below configured window")

    finished = relocalization_stability_step(
        evidence=evidence,
        current_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        previous_pose=before_deadline["previous_stable_pose"],
        stable_since=before_deadline["stable_since"],
        now_time=22.0,
        stability_window_s=2.0,
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(finished["stable"], True, "continuous stable window confirms at deadline")
    assert_equal(finished["code"], "stability_window_satisfied", "window satisfied code")

    changed = relocalization_stability_step(
        evidence=evidence,
        current_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.2},
        previous_pose=before_deadline["previous_stable_pose"],
        stable_since=before_deadline["stable_since"],
        now_time=22.1,
        stability_window_s=2.0,
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(changed["stable"], False, "heading movement restarts stability window")
    assert_equal(changed["stable_since"], 22.1, "changed pose starts a new window")

    invalid_anchor = relocalization_stability_step(
        evidence=evidence,
        current_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        previous_pose={"x": float("nan"), "y": 2.0, "z": 0.0, "yaw": 0.0},
        stable_since=20.0,
        now_time=22.1,
        stability_window_s=2.0,
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(invalid_anchor["stable"], False, "invalid prior anchor cannot preserve stability")
    assert_equal(invalid_anchor["stable_since"], 22.1, "invalid anchor restarts window")

    not_ready = relocalization_stability_step(
        evidence={**evidence, "ready_to_finish_wait": False},
        current_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        previous_pose=before_deadline["previous_stable_pose"],
        stable_since=before_deadline["stable_since"],
        now_time=25.0,
        stability_window_s=2.0,
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(not_ready["stable"], False, "missing evidence cannot confirm stability")
    assert_equal(not_ready["stable_since"], None, "missing evidence resets window")

    immediate = relocalization_stability_step(
        evidence=evidence,
        current_pose={"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
        previous_pose=None,
        stable_since=None,
        now_time=30.0,
        stability_window_s=0.0,
        pose_tolerance_m=0.50,
        yaw_tolerance_rad=0.45,
    )
    assert_equal(immediate["stable"], True, "zero window preserves immediate verification mode")

    incomplete_verification = manual_relocalization_verification_payload(
        tcp_2101_accepted=True,
        tcp_2101_result="success: x=1.000 y=2.000 z=0.000 yaw=0.000",
        tcp_2101_ambiguous=False,
        localization_ok=True,
        pose_ok=True,
        pose_near_request=True,
        scan_ok=True,
        local_costmap_ok=True,
        global_costmap_ok=True,
        tcp_pose_near_request=True,
        stability_confirmed=False,
    )
    assert_equal(
        incomplete_verification["factory_pose_accepted"],
        False,
        "final pose evidence cannot bypass an incomplete stability window",
    )
    assert_equal(
        incomplete_verification["checks"]["stability_window"],
        "fail",
        "incomplete stability is exposed to the operator",
    )


def test_map_relocalization_clearance_uses_strong_current_evidence() -> None:
    raw = "success: x=1.000 y=2.000 z=0.000 yaw=0.100"
    parsed_pose = parse_tcp_2101_success_pose(raw)
    assert_equal(parsed_pose, sample_pose(), "2101 success pose is parsed")

    clearable = map_relocalization_clearance_payload(
        map_relocalization_required={"map_id": "map_a", "reason": "manual_select"},
        selected_map_id="map_a",
        selected_map_status={"ready": True, "code": "ready"},
        localization_ok=False,
        factory_localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=0.2,
        pose_timeout_s=3.0,
        relocalization_result={"last_update": 105.0, "raw": raw},
        lock_loaded_time=100.0,
        now_time=106.0,
        now_text=lambda: "fixed-time",
    )
    assert_equal(clearable["clear"], True, "fresh 2101 plus factory pose clears manual map lock")
    assert_equal(clearable["code"], "map_relocalization_lock_clearable", "clearable code")
    assert_equal(clearable["updated_at"], "fixed-time", "clearance timestamp")
    assert_equal(clearable["tcp_2101_after_lock"], True, "2101 happened after lock")
    assert_equal(clearable["pose_near_2101"], True, "pose closeness remains visible as evidence")

    reply_only = map_relocalization_clearance_payload(
        map_relocalization_required={"map_id": "map_a", "reason": "startup_sync"},
        selected_map_id="map_a",
        selected_map_status={"ready": True, "code": "ready"},
        localization_ok=False,
        factory_localization_ok=False,
        pose=sample_pose(),
        pose_age_sec=0.2,
        pose_timeout_s=3.0,
        relocalization_result={"last_update": 105.0, "raw": raw},
        lock_loaded_time=100.0,
        now_time=106.0,
    )
    assert_equal(reply_only["clear"], False, "2101 reply alone does not clear map lock")
    assert_equal(reply_only["code"], "factory_localization_not_confirmed", "factory evidence required")

    stale_pose = map_relocalization_clearance_payload(
        map_relocalization_required={"map_id": "map_a", "reason": "startup_sync"},
        selected_map_id="map_a",
        selected_map_status={"ready": True, "code": "ready"},
        localization_ok=True,
        factory_localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=9.0,
        pose_timeout_s=3.0,
        relocalization_result={"last_update": 105.0, "raw": raw},
        lock_loaded_time=100.0,
        now_time=106.0,
    )
    assert_equal(stale_pose["clear"], False, "stale pose does not clear map lock")
    assert_equal(stale_pose["code"], "pose_missing_or_stale", "fresh pose required")

    scaled_pose = map_relocalization_clearance_payload(
        map_relocalization_required={"map_id": "map_a", "reason": "manual_select"},
        selected_map_id="map_a",
        selected_map_status={"ready": True, "code": "ready"},
        localization_ok=True,
        factory_localization_ok=True,
        pose={"x": 0.001, "y": 0.002, "z": 0.0, "yaw": 0.1},
        pose_age_sec=0.2,
        pose_timeout_s=3.0,
        relocalization_result={"last_update": 105.0, "raw": raw},
        lock_loaded_time=100.0,
        now_time=106.0,
    )
    assert_equal(scaled_pose["clear"], False, "scaled pose does not clear map lock")
    assert_equal(scaled_pose["code"], "pose_not_near_tcp_2101", "pose/2101 mismatch blocks clear")

    startup_sync = map_relocalization_clearance_payload(
        map_relocalization_required={"map_id": "map_a", "reason": "startup_sync"},
        selected_map_id="map_a",
        selected_map_status={"ready": True, "code": "ready"},
        localization_ok=True,
        factory_localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=0.2,
        pose_timeout_s=3.0,
        relocalization_result=None,
        lock_loaded_time=100.0,
        now_time=106.0,
    )
    assert_equal(startup_sync["clear"], True, "startup sync lock clears from current factory pose evidence")
    assert_equal(
        startup_sync["code"],
        "startup_map_relocalization_lock_clearable",
        "startup lock clearable code",
    )

    manual_without_2101 = map_relocalization_clearance_payload(
        map_relocalization_required={"map_id": "map_a", "reason": "manual_select"},
        selected_map_id="map_a",
        selected_map_status={"ready": True, "code": "ready"},
        localization_ok=True,
        factory_localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=0.2,
        pose_timeout_s=3.0,
        relocalization_result=None,
        lock_loaded_time=100.0,
        now_time=106.0,
    )
    assert_equal(manual_without_2101["clear"], False, "manual map switch still requires 2101 success")
    assert_equal(manual_without_2101["code"], "tcp_2101_not_success", "manual lock keeps 2101 gate")


def test_localization_status_explains_success_reply_but_map_lock() -> None:
    status = localization_status_payload(
        localization_ok=False,
        factory_localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=0.2,
        pose_timeout_s=3.0,
        relocalization_result={
            "last_update": 95.0,
            "raw": "success: x=1.000 y=2.000 z=0.000 yaw=0.100",
        },
        map_relocalization_required={
            "reason": "startup_sync",
            "message": "启动后已把当前固定地图同步到 Nav2，必须重新按开发手册2101定位",
        },
        now_time=100.0,
    )
    assert_equal(status["confirmed"], False, "map relocalization lock still blocks confirmation")
    assert_equal(status["code"], "map_relocalization_required", "map lock code")
    assert_equal(status["tcp_2101_accepted"], True, "2101 reply remains visible")
    assert_equal(status["factory_localization_ok"], True, "factory localization evidence remains visible")
    assert_equal(status["pose_fresh"], True, "fresh pose evidence remains visible")
    assert_true(status["message"].startswith("重定位失败"), "partial success is reported as final failure")
    assert_true("已收到 2101 回执" in status["message"], "2101 reply is only explanatory evidence")
    assert_true("重定位要求" in status["message"], "map relocalization requirement is explained")


def test_relocalization_response_separates_sent_from_confirmed() -> None:
    failed = relocalization_response_payload(
        {"factory_pose_accepted": False, "navigation_ready": False, "message": "未看到地图位姿更新"},
        now_text=lambda: "fixed-time",
    )
    assert_equal(failed["confirmed"], False, "published initialpose is not treated as confirmed")
    assert_equal(failed["code"], "localization_not_confirmed", "unconfirmed code")

    confirmed = relocalization_response_payload(
        {"factory_pose_accepted": True, "navigation_ready": False, "message": "定位已确认"},
    )
    assert_equal(confirmed["confirmed"], True, "factory accepted pose is confirmed")
    assert_equal(confirmed["code"], "localized_confirmed", "confirmed code")
    assert_true("task_ready" not in confirmed, "relocalization response does not include task readiness")
    assert_true("task_readiness" not in confirmed, "relocalization response does not include task readiness payload")


def test_initialpose_api_response_payload() -> None:
    response = initialpose_api_response_payload(
        localization_status={
            "confirmed": True,
            "navigation_ready": False,
            "code": "localized_confirmed",
            "message": "重定位成功",
        },
        verification={"factory_pose_accepted": True, "navigation_ready": False},
        topic="/initialpose",
        publish_repeats="10",
        frame_id="map",
        floor="F20",
        pose={"x": "1.5", "y": 2, "z": 0, "yaw": "0.3"},
    )
    assert_equal(response["ok"], True, "initialpose API request was accepted for publishing")
    assert_equal(response["confirmed"], True, "confirmed comes from localization status")
    assert_true("task_ready" not in response, "initialpose API does not include task readiness")
    assert_true("task_readiness" not in response, "initialpose API does not include task readiness payload")
    assert_equal(response["navigation_ready"], False, "navigation readiness remains separate")
    assert_equal(response["code"], "localized_confirmed", "status code is copied")
    assert_equal(response["topic"], "/initialpose", "topic is copied")
    assert_equal(response["publish_repeats"], 10, "publish repeats normalized")
    assert_equal(response["frame_id"], "map", "frame is copied")
    assert_equal(response["floor"], "F20", "floor is copied")
    assert_equal(
        response["pose"],
        {"x": 1.5, "y": 2.0, "z": 0.0, "yaw": 0.3},
        "pose is normalized",
    )


def test_manual_relocalization_requires_tcp_2101_success() -> None:
    failed = manual_relocalization_verification_payload(
        tcp_2101_accepted=False,
        tcp_2101_result="failed: ErrorCode=0x0001 初始化定位失败",
        tcp_2101_ambiguous=False,
        localization_ok=True,
        pose_ok=True,
        pose_near_request=True,
        scan_ok=True,
        local_costmap_ok=True,
        global_costmap_ok=True,
    )
    assert_equal(failed["factory_pose_accepted"], False, "manual 2101 failure blocks confirmation")
    assert_equal(failed["tcp_2101_required"], True, "manual 2101 is required")
    assert_equal(failed["tcp_2101_diagnostic_only"], False, "manual 2101 is not diagnostic-only")
    assert_equal(failed["checks"]["manual_tcp_2101"], "fail", "manual check fails")

    ok = manual_relocalization_verification_payload(
        tcp_2101_accepted=True,
        tcp_2101_result="success: x=1.000 y=2.000 z=0.000 yaw=0.100",
        tcp_2101_ambiguous=False,
        localization_ok=True,
        pose_ok=True,
        pose_near_request=True,
        scan_ok=True,
        local_costmap_ok=True,
        global_costmap_ok=True,
    )
    assert_equal(ok["factory_pose_accepted"], True, "manual 2101 and fresh pose confirm localization")
    assert_equal(ok["navigation_ready"], True, "navigation is ready when all checks pass")
    assert_true("2101/1" in ok["message"], "manual reference is visible in success message")

    reply_only = manual_relocalization_verification_payload(
        tcp_2101_accepted=True,
        tcp_2101_result="success: x=1.000 y=2.000 z=0.000 yaw=0.100",
        tcp_2101_ambiguous=False,
        localization_ok=False,
        pose_ok=True,
        pose_near_request=True,
        scan_ok=True,
        local_costmap_ok=True,
        global_costmap_ok=True,
    )
    assert_equal(reply_only["tcp_2101_accepted"], True, "2101 reply is preserved")
    assert_equal(reply_only["factory_pose_accepted"], False, "2101 reply alone does not confirm localization")
    assert_true("返回成功" in reply_only["message"], "partial success message is explicit")

    ambiguous = manual_relocalization_verification_payload(
        tcp_2101_accepted=False,
        tcp_2101_result="pending_verification: ErrorCode=0xFFFF firmware reply ambiguous",
        tcp_2101_ambiguous=True,
        localization_ok=True,
        pose_ok=True,
        pose_near_request=True,
        scan_ok=True,
        local_costmap_ok=True,
        global_costmap_ok=True,
    )
    assert_equal(ambiguous["factory_pose_accepted"], True, "fresh target pose verifies 0xFFFF")
    assert_equal(ambiguous["tcp_2101_accepted"], False, "ambiguous reply remains diagnostic")
    assert_equal(ambiguous["tcp_2101_verified_by_pose"], True, "pose evidence is explicit")
    assert_equal(ambiguous["checks"]["manual_tcp_2101"], "warn", "reply mismatch stays visible")


def main() -> int:
    test_parse_initialpose_request()
    print("[OK] test_parse_initialpose_request")
    test_localization_status_requires_fresh_map_pose()
    print("[OK] test_localization_status_requires_fresh_map_pose")
    test_factory_localization_source_precedence()
    print("[OK] test_factory_localization_source_precedence")
    test_localization_status_reports_motion_away_from_tcp_pose_as_confirmed()
    print("[OK] test_localization_status_reports_motion_away_from_tcp_pose_as_confirmed")
    test_localization_status_uses_factory_status_when_localization_topic_disagrees()
    print("[OK] test_localization_status_uses_factory_status_when_localization_topic_disagrees")
    test_relocalization_sample_evidence()
    print("[OK] test_relocalization_sample_evidence")
    test_cross_floor_platform_yaw_tolerance_is_strict()
    print("[OK] test_cross_floor_platform_yaw_tolerance_is_strict")
    test_relocalization_stability_window_is_continuous()
    print("[OK] test_relocalization_stability_window_is_continuous")
    test_map_relocalization_clearance_uses_strong_current_evidence()
    print("[OK] test_map_relocalization_clearance_uses_strong_current_evidence")
    test_localization_status_explains_success_reply_but_map_lock()
    print("[OK] test_localization_status_explains_success_reply_but_map_lock")
    test_relocalization_response_separates_sent_from_confirmed()
    print("[OK] test_relocalization_response_separates_sent_from_confirmed")
    test_initialpose_api_response_payload()
    print("[OK] test_initialpose_api_response_payload")
    test_manual_relocalization_requires_tcp_2101_success()
    print("[OK] test_manual_relocalization_requires_tcp_2101_success")
    print("[OK] localization contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
