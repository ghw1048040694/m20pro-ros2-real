#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.localization_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.localization_contract import (  # noqa: E402
    initialpose_api_response_payload,
    localization_status_payload,
    map_relocalization_clearance_payload,
    manual_relocalization_verification_payload,
    pose_tcp_2101_consistency_payload,
    parse_tcp_2101_success_pose,
    parse_initialpose_request,
    relocalization_sample_evidence,
    relocalization_response_payload,
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
        task_readiness={"ready": True, "code": "ready", "message": "任务可启动"},
        now_text=lambda: "fixed-time",
    )
    assert_equal(status["confirmed"], True, "fresh localized pose is confirmed")
    assert_equal(status["task_ready"], True, "task readiness is copied")
    assert_equal(status["code"], "localized_ready", "ready code")
    assert_equal(status["updated_at"], "fixed-time", "timestamp")

    stale = localization_status_payload(
        localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=9.0,
        pose_timeout_s=3.0,
        task_readiness={"ready": True, "code": "ready", "message": "任务可启动"},
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


def test_localization_status_rejects_scaled_tcp_pose_mismatch() -> None:
    status = localization_status_payload(
        localization_ok=True,
        pose={"x": -0.011, "y": -0.003, "z": 0.0, "yaw": -1.55},
        pose_age_sec=0.1,
        pose_timeout_s=3.0,
        relocalization_result={
            "last_update": 100.0,
            "raw": "success: x=-10.923 y=-3.264 z=0.000 yaw=-1.549",
        },
        task_readiness={"ready": True, "code": "ready", "message": "任务可启动"},
        now_time=101.0,
    )
    assert_equal(status["confirmed"], False, "scaled pose mismatch is not confirmed")
    assert_equal(status["code"], "pose_not_near_tcp_2101", "scaled pose mismatch code")
    assert_true(status["pose_error_m"] > 10.0, "scaled pose mismatch distance is visible")
    assert_true("坐标单位" in status["message"], "operator sees coordinate-unit hint")

    consistent = pose_tcp_2101_consistency_payload(
        {"x": -10.9, "y": -3.3, "z": 0.0, "yaw": -1.55},
        "success: x=-10.923 y=-3.264 z=0.000 yaw=-1.549",
    )
    assert_equal(consistent["pose_near_2101"], True, "meter-scale pose matches 2101")


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


def test_localization_status_reports_task_page_blocker() -> None:
    status = localization_status_payload(
        localization_ok=True,
        pose=sample_pose(),
        pose_age_sec=0.2,
        pose_timeout_s=3.0,
        task_readiness={
            "ready": False,
            "code": "navigation_not_ready",
            "message": "代价地图尚未恢复",
        },
    )
    assert_equal(status["confirmed"], True, "localization can be confirmed while task is blocked")
    assert_equal(status["task_ready"], False, "task blocker is copied")
    assert_equal(status["code"], "localized_task_not_ready", "task blocker code")
    assert_true(status["message"].startswith("重定位成功"), "localization success is separate from task blockers")
    assert_true("代价地图尚未恢复" in status["message"], "task blocker message is visible")


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
        task_readiness={
            "ready": False,
            "code": "map_relocalization_required",
            "message": "Nav2 已加载当前固定地图，请先按开发手册2101完成重定位",
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
    assert_true("重定位锁" in status["message"], "map lock blocker is explained")


def test_relocalization_response_separates_sent_from_confirmed() -> None:
    failed = relocalization_response_payload(
        {"factory_pose_accepted": False, "navigation_ready": False, "message": "未看到地图位姿更新"},
        {"ready": False, "code": "localization_not_confirmed", "message": "定位未确认"},
        now_text=lambda: "fixed-time",
    )
    assert_equal(failed["confirmed"], False, "published initialpose is not treated as confirmed")
    assert_equal(failed["code"], "localization_not_confirmed", "unconfirmed code")

    blocked = relocalization_response_payload(
        {"factory_pose_accepted": True, "navigation_ready": False, "message": "定位已确认"},
        {"ready": False, "code": "navigation_not_ready", "message": "代价地图尚未恢复"},
    )
    assert_equal(blocked["confirmed"], True, "factory accepted pose is confirmed")
    assert_equal(blocked["task_ready"], False, "task readiness remains blocked")
    assert_equal(blocked["code"], "localized_task_not_ready", "blocked code")
    assert_true(blocked["message"].startswith("重定位成功"), "task blocker does not change localization success")
    assert_true("代价地图尚未恢复" in blocked["message"], "blocked reason is visible")


def test_initialpose_api_response_payload() -> None:
    response = initialpose_api_response_payload(
        localization_status={
            "confirmed": True,
            "task_ready": False,
            "navigation_ready": False,
            "code": "localized_task_not_ready",
            "message": "重定位成功：定位已确认；但任务页暂不可启动：代价地图尚未恢复",
        },
        verification={"factory_pose_accepted": True, "navigation_ready": False},
        task_readiness={"ready": False, "code": "navigation_not_ready"},
        topic="/initialpose",
        publish_repeats="10",
        frame_id="map",
        floor="F20",
        pose={"x": "1.5", "y": 2, "z": 0, "yaw": "0.3"},
    )
    assert_equal(response["ok"], True, "initialpose API request was accepted for publishing")
    assert_equal(response["confirmed"], True, "confirmed comes from localization status")
    assert_equal(response["task_ready"], False, "task readiness remains separate")
    assert_equal(response["navigation_ready"], False, "navigation readiness remains separate")
    assert_equal(response["code"], "localized_task_not_ready", "status code is copied")
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


def main() -> int:
    test_parse_initialpose_request()
    print("[OK] test_parse_initialpose_request")
    test_localization_status_requires_fresh_map_pose()
    print("[OK] test_localization_status_requires_fresh_map_pose")
    test_localization_status_rejects_scaled_tcp_pose_mismatch()
    print("[OK] test_localization_status_rejects_scaled_tcp_pose_mismatch")
    test_relocalization_sample_evidence()
    print("[OK] test_relocalization_sample_evidence")
    test_map_relocalization_clearance_uses_strong_current_evidence()
    print("[OK] test_map_relocalization_clearance_uses_strong_current_evidence")
    test_localization_status_reports_task_page_blocker()
    print("[OK] test_localization_status_reports_task_page_blocker")
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
