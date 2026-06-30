#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.navigation_readiness_contract."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.navigation_readiness_contract import (  # noqa: E402
    navigation_readiness_disabled_payload,
    navigation_readiness_payload,
    navigation_readiness_wait_timeout_payload,
    should_check_navigation_readiness,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def scan(**extra) -> dict:
    payload = {"last_update": 99.0, "finite_ranges": 80}
    payload.update(extra)
    return payload


def grid(**extra) -> dict:
    payload = {"last_update": 99.0, "width": 100, "height": 100}
    payload.update(extra)
    return payload


def lifecycle(active: bool = True) -> dict:
    return {
        "/map_server": {"active": active, "message": "ok" if active else "inactive"},
        "/controller_server": {"active": True, "message": "ok"},
    }


def payload(**overrides) -> dict:
    base = {
        "scan": scan(),
        "local_costmap": grid(),
        "global_costmap": grid(),
        "lifecycle": lifecycle(),
        "check_lifecycle": True,
        "timeout_s": 2.0,
        "now": 100.0,
        "min_update_time": None,
        "now_text": lambda: "fixed-time",
    }
    base.update(overrides)
    return navigation_readiness_payload(**base)


def test_ready() -> None:
    result = payload()
    assert_equal(result["ready"], True, "ready")
    assert_equal(result["message"], "Nav2、/scan 和代价地图已就绪", "ready message")
    assert_equal(result["updated_at"], "fixed-time", "timestamp")


def test_scan_missing() -> None:
    result = payload(scan=scan(finite_ranges=0))
    assert_equal(result["ready"], False, "scan missing not ready")
    assert_equal(result["message"], "导航链路尚未收到新鲜 /scan", "scan message")


def test_costmap_missing() -> None:
    result = payload(local_costmap=grid(width=0))
    assert_equal(result["ready"], False, "costmap missing not ready")
    assert_equal(result["message"], "已定位但代价地图尚未恢复，等待 local/global costmap", "costmap message")


def test_lifecycle_inactive() -> None:
    result = payload(lifecycle=lifecycle(active=False))
    assert_equal(result["ready"], False, "inactive lifecycle not ready")
    assert_equal(result["message"], "Nav2 lifecycle 尚未全部 active，等待启动门完成", "lifecycle message")


def test_waits_for_post_reset_data() -> None:
    result = payload(min_update_time=100.5)
    assert_equal(result["ready"], False, "old data after reset not ready")
    assert_equal(result["message"], "等待复位后的 /scan 和 local/global costmap 新数据", "post-reset message")
    assert_equal(result["checks"]["min_update_time"], 100.5, "min update time recorded")


def test_lifecycle_check_can_be_disabled() -> None:
    result = payload(lifecycle=None, check_lifecycle=False)
    assert_equal(result["ready"], True, "ready without lifecycle check")
    assert_equal("lifecycle" in result["checks"], False, "lifecycle omitted")


def test_should_check_navigation_readiness() -> None:
    assert_equal(
        should_check_navigation_readiness(
            require_nav_ready=False,
            require_localization_ok=True,
            localization_ok=True,
            pose_is_plausible=True,
            pose_age_sec=0.2,
            pose_timeout_s=2.0,
        ),
        False,
        "disabled nav readiness check",
    )
    assert_equal(
        should_check_navigation_readiness(
            require_nav_ready=True,
            require_localization_ok=True,
            localization_ok=False,
            pose_is_plausible=True,
            pose_age_sec=0.2,
            pose_timeout_s=2.0,
        ),
        False,
        "unlocalized blocks navigation readiness check",
    )
    assert_equal(
        should_check_navigation_readiness(
            require_nav_ready=True,
            require_localization_ok=False,
            localization_ok=False,
            pose_is_plausible=False,
            pose_age_sec=0.2,
            pose_timeout_s=2.0,
        ),
        False,
        "implausible pose blocks navigation readiness check",
    )
    assert_equal(
        should_check_navigation_readiness(
            require_nav_ready=True,
            require_localization_ok=False,
            localization_ok=False,
            pose_is_plausible=True,
            pose_age_sec=3.0,
            pose_timeout_s=2.0,
        ),
        False,
        "stale pose blocks navigation readiness check",
    )
    assert_equal(
        should_check_navigation_readiness(
            require_nav_ready=True,
            require_localization_ok=True,
            localization_ok=True,
            pose_is_plausible=True,
            pose_age_sec=0.2,
            pose_timeout_s=2.0,
        ),
        True,
        "fresh localized pose enables navigation readiness check",
    )


def test_navigation_readiness_disabled_payload() -> None:
    result = navigation_readiness_disabled_payload(now_text=lambda: "fixed-time")
    assert_equal(result["ready"], True, "disabled readiness is ready")
    assert_equal(result["code"], "ready", "disabled readiness code")
    assert_equal(result["required"], False, "disabled readiness required flag")
    assert_equal(result["message"], "任务启动前不要求 Nav2 readiness", "disabled readiness message")
    assert_equal(result["updated_at"], "fixed-time", "disabled readiness timestamp")


def test_navigation_readiness_wait_timeout_payload() -> None:
    last_ready = payload(scan=scan(finite_ranges=0))
    result = navigation_readiness_wait_timeout_payload(
        last_ready=last_ready,
        timeout_s=3.5,
        min_update_time=100.5,
        now_text=lambda: "fixed-time",
    )
    assert_equal(result["ready"], False, "timeout readiness is not ready")
    assert_equal(result["code"], "navigation_not_ready_after_reset", "timeout code")
    assert_equal(
        result["message"],
        "任务启动复位后 Nav2/代价地图未在 3.5 秒内恢复，未下发目标；请重新定位或查看 costmap/Nav2 状态",
        "timeout message",
    )
    assert_equal(result["navigation_readiness"]["message"], "导航链路尚未收到新鲜 /scan", "last readiness retained")
    assert_equal(result["wait_timeout_s"], 3.5, "timeout seconds")
    assert_equal(result["min_update_time"], 100.5, "min update time retained")
    assert_equal(result["updated_at"], "fixed-time", "timeout timestamp")


def main() -> int:
    for test in (
        test_ready,
        test_scan_missing,
        test_costmap_missing,
        test_lifecycle_inactive,
        test_waits_for_post_reset_data,
        test_lifecycle_check_can_be_disabled,
        test_should_check_navigation_readiness,
        test_navigation_readiness_disabled_payload,
        test_navigation_readiness_wait_timeout_payload,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] navigation readiness contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
