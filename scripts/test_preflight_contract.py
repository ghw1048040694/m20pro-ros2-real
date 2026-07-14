#!/usr/bin/env python3
"""Offline tests for m20pro_cloud_bridge.preflight_contract."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.preflight_contract import (  # noqa: E402
    preflight_base_topics_item,
    preflight_context,
    preflight_costmap_items,
    preflight_lifecycle_deferred_item,
    preflight_lifecycle_item,
    preflight_localization_item,
    preflight_map_item,
    preflight_map_pose_item,
    preflight_motion_mode_item,
    preflight_navigation_topics_item,
    preflight_node_item,
    preflight_navigation_status_item,
    preflight_odom_item,
    preflight_perception_items,
    preflight_result_payload,
)


def assert_equal(actual, expected, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


def item(key: str, status: str, group: str = "base", label: Optional[str] = None) -> dict:
    return {"key": key, "label": label or key, "status": status, "message": "", "group": group}


def now_text() -> str:
    return "2026-06-27 04:20:00"


def test_ready_field_navigation() -> None:
    payload = preflight_result_payload(
        [item("nodes", "ok"), item("local_costmap", "ok", "navigation")],
        mode="move",
        site="field",
        workstation_mode=False,
        map_ok=True,
        perception_ok=True,
        timestamp=100.0,
        now_text=now_text,
    )
    assert_equal(payload["ok"], True, "ok")
    assert_equal(payload["navigation_ready"], True, "navigation ready")
    assert_equal(payload["relocalization_ready"], True, "relocalization ready")
    assert_equal(payload["summary"], "基础自检通过，导航已就绪", "summary")
    assert_equal(payload["time_text"], now_text(), "time text")


def test_workstation_navigation_deferred() -> None:
    payload = preflight_result_payload(
        [item("nodes", "ok"), item("local_costmap", "warn", "navigation")],
        mode="move",
        site="auto",
        workstation_mode=True,
        map_ok=True,
        perception_ok=True,
        timestamp=100.0,
        now_text=now_text,
    )
    assert_equal(payload["ok"], True, "base ok")
    assert_equal(payload["navigation_ready"], False, "navigation deferred")
    assert_equal(payload["site"], "workstation", "site")
    assert_equal(payload["site_mode"], "workstation", "site mode")
    assert_equal(payload["summary"], "基础自检通过，导航待重定位后确认", "summary")


def test_perception_chain_failure_is_added() -> None:
    payload = preflight_result_payload(
        [item("nodes", "ok"), item("topics", "ok"), item("map", "ok")],
        mode="move",
        site="field",
        workstation_mode=False,
        map_ok=True,
        perception_ok=False,
        timestamp=100.0,
        now_text=now_text,
    )
    assert_equal(payload["ok"], False, "not ok")
    assert_equal(payload["failures"], 1, "perception failure added")
    assert_equal(payload["items"][-1]["key"], "perception_chain", "items input is not mutated")
    assert_equal(payload["summary"], "基础自检未通过：1 项失败", "summary")


def test_motion_mode_item() -> None:
    move_ok = preflight_motion_mode_item(
        requested_mode="move",
        motion={"mode": "move", "message": "已确认 move：运动控制已放开"},
    )
    assert_equal(move_ok["status"], "ok", "move mode ok")
    assert_equal(move_ok["message"], "已确认 move：运动控制已放开", "move message")

    move_blocked = preflight_motion_mode_item(
        requested_mode="move",
        motion={"mode": "shadow", "message": "当前是 shadow：不会下发运动控制"},
    )
    assert_equal(move_blocked["status"], "fail", "move mode blocks shadow")
    assert_equal(move_blocked["group"], "base", "motion mode blocks base preflight")

    shadow_unknown = preflight_motion_mode_item(
        requested_mode="shadow",
        motion={"mode": "unknown", "message": "未找到全量 real 启动进程"},
    )
    assert_equal(shadow_unknown["status"], "warn", "shadow preflight warns on unknown motion mode")


def test_preflight_context() -> None:
    field = preflight_context(
        {"mode": "move", "site": "field"},
        localization_ok=True,
        navigation_status="location=0; nav=idle",
    )
    assert_equal(field["mode"], "move", "field mode")
    assert_equal(field["site"], "field", "field site")
    assert_equal(field["localized"], True, "field localized")
    assert_equal(field["unlocalized"], False, "field unlocalized")
    assert_equal(field["workstation_mode"], False, "field workstation mode")
    assert_equal(field["defer_nav2_startup_checks"], False, "field lifecycle checks are strict")

    auto_unlocalized = preflight_context(
        {"mode": "bad", "site": "auto"},
        localization_ok=False,
        navigation_status="location=1; nav=idle",
    )
    assert_equal(auto_unlocalized["mode"], "move", "bad mode normalizes to move")
    assert_equal(auto_unlocalized["workstation_mode"], True, "auto unlocalized is workstation-safe")
    assert_equal(auto_unlocalized["defer_nav2_startup_checks"], True, "auto unlocalized defers Nav2 checks")

    explicit_workstation = preflight_context(
        {"mode": "shadow", "site": "desk"},
        localization_ok=True,
        navigation_status="location=0; nav=idle",
    )
    assert_equal(explicit_workstation["mode"], "shadow", "shadow mode remains")
    assert_equal(explicit_workstation["workstation_mode"], True, "explicit desk is workstation")
    assert_equal(explicit_workstation["defer_nav2_startup_checks"], True, "explicit workstation defers Nav2 checks")

    map_locked = preflight_context(
        {"mode": "move", "site": "auto"},
        localization_ok=True,
        navigation_status="location=0; nav=idle",
        map_relocalization_required={"map_id": "builtin_F20"},
    )
    assert_equal(map_locked["localized"], False, "map lock overrides raw factory localization")
    assert_equal(map_locked["unlocalized"], True, "map lock requires relocalization")
    assert_equal(map_locked["relocalization_locked"], True, "map lock is explicit")
    assert_equal(map_locked["defer_nav2_startup_checks"], True, "map lock defers strict Nav2 checks")


def test_node_and_topic_items() -> None:
    nodes_ok = preflight_node_item(
        ["m20pro_tcp_bridge", "map_server"],
        ["m20pro_tcp_bridge", "map_server"],
    )
    assert_equal(nodes_ok["status"], "ok", "nodes ok")
    assert_equal(nodes_ok["message"], "全部在线", "nodes ok message")

    nodes_missing = preflight_node_item(["m20pro_tcp_bridge"], ["m20pro_tcp_bridge", "map_server"])
    assert_equal(nodes_missing["status"], "fail", "missing node fails")
    assert_equal(nodes_missing["message"], "缺少：/map_server", "missing node message")

    topics_ok = preflight_base_topics_item(
        ["/m20pro_tcp_bridge/navigation_status", "/map"],
        ["/m20pro_tcp_bridge/navigation_status", "/map"],
    )
    assert_equal(topics_ok["status"], "ok", "topics ok")
    assert_equal(topics_ok["message"], "全部存在", "topics ok message")

    base_missing = preflight_base_topics_item(
        ["/map"],
        ["/m20pro_tcp_bridge/navigation_status", "/map"],
    )
    assert_equal(base_missing["status"], "fail", "base topic missing fails")
    assert_equal(
        base_missing["message"],
        "缺少：/m20pro_tcp_bridge/navigation_status",
        "base topic missing message",
    )

    navigation_missing = preflight_navigation_topics_item(
        ["/scan"],
        ["/scan", "/m20pro_tcp_bridge/map_pose"],
    )
    assert_equal(navigation_missing["status"], "warn", "navigation topic missing warns")
    assert_equal(
        navigation_missing["message"],
        "重定位后应出现：/m20pro_tcp_bridge/map_pose",
        "navigation topic missing message",
    )
    assert_equal(navigation_missing["group"], "navigation", "navigation topic group")


def test_odom_navigation_status_and_lifecycle_items() -> None:
    odom_ok = preflight_odom_item({}, odom_ok=True, odom_finite=True, age_text="0.2s 前")
    assert_equal(odom_ok["status"], "ok", "odom ok")
    assert_equal(odom_ok["message"], "位姿有效 / 0.2s 前", "odom ok message")

    odom_missing = preflight_odom_item({}, odom_ok=False, odom_finite=False, age_text="")
    assert_equal(odom_missing["status"], "warn", "odom missing warns")
    assert_equal(
        odom_missing["message"],
        "未收到有效 /ODOM；原厂未定位时可能出现 inf/异常坐标",
        "odom missing message",
    )

    nav_ok = preflight_navigation_status_item("location=0; nav=idle")
    assert_equal(nav_ok["status"], "ok", "navigation status ok")
    assert_equal(nav_ok["message"], "location=0; nav=idle", "navigation status text")

    nav_missing = preflight_navigation_status_item("")
    assert_equal(nav_missing["status"], "warn", "navigation status missing warns")
    assert_equal(nav_missing["message"], "暂未收到 navigation_status", "navigation status missing message")

    deferred = preflight_lifecycle_deferred_item()
    assert_equal(deferred["key"], "nav2_lifecycle_deferred", "deferred lifecycle key")
    assert_equal(deferred["status"], "info", "deferred lifecycle info")
    assert_equal(
        deferred["message"],
        "当前在工位/未重定位，Nav2 可由启动门延后激活；重定位后再确认 active",
        "deferred lifecycle message",
    )

    active = preflight_lifecycle_item("/planner_server", {"active": True, "message": "active"})
    assert_equal(active["status"], "ok", "active lifecycle ok")
    assert_equal(active["label"], "/planner_server 生命周期", "active lifecycle label")

    inactive = preflight_lifecycle_item("/planner_server", {"active": False, "message": "inactive"})
    assert_equal(inactive["status"], "warn", "inactive lifecycle warns")
    assert_equal(inactive["message"], "inactive", "inactive lifecycle message")

    locked_localization = preflight_localization_item(
        False,
        map_relocalization_required={"map_id": "builtin_F20"},
    )
    assert_equal(locked_localization["status"], "warn", "map lock warns")
    assert_equal(
        locked_localization["message"],
        "当前地图要求重新定位；完成开发手册 2101 定位前不要开始移动任务",
        "map lock localization message",
    )


def test_perception_items() -> None:
    ready = preflight_perception_items(
        {"finite_ranges": 36},
        scan_ok=True,
        scan_age_text="0.4s 前",
        finite_ranges=36,
    )
    assert_equal(ready["perception_ok"], True, "edge scan makes perception usable")
    assert_equal(ready["items"][0]["status"], "ok", "edge scan item ok")
    assert_equal(ready["items"][1]["status"], "ok", "scan item ok")

    missing = preflight_perception_items(
        {},
        scan_ok=False,
        scan_age_text="",
        finite_ranges=0,
    )
    assert_equal(missing["perception_ok"], False, "missing perception fails")
    assert_equal(missing["items"][0]["status"], "fail", "missing edge scan fails")
    assert_equal(missing["items"][1]["message"], "未收到 /scan；未定位或 TF 未建立时可能暂时没有", "missing scan message")

    malformed = preflight_perception_items(
        {},
        scan_ok=True,
        scan_age_text="0.1s 前",
        finite_ranges="bad",
    )
    assert_equal(malformed["perception_ok"], False, "malformed counts do not crash or pass")
    assert_equal(malformed["finite_ranges"], 0, "malformed scan count defaults to zero")


def test_costmap_items() -> None:
    ready = preflight_costmap_items(
        {"width": 120, "height": 80},
        {"width": 400, "height": 300},
        local_ok=True,
        global_ok=True,
        local_age_text="0.3s 前",
        global_age_text="0.4s 前",
        deferred=False,
    )
    assert_equal(ready[0]["status"], "ok", "local costmap ready")
    assert_equal(ready[0]["message"], "120x80 / 0.3s 前", "local costmap message")
    assert_equal(ready[1]["status"], "ok", "global costmap ready")
    assert_equal(ready[1]["message"], "400x300 / 0.4s 前", "global costmap message")

    deferred = preflight_costmap_items(
        {},
        {},
        local_ok=False,
        global_ok=False,
        local_age_text="",
        global_age_text="",
        deferred=True,
    )
    assert_equal(deferred[0]["status"], "info", "local costmap deferred")
    assert_equal(deferred[1]["status"], "info", "global costmap deferred")
    assert_equal(
        deferred[0]["message"],
        "未重定位前 Nav2/costmap 允许延后启动；先完成重定位再严格检查",
        "deferred costmap message",
    )

    localized_missing = preflight_costmap_items(
        {},
        {},
        local_ok=False,
        global_ok=False,
        local_age_text="",
        global_age_text="",
        deferred=False,
    )
    assert_equal(localized_missing[0]["status"], "warn", "localized local costmap warns")
    assert_equal(localized_missing[0]["message"], "已定位但未收到 local_costmap；不要开始移动任务", "local missing")
    assert_equal(localized_missing[1]["status"], "warn", "localized global costmap warns")
    assert_equal(localized_missing[1]["message"], "已定位但未收到 global_costmap；不要开始移动任务", "global missing")

    malformed = preflight_costmap_items(
        {"width": "bad", "height": 10},
        {"width": 10, "height": None},
        local_ok=True,
        global_ok=True,
        local_age_text="0.1s 前",
        global_age_text="0.1s 前",
        deferred=False,
    )
    assert_equal(malformed[0]["status"], "warn", "malformed local size warns")
    assert_equal(malformed[1]["status"], "warn", "malformed global size warns")


def test_map_item() -> None:
    ok = preflight_map_item({"width": 100, "height": 100})
    assert_equal(ok["status"], "ok", "map loaded")
    assert_equal(ok["message"], "已加载 /map", "map loaded message")

    missing = preflight_map_item({})
    assert_equal(missing["status"], "fail", "missing map fails")
    assert_equal(missing["message"], "未收到 /map", "missing map message")


def test_localization_item() -> None:
    ok = preflight_localization_item(True)
    assert_equal(ok["status"], "ok", "localization ok")
    assert_equal(ok["message"], "localization_ok=true", "localization ok message")
    assert_equal(ok["group"], "navigation", "localization is navigation info")

    warn = preflight_localization_item(False)
    assert_equal(warn["status"], "warn", "localization unconfirmed warns")
    assert_equal(
        warn["message"],
        "当前未重定位；完成定位确认前不要开始移动任务",
        "localization warning message",
    )


def test_map_pose_item() -> None:
    ok = preflight_map_pose_item({"x": 1.234, "y": -2.345}, pose_ok=True, age_text="0.4s 前")
    assert_equal(ok["status"], "ok", "map pose ok")
    assert_equal(ok["message"], "x=1.23 y=-2.35 / 0.4s 前", "map pose message")
    assert_equal(ok["group"], "navigation", "map pose is navigation info")

    missing = preflight_map_pose_item({}, pose_ok=False, age_text="")
    assert_equal(missing["status"], "warn", "missing map pose warns")
    assert_equal(
        missing["message"],
        "未收到有效 /m20pro_tcp_bridge/map_pose；到测试场地后先重定位",
        "missing map pose message",
    )


def main() -> int:
    for test in (
        test_ready_field_navigation,
        test_workstation_navigation_deferred,
        test_perception_chain_failure_is_added,
        test_motion_mode_item,
        test_preflight_context,
        test_node_and_topic_items,
        test_odom_navigation_status_and_lifecycle_items,
        test_perception_items,
        test_costmap_items,
        test_map_item,
        test_localization_item,
        test_map_pose_item,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] preflight contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
