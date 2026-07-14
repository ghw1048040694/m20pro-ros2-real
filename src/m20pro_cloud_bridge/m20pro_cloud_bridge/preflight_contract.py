"""Pure preflight summary helpers for the M20Pro web dashboard."""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional


NowText = Callable[[], str]


def preflight_context(
    payload: Dict[str, Any],
    *,
    localization_ok: Any,
    navigation_status: Any,
    map_relocalization_required: Any = None,
) -> Dict[str, Any]:
    mode = str((payload or {}).get("mode") or "move").strip()
    if mode not in ("move", "shadow"):
        mode = "move"
    site = str((payload or {}).get("site") or "auto").strip().lower()
    explicit_workstation = site in ("workstation", "bench", "desk", "office", "charging")
    auto_site = site in ("", "auto", "unknown")
    nav_status_text = str(navigation_status or "")
    relocalization_locked = bool(map_relocalization_required)
    localized = localization_ok is True and not relocalization_locked
    unlocalized = (not localized) or ("location=1" in nav_status_text.lower())
    workstation_mode = explicit_workstation or (auto_site and unlocalized)
    return {
        "mode": mode,
        "site": site,
        "navigation_status_text": nav_status_text,
        "localized": localized,
        "unlocalized": unlocalized,
        "relocalization_locked": relocalization_locked,
        "workstation_mode": workstation_mode,
        "defer_nav2_startup_checks": workstation_mode or unlocalized,
    }


def _nonnegative_int(value: Any, default: int = 0) -> int:
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return default


def preflight_node_item(node_names: List[str], required_nodes: List[str]) -> Dict[str, Any]:
    available = set(str(name) for name in node_names)
    missing = [str(name) for name in required_nodes if str(name) not in available]
    return {
        "key": "nodes",
        "label": "核心节点",
        "status": "ok" if not missing else "fail",
        "message": "全部在线" if not missing else "缺少：" + "、".join(f"/{name}" for name in missing),
        "group": "base",
    }


def _preflight_topic_item(
    topic_names: List[str],
    required_topics: List[str],
    *,
    key: str,
    label: str,
    missing_message_prefix: str,
    missing_status: str,
    group: str,
) -> Dict[str, Any]:
    available = set(str(name) for name in topic_names)
    missing = [str(topic) for topic in required_topics if str(topic) not in available]
    return {
        "key": key,
        "label": label,
        "status": "ok" if not missing else missing_status,
        "message": "全部存在" if not missing else missing_message_prefix + "、".join(missing),
        "group": group,
    }


def preflight_base_topics_item(topic_names: List[str], required_topics: List[str]) -> Dict[str, Any]:
    return _preflight_topic_item(
        topic_names,
        required_topics,
        key="topics",
        label="基础话题",
        missing_message_prefix="缺少：",
        missing_status="fail",
        group="base",
    )


def preflight_navigation_topics_item(topic_names: List[str], required_topics: List[str]) -> Dict[str, Any]:
    return _preflight_topic_item(
        topic_names,
        required_topics,
        key="navigation_topics",
        label="导航话题",
        missing_message_prefix="重定位后应出现：",
        missing_status="warn",
        group="navigation",
    )


def preflight_odom_item(
    odom: Dict[str, Any],
    *,
    odom_ok: bool,
    odom_finite: bool,
    age_text: str,
) -> Dict[str, Any]:
    return {
        "key": "odom",
        "label": "原厂里程计",
        "status": "ok" if odom_ok and odom_finite else "warn",
        "message": (
            f"位姿有效 / {age_text}"
            if age_text and odom_finite
            else "未收到有效 /ODOM；原厂未定位时可能出现 inf/异常坐标"
        ),
        "group": "navigation",
    }


def preflight_navigation_status_item(navigation_status: Any) -> Dict[str, Any]:
    text = str(navigation_status or "")
    return {
        "key": "navigation_status",
        "label": "原厂导航状态",
        "status": "ok" if text else "warn",
        "message": text or "暂未收到 navigation_status",
        "group": "base",
    }


def preflight_lifecycle_deferred_item() -> Dict[str, Any]:
    return {
        "key": "nav2_lifecycle_deferred",
        "label": "Nav2 生命周期",
        "status": "info",
        "message": "当前在工位/未重定位，Nav2 可由启动门延后激活；重定位后再确认 active",
        "group": "navigation",
    }


def preflight_lifecycle_item(node_name: str, lifecycle: Dict[str, Any]) -> Dict[str, Any]:
    name = str(node_name)
    return {
        "key": f"lifecycle:{name}",
        "label": f"{name} 生命周期",
        "status": "ok" if (lifecycle or {}).get("active") else "warn",
        "message": str((lifecycle or {}).get("message", "")),
        "group": "navigation",
    }


def preflight_perception_items(
    scan: Dict[str, Any],
    *,
    scan_ok: bool,
    scan_age_text: str,
    finite_ranges: int,
) -> Dict[str, Any]:
    scan_ranges = _nonnegative_int(finite_ranges)
    perception_ok = bool(scan_ok and scan_ranges > 0)
    scan_message = (
        f"有效距离 {scan_ranges} / {scan_age_text}"
        if scan_age_text
        else "未收到 /scan；未定位或 TF 未建立时可能暂时没有"
    )
    return {
        "perception_ok": perception_ok,
        "finite_ranges": scan_ranges,
        "items": [
            {
                "key": "edge_scan",
                "label": "106 边缘激光",
                "status": "ok" if perception_ok else "fail",
                "message": (
                    f"edge scan 已输出 /scan，有效距离 {scan_ranges} / {scan_age_text}"
                    if perception_ok
                    else "edge scan 尚未输出可用 /scan；检查 106 m20pro-edge-scan-106.service"
                ),
                "group": "base",
            },
            {
                "key": "scan",
                "label": "二维激光",
                "status": "ok" if scan_ok and scan_ranges > 0 else "warn",
                "message": scan_message,
                "group": "navigation",
            },
        ],
    }


def preflight_costmap_items(
    local_costmap: Dict[str, Any],
    global_costmap: Dict[str, Any],
    *,
    local_ok: bool,
    global_ok: bool,
    local_age_text: str,
    global_age_text: str,
    deferred: bool,
) -> List[Dict[str, Any]]:
    def costmap_item(
        key: str,
        label: str,
        payload: Dict[str, Any],
        *,
        fresh: bool,
        age_text: str,
        missing_message: str,
    ) -> Dict[str, Any]:
        width = _nonnegative_int((payload or {}).get("width", 0)) if isinstance(payload, dict) else 0
        height = _nonnegative_int((payload or {}).get("height", 0)) if isinstance(payload, dict) else 0
        size_ok = width > 0 and height > 0
        if deferred:
            status = "ok" if fresh and size_ok else "info"
            missing = "未重定位前 Nav2/costmap 允许延后启动；先完成重定位再严格检查"
        else:
            status = "ok" if fresh and size_ok else "warn"
            missing = missing_message
        message = f"{width}x{height} / {age_text}" if isinstance(payload, dict) and size_ok else missing
        return {
            "key": key,
            "label": label,
            "status": status,
            "message": message,
            "group": "navigation",
        }

    return [
        costmap_item(
            "local_costmap",
            "局部代价地图",
            local_costmap,
            fresh=local_ok,
            age_text=local_age_text,
            missing_message="已定位但未收到 local_costmap；不要开始移动任务",
        ),
        costmap_item(
            "global_costmap",
            "全局代价地图",
            global_costmap,
            fresh=global_ok,
            age_text=global_age_text,
            missing_message="已定位但未收到 global_costmap；不要开始移动任务",
        ),
    ]


def preflight_map_item(map_payload: Dict[str, Any]) -> Dict[str, Any]:
    map_available = isinstance(map_payload, dict) and bool(map_payload)
    return {
        "key": "map",
        "label": "地图",
        "status": "ok" if map_available else "fail",
        "message": "已加载 /map" if map_available else "未收到 /map",
        "group": "base",
    }


def preflight_map_pose_item(
    pose: Dict[str, Any],
    *,
    pose_ok: bool,
    age_text: str,
) -> Dict[str, Any]:
    if pose_ok:
        message = "x=%.2f y=%.2f / %s" % (
            float(pose.get("x", 0.0)),
            float(pose.get("y", 0.0)),
            str(age_text or "未知"),
        )
    else:
        message = "未收到有效 /m20pro_tcp_bridge/map_pose；到测试场地后先重定位"
    return {
        "key": "map_pose",
        "label": "地图位姿",
        "status": "ok" if pose_ok else "warn",
        "message": message,
        "group": "navigation",
    }


def preflight_localization_item(
    localization_ok: Any,
    *,
    map_relocalization_required: Any = None,
) -> Dict[str, Any]:
    confirmed = localization_ok is True
    if confirmed:
        message = "localization_ok=true"
    elif map_relocalization_required:
        message = "当前地图要求重新定位；完成开发手册 2101 定位前不要开始移动任务"
    else:
        message = "当前未重定位；完成定位确认前不要开始移动任务"
    return {
        "key": "localization",
        "label": "定位状态",
        "status": "ok" if confirmed else "warn",
        "message": message,
        "group": "navigation",
    }


def preflight_motion_mode_item(*, requested_mode: str, motion: Dict[str, Any]) -> Dict[str, Any]:
    detected_mode = str((motion or {}).get("mode") or "").strip()
    message = str((motion or {})["message"])
    if str(requested_mode or "").strip() == "move":
        return {
            "key": "motion_mode",
            "label": "运动模式",
            "status": "ok" if detected_mode == "move" else "fail",
            "message": message,
            "group": "base",
        }
    return {
        "key": "motion_mode",
        "label": "运动模式",
        "status": "ok" if detected_mode in ("shadow", "move") else "warn",
        "message": message,
        "group": "base",
    }


def preflight_result_payload(
    items: List[Dict[str, Any]],
    *,
    mode: str,
    site: str,
    workstation_mode: bool,
    map_ok: bool,
    perception_ok: bool,
    timestamp: float,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    output_items = [dict(item) for item in items]
    failures = [item for item in output_items if item.get("status") == "fail" and item.get("group") == "base"]
    if not perception_ok and not any(item.get("key") == "lidar_points" for item in failures):
        perception_failure = {
            "key": "perception_chain",
            "label": "感知链路",
            "status": "fail",
            "message": "106 edge scan 和 /scan 不可用",
            "group": "base",
        }
        output_items.append(perception_failure)
        failures.append(perception_failure)
    navigation_failures = [
        item
        for item in output_items
        if item.get("group") == "navigation" and item.get("status") in ("fail", "warn")
    ]
    warnings = [item for item in output_items if item.get("status") == "warn"]
    relocalization_blockers = [
        item
        for item in failures
        if item.get("key") in ("nodes", "topics", "lidar_points", "perception_chain", "map")
    ]
    relocalization_ready = bool(map_ok and perception_ok and not relocalization_blockers)
    failure_labels = "、".join(str(item.get("label") or item.get("key")) for item in failures)
    if not failures:
        summary = (
            "基础自检通过，导航已就绪"
            if not navigation_failures
            else "基础自检通过，导航待重定位后确认"
        )
    elif relocalization_ready:
        summary = (
            f"基础自检未通过：{len(failures)} 项失败"
            f"（{failure_labels}）；地图/点云/scan 可用，仍可先做重定位排查，不要开始移动任务"
        )
    else:
        summary = f"基础自检未通过：{len(failures)} 项失败"
    site_value = "workstation" if workstation_mode else site
    return {
        "ok": not failures,
        "navigation_ready": not navigation_failures,
        "relocalization_ready": relocalization_ready,
        "mode": mode,
        "site": site_value,
        "site_mode": "workstation" if workstation_mode else "field",
        "workstation_mode": workstation_mode,
        "timestamp": timestamp,
        "time_text": (now_text or (lambda: ""))(),
        "age_sec": 0.0,
        "items": output_items,
        "failures": len(failures),
        "navigation_warnings": len(navigation_failures),
        "warnings": len(warnings),
        "summary": summary,
    }
