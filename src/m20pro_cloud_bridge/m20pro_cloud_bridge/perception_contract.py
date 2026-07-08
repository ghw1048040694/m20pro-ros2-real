"""Pure perception-chain status helpers for the M20Pro web dashboard."""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional


NowText = Callable[[], str]


def default_now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def safe_float(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _payload_age(payload: Dict[str, Any], now: float) -> Optional[float]:
    last_update = safe_float(payload.get("last_update"))
    return None if last_update is None else max(0.0, now - last_update)


def perception_status_payload(
    runtime_state: Dict[str, Any],
    *,
    now: float,
    scan_timeout_s: float = 2.0,
    lidar_timeout_s: float = 2.0,
    relay_timeout_s: float = 4.0,
    perception_mode: str = "local_fusion",
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    relay = runtime_state.get("lidar_relay_status") if isinstance(runtime_state.get("lidar_relay_status"), dict) else {}
    lidar = runtime_state.get("lidar_points") if isinstance(runtime_state.get("lidar_points"), dict) else {}
    scan = runtime_state.get("scan") if isinstance(runtime_state.get("scan"), dict) else {}

    relay_age = _payload_age(relay, now)
    lidar_age = _payload_age(lidar, now)
    scan_age = _payload_age(scan, now)

    relay_messages = safe_int(relay.get("messages"), 0)
    relay_published = safe_int(relay.get("messages_published"), 0)
    input_publishers = safe_int(relay.get("input_publisher_count"), -1)
    relay_output_points = safe_int(relay.get("output_width"), 0) * max(1, safe_int(relay.get("output_height"), 1))
    lidar_points = safe_int(lidar.get("width"), 0) * max(1, safe_int(lidar.get("height"), 1))
    finite_ranges = safe_int(scan.get("finite_ranges"), 0)

    scan_ok = bool(scan_age is not None and scan_age <= max(0.1, float(scan_timeout_s)) and finite_ranges > 0)
    lidar_ok = bool(lidar_age is not None and lidar_age <= max(0.1, float(lidar_timeout_s)) and lidar_points > 0)
    relay_ok = bool(
        relay_age is not None
        and relay_age <= max(0.1, float(relay_timeout_s))
        and relay_messages > 0
        and relay_published > 0
    )

    mode = str(perception_mode or "local_fusion").strip() or "local_fusion"
    edge_scan_mode = mode == "edge_scan"

    if edge_scan_mode:
        if not scan_ok:
            code = "scan_unavailable"
            message = "edge scan 模式下尚未收到新鲜二维激光；检查 106 edge scan 服务和 DDS 轻量 scan 链路"
            ready = False
            severity = "fail"
        else:
            code = "perception_ready"
            message = "edge scan 和 /scan 感知链路已有新鲜数据"
            ready = True
            severity = "ok"
    elif input_publishers == 0 and relay_messages <= 0:
        code = "factory_lidar_points_publisher_missing"
        message = "原厂 /LIDAR/POINTS 当前没有 DDS publisher；rsdriver 到 ROS2 点云端点未建立，relay 和 /scan 不会有数据"
        ready = False
        severity = "fail"
    elif not relay_ok:
        code = "lidar_relay_no_samples"
        message = "点云 relay 没有收到或发布样本；检查 /LIDAR/POINTS、DDS profile 和 relay 日志"
        ready = False
        severity = "fail"
    elif not lidar_ok:
        code = "lidar_relay_output_unavailable"
        message = "点云 relay 已运行但前端未收到新鲜 relay 点云"
        ready = False
        severity = "fail"
    elif not scan_ok:
        code = "scan_unavailable"
        message = "relay 点云已有数据，但 /scan 尚未产生有效距离；检查 pointcloud_fusion"
        ready = False
        severity = "fail"
    else:
        code = "perception_ready"
        message = "点云 relay 和 /scan 均有新鲜数据"
        ready = True
        severity = "ok"

    return {
        "ready": ready,
        "code": code,
        "message": message,
        "severity": severity,
        "mode": mode,
        "scan": {
            "ok": scan_ok,
            "age_sec": scan_age,
            "finite_ranges": finite_ranges,
            "frame_id": scan.get("frame_id"),
        },
        "lidar_points": {
            "ok": lidar_ok,
            "age_sec": lidar_age,
            "points": lidar_points,
            "source": lidar.get("source"),
            "frame_id": lidar.get("frame_id"),
        },
        "relay": {
            "ok": False if edge_scan_mode else relay_ok,
            "not_used": edge_scan_mode,
            "age_sec": relay_age,
            "input_topic": relay.get("input_topic"),
            "input_publisher_count": input_publishers,
            "messages": relay_messages,
            "messages_published": relay_published,
            "output_points": relay_output_points,
            "cloud_reliability": relay.get("cloud_reliability"),
            "subscription_modes": relay.get("subscription_modes"),
            "last_subscription_mode": relay.get("last_subscription_mode"),
            "input_rate_hz": relay.get("input_rate_hz"),
            "publish_rate_hz": relay.get("publish_rate_hz"),
            "downsample_method": relay.get("downsample_method"),
        },
        "updated_at": (now_text or default_now_text)(),
    }
