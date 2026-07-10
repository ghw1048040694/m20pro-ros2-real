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
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    scan = runtime_state.get("scan") if isinstance(runtime_state.get("scan"), dict) else {}
    scan_age = _payload_age(scan, now)
    finite_ranges = safe_int(scan.get("finite_ranges"), 0)
    scan_ok = bool(scan_age is not None and scan_age <= max(0.1, float(scan_timeout_s)) and finite_ranges > 0)
    ready = scan_ok
    code = "perception_ready" if ready else "scan_unavailable"
    message = (
        "106 edge scan 和 /scan 感知链路已有新鲜数据"
        if ready
        else "尚未收到新鲜 /scan；检查 106 m20pro-edge-scan-106.service 和 DDS 轻量链路"
    )

    return {
        "ready": ready,
        "code": code,
        "message": message,
        "severity": "ok" if ready else "fail",
        "mode": "edge_scan",
        "scan": {
            "ok": scan_ok,
            "age_sec": scan_age,
            "finite_ranges": finite_ranges,
            "frame_id": scan.get("frame_id"),
        },
        "updated_at": (now_text or default_now_text)(),
    }
