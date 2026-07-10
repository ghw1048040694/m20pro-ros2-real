"""Pure navigation readiness helpers for relocalization and diagnostics."""

from __future__ import annotations

from typing import Any, Callable, Dict, Optional

from .task_contract import readiness_failure, readiness_success


NowText = Callable[[], str]


def _fresh_payload(
    payload: Dict[str, Any],
    *,
    now: float,
    timeout_s: float,
    min_update_time: Optional[float],
    require_points: bool,
) -> Dict[str, Any]:
    last_update = payload.get("last_update")
    if last_update is None:
        return {"ok": False, "age_sec": None, "after_min_update": False}
    try:
        last_update_s = float(last_update)
        age = max(0.0, float(now) - last_update_s)
    except (TypeError, ValueError):
        return {"ok": False, "age_sec": None, "after_min_update": False}
    after_min = min_update_time is None or last_update_s >= float(min_update_time)
    if require_points:
        data_ok = int(payload.get("finite_ranges", 0) or 0) > 0
    else:
        data_ok = bool(payload.get("width")) and bool(payload.get("height"))
    return {
        "ok": bool(age <= float(timeout_s) and after_min and data_ok),
        "age_sec": age,
        "after_min_update": after_min,
    }


def navigation_readiness_payload(
    *,
    scan: Dict[str, Any],
    local_costmap: Dict[str, Any],
    global_costmap: Dict[str, Any],
    lifecycle: Optional[Dict[str, Dict[str, Any]]],
    check_lifecycle: bool,
    timeout_s: float,
    now: float,
    min_update_time: Optional[float],
    now_text: NowText,
) -> Dict[str, Any]:
    scan_fresh = _fresh_payload(
        scan,
        now=now,
        timeout_s=timeout_s,
        min_update_time=min_update_time,
        require_points=True,
    )
    local_fresh = _fresh_payload(
        local_costmap,
        now=now,
        timeout_s=timeout_s,
        min_update_time=min_update_time,
        require_points=False,
    )
    global_fresh = _fresh_payload(
        global_costmap,
        now=now,
        timeout_s=timeout_s,
        min_update_time=min_update_time,
        require_points=False,
    )
    lifecycle_payload = dict(lifecycle or {})
    lifecycle_ok = True
    if check_lifecycle:
        lifecycle_ok = all(item.get("active") for item in lifecycle_payload.values())

    checks: Dict[str, Any] = {
        "scan": {
            "ok": scan_fresh["ok"],
            "age_sec": scan_fresh["age_sec"],
            "finite_ranges": scan.get("finite_ranges"),
            "after_min_update": scan_fresh["after_min_update"],
        },
        "local_costmap": {
            "ok": local_fresh["ok"],
            "age_sec": local_fresh["age_sec"],
            "width": local_costmap.get("width"),
            "height": local_costmap.get("height"),
            "after_min_update": local_fresh["after_min_update"],
        },
        "global_costmap": {
            "ok": global_fresh["ok"],
            "age_sec": global_fresh["age_sec"],
            "width": global_costmap.get("width"),
            "height": global_costmap.get("height"),
            "after_min_update": global_fresh["after_min_update"],
        },
    }
    if min_update_time is not None:
        checks["min_update_time"] = float(min_update_time)
    if check_lifecycle:
        checks["lifecycle"] = lifecycle_payload

    ready = bool(scan_fresh["ok"] and local_fresh["ok"] and global_fresh["ok"] and lifecycle_ok)
    if ready:
        message = "Nav2、/scan 和代价地图已就绪"
    elif min_update_time is not None and not (
        scan_fresh["after_min_update"]
        and local_fresh["after_min_update"]
        and global_fresh["after_min_update"]
    ):
        message = "等待复位后的 /scan 和 local/global costmap 新数据"
    elif not scan_fresh["ok"]:
        message = "导航链路尚未收到新鲜 /scan"
    elif not local_fresh["ok"] or not global_fresh["ok"]:
        message = "已定位但代价地图尚未恢复，等待 local/global costmap"
    else:
        message = "Nav2 lifecycle 尚未全部 active，等待启动门完成"

    return {
        "ready": ready,
        "message": message,
        "timeout_s": float(timeout_s),
        "checks": checks,
        "updated_at": now_text(),
    }


def should_check_navigation_readiness(
    *,
    require_nav_ready: bool,
    require_localization_ok: bool,
    localization_ok: Any,
    pose_is_plausible: bool,
    pose_age_sec: Optional[float],
    pose_timeout_s: float,
) -> bool:
    if not bool(require_nav_ready):
        return False
    if bool(require_localization_ok) and localization_ok is not True:
        return False
    return bool(
        pose_is_plausible
        and pose_age_sec is not None
        and float(pose_age_sec) <= float(pose_timeout_s)
    )


def navigation_readiness_disabled_payload(*, now_text: NowText) -> Dict[str, Any]:
    return readiness_success(
        "当前不要求检查 Nav2 readiness",
        {"required": False},
        now_text=now_text,
    )


def navigation_readiness_wait_timeout_payload(
    *,
    last_ready: Dict[str, Any],
    timeout_s: float,
    min_update_time: Optional[float],
    now_text: NowText,
) -> Dict[str, Any]:
    return readiness_failure(
        "navigation_not_ready_after_reset",
        "任务启动复位后 Nav2/代价地图未在 %.1f 秒内恢复，未下发目标；请重新定位或查看 costmap/Nav2 状态"
        % float(timeout_s),
        {
            "navigation_readiness": dict(last_ready or {}),
            "wait_timeout_s": float(timeout_s),
            "min_update_time": min_update_time,
        },
        now_text=now_text,
    )
