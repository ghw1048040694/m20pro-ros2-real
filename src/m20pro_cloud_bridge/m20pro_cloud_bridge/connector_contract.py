"""Versioned identity and safety policy for unified navigation connectors."""

from __future__ import annotations

import math
import time
from typing import Any, Dict


def _text(value: Any) -> str:
    return str(value or "").strip()


def _positive(value: Any) -> Any:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) and number > 0.0 else None


def _corridor_geometry(raw: Dict[str, Any]) -> Any:
    corridor = raw.get("corridor") if isinstance(raw.get("corridor"), dict) else {}
    width = _positive(corridor.get("width_m"))
    lookahead = _positive(corridor.get("lookahead_m"))
    if width is None or lookahead is None:
        return None
    return {"width_m": width, "lookahead_m": lookahead}


def connector_terrain_guard_profile(route: Dict[str, Any]) -> Dict[str, Any]:
    """Return the immutable 106 terrain identity carried by one connector.

    This record deliberately contains identity and policy only.  The
    point-cloud classifier and its geometry thresholds remain a 106-local
    implementation; 104 must never duplicate them or accept another edge's
    profile.  New routes default to a shadow profile and stop-only policy until
    the physical corridor has been recorded and certified.
    """
    route_id = _text(route.get("id")) or "unidentified_connector"
    raw = route.get("terrain_guard") if isinstance(route, dict) else None
    raw = raw if isinstance(raw, dict) else {}
    profile_id = _text(raw.get("profile_id")) or f"{route_id}:terrain"
    corridor_version = _text(raw.get("corridor_version")) or "shadow-v1"
    policy = _text(raw.get("motion_policy")) or "stop_only"
    if policy not in {"stop_only", "certified_connector"}:
        policy = "stop_only"
    return {
        "profile_id": profile_id,
        "corridor_version": corridor_version,
        "motion_policy": policy,
        "certified_motion": bool(raw.get("certified_motion", False))
        and policy == "certified_connector",
        "source": "106_local_pointcloud",
        "corridor": _corridor_geometry(raw),
    }


def terrain_guard_profile_for_route(route_id: Any, payload: Any = None) -> Dict[str, Any]:
    """Normalize a route-edit payload through the same profile rule."""
    raw = payload if isinstance(payload, dict) else {}
    profile = connector_terrain_guard_profile(
        {
            "id": _text(route_id),
            "terrain_guard": raw.get("terrain_guard"),
        }
    )
    # Browser/API route editing is never a certification authority.
    profile["motion_policy"] = "stop_only"
    profile["certified_motion"] = False
    return profile


def connector_terrain_status_decision(
    profile: Dict[str, Any],
    status: Any,
    *,
    now_unix_s: Any = None,
    timeout_s: float = 1.0,
) -> Dict[str, Any]:
    """Validate one fresh 106 status before a connector map transaction.

    This gate authorizes only the *map-switch evidence* that the robot is at a
    traversable connector state.  It never turns the profile into a motion
    lease; motion certification remains a separate, future field-validated
    step.
    """
    if not isinstance(status, dict):
        return {"ok": False, "code": "terrain_guard_status_missing", "message": "缺少 106 terrain_guard 状态"}
    expected_profile = connector_terrain_guard_profile({"id": profile.get("profile_id"), "terrain_guard": profile})
    expected_profile_id = _text(expected_profile.get("profile_id"))
    expected_version = _text(expected_profile.get("corridor_version"))
    status_profile_id = _text(status.get("profile_id"))
    if not status_profile_id:
        route_id = _text(status.get("route_id"))
        status_profile_id = f"{route_id}:terrain" if route_id else ""
    status_version = _text(status.get("corridor_version"))
    if status_profile_id != expected_profile_id or status_version != expected_version:
        return {
            "ok": False,
            "code": "terrain_guard_profile_mismatch",
            "message": "106 terrain_guard profile 与当前连接边不一致",
            "expected_profile_id": expected_profile_id,
            "status_profile_id": status_profile_id or None,
            "expected_corridor_version": expected_version,
            "status_corridor_version": status_version or None,
        }

    state = _text(status.get("state")).lower()
    if state != "traversable":
        return {
            "ok": False,
            "code": "terrain_guard_not_traversable",
            "message": "106 terrain_guard 当前未确认楼梯走廊可通行",
            "state": state or "unknown",
            "reason": _text(status.get("reason")) or "terrain_state_unknown",
        }

    try:
        now_value = time.time() if now_unix_s is None else float(now_unix_s)
        stamp = float(status.get("stamp_unix_s"))
    except (TypeError, ValueError):
        now_value = time.time()
        stamp = float("nan")
    timeout = max(0.1, float(timeout_s))
    age = now_value - stamp if math.isfinite(stamp) else float("inf")
    if not math.isfinite(age) or age < -0.5 or age > timeout:
        return {
            "ok": False,
            "code": "terrain_guard_status_stale",
            "message": "106 terrain_guard 状态已过期",
            "status_age_s": age if math.isfinite(age) else None,
            "timeout_s": timeout,
        }
    try:
        cloud_age = float(status.get("cloud_age_s"))
    except (TypeError, ValueError):
        cloud_age = float("inf")
    if not math.isfinite(cloud_age) or cloud_age > timeout:
        return {
            "ok": False,
            "code": "terrain_guard_cloud_stale",
            "message": "106 terrain_guard 点云已过期",
            "cloud_age_s": cloud_age if math.isfinite(cloud_age) else None,
            "timeout_s": timeout,
        }
    return {
        "ok": True,
        "code": "terrain_guard_traversable",
        "message": "106 terrain_guard 已确认连接边走廊可通行",
        "state": state,
        "reason": _text(status.get("reason")) or "step_profile_continuous",
        "status_age_s": age,
        "cloud_age_s": cloud_age,
        "certified_motion": bool(status.get("certified_motion", False))
        and bool(expected_profile.get("certified_motion", False)),
    }
