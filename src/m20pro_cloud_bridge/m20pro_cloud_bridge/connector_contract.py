"""Versioned identity and safety policy for unified navigation connectors."""

from __future__ import annotations

from typing import Any, Dict


def _text(value: Any) -> str:
    return str(value or "").strip()


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
