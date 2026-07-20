from __future__ import annotations

import json
import math
from typing import Any, Dict


VALID_STATES = {"clear", "blocked", "unknown"}


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError):
        return default
    return result


def parse_stair_clearance(text: Any, *, received_monotonic: float) -> Dict[str, Any]:
    try:
        payload = json.loads(str(text or ""))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        return {"ok": False, "code": "stair_clearance_invalid_json", "message": str(exc)}
    if not isinstance(payload, dict):
        return {"ok": False, "code": "stair_clearance_not_object", "message": "净空状态不是对象"}
    state = str(payload.get("state") or "").strip().lower()
    session_id = str(payload.get("session_id") or "").strip()
    profile_hash = str(payload.get("profile_hash") or "").strip()
    sequence = _safe_int(payload.get("sequence"), -1)
    if (
        payload.get("active") is not True
        or state not in VALID_STATES
        or not session_id
        or len(profile_hash) != 64
        or any(ch not in "0123456789abcdef" for ch in profile_hash)
        or sequence < 1
    ):
        return {
            "ok": False,
            "code": "stair_clearance_fields_invalid",
            "message": "净空状态缺少 active、session_id、profile_hash、合法 state 或正序列号",
        }
    nearest = payload.get("nearest_obstacle_m")
    try:
        nearest = float(nearest) if nearest is not None else None
    except (TypeError, ValueError):
        nearest = None
    if nearest is not None and not math.isfinite(nearest):
        nearest = None
    return {
        "ok": True,
        "session_id": session_id,
        "profile_hash": profile_hash,
        "state": state,
        "reason": str(payload.get("reason") or ""),
        "sequence": sequence,
        "nearest_obstacle_m": nearest,
        "corridor_points": max(0, _safe_int(payload.get("corridor_points"))),
        "profile_bins": max(0, _safe_int(payload.get("profile_bins"))),
        "obstacle_points": max(0, _safe_int(payload.get("obstacle_points"))),
        "received_monotonic": float(received_monotonic),
        "raw": payload,
    }


def stair_clearance_gate_decision(
    *,
    session_id: str,
    profile_hash: str,
    phase: str,
    sample: Dict[str, Any],
    clear_samples: int,
    required_clear_samples: int,
    started_monotonic: float,
    now_monotonic: float,
    startup_timeout_s: float,
    stale_timeout_s: float,
) -> Dict[str, Any]:
    if not session_id:
        return {"action": "idle"}
    elapsed = max(0.0, float(now_monotonic) - float(started_monotonic))
    matching_session = bool(sample.get("ok")) and str(sample.get("session_id") or "") == str(
        session_id
    )
    if matching_session and str(sample.get("profile_hash") or "") != str(profile_hash):
        return {"action": "abort", "reason": "stair_profile_mismatch"}
    matching = matching_session and str(sample.get("profile_hash") or "") == str(profile_hash)
    if phase in ("waiting_traverse", "waiting_exit"):
        if matching and sample.get("state") == "blocked":
            return {"action": "abort", "reason": "stair_clearance_blocked"}
        if matching and sample.get("state") == "clear" and clear_samples >= max(1, required_clear_samples):
            return {"action": "start_motion"}
        if elapsed >= max(0.1, float(startup_timeout_s)):
            reason = "stair_clearance_unknown" if matching else "stair_clearance_timeout"
            return {"action": "abort", "reason": reason}
        return {"action": "wait"}
    if phase in ("traversing", "exiting"):
        if not matching:
            return {"action": "abort", "reason": "stair_clearance_missing"}
        age = max(0.0, float(now_monotonic) - float(sample.get("received_monotonic") or 0.0))
        if age > max(0.1, float(stale_timeout_s)):
            return {"action": "abort", "reason": "stair_clearance_stale", "age_s": age}
        if sample.get("state") != "clear":
            return {
                "action": "abort",
                "reason": "stair_clearance_blocked" if sample.get("state") == "blocked" else "stair_clearance_unknown",
            }
        return {"action": "continue"}
    return {"action": "hold"}
