import math
from typing import Any, Dict, Optional


def _angle_error(a: float, b: float) -> float:
    return abs((float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi)


def stationary_drift_decision(
    *,
    anchor_pose: Optional[Dict[str, Any]],
    candidate: Dict[str, float],
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> Dict[str, Any]:
    """Keep an idle robot anchored instead of accepting cumulative TF drift."""

    if anchor_pose is None:
        return {"accept": True, "reason": "stationary_anchor_started"}

    distance = math.hypot(
        float(candidate["x"]) - float(anchor_pose["x"]),
        float(candidate["y"]) - float(anchor_pose["y"]),
    )
    yaw_error = _angle_error(float(candidate["yaw"]), float(anchor_pose["yaw"]))
    if (
        distance <= max(0.0, float(position_tolerance_m))
        and yaw_error <= max(0.0, float(yaw_tolerance_rad))
    ):
        return {"accept": True, "reason": ""}
    return {
        "accept": False,
        "reason": "stationary_drift_requires_relocalization distance=%.2f yaw=%.2f"
        % (distance, yaw_error),
    }


def stable_jump_decision(
    *,
    last_pose: Optional[Dict[str, Any]],
    pending_pose: Optional[Dict[str, Any]],
    candidate: Dict[str, float],
    now_s: float,
    jump_limit_m: float,
    accept_after_s: float,
    candidate_radius_m: float,
    candidate_yaw_tolerance_rad: float,
    allow_stable_recovery: bool,
) -> Dict[str, Any]:
    """Reject isolated pose jumps while allowing a confirmed stable source to recover."""

    if last_pose is None or jump_limit_m <= 0.0:
        return {"accept": True, "pending_pose": None, "reason": ""}

    distance = math.hypot(
        float(candidate["x"]) - float(last_pose["x"]),
        float(candidate["y"]) - float(last_pose["y"]),
    )
    if distance <= jump_limit_m:
        return {"accept": True, "pending_pose": None, "reason": ""}

    if not allow_stable_recovery or accept_after_s <= 0.0:
        return {
            "accept": False,
            "pending_pose": None,
            "reason": "jump_requires_relocalization distance=%.2f" % distance,
        }

    same_candidate = bool(
        pending_pose is not None
        and math.hypot(
            float(candidate["x"]) - float(pending_pose["x"]),
            float(candidate["y"]) - float(pending_pose["y"]),
        )
        <= max(0.01, float(candidate_radius_m))
        and _angle_error(float(candidate["yaw"]), float(pending_pose["yaw"]))
        <= max(0.01, float(candidate_yaw_tolerance_rad))
    )
    if not same_candidate:
        pending = dict(candidate)
        pending["first_seen_s"] = float(now_s)
        return {
            "accept": False,
            "pending_pose": pending,
            "reason": "jump_candidate_started distance=%.2f" % distance,
        }

    stable_for_s = max(0.0, float(now_s) - float(pending_pose.get("first_seen_s", now_s)))
    if stable_for_s < accept_after_s:
        return {
            "accept": False,
            "pending_pose": pending_pose,
            "reason": "jump_waiting_for_stability distance=%.2f stable_for=%.1fs"
            % (distance, stable_for_s),
        }

    return {
        "accept": True,
        "pending_pose": None,
        "reason": "stable_jump_recovered distance=%.2f stable_for=%.1fs"
        % (distance, stable_for_s),
    }
