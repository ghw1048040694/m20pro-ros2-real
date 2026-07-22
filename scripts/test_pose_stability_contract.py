#!/usr/bin/env python3
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "m20pro_navigation"))

from m20pro_navigation.pose_stability_contract import (
    stationary_drift_decision,
    stationary_pose_update,
    stable_jump_decision,
)


def decide(last, pending, candidate, now_s, *, allow=True):
    return stable_jump_decision(
        last_pose=last,
        pending_pose=pending,
        candidate=candidate,
        now_s=now_s,
        jump_limit_m=0.6,
        accept_after_s=8.0,
        candidate_radius_m=0.3,
        candidate_yaw_tolerance_rad=0.35,
        allow_stable_recovery=allow,
    )


def main() -> None:
    last = {"x": 1.0, "y": 1.0, "z": 0.0, "yaw": 0.0}
    candidate = {"x": 2.2, "y": 1.0, "z": 0.0, "yaw": 0.1}

    started = decide(last, None, candidate, 10.0)
    assert not started["accept"]
    assert started["reason"].startswith("jump_candidate_started")

    waiting = decide(last, started["pending_pose"], {**candidate, "x": 2.25}, 16.0)
    assert not waiting["accept"]
    assert waiting["reason"].startswith("jump_waiting_for_stability")

    recovered = decide(last, waiting["pending_pose"], {**candidate, "x": 2.24}, 18.2)
    assert recovered["accept"]
    assert recovered["reason"].startswith("stable_jump_recovered")

    unlocalized = decide(last, started["pending_pose"], candidate, 30.0, allow=False)
    assert not unlocalized["accept"]
    assert unlocalized["pending_pose"] is None
    assert unlocalized["reason"].startswith("jump_requires_relocalization")

    discontinuous = decide(last, started["pending_pose"], {**candidate, "x": 3.0}, 30.0)
    assert not discontinuous["accept"]
    assert discontinuous["pending_pose"]["first_seen_s"] == 30.0

    near = decide(last, None, {**candidate, "x": 1.4}, 10.0)
    assert near["accept"]

    # The production bridge must pass zero here; a stable but uncommanded
    # hypothesis is never sufficient evidence to replace the last pose.
    permanently_rejected = stable_jump_decision(
        last_pose=last,
        pending_pose=None,
        candidate=candidate,
        now_s=10.0,
        jump_limit_m=0.6,
        accept_after_s=0.0,
        candidate_radius_m=0.3,
        candidate_yaw_tolerance_rad=0.35,
        allow_stable_recovery=True,
    )
    assert not permanently_rejected["accept"]
    assert permanently_rejected["reason"].startswith("jump_requires_relocalization")

    anchor = {"x": 1.0, "y": 1.0, "yaw": 0.0}
    assert stationary_drift_decision(
        anchor_pose=anchor,
        candidate={"x": 1.1, "y": 1.0, "yaw": 0.1},
        position_tolerance_m=0.2,
        yaw_tolerance_rad=0.3,
    )["accept"]
    drift = stationary_drift_decision(
        anchor_pose=anchor,
        candidate={"x": 1.3, "y": 1.0, "yaw": 0.0},
        position_tolerance_m=0.2,
        yaw_tolerance_rad=0.3,
    )
    assert not drift["accept"]
    assert drift["reason"].startswith("stationary_drift_requires_relocalization")

    anchored = stationary_pose_update(
        anchor_pose=anchor,
        candidate={"x": 1.1, "y": 1.0, "z": 0.4, "yaw": 0.1},
        position_tolerance_m=0.2,
        yaw_tolerance_rad=0.3,
    )
    assert anchored["accept"]
    assert anchored["pose"]["x"] == anchor["x"]
    assert anchored["pose"]["yaw"] == anchor["yaw"]

    held_update = stationary_pose_update(
        anchor_pose=anchor,
        candidate={"x": 1.3, "y": 1.0, "z": 0.0, "yaw": 0.0},
        position_tolerance_m=0.2,
        yaw_tolerance_rad=0.3,
    )
    assert held_update["accept"]
    assert held_update["reason"].startswith("stationary_drift_requires_relocalization")
    assert held_update["pose"]["x"] == anchor["x"]

    print("pose stability contract tests passed")


if __name__ == "__main__":
    main()
