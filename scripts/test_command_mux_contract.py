#!/usr/bin/env python3
"""Offline safety contract for exclusive Nav2/operator velocity arbitration."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.command_mux_contract import (  # noqa: E402
    CommandMuxArbiter,
    normalized_teleop_command,
    teleop_release_decision,
)


LIMITS = {
    "max_forward_speed_mps": 0.18,
    "max_reverse_speed_mps": 0.12,
    "max_lateral_speed_mps": 0.12,
    "max_angular_speed_radps": 0.45,
}


def main() -> None:
    arbiter = CommandMuxArbiter(
        navigation_timeout_s=0.6,
        teleop_timeout_s=0.35,
        teleop_limits=LIMITS,
    )
    assert arbiter.mode == "locked"
    assert not arbiter.accept("navigation", {"linear_x": 0.3}, now=1.0)["publish"]

    transition = arbiter.set_mode("navigation", reason="task_start")
    assert transition["publish"] and transition["command"]["linear_x"] == 0.0
    nav = arbiter.accept("navigation", {"linear_x": 0.3}, now=2.0)
    assert nav["publish"] and nav["command"]["linear_x"] == 0.3
    assert not arbiter.accept("teleop", {"linear_x": 0.1}, now=2.1)["publish"]
    assert not arbiter.watchdog(now=2.5)["publish"]
    timed_out = arbiter.watchdog(now=2.61)
    assert timed_out["publish"] and timed_out["command"]["linear_x"] == 0.0

    arbiter.set_mode("teleop", reason="operator_takeover")
    assert not arbiter.accept("navigation", {"linear_x": 0.3}, now=3.0)["publish"]
    clamped = arbiter.accept(
        "teleop",
        {"linear_x": 1.0, "linear_y": -1.0, "angular_z": 2.0},
        now=3.0,
    )
    assert clamped["command"] == {
        "linear_x": 0.18,
        "linear_y": -0.12,
        "angular_z": 0.45,
    }
    assert arbiter.watchdog(now=3.36)["publish"]
    arbiter.set_mode("locked", reason="operator_release")
    assert not arbiter.accept("navigation", {"linear_x": 0.3}, now=4.0)["publish"]

    reverse = normalized_teleop_command(
        {"linear_x": -1, "linear_y": 0, "angular_z": 0}, **LIMITS
    )
    assert reverse["linear_x"] == -0.12
    diagonal = normalized_teleop_command(
        {"linear_x": 1, "linear_y": 1, "angular_z": 0}, **LIMITS
    )
    assert diagonal["linear_x"] < 0.18 and diagonal["linear_y"] < 0.12
    try:
        normalized_teleop_command(
            {"linear_x": 1.1, "linear_y": 0, "angular_z": 0}, **LIMITS
        )
    except ValueError:
        pass
    else:
        raise AssertionError("out-of-range browser command accepted")

    stale_release = teleop_release_decision(
        active=False,
        force=False,
        request_session_id="old_session",
        active_session_id="old_session",
    )
    assert stale_release["ok"] and not stale_release["lock_mux"]
    wrong_owner = teleop_release_decision(
        active=True,
        force=False,
        request_session_id="old_session",
        active_session_id="new_session",
    )
    assert not wrong_owner["ok"] and not wrong_owner["lock_mux"]
    emergency = teleop_release_decision(
        active=False,
        force=True,
        request_session_id="",
        active_session_id="",
    )
    assert emergency["ok"] and emergency["lock_mux"]

    print("command mux contract tests passed")


if __name__ == "__main__":
    main()
