"""Pure command-source arbitration for navigation and operator takeover."""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional


COMMAND_MODES = {"locked", "navigation", "teleop"}
ZERO_COMMAND = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}


def _finite(value: Any) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("command values must be finite")
    return number


def normalized_teleop_command(
    payload: Mapping[str, Any],
    *,
    max_forward_speed_mps: float,
    max_reverse_speed_mps: float,
    max_lateral_speed_mps: float,
    max_angular_speed_radps: float,
) -> Dict[str, float]:
    """Validate normalized browser axes and convert them to bounded velocities."""
    axes = {
        "linear_x": _finite(payload.get("linear_x", 0.0)),
        "linear_y": _finite(payload.get("linear_y", 0.0)),
        "angular_z": _finite(payload.get("angular_z", 0.0)),
    }
    if any(abs(value) > 1.0 for value in axes.values()):
        raise ValueError("normalized command axes must be within [-1, 1]")
    linear_norm = math.hypot(axes["linear_x"], axes["linear_y"])
    if linear_norm > 1.0:
        axes["linear_x"] /= linear_norm
        axes["linear_y"] /= linear_norm
    linear_limit = (
        float(max_forward_speed_mps)
        if axes["linear_x"] >= 0.0
        else float(max_reverse_speed_mps)
    )
    return {
        "linear_x": axes["linear_x"] * linear_limit,
        "linear_y": axes["linear_y"] * float(max_lateral_speed_mps),
        "angular_z": axes["angular_z"] * float(max_angular_speed_radps),
    }


def clamp_teleop_velocity(
    command: Mapping[str, Any],
    *,
    max_forward_speed_mps: float,
    max_reverse_speed_mps: float,
    max_lateral_speed_mps: float,
    max_angular_speed_radps: float,
) -> Dict[str, float]:
    linear_x = _finite(command.get("linear_x", 0.0))
    linear_y = _finite(command.get("linear_y", 0.0))
    angular_z = _finite(command.get("angular_z", 0.0))
    linear_x = min(float(max_forward_speed_mps), max(-float(max_reverse_speed_mps), linear_x))
    linear_y = min(float(max_lateral_speed_mps), max(-float(max_lateral_speed_mps), linear_y))
    angular_z = min(float(max_angular_speed_radps), max(-float(max_angular_speed_radps), angular_z))
    return {
        "linear_x": linear_x,
        "linear_y": linear_y,
        "angular_z": angular_z,
    }


def command_is_nonzero(command: Mapping[str, Any]) -> bool:
    return any(abs(float(command.get(key, 0.0))) > 1e-6 for key in ZERO_COMMAND)


def teleop_release_decision(
    *,
    active: bool,
    force: bool,
    request_session_id: str,
    active_session_id: str,
) -> Dict[str, Any]:
    """Prevent a stale browser release from locking a newer navigation task."""
    if not active:
        return {
            "ok": True,
            "release": False,
            "lock_mux": bool(force),
        }
    if not force and str(request_session_id) != str(active_session_id):
        return {
            "ok": False,
            "release": False,
            "lock_mux": False,
            "code": "teleop_session_mismatch",
        }
    return {
        "ok": True,
        "release": True,
        "lock_mux": True,
    }


class CommandMuxArbiter:
    """Fail-closed state machine; source changes always emit a zero command."""

    def __init__(
        self,
        *,
        navigation_timeout_s: float,
        teleop_timeout_s: float,
        teleop_limits: Mapping[str, float],
    ) -> None:
        self.navigation_timeout_s = float(navigation_timeout_s)
        self.teleop_timeout_s = float(teleop_timeout_s)
        self.teleop_limits = dict(teleop_limits)
        self.mode = "locked"
        self.last_source_time: Dict[str, Optional[float]] = {
            "navigation": None,
            "teleop": None,
        }
        self.output_nonzero = False
        self.last_stop_reason = "startup_locked"

    def set_mode(self, mode: str, *, reason: str) -> Dict[str, Any]:
        if mode not in COMMAND_MODES:
            raise ValueError("unknown command mode: %s" % mode)
        previous = self.mode
        self.mode = mode
        self.last_source_time = {"navigation": None, "teleop": None}
        self.output_nonzero = False
        self.last_stop_reason = str(reason or "mode_change")
        return {
            "publish": True,
            "command": dict(ZERO_COMMAND),
            "mode": mode,
            "previous_mode": previous,
            "reason": self.last_stop_reason,
        }

    def accept(self, source: str, command: Mapping[str, Any], *, now: float) -> Dict[str, Any]:
        if source not in ("navigation", "teleop"):
            raise ValueError("unknown command source: %s" % source)
        if source != self.mode:
            return {"publish": False, "mode": self.mode, "source": source}
        try:
            if source == "teleop":
                output = clamp_teleop_velocity(command, **self.teleop_limits)
            else:
                output = {
                    "linear_x": _finite(command.get("linear_x", 0.0)),
                    "linear_y": _finite(command.get("linear_y", 0.0)),
                    "angular_z": _finite(command.get("angular_z", 0.0)),
                }
        except (TypeError, ValueError, OverflowError):
            output = dict(ZERO_COMMAND)
            self.last_stop_reason = "%s_invalid" % source
        self.last_source_time[source] = float(now)
        self.output_nonzero = command_is_nonzero(output)
        return {
            "publish": True,
            "command": output,
            "mode": self.mode,
            "source": source,
        }

    def watchdog(self, *, now: float) -> Dict[str, Any]:
        if self.mode not in ("navigation", "teleop") or not self.output_nonzero:
            return {"publish": False, "mode": self.mode}
        last_update = self.last_source_time.get(self.mode)
        timeout_s = (
            self.navigation_timeout_s if self.mode == "navigation" else self.teleop_timeout_s
        )
        if last_update is not None and float(now) - float(last_update) <= timeout_s:
            return {"publish": False, "mode": self.mode}
        self.output_nonzero = False
        self.last_stop_reason = "%s_timeout" % self.mode
        return {
            "publish": True,
            "command": dict(ZERO_COMMAND),
            "mode": self.mode,
            "reason": self.last_stop_reason,
        }
