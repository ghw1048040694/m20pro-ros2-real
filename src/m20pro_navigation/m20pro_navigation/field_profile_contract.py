"""Single-source field profile validation and platform-specific rendering."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping

import yaml


SCHEMA_VERSION = 1
PROFILE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")

TOP_LEVEL_KEYS = {
    "schema_version",
    "profile_name",
    "scan",
    "stair",
    "stair_safety",
    "navigation",
}
SCAN_KEYS = {
    "height_min_m",
    "height_max_m",
    "publish_hz",
    "bin_hold_s",
    "angle_increment_rad",
    "range_min_m",
    "range_max_m",
}
STAIR_KEYS = {
    "max_step_height_m",
    "obstacle_height_margin_m",
    "forward_min_m",
    "forward_max_m",
    "corridor_half_width_m",
    "min_corridor_points",
    "min_profile_bins",
    "min_obstacle_points",
    "profile_hold_s",
    "mode_timeout_s",
}
STAIR_SAFETY_KEYS = {
    "required_clear_samples",
    "startup_timeout_s",
    "stale_timeout_s",
}
NAVIGATION_KEYS = {
    "controller_frequency_hz",
    "xy_goal_tolerance_m",
    "yaw_goal_tolerance_rad",
    "inflation_radius_m",
}


class FieldProfileError(ValueError):
    pass


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise FieldProfileError(f"{path} must be a mapping")
    return value


def _exact_keys(value: Mapping[str, Any], expected: Iterable[str], path: str) -> None:
    expected_set = set(expected)
    actual = set(value)
    missing = sorted(expected_set - actual)
    unknown = sorted(actual - expected_set)
    if missing:
        raise FieldProfileError(f"{path} missing keys: {', '.join(missing)}")
    if unknown:
        raise FieldProfileError(f"{path} unknown keys: {', '.join(unknown)}")


def _number(
    value: Any,
    path: str,
    *,
    minimum: float,
    maximum: float,
) -> float:
    if isinstance(value, bool):
        raise FieldProfileError(f"{path} must be a number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise FieldProfileError(f"{path} must be a number") from exc
    if not math.isfinite(result) or result < minimum or result > maximum:
        raise FieldProfileError(
            f"{path} must be within [{minimum:g}, {maximum:g}], got {value!r}"
        )
    return result


def _integer(
    value: Any,
    path: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise FieldProfileError(f"{path} must be an integer")
    if value < minimum or value > maximum:
        raise FieldProfileError(
            f"{path} must be an integer within [{minimum}, {maximum}], got {value!r}"
        )
    return value


def validate_field_profile(raw: Any) -> Dict[str, Any]:
    source = _mapping(raw, "profile")
    _exact_keys(source, TOP_LEVEL_KEYS, "profile")

    schema_version = _integer(
        source["schema_version"], "schema_version", minimum=SCHEMA_VERSION, maximum=SCHEMA_VERSION
    )
    profile_name = str(source["profile_name"] or "").strip()
    if not PROFILE_NAME_PATTERN.fullmatch(profile_name):
        raise FieldProfileError(
            "profile_name must use 1-64 ASCII letters, digits, dot, underscore, or dash"
        )

    scan_raw = _mapping(source["scan"], "scan")
    stair_raw = _mapping(source["stair"], "stair")
    safety_raw = _mapping(source["stair_safety"], "stair_safety")
    navigation_raw = _mapping(source["navigation"], "navigation")
    _exact_keys(scan_raw, SCAN_KEYS, "scan")
    _exact_keys(stair_raw, STAIR_KEYS, "stair")
    _exact_keys(safety_raw, STAIR_SAFETY_KEYS, "stair_safety")
    _exact_keys(navigation_raw, NAVIGATION_KEYS, "navigation")

    scan = {
        "height_min_m": _number(scan_raw["height_min_m"], "scan.height_min_m", minimum=-2.0, maximum=0.0),
        "height_max_m": _number(scan_raw["height_max_m"], "scan.height_max_m", minimum=0.1, maximum=3.0),
        "publish_hz": _number(scan_raw["publish_hz"], "scan.publish_hz", minimum=2.0, maximum=12.0),
        "bin_hold_s": _number(scan_raw["bin_hold_s"], "scan.bin_hold_s", minimum=0.2, maximum=1.5),
        "angle_increment_rad": _number(
            scan_raw["angle_increment_rad"],
            "scan.angle_increment_rad",
            minimum=0.002,
            maximum=0.05,
        ),
        "range_min_m": _number(scan_raw["range_min_m"], "scan.range_min_m", minimum=0.05, maximum=1.0),
        "range_max_m": _number(scan_raw["range_max_m"], "scan.range_max_m", minimum=2.0, maximum=30.0),
    }
    if scan["height_min_m"] >= scan["height_max_m"]:
        raise FieldProfileError("scan.height_min_m must be below scan.height_max_m")
    if scan["range_min_m"] >= scan["range_max_m"]:
        raise FieldProfileError("scan.range_min_m must be below scan.range_max_m")

    stair = {
        "max_step_height_m": _number(
            stair_raw["max_step_height_m"],
            "stair.max_step_height_m",
            minimum=0.10,
            maximum=0.40,
        ),
        "obstacle_height_margin_m": _number(
            stair_raw["obstacle_height_margin_m"],
            "stair.obstacle_height_margin_m",
            minimum=0.01,
            maximum=0.15,
        ),
        "forward_min_m": _number(stair_raw["forward_min_m"], "stair.forward_min_m", minimum=0.1, maximum=1.0),
        "forward_max_m": _number(stair_raw["forward_max_m"], "stair.forward_max_m", minimum=0.8, maximum=5.0),
        "corridor_half_width_m": _number(
            stair_raw["corridor_half_width_m"],
            "stair.corridor_half_width_m",
            minimum=0.30,
            maximum=1.50,
        ),
        "min_corridor_points": _integer(
            stair_raw["min_corridor_points"],
            "stair.min_corridor_points",
            minimum=20,
            maximum=100000,
        ),
        "min_profile_bins": _integer(
            stair_raw["min_profile_bins"],
            "stair.min_profile_bins",
            minimum=3,
            maximum=40,
        ),
        "min_obstacle_points": _integer(
            stair_raw["min_obstacle_points"],
            "stair.min_obstacle_points",
            minimum=3,
            maximum=10000,
        ),
        "profile_hold_s": _number(
            stair_raw["profile_hold_s"], "stair.profile_hold_s", minimum=0.2, maximum=0.75
        ),
        "mode_timeout_s": _number(
            stair_raw["mode_timeout_s"], "stair.mode_timeout_s", minimum=1.0, maximum=5.0
        ),
    }
    if stair["forward_min_m"] >= stair["forward_max_m"]:
        raise FieldProfileError("stair.forward_min_m must be below stair.forward_max_m")
    stair["obstacle_height_m"] = (
        stair["max_step_height_m"] + stair["obstacle_height_margin_m"]
    )

    stair_safety = {
        "required_clear_samples": _integer(
            safety_raw["required_clear_samples"],
            "stair_safety.required_clear_samples",
            minimum=1,
            maximum=10,
        ),
        "startup_timeout_s": _number(
            safety_raw["startup_timeout_s"],
            "stair_safety.startup_timeout_s",
            minimum=2.0,
            maximum=20.0,
        ),
        "stale_timeout_s": _number(
            safety_raw["stale_timeout_s"],
            "stair_safety.stale_timeout_s",
            minimum=0.5,
            maximum=3.0,
        ),
    }
    minimum_startup_s = stair_safety["required_clear_samples"] / scan["publish_hz"] + 1.0
    if stair_safety["startup_timeout_s"] < minimum_startup_s:
        raise FieldProfileError(
            "stair_safety.startup_timeout_s is too short for required_clear_samples and scan.publish_hz"
        )
    if stair_safety["stale_timeout_s"] > stair["mode_timeout_s"]:
        raise FieldProfileError(
            "stair_safety.stale_timeout_s must not exceed stair.mode_timeout_s"
        )

    navigation = {
        "controller_frequency_hz": _number(
            navigation_raw["controller_frequency_hz"],
            "navigation.controller_frequency_hz",
            minimum=2.0,
            maximum=20.0,
        ),
        "xy_goal_tolerance_m": _number(
            navigation_raw["xy_goal_tolerance_m"],
            "navigation.xy_goal_tolerance_m",
            minimum=0.10,
            maximum=0.80,
        ),
        "yaw_goal_tolerance_rad": _number(
            navigation_raw["yaw_goal_tolerance_rad"],
            "navigation.yaw_goal_tolerance_rad",
            minimum=0.05,
            maximum=0.80,
        ),
        "inflation_radius_m": _number(
            navigation_raw["inflation_radius_m"],
            "navigation.inflation_radius_m",
            minimum=0.40,
            maximum=2.00,
        ),
    }

    normalized = {
        "schema_version": schema_version,
        "profile_name": profile_name,
        "scan": scan,
        "stair": {key: value for key, value in stair.items() if key != "obstacle_height_m"},
        "stair_safety": stair_safety,
        "navigation": navigation,
    }
    canonical = json.dumps(
        normalized,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("ascii")
    profile_hash = hashlib.sha256(canonical).hexdigest()
    normalized["profile_hash"] = profile_hash
    normalized["stair"]["obstacle_height_m"] = stair["obstacle_height_m"]
    return normalized


def load_field_profile(path: Any) -> Dict[str, Any]:
    profile_path = Path(path).expanduser().resolve()
    try:
        raw = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise FieldProfileError(f"cannot read field profile {profile_path}: {exc}") from exc
    profile = validate_field_profile(raw)
    profile["source_path"] = str(profile_path)
    return profile


def _format_env_value(value: Any) -> str:
    if isinstance(value, float):
        return format(value, ".12g")
    return str(value)


def edge_environment(profile: Mapping[str, Any]) -> Dict[str, Any]:
    scan = profile["scan"]
    stair = profile["stair"]
    return {
        "INPUT_TOPIC": "/LIDAR/POINTS",
        "OUTPUT_TOPIC": "/scan",
        "DURATION_S": 0,
        "DOMAIN": 0,
        "USE_SHM": 0,
        "PREFIX": "rt",
        "HEIGHT_MIN": scan["height_min_m"],
        "HEIGHT_MAX": scan["height_max_m"],
        "MAX_PUBLISH_HZ": scan["publish_hz"],
        "BIN_HOLD_S": scan["bin_hold_s"],
        "MAX_POINTS": 0,
        "FRAME_ID": "m20pro_base_link",
        "ANGLE_INCREMENT": scan["angle_increment_rad"],
        "RANGE_MAX": scan["range_max_m"],
        "RANGE_MIN": scan["range_min_m"],
        "STAIR_SCAN_TOPIC": "/m20pro/stair_obstacle_scan",
        "STAIR_STATUS_TOPIC": "/m20pro/stair_clearance",
        "STAIR_MODE_TOPIC": "/m20pro/stair_perception_mode",
        "STAIR_FORWARD_MIN": stair["forward_min_m"],
        "STAIR_FORWARD_MAX": stair["forward_max_m"],
        "STAIR_HALF_WIDTH": stair["corridor_half_width_m"],
        "STAIR_OBSTACLE_HEIGHT": stair["obstacle_height_m"],
        "STAIR_MAX_STEP_HEIGHT": stair["max_step_height_m"],
        "STAIR_MIN_CORRIDOR_POINTS": stair["min_corridor_points"],
        "STAIR_MIN_PROFILE_BINS": stair["min_profile_bins"],
        "STAIR_MIN_OBSTACLE_POINTS": stair["min_obstacle_points"],
        "STAIR_PROFILE_HOLD_S": stair["profile_hold_s"],
        "STAIR_MODE_TIMEOUT_S": stair["mode_timeout_s"],
        "FIELD_PROFILE_NAME": profile["profile_name"],
        "FIELD_PROFILE_HASH": profile["profile_hash"],
    }


def render_edge_environment(profile: Mapping[str, Any]) -> str:
    lines = [
        "# Generated from m20pro_field_profile.yaml. Do not edit this file.",
        f"# profile_hash={profile['profile_hash']}",
    ]
    lines.extend(
        f"{key}={_format_env_value(value)}"
        for key, value in edge_environment(profile).items()
    )
    return "\n".join(lines) + "\n"
