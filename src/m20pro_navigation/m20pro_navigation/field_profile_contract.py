"""Single-source field profile validation and platform-specific rendering."""

from __future__ import annotations

import hashlib
import json
import math
import re
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping

import yaml


SCHEMA_VERSION = 3
PROFILE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")

TOP_LEVEL_KEYS = {
    "schema_version",
    "profile_name",
    "scan",
    "stair",
    "stair_safety",
    "stair_transition",
    "navigation",
    "teleoperation",
    "localization",
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
    "controller",
    "goal",
    "progress",
    "local_planner",
    "costmap",
    "global_planner",
}

STAIR_TRANSITION_SPEC = {
    "entry_tolerance_m": ("number", 0.30, 2.00),
    "floor_switch_timeout_s": ("number", 30.0, 300.0),
    "post_switch_goal_delay_s": ("number", 0.10, 10.0),
    "duplicate_goal_tolerance_m": ("number", 0.01, 0.50),
    "duplicate_goal_yaw_tolerance_rad": ("number", 0.02, 0.80),
}
CONTROLLER_SPEC = {
    "frequency_hz": ("number", 2.0, 20.0),
    "max_linear_speed_mps": ("number", 0.10, 0.80),
    "max_angular_speed_radps": ("number", 0.40, 1.20),
    "linear_acceleration_limit_mps2": ("number", 0.10, 5.00),
    "angular_acceleration_limit_radps2": ("number", 0.10, 5.00),
    "linear_deceleration_limit_mps2": ("number", 0.10, 5.00),
    "angular_deceleration_limit_radps2": ("number", 0.10, 5.00),
    "stopped_linear_speed_mps": ("number", 0.01, 0.30),
    "recovery_min_angular_speed_radps": ("number", 0.10, 0.80),
    "recovery_simulation_time_s": ("number", 0.50, 5.00),
}
GOAL_SPEC = {
    "xy_tolerance_m": ("number", 0.10, 0.80),
    "yaw_tolerance_rad": ("number", 0.05, 0.80),
}
PROGRESS_SPEC = {
    "required_movement_radius_m": ("number", 0.02, 0.50),
    "movement_time_allowance_s": ("number", 3.0, 60.0),
}
LOCAL_PLANNER_SPEC = {
    "simulation_time_s": ("number", 0.50, 5.00),
    "linear_velocity_samples": ("integer", 3, 50),
    "angular_velocity_samples": ("integer", 5, 80),
}
COSTMAP_SPEC = {
    "local_update_frequency_hz": ("number", 2.0, 20.0),
    "local_publish_frequency_hz": ("number", 0.5, 10.0),
    "global_update_frequency_hz": ("number", 0.2, 10.0),
    "global_publish_frequency_hz": ("number", 0.1, 5.0),
    "obstacle_range_m": ("number", 1.0, 8.0),
    "raytrace_range_m": ("number", 1.2, 10.0),
    "observation_persistence_s": ("number", 0.0, 2.0),
    "inflation_radius_m": ("number", 0.40, 2.00),
    "inflation_cost_scaling_factor": ("number", 0.50, 10.0),
}
GLOBAL_PLANNER_SPEC = {
    "expected_frequency_hz": ("number", 0.10, 10.0),
    "goal_tolerance_m": ("number", 0.10, 2.00),
}
TELEOPERATION_SPEC = {
    "watchdog_rate_hz": ("number", 10.0, 50.0),
    "command_timeout_s": ("number", 0.15, 1.0),
    "navigation_timeout_s": ("number", 0.20, 2.0),
    "max_forward_speed_mps": ("number", 0.05, 0.40),
    "max_reverse_speed_mps": ("number", 0.05, 0.30),
    "max_lateral_speed_mps": ("number", 0.05, 0.30),
    "max_angular_speed_radps": ("number", 0.10, 0.80),
}
LOCALIZATION_SPEC = {
    "tf_fallback_max_age_s": ("number", 0.20, 5.00),
    "pose_jump_reject_m": ("number", 0.20, 3.00),
    "pose_jump_candidate_radius_m": ("number", 0.05, 1.00),
    "pose_jump_candidate_yaw_tolerance_rad": ("number", 0.05, 1.50),
    "stationary_drift_reject_m": ("number", 0.05, 1.00),
    "stationary_drift_reject_yaw_rad": ("number", 0.05, 1.00),
    "motion_command_hold_s": ("number", 0.10, 5.00),
    "command_linear_deadband_mps": ("number", 0.005, 0.20),
    "command_angular_deadband_rad_s": ("number", 0.01, 0.50),
    "filter_hold_last_good_s": ("number", 0.10, 5.00),
    "relocalization_jump_grace_s": ("number", 0.50, 10.0),
    "relocalization_jump_grace_radius_m": ("number", 0.20, 5.00),
    "odom_rebase_jump_m": ("number", 0.20, 3.00),
    "odom_rebase_jump_yaw_rad": ("number", 0.20, 3.14),
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


def _validated_group(
    value: Mapping[str, Any],
    path: str,
    specification: Mapping[str, Any],
) -> Dict[str, Any]:
    _exact_keys(value, specification, path)
    result: Dict[str, Any] = {}
    for key, (kind, minimum, maximum) in specification.items():
        field_path = f"{path}.{key}"
        if kind == "integer":
            result[key] = _integer(
                value[key], field_path, minimum=int(minimum), maximum=int(maximum)
            )
        else:
            result[key] = _number(
                value[key], field_path, minimum=float(minimum), maximum=float(maximum)
            )
    return result


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
    transition_raw = _mapping(source["stair_transition"], "stair_transition")
    navigation_raw = _mapping(source["navigation"], "navigation")
    teleoperation_raw = _mapping(source["teleoperation"], "teleoperation")
    localization_raw = _mapping(source["localization"], "localization")
    _exact_keys(scan_raw, SCAN_KEYS, "scan")
    _exact_keys(stair_raw, STAIR_KEYS, "stair")
    _exact_keys(safety_raw, STAIR_SAFETY_KEYS, "stair_safety")
    _exact_keys(navigation_raw, NAVIGATION_KEYS, "navigation")

    controller_raw = _mapping(navigation_raw["controller"], "navigation.controller")
    goal_raw = _mapping(navigation_raw["goal"], "navigation.goal")
    progress_raw = _mapping(navigation_raw["progress"], "navigation.progress")
    local_planner_raw = _mapping(
        navigation_raw["local_planner"], "navigation.local_planner"
    )
    costmap_raw = _mapping(navigation_raw["costmap"], "navigation.costmap")
    global_planner_raw = _mapping(
        navigation_raw["global_planner"], "navigation.global_planner"
    )

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

    stair_transition = _validated_group(
        transition_raw, "stair_transition", STAIR_TRANSITION_SPEC
    )

    controller = _validated_group(
        controller_raw, "navigation.controller", CONTROLLER_SPEC
    )
    if controller["stopped_linear_speed_mps"] >= controller["max_linear_speed_mps"]:
        raise FieldProfileError(
            "navigation.controller.stopped_linear_speed_mps must be below max_linear_speed_mps"
        )
    if (
        controller["recovery_min_angular_speed_radps"]
        > controller["max_angular_speed_radps"]
    ):
        raise FieldProfileError(
            "navigation.controller.recovery_min_angular_speed_radps must not exceed max_angular_speed_radps"
        )

    goal = _validated_group(goal_raw, "navigation.goal", GOAL_SPEC)
    progress = _validated_group(progress_raw, "navigation.progress", PROGRESS_SPEC)
    if progress["required_movement_radius_m"] > goal["xy_tolerance_m"]:
        raise FieldProfileError(
            "navigation.progress.required_movement_radius_m must not exceed goal.xy_tolerance_m"
        )

    local_planner = _validated_group(
        local_planner_raw, "navigation.local_planner", LOCAL_PLANNER_SPEC
    )
    costmap = _validated_group(costmap_raw, "navigation.costmap", COSTMAP_SPEC)
    if costmap["local_publish_frequency_hz"] > costmap["local_update_frequency_hz"]:
        raise FieldProfileError(
            "navigation.costmap.local_publish_frequency_hz must not exceed local_update_frequency_hz"
        )
    if costmap["global_publish_frequency_hz"] > costmap["global_update_frequency_hz"]:
        raise FieldProfileError(
            "navigation.costmap.global_publish_frequency_hz must not exceed global_update_frequency_hz"
        )
    if costmap["raytrace_range_m"] < costmap["obstacle_range_m"]:
        raise FieldProfileError(
            "navigation.costmap.raytrace_range_m must not be below obstacle_range_m"
        )

    global_planner = _validated_group(
        global_planner_raw, "navigation.global_planner", GLOBAL_PLANNER_SPEC
    )
    if global_planner["goal_tolerance_m"] < goal["xy_tolerance_m"]:
        raise FieldProfileError(
            "navigation.global_planner.goal_tolerance_m must not be below goal.xy_tolerance_m"
        )

    navigation = {
        "controller": controller,
        "goal": goal,
        "progress": progress,
        "local_planner": local_planner,
        "costmap": costmap,
        "global_planner": global_planner,
    }

    teleoperation = _validated_group(
        teleoperation_raw, "teleoperation", TELEOPERATION_SPEC
    )
    for speed_key in (
        "max_forward_speed_mps",
        "max_reverse_speed_mps",
        "max_lateral_speed_mps",
    ):
        if teleoperation[speed_key] > controller["max_linear_speed_mps"]:
            raise FieldProfileError(
                "teleoperation.%s must not exceed navigation.controller.max_linear_speed_mps"
                % speed_key
            )
    if teleoperation["max_angular_speed_radps"] > controller["max_angular_speed_radps"]:
        raise FieldProfileError(
            "teleoperation.max_angular_speed_radps must not exceed navigation.controller.max_angular_speed_radps"
        )
    if teleoperation["watchdog_rate_hz"] * teleoperation["command_timeout_s"] < 3.0:
        raise FieldProfileError(
            "teleoperation watchdog must provide at least three checks per command timeout"
        )

    localization = _validated_group(
        localization_raw, "localization", LOCALIZATION_SPEC
    )
    if localization["pose_jump_candidate_radius_m"] > localization["pose_jump_reject_m"]:
        raise FieldProfileError(
            "localization.pose_jump_candidate_radius_m must not exceed pose_jump_reject_m"
        )
    if localization["stationary_drift_reject_m"] > localization["pose_jump_reject_m"]:
        raise FieldProfileError(
            "localization.stationary_drift_reject_m must not exceed pose_jump_reject_m"
        )
    if localization["relocalization_jump_grace_radius_m"] < localization["pose_jump_reject_m"]:
        raise FieldProfileError(
            "localization.relocalization_jump_grace_radius_m must not be below pose_jump_reject_m"
        )
    if localization["odom_rebase_jump_m"] < localization["pose_jump_reject_m"]:
        raise FieldProfileError(
            "localization.odom_rebase_jump_m must not be below pose_jump_reject_m"
        )

    normalized = {
        "schema_version": schema_version,
        "profile_name": profile_name,
        "scan": scan,
        "stair": {key: value for key, value in stair.items() if key != "obstacle_height_m"},
        "stair_safety": stair_safety,
        "stair_transition": stair_transition,
        "navigation": navigation,
        "teleoperation": teleoperation,
        "localization": localization,
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


def _replace_field_placeholders(
    source: Mapping[str, Any], replacements: Mapping[str, Any]
) -> Dict[str, Any]:
    rendered = deepcopy(source)
    used = set()

    def replace(value: Any, path: str) -> Any:
        if isinstance(value, dict):
            return {
                key: replace(item, f"{path}.{key}" if path else str(key))
                for key, item in value.items()
            }
        if isinstance(value, list):
            return [replace(item, f"{path}[{index}]") for index, item in enumerate(value)]
        if isinstance(value, str) and value.startswith("__FIELD_PROFILE_"):
            if value not in replacements:
                raise FieldProfileError(
                    f"unknown field profile placeholder {value!r} at {path}"
                )
            used.add(value)
            return replacements[value]
        return value

    rendered = replace(rendered, "")
    missing = sorted(set(replacements) - used)
    if missing:
        raise FieldProfileError(
            "parameter template missing field profile placeholders: "
            + ", ".join(missing)
        )
    return rendered


def render_nav2_parameters(
    source: Mapping[str, Any], profile: Mapping[str, Any]
) -> Dict[str, Any]:
    """Render the Nav2 template with native numeric parameter types."""
    navigation = profile["navigation"]
    controller = navigation["controller"]
    goal = navigation["goal"]
    progress = navigation["progress"]
    local_planner = navigation["local_planner"]
    costmap = navigation["costmap"]
    global_planner = navigation["global_planner"]
    replacements = {
        "__FIELD_PROFILE_CONTROLLER_FREQUENCY__": controller["frequency_hz"],
        "__FIELD_PROFILE_PROGRESS_RADIUS__": progress["required_movement_radius_m"],
        "__FIELD_PROFILE_PROGRESS_TIME__": progress["movement_time_allowance_s"],
        "__FIELD_PROFILE_XY_GOAL_TOLERANCE__": goal["xy_tolerance_m"],
        "__FIELD_PROFILE_YAW_GOAL_TOLERANCE__": goal["yaw_tolerance_rad"],
        "__FIELD_PROFILE_MAX_LINEAR_SPEED__": controller["max_linear_speed_mps"],
        "__FIELD_PROFILE_MAX_ANGULAR_SPEED__": controller["max_angular_speed_radps"],
        "__FIELD_PROFILE_LINEAR_ACCELERATION__": controller[
            "linear_acceleration_limit_mps2"
        ],
        "__FIELD_PROFILE_ANGULAR_ACCELERATION__": controller[
            "angular_acceleration_limit_radps2"
        ],
        "__FIELD_PROFILE_LINEAR_DECELERATION__": -controller[
            "linear_deceleration_limit_mps2"
        ],
        "__FIELD_PROFILE_ANGULAR_DECELERATION__": -controller[
            "angular_deceleration_limit_radps2"
        ],
        "__FIELD_PROFILE_STOPPED_LINEAR_SPEED__": controller[
            "stopped_linear_speed_mps"
        ],
        "__FIELD_PROFILE_LINEAR_SAMPLES__": local_planner["linear_velocity_samples"],
        "__FIELD_PROFILE_ANGULAR_SAMPLES__": local_planner["angular_velocity_samples"],
        "__FIELD_PROFILE_SIMULATION_TIME__": local_planner["simulation_time_s"],
        "__FIELD_PROFILE_OBSTACLE_RANGE__": costmap["obstacle_range_m"],
        "__FIELD_PROFILE_RAYTRACE_RANGE__": costmap["raytrace_range_m"],
        "__FIELD_PROFILE_OBSERVATION_PERSISTENCE__": costmap[
            "observation_persistence_s"
        ],
        "__FIELD_PROFILE_INFLATION_RADIUS__": costmap["inflation_radius_m"],
        "__FIELD_PROFILE_INFLATION_COST_SCALING__": costmap[
            "inflation_cost_scaling_factor"
        ],
        "__FIELD_PROFILE_LOCAL_COSTMAP_UPDATE_FREQUENCY__": costmap[
            "local_update_frequency_hz"
        ],
        "__FIELD_PROFILE_LOCAL_COSTMAP_PUBLISH_FREQUENCY__": costmap[
            "local_publish_frequency_hz"
        ],
        "__FIELD_PROFILE_GLOBAL_COSTMAP_UPDATE_FREQUENCY__": costmap[
            "global_update_frequency_hz"
        ],
        "__FIELD_PROFILE_GLOBAL_COSTMAP_PUBLISH_FREQUENCY__": costmap[
            "global_publish_frequency_hz"
        ],
        "__FIELD_PROFILE_PLANNER_FREQUENCY__": global_planner["expected_frequency_hz"],
        "__FIELD_PROFILE_PLANNER_GOAL_TOLERANCE__": global_planner[
            "goal_tolerance_m"
        ],
        "__FIELD_PROFILE_RECOVERY_SIMULATION_TIME__": controller[
            "recovery_simulation_time_s"
        ],
        "__FIELD_PROFILE_RECOVERY_MIN_ANGULAR_SPEED__": controller[
            "recovery_min_angular_speed_radps"
        ],
    }
    return _replace_field_placeholders(source, replacements)


def tcp_bridge_parameters(profile: Mapping[str, Any]) -> Dict[str, Any]:
    localization = profile["localization"]
    return {
        "tf_pose_fallback_max_age_s": localization["tf_fallback_max_age_s"],
        "pose_jump_reject_m": localization["pose_jump_reject_m"],
        "pose_jump_candidate_radius_m": localization["pose_jump_candidate_radius_m"],
        "pose_jump_candidate_yaw_tolerance_rad": localization[
            "pose_jump_candidate_yaw_tolerance_rad"
        ],
        "pose_stationary_drift_reject_m": localization["stationary_drift_reject_m"],
        "pose_stationary_drift_reject_yaw_rad": localization[
            "stationary_drift_reject_yaw_rad"
        ],
        "pose_motion_command_hold_s": localization["motion_command_hold_s"],
        "pose_command_linear_deadband_mps": localization["command_linear_deadband_mps"],
        "pose_command_angular_deadband_rad_s": localization[
            "command_angular_deadband_rad_s"
        ],
        "pose_filter_hold_last_good_s": localization["filter_hold_last_good_s"],
        "pose_relocalization_jump_grace_s": localization["relocalization_jump_grace_s"],
        "pose_relocalization_jump_grace_radius_m": localization[
            "relocalization_jump_grace_radius_m"
        ],
        "odom_rebase_jump_m": localization["odom_rebase_jump_m"],
        "odom_rebase_jump_yaw_rad": localization["odom_rebase_jump_yaw_rad"],
    }


def render_m20pro_parameters(
    source: Mapping[str, Any],
    profile: Mapping[str, Any],
    *,
    enable_axis_command: bool,
) -> Dict[str, Any]:
    """Render the final production node parameters with native value types."""
    if type(enable_axis_command) is not bool:
        raise FieldProfileError("enable_axis_command must be a boolean")
    localization = tcp_bridge_parameters(profile)
    placeholder_by_parameter = {
        "tf_pose_fallback_max_age_s": "__FIELD_PROFILE_TF_FALLBACK_MAX_AGE__",
        "pose_jump_reject_m": "__FIELD_PROFILE_POSE_JUMP_REJECT__",
        "pose_jump_candidate_radius_m": "__FIELD_PROFILE_POSE_JUMP_CANDIDATE_RADIUS__",
        "pose_jump_candidate_yaw_tolerance_rad": "__FIELD_PROFILE_POSE_JUMP_CANDIDATE_YAW__",
        "pose_stationary_drift_reject_m": "__FIELD_PROFILE_STATIONARY_DRIFT_REJECT__",
        "pose_stationary_drift_reject_yaw_rad": "__FIELD_PROFILE_STATIONARY_DRIFT_REJECT_YAW__",
        "pose_motion_command_hold_s": "__FIELD_PROFILE_MOTION_COMMAND_HOLD__",
        "pose_command_linear_deadband_mps": "__FIELD_PROFILE_LINEAR_DEADBAND__",
        "pose_command_angular_deadband_rad_s": "__FIELD_PROFILE_ANGULAR_DEADBAND__",
        "pose_filter_hold_last_good_s": "__FIELD_PROFILE_FILTER_HOLD_LAST_GOOD__",
        "pose_relocalization_jump_grace_s": "__FIELD_PROFILE_RELOCALIZATION_GRACE__",
        "pose_relocalization_jump_grace_radius_m": "__FIELD_PROFILE_RELOCALIZATION_GRACE_RADIUS__",
        "odom_rebase_jump_m": "__FIELD_PROFILE_ODOM_REBASE_JUMP__",
        "odom_rebase_jump_yaw_rad": "__FIELD_PROFILE_ODOM_REBASE_JUMP_YAW__",
    }
    replacements = {
        placeholder: localization[parameter]
        for parameter, placeholder in placeholder_by_parameter.items()
    }
    rendered = _replace_field_placeholders(source, replacements)
    bridge = rendered.setdefault("m20pro_tcp_bridge", {}).setdefault(
        "ros__parameters", {}
    )
    bridge["enable_axis_command"] = enable_axis_command
    bridge["enable_initialpose_relocalization"] = True
    bridge["enable_initialpose_3d_relocalization"] = False
    return rendered


def floor_manager_field_parameters(profile: Mapping[str, Any]) -> Dict[str, Any]:
    safety = profile["stair_safety"]
    transition = profile["stair_transition"]
    return {
        "field_profile_name": profile["profile_name"],
        "field_profile_hash": profile["profile_hash"],
        "stair_clearance_startup_timeout_s": safety["startup_timeout_s"],
        "stair_clearance_stale_timeout_s": safety["stale_timeout_s"],
        "stair_clearance_required_samples": safety["required_clear_samples"],
        "stair_entry_tolerance_m": transition["entry_tolerance_m"],
        "floor_switch_timeout_s": transition["floor_switch_timeout_s"],
        "post_switch_goal_delay_s": transition["post_switch_goal_delay_s"],
        "duplicate_goal_tolerance_m": transition["duplicate_goal_tolerance_m"],
        "duplicate_goal_yaw_tolerance_rad": transition[
            "duplicate_goal_yaw_tolerance_rad"
        ],
    }


def command_mux_field_parameters(profile: Mapping[str, Any]) -> Dict[str, Any]:
    teleoperation = profile["teleoperation"]
    return {
        "watchdog_rate_hz": teleoperation["watchdog_rate_hz"],
        "teleop_command_timeout_s": teleoperation["command_timeout_s"],
        "navigation_command_timeout_s": teleoperation["navigation_timeout_s"],
        "teleop_max_forward_speed_mps": teleoperation["max_forward_speed_mps"],
        "teleop_max_reverse_speed_mps": teleoperation["max_reverse_speed_mps"],
        "teleop_max_lateral_speed_mps": teleoperation["max_lateral_speed_mps"],
        "teleop_max_angular_speed_radps": teleoperation["max_angular_speed_radps"],
    }


def web_teleoperation_field_parameters(profile: Mapping[str, Any]) -> Dict[str, Any]:
    teleoperation = profile["teleoperation"]
    return {
        "teleop_command_timeout_s": teleoperation["command_timeout_s"],
        "teleop_max_forward_speed_mps": teleoperation["max_forward_speed_mps"],
        "teleop_max_reverse_speed_mps": teleoperation["max_reverse_speed_mps"],
        "teleop_max_lateral_speed_mps": teleoperation["max_lateral_speed_mps"],
        "teleop_max_angular_speed_radps": teleoperation["max_angular_speed_radps"],
    }


def web_navigation_field_parameters(profile: Mapping[str, Any]) -> Dict[str, Any]:
    """Expose task-layer navigation thresholds from the field profile.

    The dashboard uses this value only for progress and Nav2-success evidence;
    Nav2 itself receives the same value through the rendered goal checker.
    Keeping both consumers on the profile prevents a hidden 0.3 m fallback from
    disagreeing with the field 0.35 m configuration.
    """
    goal = profile["navigation"]["goal"]
    return {
        "goal_reached_tolerance_m": goal["xy_tolerance_m"],
    }


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
