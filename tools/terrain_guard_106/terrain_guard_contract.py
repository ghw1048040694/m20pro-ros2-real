"""Pure, fail-closed terrain checks for the 106-local stair guard.

This module deliberately has no ROS dependency.  The 106 node converts the
vendor point cloud into ``(x, y, z)`` tuples and calls :func:`inspect_cloud`;
104 only receives the small JSON result, never the raw cloud.
"""

from __future__ import annotations

import math
from statistics import median
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float, float]


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _text(value: Any) -> str:
    return str(value or "").strip()


def _positive(value: Any, default: float) -> Optional[float]:
    number = _finite(value)
    if number is None or number <= 0.0:
        return default if default > 0.0 else None
    return number


def _int_at_least(value: Any, default: int, minimum: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        number = default
    return max(minimum, number)


def _error(code: str, message: str, **extra: Any) -> Dict[str, Any]:
    return {
        "ok": False,
        "code": code,
        "message": message,
        **extra,
    }


def normalize_corridor(request: Dict[str, Any]) -> Dict[str, Any]:
    """Validate the local rectangular corridor carried by a stair request."""
    if not isinstance(request, dict):
        return _error("terrain_request_invalid", "terrain request must be an object")
    corridor = request.get("corridor")
    if not isinstance(corridor, dict):
        corridor = request.get("geometry")
    if not isinstance(corridor, dict):
        return _error("corridor_geometry_missing", "stair request has no corridor geometry")

    width_m = _positive(corridor.get("width_m"), 0.0)
    lookahead_m = _positive(corridor.get("lookahead_m"), 0.0)
    if width_m is None or lookahead_m is None:
        return _error("corridor_geometry_invalid", "corridor width and lookahead must be positive")

    direction = _text(request.get("direction") or corridor.get("direction") or "forward").lower()
    if direction not in {"forward", "reverse"}:
        return _error("corridor_direction_invalid", "corridor direction must be forward or reverse")

    min_step = _positive(corridor.get("min_step_height_m"), 0.04)
    max_step = _positive(corridor.get("max_step_height_m"), 0.24)
    obstacle_height = _positive(corridor.get("obstacle_height_m"), 0.22)
    bin_size = _positive(corridor.get("bin_size_m"), 0.12)
    if min_step is None or max_step is None or obstacle_height is None or bin_size is None:
        return _error("corridor_limits_invalid", "terrain limits must be positive")
    if min_step >= max_step:
        return _error("corridor_step_limits_invalid", "min step height must be below max step height")

    return {
        "width_m": width_m,
        "lookahead_m": lookahead_m,
        "direction": direction,
        "min_step_height_m": min_step,
        "max_step_height_m": max_step,
        "obstacle_height_m": obstacle_height,
        "bin_size_m": bin_size,
        "min_points_per_bin": _int_at_least(corridor.get("min_points_per_bin"), 4, 1),
        "min_step_count": _int_at_least(corridor.get("min_step_count"), 2, 1),
        "min_coverage": min(1.0, max(0.1, _finite(corridor.get("min_coverage")) or 0.55)),
        "obstacle_distance_m": _positive(corridor.get("obstacle_distance_m"), lookahead_m),
    }


def _normalized_points(points: Iterable[Any]) -> List[Point]:
    normalized: List[Point] = []
    for raw in points:
        try:
            x, y, z = raw[0], raw[1], raw[2]
        except (IndexError, KeyError, TypeError):
            continue
        values = (_finite(x), _finite(y), _finite(z))
        if all(value is not None for value in values):
            normalized.append((values[0], values[1], values[2]))  # type: ignore[arg-type]
    return normalized


def _median_or_none(values: Sequence[float]) -> Optional[float]:
    return float(median(values)) if values else None


def _base_result(state: str, *, reason: str, confidence: float = 0.0, **extra: Any) -> Dict[str, Any]:
    return {
        "state": state,
        "reason": reason,
        "confidence": min(1.0, max(0.0, float(confidence))),
        # Classification and motion authorization are deliberately separate.
        # The 106 shadow guard never grants a velocity permission by itself.
        "traversable": state == "traversable",
        "permit_motion": False,
        "hazard_type": None,
        "hazard_distance_m": None,
        **extra,
    }


def inspect_cloud(
    points: Iterable[Any],
    *,
    request: Dict[str, Any],
    cloud_age_s: Optional[float],
    cloud_timeout_s: float = 0.75,
) -> Dict[str, Any]:
    """Return a bounded terrain status without publishing or commanding motion.

    The classifier is intentionally conservative.  A complete, monotonic
    sequence of bounded step rises/falls is required for ``traversable``;
    missing coverage, abrupt rises, and isolated high returns are blocked or
    unknown rather than guessed safe.
    """
    limits = normalize_corridor(request)
    if not limits.get("ok", True):
        return _base_result("unknown", reason=str(limits.get("code") or "corridor_invalid"))

    age = _finite(cloud_age_s)
    timeout = _positive(cloud_timeout_s, 0.75) or 0.75
    if age is None or age > timeout:
        return _base_result(
            "stale",
            reason="pointcloud_stale",
            hazard_type="pointcloud_stale",
            hazard_distance_m=0.0,
        )

    width = float(limits["width_m"])
    lookahead = float(limits["lookahead_m"])
    direction = str(limits["direction"])
    selected: List[Tuple[float, float, float]] = []
    for x, y, z in _normalized_points(points):
        longitudinal = -x if direction == "reverse" else x
        if 0.0 <= longitudinal <= lookahead and abs(y) <= width * 0.5:
            selected.append((longitudinal, y, z))

    if not selected:
        return _base_result("unknown", reason="corridor_no_points")

    bin_size = float(limits["bin_size_m"])
    bin_count = max(1, int(math.ceil(lookahead / bin_size)))
    bins: List[List[float]] = [[] for _ in range(bin_count)]
    for longitudinal, _y, z in selected:
        index = min(bin_count - 1, int(longitudinal / bin_size))
        bins[index].append(z)

    min_points = int(limits["min_points_per_bin"])
    usable = [index for index, values in enumerate(bins) if len(values) >= min_points]
    coverage = len(usable) / float(bin_count)
    if coverage < float(limits["min_coverage"]):
        return _base_result(
            "unknown",
            reason="corridor_coverage_low",
            confidence=coverage,
            coverage=coverage,
            bins_with_points=len(usable),
            bin_count=bin_count,
        )

    # A stair profile may end at the visible range, but it may not have a
    # blind spot in front of the robot.  Looking only at valid adjacent pairs
    # would otherwise allow two isolated returns to grant motion permission.
    if not usable or usable[0] != 0:
        return _base_result(
            "unknown",
            reason="corridor_front_coverage_missing",
            confidence=coverage,
            coverage=coverage,
            bins_with_points=len(usable),
            bin_count=bin_count,
        )
    last_usable = usable[-1]
    if usable != list(range(last_usable + 1)):
        return _base_result(
            "unknown",
            reason="corridor_profile_gap",
            confidence=coverage,
            coverage=coverage,
            bins_with_points=len(usable),
            bin_count=bin_count,
        )

    # Do not infer terrain beyond the last continuously observed bin.
    profiles = [_median_or_none(bins[index]) for index in range(last_usable + 1)]
    valid_profiles = [value for value in profiles if value is not None]
    baseline = min(valid_profiles) if valid_profiles else 0.0
    obstacle_height = float(limits["obstacle_height_m"])
    obstacle_distance = float(limits["obstacle_distance_m"])

    high_bins = [
        index
        for index, values in enumerate(bins)
        if values and max(values) - baseline >= obstacle_height
        and index * bin_size <= obstacle_distance
    ]

    step_deltas: List[Tuple[int, float]] = []
    invalid_deltas: List[Tuple[int, float]] = []
    flat_tolerance = float(limits["min_step_height_m"]) * 0.5
    for index in range(len(profiles) - 1):
        left = profiles[index]
        right = profiles[index + 1]
        if left is None or right is None:
            continue
        delta = right - left
        if abs(delta) >= flat_tolerance and float(limits["min_step_height_m"]) <= abs(delta) <= float(limits["max_step_height_m"]):
            step_deltas.append((index, delta))
        elif abs(delta) < flat_tolerance:
            continue
        elif abs(delta) < float(limits["min_step_height_m"]):
            invalid_deltas.append((index, delta))
        else:
            step_deltas.append((index, delta))

    min_steps = int(limits["min_step_count"])
    max_step = float(limits["max_step_height_m"])
    abrupt_bins = []
    for index in range(len(profiles) - 1):
        left = profiles[index]
        right = profiles[index + 1]
        if left is not None and right is not None and abs(right - left) > max_step:
            abrupt_bins.append(index)

    if abrupt_bins:
        hazard_index = abrupt_bins[0]
        return _base_result(
            "blocked",
            reason="step_height_out_of_range",
            confidence=coverage,
            coverage=coverage,
            hazard_type="step_height_out_of_range",
            hazard_distance_m=hazard_index * bin_size,
            step_count=0,
        )

    if invalid_deltas:
        index, _delta = invalid_deltas[0]
        return _base_result(
            "unknown",
            reason="step_profile_unverified",
            confidence=coverage * 0.5,
            coverage=coverage,
            hazard_distance_m=index * bin_size,
            step_count=0,
        )

    signs = [1 if delta > 0 else -1 for _index, delta in step_deltas]
    positive = sum(1 for sign in signs if sign > 0)
    negative = sum(1 for sign in signs if sign < 0)
    dominant_sign = 1 if positive >= negative else -1
    coherent_steps = sum(1 for sign in signs if sign == dominant_sign)

    if positive and negative:
        return _base_result(
            "unknown",
            reason="step_profile_direction_inconsistent",
            confidence=coverage * 0.5,
            coverage=coverage,
            step_count=coherent_steps,
        )

    # An isolated jump that is much taller than the other steps is more
    # likely an object in the corridor than a staircase.  Fail closed instead
    # of treating it as a new staircase level.
    step_heights = [abs(delta) for _index, delta in step_deltas]
    if len(step_heights) >= min_steps:
        # Use the smallest observed step as the baseline.  A single tall
        # return must not be hidden by averaging it with normal steps.
        reference_height = min(step_heights)
        if any(
            height > max(reference_height * 1.75, reference_height + 0.08)
            for height in step_heights
        ):
            return _base_result(
                "blocked",
                reason="step_profile_inconsistent",
                confidence=coverage,
                coverage=coverage,
                hazard_type="step_profile_inconsistent",
                hazard_distance_m=next(
                    index * bin_size
                    for index, delta in step_deltas
                    if abs(delta) > max(reference_height * 1.75, reference_height + 0.08)
                ),
                step_count=coherent_steps,
            )

    if coherent_steps >= min_steps:
        return _base_result(
            "traversable",
            reason="step_profile_continuous",
            confidence=min(1.0, coverage * (0.5 + 0.5 * coherent_steps / max(1, len(step_deltas)))),
            coverage=coverage,
            step_count=coherent_steps,
            step_direction="up" if dominant_sign > 0 else "down",
        )

    if high_bins:
        return _base_result(
            "blocked",
            reason="high_obstacle_in_corridor",
            confidence=coverage,
            coverage=coverage,
            hazard_type="high_obstacle_in_corridor",
            hazard_distance_m=high_bins[0] * bin_size,
            step_count=coherent_steps,
        )

    return _base_result(
        "unknown",
        reason="step_profile_unverified",
        confidence=coverage * 0.5,
        coverage=coverage,
        step_count=coherent_steps,
    )
