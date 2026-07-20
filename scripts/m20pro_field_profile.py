#!/usr/bin/env python3
"""Validate and render the single M20Pro field profile."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.field_profile_contract import (  # noqa: E402
    FieldProfileError,
    load_field_profile,
    render_edge_environment,
    render_m20pro_parameters,
    render_nav2_parameters,
)


DEFAULT_PROFILE = ROOT / "src/m20pro_bringup/config/m20pro_field_profile.yaml"


def profile_summary(profile: dict) -> dict:
    return {
        "profile_name": profile["profile_name"],
        "profile_hash": profile["profile_hash"],
        "source_path": profile["source_path"],
        "scan": profile["scan"],
        "stair": profile["stair"],
        "stair_safety": profile["stair_safety"],
        "stair_transition": profile["stair_transition"],
        "navigation": profile["navigation"],
        "localization": profile["localization"],
    }


def render_yaml(
    command: str,
    profile: dict,
    source: Path,
    output: Path,
    axis_enabled: bool,
) -> None:
    try:
        raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise FieldProfileError(f"cannot read parameter template {source}: {exc}") from exc
    if not isinstance(raw, dict):
        raise FieldProfileError(f"parameter template {source} must be a mapping")
    if command == "render-real-yaml":
        rendered = render_m20pro_parameters(
            raw, profile, enable_axis_command=axis_enabled
        )
    else:
        rendered = render_nav2_parameters(raw, profile)
    text = yaml.safe_dump(rendered, allow_unicode=True, sort_keys=False)
    if "__FIELD_PROFILE_" in text:
        raise FieldProfileError(f"rendered parameter file still has placeholders: {source}")
    try:
        output.write_text(text, encoding="utf-8")
    except OSError as exc:
        raise FieldProfileError(f"cannot write rendered parameters {output}: {exc}") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command",
        choices=(
            "check",
            "show-json",
            "render-edge-env",
            "render-real-yaml",
            "render-nav2-yaml",
        ),
        nargs="?",
        default="check",
    )
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--axis-enabled", choices=("true", "false"), default="false")
    args = parser.parse_args()
    try:
        profile = load_field_profile(args.profile)
        if args.command in ("render-real-yaml", "render-nav2-yaml"):
            if args.input is None or args.output is None:
                parser.error(f"{args.command} requires --input and --output")
            render_yaml(
                args.command,
                profile,
                args.input,
                args.output,
                args.axis_enabled == "true",
            )
            return 0
    except FieldProfileError as exc:
        print(f"field profile invalid: {exc}", file=sys.stderr)
        return 2

    if args.command == "render-edge-env":
        sys.stdout.write(render_edge_environment(profile))
        return 0
    if args.command == "show-json":
        print(json.dumps(profile_summary(profile), ensure_ascii=False, indent=2))
        return 0

    summary = profile_summary(profile)
    print(f"field profile OK: {summary['profile_name']}")
    print(f"profile hash: {summary['profile_hash']}")
    print(
        "stair max/obstacle height: %.3f / %.3f m"
        % (
            summary["stair"]["max_step_height_m"],
            summary["stair"]["obstacle_height_m"],
        )
    )
    print(
        "stair corridor: %.2f..%.2f m, half width %.2f m"
        % (
            summary["stair"]["forward_min_m"],
            summary["stair"]["forward_max_m"],
            summary["stair"]["corridor_half_width_m"],
        )
    )
    print(
        "clear samples/startup/stale/mode timeout: %d / %.2f / %.2f / %.2f s"
        % (
            summary["stair_safety"]["required_clear_samples"],
            summary["stair_safety"]["startup_timeout_s"],
            summary["stair_safety"]["stale_timeout_s"],
            summary["stair"]["mode_timeout_s"],
        )
    )
    print(
        "navigation speed/goal/costmap: %.2f m/s / %.2f rad/s / %.2f m / %.2f rad / %.2f m"
        % (
            summary["navigation"]["controller"]["max_linear_speed_mps"],
            summary["navigation"]["controller"]["max_angular_speed_radps"],
            summary["navigation"]["goal"]["xy_tolerance_m"],
            summary["navigation"]["goal"]["yaw_tolerance_rad"],
            summary["navigation"]["costmap"]["inflation_radius_m"],
        )
    )
    print("editable field parameters: 67 (navigation: 28)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
