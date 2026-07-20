#!/usr/bin/env python3
"""Validate and render the single M20Pro field profile."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.field_profile_contract import (  # noqa: E402
    FieldProfileError,
    load_field_profile,
    render_edge_environment,
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
        "navigation": profile["navigation"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command",
        choices=("check", "show-json", "render-edge-env"),
        nargs="?",
        default="check",
    )
    args = parser.parse_args()
    try:
        profile = load_field_profile(DEFAULT_PROFILE)
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
