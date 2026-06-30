#!/usr/bin/env python3
"""Read-only battery gate for Codex goal-mode work on the real robot.

This script only queries the web dashboard /api/state endpoint. It does not
start tasks, publish ROS goals, change relocalization, or send motion.
"""

from __future__ import annotations

import argparse
import json
import urllib.request
from typing import Any, Dict, Optional


def fetch_state(base_url: str, timeout_s: float) -> Dict[str, Any]:
    with urllib.request.urlopen(f"{base_url.rstrip('/')}/api/state", timeout=timeout_s) as response:
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise RuntimeError("unexpected /api/state payload")
    return payload


def number(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://10.21.31.104:8080")
    parser.add_argument("--min-level", type=float, default=25.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    try:
        state = fetch_state(args.url, args.timeout)
    except Exception as exc:
        print(f"[goal_battery_gate] BLOCK: cannot read robot battery from {args.url}: {exc}")
        return 2

    battery = state.get("battery") if isinstance(state.get("battery"), dict) else {}
    primary = battery.get("primary") if isinstance(battery.get("primary"), dict) else {}
    level = number(primary.get("level"))
    active_task = state.get("active_task") is not None
    readiness = state.get("task_readiness") if isinstance(state.get("task_readiness"), dict) else {}

    print("[goal_battery_gate] read-only")
    print(f"url={args.url}")
    print(f"battery_level={'-' if level is None else f'{level:.0f}%'}")
    print(f"min_level={args.min_level:.0f}%")
    print(f"active_task={'true' if active_task else 'false'}")
    print(f"task_readiness={readiness.get('code') or '-'}")

    if level is None:
        print("[goal_battery_gate] BLOCK: battery level unavailable; stop goal-mode field work.")
        return 2
    if level < args.min_level:
        print("[goal_battery_gate] BLOCK: battery below threshold; stop goal-mode field work and charge.")
        return 2

    print("[goal_battery_gate] OK: battery is sufficient for non-destructive goal-mode work.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
