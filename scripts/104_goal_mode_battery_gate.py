#!/usr/bin/env python3
"""Read-only battery display probe for Codex goal-mode work on the real robot.

This script only queries the web dashboard /api/state endpoint. It does not
start tasks, publish ROS goals, change relocalization, send motion, or block
field work through its exit status.
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
        print(f"[goal_battery_gate] WARN: cannot read robot battery from {args.url}: {exc}")
        return 0

    battery = state.get("battery") if isinstance(state.get("battery"), dict) else {}
    primary = battery.get("primary") if isinstance(battery.get("primary"), dict) else {}
    level = number(primary.get("level"))
    active_task = state.get("active_task") is not None
    localization_status = state.get("localization_status") if isinstance(state.get("localization_status"), dict) else {}

    print("[goal_battery_gate] read-only")
    print(f"url={args.url}")
    print(f"battery_level={'-' if level is None else f'{level:.0f}%'}")
    print(f"min_level={args.min_level:.0f}%")
    print(f"active_task={'true' if active_task else 'false'}")
    print(f"localization_status={localization_status.get('code') or '-'}")

    if level is None:
        print("[goal_battery_gate] INFO: battery level unavailable; operator should judge from field hardware.")
        return 0
    if level < args.min_level:
        print("[goal_battery_gate] INFO: battery below reference level; display only, not a software gate.")
        return 0

    print("[goal_battery_gate] OK: battery display is available.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
