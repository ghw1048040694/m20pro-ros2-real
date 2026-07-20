#!/usr/bin/env python3
"""Offline tests for the fail-closed stair perception contract."""

from __future__ import annotations

import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.stair_safety_contract import (  # noqa: E402
    parse_stair_clearance,
    stair_clearance_gate_decision,
)


def sample(state: str = "clear", sequence: object = 1, **extra: object) -> dict:
    payload = {
        "version": 1,
        "active": True,
        "session_id": "session-1",
        "profile_hash": "a" * 64,
        "sequence": sequence,
        "state": state,
        "corridor_points": 120,
        "profile_bins": 12,
        "obstacle_points": 0,
    }
    payload.update(extra)
    return parse_stair_clearance(json.dumps(payload), received_monotonic=10.0)


def decision(*, phase: str, parsed: dict, clear_samples: int, now: float) -> dict:
    return stair_clearance_gate_decision(
        session_id="session-1",
        profile_hash="a" * 64,
        phase=phase,
        sample=parsed,
        clear_samples=clear_samples,
        required_clear_samples=3,
        started_monotonic=8.0,
        now_monotonic=now,
        startup_timeout_s=4.0,
        stale_timeout_s=1.0,
    )


def test_parser_is_strict_and_never_raises() -> None:
    assert sample()["ok"] is True
    assert parse_stair_clearance("not-json", received_monotonic=1.0)["ok"] is False
    assert sample(sequence="bad")["ok"] is False
    assert sample(active=False)["ok"] is False
    assert sample(profile_hash="")["ok"] is False
    malformed_counts = sample(corridor_points="bad", profile_bins=None, obstacle_points=-3)
    assert malformed_counts["ok"] is True
    assert malformed_counts["corridor_points"] == 0
    assert malformed_counts["obstacle_points"] == 0


def test_startup_requires_consecutive_clear_samples() -> None:
    assert decision(phase="waiting_traverse", parsed=sample(), clear_samples=2, now=10.0)["action"] == "wait"
    assert decision(phase="waiting_traverse", parsed=sample(), clear_samples=3, now=10.0)["action"] == "start_motion"
    blocked = decision(phase="waiting_traverse", parsed=sample("blocked"), clear_samples=0, now=10.0)
    assert blocked == {"action": "abort", "reason": "stair_clearance_blocked"}
    timed_out = decision(phase="waiting_traverse", parsed={}, clear_samples=0, now=12.1)
    assert timed_out == {"action": "abort", "reason": "stair_clearance_timeout"}


def test_motion_fails_closed_on_bad_or_stale_data() -> None:
    assert decision(phase="traversing", parsed=sample(), clear_samples=3, now=10.5)["action"] == "continue"
    assert decision(phase="exiting", parsed=sample("blocked"), clear_samples=0, now=10.5)["reason"] == "stair_clearance_blocked"
    assert decision(phase="traversing", parsed=sample("unknown"), clear_samples=0, now=10.5)["reason"] == "stair_clearance_unknown"
    assert decision(phase="traversing", parsed={}, clear_samples=0, now=10.5)["reason"] == "stair_clearance_missing"
    stale = decision(phase="traversing", parsed=sample(), clear_samples=3, now=11.1)
    assert stale["action"] == "abort"
    assert stale["reason"] == "stair_clearance_stale"
    mismatch = decision(
        phase="traversing",
        parsed=sample(profile_hash="b" * 64),
        clear_samples=3,
        now=10.5,
    )
    assert mismatch == {"action": "abort", "reason": "stair_profile_mismatch"}


def main() -> int:
    for test in (
        test_parser_is_strict_and_never_raises,
        test_startup_requires_consecutive_clear_samples,
        test_motion_fails_closed_on_bad_or_stale_data,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] stair safety contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
