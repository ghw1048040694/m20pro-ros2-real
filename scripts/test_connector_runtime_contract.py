#!/usr/bin/env python3
"""Offline tests for connector component readiness admission."""

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.connector_runtime_contract import (  # noqa: E402
    connector_runtime_readiness,
)


def record(component: str, **extra) -> dict:
    parsed = {
        "component": component,
        "enabled": True,
        "ready": True,
        "busy": False,
    }
    parsed.update(extra)
    return {"last_update": 100.0, "parsed": parsed}


def check(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def decide(executor=None, now: float = 101.0) -> dict:
    return connector_runtime_readiness(
        executor_status=(
            executor
            if executor is not None
            else record("stair_executor")
        ),
        now_unix_s=now,
        timeout_s=3.5,
    )


def main() -> int:
    check(decide()["ok"], "fresh enabled executor admits connector start")

    missing = connector_runtime_readiness(
        executor_status=None,
        now_unix_s=101.0,
        timeout_s=3.5,
    )
    check(missing["code"] == "stair_executor_status_missing", "missing executor fails before start")

    disabled = decide(executor=record("stair_executor", enabled=False, ready=False))
    check(disabled["code"] == "stair_executor_disabled", "disabled executor cannot be inferred ready")

    stale = decide(now=104.0)
    check(stale["code"] == "stair_executor_status_stale", "stale heartbeat fails closed")

    busy = decide(
        executor=record(
            "stair_executor",
            busy=True,
            request_id="previous",
        )
    )
    check(busy["code"] == "stair_executor_busy", "prior connector ownership blocks overlap")

    wrong_identity = decide(executor=record("other_component"))
    check(
        wrong_identity["code"] == "stair_executor_identity_mismatch",
        "component identity is explicit",
    )

    print("connector runtime contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
