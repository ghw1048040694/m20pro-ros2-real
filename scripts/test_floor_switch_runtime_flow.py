#!/usr/bin/env python3
"""Execute the real Web floor-switch method against deterministic adapters."""

from __future__ import annotations

import sys
import threading
import types
from pathlib import Path
from typing import Any, Dict, List


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.web_dashboard_node import WebDashboardNode  # noqa: E402


def assert_equal(actual: Any, expected: Any, message: str) -> None:
    if actual != expected:
        raise AssertionError(f"{message}: expected {expected!r}, got {actual!r}")


class _Parameter:
    def __init__(self, value: Any) -> None:
        self.value = value


class _Logger:
    def __init__(self) -> None:
        self.exceptions: List[str] = []

    def exception(self, message: str) -> None:
        self.exceptions.append(message)


class _FloorPublisher:
    def __init__(self, runtime: "Runtime") -> None:
        self.runtime = runtime

    def publish(self, message: Any) -> None:
        value = str(message.data or "")
        self.runtime.events.append(f"floor:{value}")
        self.runtime.floor_messages.append(value)


class Runtime:
    """Small adapter surface required by the production transaction method."""

    def __init__(self, *, activation_ok: bool = True) -> None:
        self.request_id = "switch-1"
        self.events: List[str] = []
        self.results: List[Dict[str, Any]] = []
        self.persisted: List[Dict[str, Any]] = []
        self.floor_messages: List[str] = []
        self.activation_ok = activation_ok
        self._data_lock = threading.RLock()
        self._floor_switch_lock = threading.RLock()
        self._floor_switch_inflight = self.request_id
        self._settings: Dict[str, Any] = {
            "selected_map_id": "map-f1",
            "active_task": {
                "task_id": "task-1",
                "status": "running",
                "multi_floor": True,
                "connector_request_id": self.request_id,
                "connector_route_id": "route-up",
                "connector_plan_id": "plan-1",
                "connector_map_epoch": 7,
                "last_floor_goal_source_floor": "F1",
                "last_floor_goal_target_floor": "F2",
            },
        }
        self._floor_routes = [
            {
                "id": "route-up",
                "source_floor": "F1",
                "target_floor": "F2",
                "source_map_id": "map-f1",
                "target_map_id": "map-f2",
                "direction": "up",
                "entry": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "source_platform": {"x": 2.0, "y": 0.0, "yaw": 0.0},
                "target_platform": {"x": 0.5, "y": 0.0, "yaw": 0.0},
                "post_exit": {"x": 1.5, "y": 0.0, "yaw": 0.0},
            }
        ]
        self.floor_context_pub = _FloorPublisher(self)
        self.logger = _Logger()

        for name in (
            "_advance_floor_switch_transaction",
            "_fail_floor_switch_transaction",
            "_floor_switch_task_is_active",
        ):
            method = getattr(WebDashboardNode, name)
            setattr(self, name, types.MethodType(method, self))

    def get_logger(self) -> _Logger:
        return self.logger

    def get_parameter(self, name: str) -> _Parameter:
        values = {
            "cross_floor_platform_position_tolerance_m": 0.50,
            "cross_floor_platform_yaw_tolerance_rad": 0.35,
        }
        return _Parameter(values[name])

    def _persist_floor_switch_transaction(self, transaction: Dict[str, Any]) -> None:
        snapshot = dict(transaction)
        self.persisted.append(snapshot)
        self._settings["floor_switch_transaction"] = snapshot

    def _activate_cross_floor_target_map(self, map_id: str) -> Dict[str, Any]:
        self.events.append("activate")
        if not self.activation_ok:
            return {
                "ok": False,
                "code": "factory_map_apply_failed",
                "message": "106目标地图激活失败",
                "nav2_load_map": {"ok": True},
                "factory_apply_map": {"ok": False},
            }
        return {
            "ok": True,
            "code": "floor_switch_maps_activated",
            "selected_map_id": map_id,
            "nav2_load_map": {"ok": True, "result": 0},
            "factory_apply_map": {"ok": True, "returncode": 0},
        }

    def _publish_initialpose(
        self,
        payload: Dict[str, Any],
        **kwargs: Any,
    ) -> Dict[str, Any]:
        self.events.append("relocalize")
        assert_equal(payload["floor"], "F2", "relocalization target floor")
        assert_equal(payload["x"], 0.5, "relocalization target x")
        assert_equal(kwargs["allow_active_task"], True, "active task relocalization allowed")
        assert_equal(kwargs["require_lifecycle"], False, "no lifecycle gate")
        assert_equal(kwargs["stability_window_s"], 0.0, "no extra stability gate")
        return {
            "confirmed": True,
            "verification": {"factory_pose_accepted": True},
        }

    def _publish_floor_switch_result(self, payload: Dict[str, Any]) -> None:
        self.events.append("result")
        self.results.append(dict(payload))


def _request(runtime: Runtime) -> Dict[str, Any]:
    return {
        "request_id": runtime.request_id,
        "route_id": "route-up",
        "plan_id": "plan-1",
        "map_epoch": 7,
        "source_floor": "F1",
        "target_floor": "F2",
        "target_map_id": "map-f2",
    }


def test_success_flow_orders_activation_relocalization_floor_and_result() -> None:
    runtime = Runtime()
    WebDashboardNode._run_floor_switch_transaction(runtime, _request(runtime))

    assert_equal(runtime.events, ["activate", "relocalize", "floor:F2", "result"], "runtime order")
    assert_equal(runtime.floor_messages, ["F2"], "target floor published once")
    assert_equal(runtime.results[-1]["ok"], True, "success result")
    assert_equal(runtime.results[-1]["target_map_id"], "map-f2", "success map identity")
    assert_equal(runtime.persisted[-1]["state"], "COMMITTED", "transaction committed")
    assert_equal(runtime._floor_switch_inflight, None, "inflight marker cleared")
    assert_equal(runtime.logger.exceptions, [], "no runtime exception")


def test_map_activation_failure_stops_before_relocalization() -> None:
    runtime = Runtime(activation_ok=False)
    WebDashboardNode._run_floor_switch_transaction(runtime, _request(runtime))

    assert_equal(runtime.events, ["activate", "result"], "failure stops before 2101")
    assert_equal(runtime.floor_messages, [], "failed switch does not change floor context")
    assert_equal(runtime.results[-1]["ok"], False, "failure result")
    assert_equal(runtime.results[-1]["code"], "factory_map_apply_failed", "failure code")
    assert_equal(runtime.persisted[-1]["state"], "FAILED", "transaction failed")
    assert_equal(runtime._floor_switch_inflight, None, "failed inflight marker cleared")


def main() -> int:
    for test in (
        test_success_flow_orders_activation_relocalization_floor_and_result,
        test_map_activation_failure_stops_before_relocalization,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] floor switch runtime flow tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
