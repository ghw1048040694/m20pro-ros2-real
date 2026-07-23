"""ROS 2 JSON adapter for the fail-closed stair connector reducer.

This node is intentionally not part of the current real launch.  When a
future field-certified connector is ready, an orchestrator can start it and
consume semantic actions through the existing arbiter/map transaction.  The
node itself has no Twist, gait, map, or factory client publishers.
"""

from __future__ import annotations

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .stair_executor_contract import create_connector_execution, step_connector_execution


def _json_message(payload: Dict[str, Any]) -> String:
    message = String()
    message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return message


def _parse(value: Any) -> Optional[Dict[str, Any]]:
    try:
        result = json.loads(str(value or ""))
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return result if isinstance(result, dict) else None


class StairExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_stair_executor")
        self.declare_parameter("enabled", False)
        self.declare_parameter("start_topic", "/m20pro/stair_executor/start")
        self.declare_parameter("event_topic", "/m20pro/stair_executor/event")
        self.declare_parameter("action_topic", "/m20pro/stair_executor/action")
        self.declare_parameter("status_topic", "/m20pro/stair_executor/status")
        self.declare_parameter("terrain_request_topic", "/m20pro/terrain_guard/request")
        self.declare_parameter("terrain_status_topic", "/m20pro/terrain_guard/status")
        self.declare_parameter("stage_timeout_s", 180.0)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._execution: Optional[Dict[str, Any]] = None
        self._action_pub = self.create_publisher(
            String, str(self.get_parameter("action_topic").value), 10
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._terrain_request_pub = self.create_publisher(
            String, str(self.get_parameter("terrain_request_topic").value), 10
        )
        self.create_subscription(
            String, str(self.get_parameter("start_topic").value), self._on_start, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("event_topic").value), self._on_event, 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("terrain_status_topic").value),
            self._on_terrain_status,
            10,
        )
        self._publish_status(
            {
                "ok": self._enabled,
                "code": "stair_executor_ready" if self._enabled else "stair_execution_retired",
                "message": "楼梯语义执行器已启动" if self._enabled else "楼梯执行器默认关闭",
                "state": "IDLE",
                "actions": [],
            }
        )

    def _on_start(self, message: String) -> None:
        payload = _parse(message.data)
        if payload is None:
            self._publish_status(self._error("connector_start_invalid", "楼梯执行启动请求不是 JSON 对象"))
            return
        if not self._enabled:
            self._publish_status(self._error("stair_execution_retired", "楼梯执行器当前关闭"))
            return
        if self._execution and str(self._execution.get("state") or "") not in {
            "COMPLETED",
            "STOPPED",
            "FAILED",
        }:
            self._publish_status(self._error("connector_busy", "已有楼梯连接边正在执行"))
            return
        result = create_connector_execution(
            payload.get("route") if isinstance(payload.get("route"), dict) else {},
            request_id=payload.get("request_id"),
            now_monotonic=self.get_clock().now().nanoseconds / 1e9,
            stage_timeout_s=float(self.get_parameter("stage_timeout_s").value),
        )
        if result.get("ok"):
            self._execution = dict(result["execution"])
        else:
            self._execution = None
        self._publish_result(result)

    def _on_event(self, message: String) -> None:
        payload = _parse(message.data)
        if payload is None:
            self._publish_status(self._error("connector_event_invalid", "楼梯执行事件不是 JSON 对象"))
            return
        if self._execution is None:
            self._publish_status(self._error("connector_not_active", "当前没有活动楼梯连接边"))
            return
        result = step_connector_execution(
            self._execution,
            payload,
            now_monotonic=self.get_clock().now().nanoseconds / 1e9,
        )
        if isinstance(result.get("execution"), dict):
            self._execution = dict(result["execution"])
        self._publish_result(result)

    def _on_terrain_status(self, message: String) -> None:
        """Feed 106's identity-bound status into the reducer without motion I/O."""
        if self._execution is None:
            return
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            status = None
        event = {
            "type": "terrain_status",
            "request_id": (status or {}).get("request_id") if isinstance(status, dict) else None,
            "status": status,
        }
        result = step_connector_execution(
            self._execution,
            event,
            now_monotonic=self.get_clock().now().nanoseconds / 1e9,
        )
        if isinstance(result.get("execution"), dict):
            self._execution = dict(result["execution"])
        self._publish_result(result)

    @staticmethod
    def _error(code: str, message: str) -> Dict[str, Any]:
        return {"ok": False, "code": code, "message": message, "actions": []}

    def _publish_result(self, result: Dict[str, Any]) -> None:
        actions = result.get("actions") if isinstance(result.get("actions"), list) else []
        envelope = dict(result)
        envelope["execution"] = dict(self._execution or result.get("execution") or {})
        envelope["actions"] = actions
        self._publish_action(actions, envelope)
        self._publish_terrain_request(actions, envelope)
        self._publish_status(envelope)

    def _publish_action(self, actions: list[Dict[str, Any]], result: Dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(
            {
                "request_id": (self._execution or {}).get("request_id"),
                "route_id": (self._execution or {}).get("route_id"),
                "actions": actions,
                "source": "m20pro_stair_executor",
                "result_code": result.get("code"),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self._action_pub.publish(message)

    def _publish_terrain_request(self, actions: list[Dict[str, Any]], result: Dict[str, Any]) -> None:
        execution = self._execution or result.get("execution") or {}
        if not isinstance(execution, dict):
            return
        request_id = str(execution.get("request_id") or "").strip()
        route_id = str(execution.get("route_id") or "").strip()
        for action in actions:
            kind = str(action.get("kind") or "")
            if kind == "request_terrain_guard":
                payload = {
                    "enabled": True,
                    "request_id": request_id,
                    "route_id": route_id,
                    "profile_id": str(action.get("profile_id") or "").strip(),
                    "corridor_version": str(action.get("corridor_version") or "").strip(),
                    "direction": str(action.get("direction") or "forward").strip(),
                    "corridor": action.get("corridor") if isinstance(action.get("corridor"), dict) else {},
                }
                self._terrain_request_pub.publish(_json_message(payload))
                return
            if kind == "resume_flat_navigation" or (
                kind == "stop" and str(execution.get("state") or "") in {"FAILED", "STOPPED"}
            ):
                self._terrain_request_pub.publish(
                    _json_message(
                        {
                            "enabled": False,
                            "request_id": request_id,
                            "route_id": route_id,
                        }
                    )
                )
                return

    def _publish_status(self, result: Dict[str, Any]) -> None:
        message = String()
        execution = result.get("execution") if isinstance(result.get("execution"), dict) else self._execution or {}
        message.data = json.dumps(
            {
                "ok": bool(result.get("ok")),
                "code": result.get("code"),
                "message": result.get("message"),
                "state": execution.get("state") or result.get("state") or "IDLE",
                "status": execution.get("status") or "idle",
                "request_id": execution.get("request_id"),
                "route_id": execution.get("route_id"),
                "terrain_guard": execution.get("terrain_guard"),
                "actions": result.get("actions") if isinstance(result.get("actions"), list) else [],
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self._status_pub.publish(message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = StairExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
