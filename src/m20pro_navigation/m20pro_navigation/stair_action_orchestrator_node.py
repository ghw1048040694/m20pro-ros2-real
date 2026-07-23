"""Optional ROS adapter for the stair-executor semantic action contract.

The adapter is intentionally disabled by default and is not included in the
real launch.  When a field-certified connector is eventually enabled, it
routes only the already-existing floor-goal, floor-switch and stop interfaces.
Gait and connector-motion actions are published as non-dispatchable semantic
intents for a separately certified motion adapter; this node never publishes
``cmd_vel`` or vendor gait commands.
"""

from __future__ import annotations

import json
from collections import deque
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from .stair_action_orchestrator_contract import (
    event_for_floor_switch_result,
    event_for_stair_status,
    translate_action_envelope,
)
from .geometry import yaw_to_quaternion


def _parse(value: Any) -> Optional[Dict[str, Any]]:
    try:
        parsed = json.loads(str(value or ""))
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return parsed if isinstance(parsed, dict) else None


def _json_message(payload: Dict[str, Any]) -> String:
    message = String()
    message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return message


class StairActionOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_stair_action_orchestrator")
        self.declare_parameter("enabled", False)
        self.declare_parameter("action_topic", "/m20pro/stair_executor/action")
        self.declare_parameter("event_topic", "/m20pro/stair_executor/event")
        self.declare_parameter("status_topic", "/m20pro/stair_executor/orchestrator_status")
        self.declare_parameter("floor_goal_topic", "/m20pro/floor_goal")
        self.declare_parameter("floor_switch_request_topic", "/m20pro/floor_switch_request")
        self.declare_parameter("stop_task_topic", "/m20pro/stop_task")
        self.declare_parameter("stair_status_topic", "/m20pro/stair_status")
        self.declare_parameter("floor_switch_result_topic", "/m20pro/floor_switch_result")
        self.declare_parameter("terrain_request_topic", "/m20pro/terrain_guard/request")
        self.declare_parameter("intent_topic", "/m20pro/stair_executor/intent")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._identity: Optional[Dict[str, Any]] = None
        self._last_sequence = 0
        self._retired_request_ids: deque[str] = deque(maxlen=128)
        self._expected_nav_label: Optional[str] = None
        self._expected_nav_stage: Optional[str] = None
        self._expected_nav_goal_seq: Optional[int] = None

        self._event_pub = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), 10
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._floor_goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("floor_goal_topic").value), 10
        )
        self._floor_switch_pub = self.create_publisher(
            String, str(self.get_parameter("floor_switch_request_topic").value), 10
        )
        self._stop_pub = self.create_publisher(
            String, str(self.get_parameter("stop_task_topic").value), 10
        )
        self._terrain_request_pub = self.create_publisher(
            String, str(self.get_parameter("terrain_request_topic").value), 10
        )
        self._intent_pub = self.create_publisher(
            String, str(self.get_parameter("intent_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("action_topic").value),
            self._on_action,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stair_status_topic").value),
            self._on_stair_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("floor_switch_result_topic").value),
            self._on_floor_switch_result,
            10,
        )
        self._publish_status(
            {
                "ok": self._enabled,
                "code": "stair_action_orchestrator_ready"
                if self._enabled
                else "stair_action_orchestrator_retired",
                "message": "楼梯动作编排器已启动"
                if self._enabled
                else "楼梯动作编排器默认关闭",
            }
        )

    def _on_action(self, message: String) -> None:
        envelope = _parse(message.data)
        if envelope is None:
            self._publish_status(
                {
                    "ok": False,
                    "code": "stair_action_envelope_invalid",
                    "message": "楼梯动作信封不是 JSON 对象",
                }
            )
            return
        if not self._enabled:
            self._publish_status(
                {
                    "ok": False,
                    "code": "stair_action_orchestrator_retired",
                    "message": "楼梯动作编排器当前关闭",
                }
            )
            return

        incoming_identity = {
            "request_id": str(envelope.get("request_id") or "").strip(),
            "route_id": str(envelope.get("route_id") or "").strip(),
            "plan_id": str(envelope.get("plan_id") or "").strip(),
            "map_epoch": envelope.get("map_epoch"),
        }
        incoming_request = str(incoming_identity.get("request_id") or "")
        if incoming_request and incoming_request in self._retired_request_ids:
            self._publish_status(
                {
                    "ok": True,
                    "code": "stair_action_retired_ignored",
                    "message": "忽略已结束楼梯连接边的迟到动作",
                    "request_id": incoming_request,
                    "ignored": True,
                }
            )
            return
        current_request = str((self._identity or {}).get("request_id") or "")
        if current_request and incoming_identity.get("request_id") != current_request:
            if current_request not in self._retired_request_ids:
                self._publish_status(
                    {
                        "ok": False,
                        "code": "stair_action_busy",
                        "message": "已有另一条楼梯连接边正在编排",
                        "active_request_id": current_request,
                    }
                )
                return
            self._identity = None
            self._last_sequence = 0
            self._expected_nav_label = None
            self._expected_nav_stage = None
            self._expected_nav_goal_seq = None

        expected = self._identity if self._identity else None
        result = translate_action_envelope(
            envelope,
            expected_identity=expected,
            last_sequence=self._last_sequence if expected else 0,
        )
        if not result.get("ok"):
            self._publish_status(result)
            return
        if result.get("ignored"):
            self._publish_status(result)
            return
        self._identity = dict(result.get("identity") or incoming_identity)
        self._last_sequence = int(result.get("sequence") or self._last_sequence)
        for command in result.get("commands") or []:
            self._apply_command(command)
        if any(
            isinstance(item, dict) and item.get("kind") == "release_terrain_guard"
            for item in (envelope.get("actions") or [])
        ):
            retired_request = str(self._identity.get("request_id") or "")
            if retired_request and retired_request not in self._retired_request_ids:
                self._retired_request_ids.append(retired_request)
            self._expected_nav_label = None
            self._expected_nav_stage = None
            self._expected_nav_goal_seq = None
        self._publish_status(result)

    def _apply_command(self, command: Dict[str, Any]) -> None:
        kind = str(command.get("kind") or "")
        if kind == "publish_floor_goal":
            pose_data = command.get("pose") if isinstance(command.get("pose"), dict) else {}
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            # floor_manager uses the header only to select the floor; it
            # converts the pose into its canonical map frame itself.
            message.header.frame_id = str(command.get("floor") or "")
            message.pose.position.x = float(pose_data.get("x", 0.0))
            message.pose.position.y = float(pose_data.get("y", 0.0))
            message.pose.position.z = float(pose_data.get("z", 0.0))
            message.pose.orientation = yaw_to_quaternion(float(pose_data.get("yaw", 0.0)))
            self._floor_goal_pub.publish(message)
            # floor_manager reports every /m20pro/floor_goal as floor_goal.
            # Keep the connector stage separately so the existing goal API
            # remains the sole Nav2 dispatch path.
            self._expected_nav_label = "floor_goal"
            self._expected_nav_stage = str(command.get("label") or "") or None
            self._expected_nav_goal_seq = None
            return
        if kind == "publish_floor_switch_request":
            self._floor_switch_pub.publish(_json_message(dict(command.get("payload") or {})))
            return
        if kind == "publish_stop_task":
            message = String()
            message.data = str(command.get("reason") or "stair_executor_stop")
            self._stop_pub.publish(message)
            self._expected_nav_label = None
            self._expected_nav_stage = None
            self._expected_nav_goal_seq = None
            return
        if kind == "publish_terrain_guard_request":
            self._terrain_request_pub.publish(
                _json_message(dict(command.get("payload") or {}))
            )
            return
        if kind == "publish_semantic_intent":
            self._intent_pub.publish(
                _json_message(
                    {
                        "request_id": (self._identity or {}).get("request_id"),
                        "route_id": (self._identity or {}).get("route_id"),
                        "plan_id": (self._identity or {}).get("plan_id"),
                        "map_epoch": (self._identity or {}).get("map_epoch"),
                        "dispatchable": False,
                        "intents": command.get("intents") or [],
                    }
                )
            )

    def _on_stair_status(self, message: String) -> None:
        if not self._enabled or not self._identity:
            return
        text = str(message.data or "").strip()
        if text.startswith("nav_goal_accepted"):
            fields = {}
            for token in text.replace(",", " ").split():
                key, separator, value = token.partition("=")
                if separator and key:
                    fields[key.strip()] = value.strip()
            if fields.get("label") == self._expected_nav_label:
                try:
                    self._expected_nav_goal_seq = int(fields.get("goal_seq"))
                except (TypeError, ValueError):
                    self._expected_nav_goal_seq = None
            return
        event = event_for_stair_status(
            text,
            identity=self._identity,
            expected_nav_label=self._expected_nav_label,
            expected_stage=self._expected_nav_stage,
            expected_goal_seq=self._expected_nav_goal_seq,
        )
        if event is not None:
            self._event_pub.publish(_json_message(event))

    def _on_floor_switch_result(self, message: String) -> None:
        if not self._enabled or not self._identity:
            return
        result = _parse(message.data)
        event = event_for_floor_switch_result(result, identity=self._identity)
        if event is not None:
            self._event_pub.publish(_json_message(event))

    def _publish_status(self, payload: Dict[str, Any]) -> None:
        self._status_pub.publish(_json_message(payload))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = StairActionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
