"""Exclusive velocity arbiter between Nav2 and browser operator takeover."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .command_mux_contract import CommandMuxArbiter


class CommandMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_command_mux")
        self.declare_parameter("navigation_cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("teleop_cmd_vel_topic", "/cmd_vel_teleop")
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/m20pro/cmd_vel_mux/status")
        self.declare_parameter("mode_request_topic", "/m20pro/cmd_vel_mux/mode_request")
        self.declare_parameter("lock_service", "/m20pro/cmd_vel_mux/lock")
        self.declare_parameter("navigation_service", "/m20pro/cmd_vel_mux/enable_navigation")
        self.declare_parameter("teleop_service", "/m20pro/cmd_vel_mux/enable_teleop")
        self.declare_parameter("initial_mode", "navigation")
        self.declare_parameter("watchdog_rate_hz", 20.0)
        self.declare_parameter("navigation_command_timeout_s", 0.6)
        self.declare_parameter("teleop_command_timeout_s", 0.8)
        self.declare_parameter("teleop_max_forward_speed_mps", 0.18)
        self.declare_parameter("teleop_max_reverse_speed_mps", 0.12)
        self.declare_parameter("teleop_max_lateral_speed_mps", 0.18)
        self.declare_parameter("teleop_max_angular_speed_radps", 0.45)

        self._lock = threading.RLock()
        self._arbiter = CommandMuxArbiter(
            navigation_timeout_s=float(self.get_parameter("navigation_command_timeout_s").value),
            teleop_timeout_s=float(self.get_parameter("teleop_command_timeout_s").value),
            teleop_limits={
                "max_forward_speed_mps": float(
                    self.get_parameter("teleop_max_forward_speed_mps").value
                ),
                "max_reverse_speed_mps": float(
                    self.get_parameter("teleop_max_reverse_speed_mps").value
                ),
                "max_lateral_speed_mps": float(
                    self.get_parameter("teleop_max_lateral_speed_mps").value
                ),
                "max_angular_speed_radps": float(
                    self.get_parameter("teleop_max_angular_speed_radps").value
                ),
            },
        )
        self._last_status_reason = "startup_locked"
        initial_mode = str(self.get_parameter("initial_mode").value).strip().lower()
        if initial_mode not in ("locked", "navigation"):
            raise RuntimeError("command mux initial_mode must be locked or navigation")
        self._arbiter.set_mode(initial_mode, reason="startup_%s" % initial_mode)
        self._last_status_reason = "startup_%s" % initial_mode
        self._output_pub = self.create_publisher(
            Twist, str(self.get_parameter("output_cmd_vel_topic").value), 10
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), status_qos
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("navigation_cmd_vel_topic").value),
            lambda msg: self._on_command("navigation", msg),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("teleop_cmd_vel_topic").value),
            lambda msg: self._on_command("teleop", msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mode_request_topic").value),
            self._on_mode_request,
            10,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter("lock_service").value),
            lambda request, response: self._set_mode(response, "locked", "operator_lock"),
        )
        self.create_service(
            Trigger,
            str(self.get_parameter("navigation_service").value),
            lambda request, response: self._set_mode(response, "navigation", "task_start"),
        )
        self.create_service(
            Trigger,
            str(self.get_parameter("teleop_service").value),
            lambda request, response: self._set_mode(response, "teleop", "operator_takeover"),
        )
        watchdog_hz = max(10.0, float(self.get_parameter("watchdog_rate_hz").value))
        self._watchdog_timer = self.create_timer(1.0 / watchdog_hz, self._tick_watchdog)
        self._watchdog_timer.cancel()
        self._publish_zero()
        self._publish_status(self._last_status_reason)

    @staticmethod
    def _twist_values(msg: Twist) -> Dict[str, float]:
        return {
            "linear_x": float(msg.linear.x),
            "linear_y": float(msg.linear.y),
            "angular_z": float(msg.angular.z),
        }

    @staticmethod
    def _to_twist(command: Dict[str, Any]) -> Twist:
        msg = Twist()
        msg.linear.x = float(command.get("linear_x", 0.0))
        msg.linear.y = float(command.get("linear_y", 0.0))
        msg.angular.z = float(command.get("angular_z", 0.0))
        return msg

    def _publish_zero(self) -> None:
        self._output_pub.publish(Twist())

    def _on_mode_request(self, msg: String) -> None:
        mode = str(msg.data or "").strip().lower()
        if mode not in ("locked", "navigation", "teleop"):
            self.get_logger().warning("ignored invalid command mux mode request: %s" % mode)
            return
        with self._lock:
            decision = self._arbiter.set_mode(mode, reason="mode_request_%s" % mode)
            self._last_status_reason = str(decision["reason"])
            self._watchdog_timer.cancel()
        self._output_pub.publish(self._to_twist(decision["command"]))
        self._publish_status(self._last_status_reason)

    def _set_mode(self, response: Any, mode: str, reason: str) -> Any:
        with self._lock:
            decision = self._arbiter.set_mode(mode, reason=reason)
            self._last_status_reason = reason
            self._watchdog_timer.cancel()
        self._output_pub.publish(self._to_twist(decision["command"]))
        self._publish_status(reason)
        response.success = True
        response.message = "command mux mode=%s" % mode
        return response

    def _on_command(self, source: str, msg: Twist) -> None:
        with self._lock:
            decision = self._arbiter.accept(
                source, self._twist_values(msg), now=time.monotonic()
            )
            output_nonzero = bool(self._arbiter.output_nonzero)
            if decision.get("publish"):
                if output_nonzero:
                    self._watchdog_timer.reset()
                else:
                    self._watchdog_timer.cancel()
        if decision.get("publish"):
            self._output_pub.publish(self._to_twist(decision["command"]))

    def _tick_watchdog(self) -> None:
        with self._lock:
            decision = self._arbiter.watchdog(now=time.monotonic())
            reason = str(decision.get("reason") or "")
            if reason:
                self._last_status_reason = reason
        if decision.get("publish"):
            self._watchdog_timer.cancel()
            self._output_pub.publish(self._to_twist(decision["command"]))
            self._publish_status(reason)

    def _publish_status(self, reason: str) -> None:
        with self._lock:
            payload = {
                "mode": self._arbiter.mode,
                "reason": str(reason or self._last_status_reason),
                "output_nonzero": bool(self._arbiter.output_nonzero),
                "updated_at": time.time(),
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        if rclpy.ok():
            try:
                self._publish_zero()
            except Exception:
                pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
