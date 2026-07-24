"""Single ROS 2 executor for one directed stair connector."""

from __future__ import annotations

import json
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .geometry import quaternion_to_yaw, yaw_to_quaternion
from .stair_executor_contract import (
    connector_motion_decision,
    connector_nav_status_event,
    create_connector_execution,
    step_connector_execution,
)


TERMINAL_STATES = {"COMPLETED", "STOPPED", "FAILED"}


def _json_message(payload: Dict[str, Any]) -> String:
    message = String()
    message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return message


def _parse_json(value: Any) -> Optional[Dict[str, Any]]:
    try:
        parsed = json.loads(str(value or ""))
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return parsed if isinstance(parsed, dict) else None


class StairExecutorNode(Node):
    """Run entry, straight stair motion, map switch and exit as one chain."""

    def __init__(self) -> None:
        super().__init__("m20pro_stair_executor")
        self.declare_parameter("enabled", False)
        self.declare_parameter("start_topic", "/m20pro/stair_executor/start")
        self.declare_parameter("status_topic", "/m20pro/stair_executor/status")
        self.declare_parameter("floor_goal_topic", "/m20pro/floor_goal")
        self.declare_parameter("floor_switch_request_topic", "/m20pro/floor_switch_request")
        self.declare_parameter("floor_switch_result_topic", "/m20pro/floor_switch_result")
        self.declare_parameter("current_floor_topic", "/m20pro/current_floor")
        self.declare_parameter("stair_status_topic", "/m20pro/stair_status")
        self.declare_parameter("stop_task_topic", "/m20pro/stop_task")
        self.declare_parameter("robot_pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("localization_ok_topic", "/m20pro_tcp_bridge/localization_ok")
        self.declare_parameter("gait_command_topic", "/m20pro/gait_command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("stage_timeout_s", 180.0)
        self.declare_parameter("motion_speed_mps", 0.12)
        self.declare_parameter("motion_command_hz", 10.0)
        self.declare_parameter("motion_pose_timeout_s", 1.5)
        self.declare_parameter("gait_settle_s", 1.0)
        self.declare_parameter("platform_tolerance_m", 0.50)
        self.declare_parameter("post_switch_goal_delay_s", 0.20)
        self.declare_parameter("floor_sync_timeout_s", 3.0)
        self.declare_parameter("heartbeat_period_s", 1.0)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._execution: Optional[Dict[str, Any]] = None
        self._expected_nav_stage: Optional[str] = None
        self._expected_nav_goal_seq: Optional[int] = None
        self._motion_active = False
        self._motion_target: Optional[Dict[str, Any]] = None
        self._motion_direction = ""
        self._motion_started_monotonic = 0.0
        self._motion_start_after_monotonic = 0.0
        self._robot_pose: Optional[Dict[str, float]] = None
        self._robot_pose_monotonic = 0.0
        self._localization_ok: Optional[bool] = None
        self._current_floor = ""
        self._pending_exit_action: Optional[Dict[str, Any]] = None
        self._pending_exit_ready_monotonic = 0.0
        self._pending_exit_deadline_monotonic = 0.0
        self._last_heartbeat_monotonic = 0.0

        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._floor_goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("floor_goal_topic").value), 10
        )
        self._floor_switch_pub = self.create_publisher(
            String, str(self.get_parameter("floor_switch_request_topic").value), 10
        )
        self._gait_pub = self.create_publisher(
            String, str(self.get_parameter("gait_command_topic").value), 10
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )

        self.create_subscription(
            String, str(self.get_parameter("start_topic").value), self._on_start, 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("floor_switch_result_topic").value),
            self._on_floor_switch_result,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("current_floor_topic").value),
            self._on_current_floor,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stair_status_topic").value),
            self._on_stair_status,
            10,
        )
        self.create_subscription(
            String, str(self.get_parameter("stop_task_topic").value), self._on_stop_task, 10
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("robot_pose_topic").value),
            self._on_robot_pose,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("localization_ok_topic").value),
            self._on_localization_ok,
            10,
        )
        motion_hz = max(2.0, float(self.get_parameter("motion_command_hz").value))
        self.create_timer(1.0 / motion_hz, self._on_tick)
        self._publish_status(
            {
                "ok": self._enabled,
                "code": "stair_executor_ready" if self._enabled else "stair_executor_disabled",
                "message": "楼梯执行器已就绪" if self._enabled else "楼梯执行器未启用",
                "execution": {},
                "actions": [],
            }
        )

    def _on_start(self, message: String) -> None:
        payload = _parse_json(message.data)
        if payload is None:
            self._publish_status(self._error("connector_start_invalid", "楼梯执行启动请求不是 JSON 对象"))
            return
        if not self._enabled:
            self._publish_status(
                self._start_error(
                    payload,
                    "stair_executor_disabled",
                    "楼梯执行器未启用",
                )
            )
            return
        if self._is_active():
            self._publish_status(
                self._start_error(
                    payload,
                    "connector_busy",
                    "已有楼梯连接边正在执行",
                )
            )
            return
        self._reset_runtime()
        result = create_connector_execution(
            payload.get("route") if isinstance(payload.get("route"), dict) else {},
            request_id=payload.get("request_id"),
            plan_id=payload.get("plan_id"),
            map_epoch=payload.get("map_epoch"),
            now_monotonic=self._now(),
            stage_timeout_s=float(self.get_parameter("stage_timeout_s").value),
        )
        self._apply_result(result)

    def _on_stair_status(self, message: String) -> None:
        if not self._is_active() or not self._expected_nav_stage:
            return
        text = str(message.data or "").strip()
        decision = connector_nav_status_event(
            text,
            expected_goal_seq=self._expected_nav_goal_seq,
        )
        action = str(decision.get("action") or "")
        if action == "accepted":
            self._expected_nav_goal_seq = int(decision["goal_seq"])
            return
        if action == "reached":
            event_type = "entry_reached" if self._expected_nav_stage == "entry" else "exit_reached"
            self._expected_nav_stage = None
            self._expected_nav_goal_seq = None
            self._apply_event(event_type)
        elif action == "failed":
            self._apply_event(
                "navigation_failed",
                reason=str(decision.get("code") or "connector_nav_goal_failed"),
            )

    def _on_floor_switch_result(self, message: String) -> None:
        if not self._is_active() or self._state() != "PLATFORM_HOLD":
            return
        payload = _parse_json(message.data)
        if payload is None or not self._identity_matches(payload):
            return
        self._apply_event(
            "floor_switch_result",
            ok=bool(payload.get("ok")),
            target_floor=payload.get("target_floor"),
            target_map_id=payload.get("target_map_id"),
        )

    def _on_stop_task(self, message: String) -> None:
        if self._is_active():
            self._apply_event("stop_requested", reason=str(message.data or "manual_stop"))

    def _on_robot_pose(self, message: PoseStamped) -> None:
        self._robot_pose = {
            "x": float(message.pose.position.x),
            "y": float(message.pose.position.y),
            "yaw": quaternion_to_yaw(message.pose.orientation),
        }
        self._robot_pose_monotonic = self._now()

    def _on_current_floor(self, message: String) -> None:
        self._current_floor = str(message.data or "").strip()

    def _on_localization_ok(self, message: Bool) -> None:
        self._localization_ok = bool(message.data)

    def _on_tick(self) -> None:
        now = self._now()
        if self._is_active():
            watchdog = step_connector_execution(
                dict(self._execution or {}),
                {"type": "watchdog_tick", **self._identity()},
                now_monotonic=now,
            )
            if watchdog.get("actions"):
                self._apply_result(watchdog)
            if self._motion_active and self._state() == "TRAVERSING":
                self._tick_motion(now)
            if self._pending_exit_action is not None:
                self._tick_pending_exit_goal(now)
        heartbeat_period = max(0.2, float(self.get_parameter("heartbeat_period_s").value))
        if now - self._last_heartbeat_monotonic >= heartbeat_period:
            self._last_heartbeat_monotonic = now
            self._publish_runtime_heartbeat()

    def _tick_motion(self, now: float) -> None:
        pose_timeout = float(self.get_parameter("motion_pose_timeout_s").value)
        if now < self._motion_start_after_monotonic:
            self._cmd_vel_pub.publish(Twist())
            return
        if self._robot_pose_monotonic > 0.0:
            pose_age = now - self._robot_pose_monotonic
        else:
            pose_age = now - self._motion_started_monotonic
        decision = connector_motion_decision(
            current_pose=self._robot_pose,
            target_pose=self._motion_target,
            pose_age_s=pose_age,
            pose_timeout_s=pose_timeout,
            tolerance_m=float(self.get_parameter("platform_tolerance_m").value),
            speed_mps=float(self.get_parameter("motion_speed_mps").value),
            direction=self._motion_direction,
        )
        action = str(decision.get("action") or "")
        if action == "move":
            command = Twist()
            command.linear.x = float(decision["linear_x"])
            self._cmd_vel_pub.publish(command)
            return
        if action == "wait":
            return
        self._stop_motion()
        if action == "reached":
            self._apply_event("platform_reached")
            return
        self._apply_event(
            "communication_timeout",
            reason=str(decision.get("code") or "connector_motion_failed"),
        )

    def _apply_event(self, event_type: str, **extra: Any) -> None:
        if self._execution is None:
            return
        result = step_connector_execution(
            dict(self._execution),
            {"type": event_type, **self._identity(), **extra},
            now_monotonic=self._now(),
        )
        self._apply_result(result)

    def _apply_result(self, result: Dict[str, Any]) -> None:
        execution = result.get("execution")
        if isinstance(execution, dict):
            self._execution = dict(execution)
        for action in result.get("actions") or []:
            if isinstance(action, dict):
                self._apply_action(action)
        if self._state() in TERMINAL_STATES:
            self._pending_exit_action = None
            self._pending_exit_ready_monotonic = 0.0
            self._pending_exit_deadline_monotonic = 0.0
        self._publish_status(result)

    def _apply_action(self, action: Dict[str, Any]) -> None:
        kind = str(action.get("kind") or "")
        if kind == "dispatch_entry_goal":
            self._publish_floor_goal(action, stage="entry")
        elif kind == "dispatch_exit_goal":
            now = self._now()
            self._pending_exit_action = dict(action)
            self._pending_exit_ready_monotonic = now + max(
                0.0, float(self.get_parameter("post_switch_goal_delay_s").value)
            )
            self._pending_exit_deadline_monotonic = now + max(
                1.0, float(self.get_parameter("floor_sync_timeout_s").value)
            )
        elif kind == "set_gait":
            message = String()
            message.data = str(action.get("gait") or "")
            self._gait_pub.publish(message)
        elif kind == "start_connector_motion":
            self._motion_target = dict(action.get("target_pose") or {})
            self._motion_direction = str(action.get("direction") or "")
            self._motion_started_monotonic = self._now()
            self._motion_start_after_monotonic = self._motion_started_monotonic + max(
                0.0, float(self.get_parameter("gait_settle_s").value)
            )
            self._motion_active = True
        elif kind == "stop_motion":
            self._stop_motion()
        elif kind == "request_floor_switch":
            self._floor_switch_pub.publish(
                _json_message(
                    {
                        **self._identity(),
                        "source_floor": action.get("source_floor"),
                        "target_floor": action.get("target_floor"),
                        "target_map_id": action.get("target_map_id"),
                    }
                )
            )

    def _tick_pending_exit_goal(self, now: float) -> None:
        action = self._pending_exit_action
        if action is None:
            return
        target_floor = str((self._execution or {}).get("target_floor") or "")
        if now >= self._pending_exit_ready_monotonic and self._current_floor == target_floor:
            self._pending_exit_action = None
            self._pending_exit_ready_monotonic = 0.0
            self._pending_exit_deadline_monotonic = 0.0
            self._publish_floor_goal(action, stage="exit")
            return
        if now < self._pending_exit_deadline_monotonic:
            return
        self._pending_exit_action = None
        self._pending_exit_ready_monotonic = 0.0
        self._pending_exit_deadline_monotonic = 0.0
        self._apply_event("communication_timeout", reason="floor_context_sync_timeout")

    def _publish_floor_goal(self, action: Dict[str, Any], *, stage: str) -> None:
        pose = action.get("pose") if isinstance(action.get("pose"), dict) else {}
        execution = self._execution or {}
        floor = execution.get("source_floor") if stage == "entry" else execution.get("target_floor")
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(floor or "")
        message.pose.position.x = float(pose.get("x", 0.0))
        message.pose.position.y = float(pose.get("y", 0.0))
        message.pose.position.z = float(pose.get("z", 0.0))
        message.pose.orientation = yaw_to_quaternion(float(pose.get("yaw", 0.0)))
        self._expected_nav_stage = stage
        self._expected_nav_goal_seq = None
        self._floor_goal_pub.publish(message)

    def _stop_motion(self) -> None:
        self._motion_active = False
        self._motion_target = None
        self._motion_direction = ""
        self._motion_start_after_monotonic = 0.0
        self._cmd_vel_pub.publish(Twist())

    def _reset_runtime(self) -> None:
        self._stop_motion()
        self._execution = None
        self._expected_nav_stage = None
        self._expected_nav_goal_seq = None
        self._motion_started_monotonic = 0.0
        self._pending_exit_action = None
        self._pending_exit_ready_monotonic = 0.0
        self._pending_exit_deadline_monotonic = 0.0

    def _is_active(self) -> bool:
        return self._execution is not None and self._state() not in TERMINAL_STATES

    def _state(self) -> str:
        return str((self._execution or {}).get("state") or "IDLE").upper()

    def _identity(self) -> Dict[str, Any]:
        execution = self._execution or {}
        return {
            "request_id": execution.get("request_id"),
            "route_id": execution.get("route_id"),
            "plan_id": execution.get("plan_id"),
            "map_epoch": execution.get("map_epoch"),
        }

    def _identity_matches(self, payload: Dict[str, Any]) -> bool:
        identity = self._identity()
        for key in ("request_id", "route_id", "plan_id"):
            if str(payload.get(key) or "") != str(identity.get(key) or ""):
                return False
        try:
            return int(payload.get("map_epoch")) == int(identity.get("map_epoch"))
        except (TypeError, ValueError):
            return False

    def _publish_runtime_heartbeat(self) -> None:
        self._publish_status(
            {
                "ok": self._enabled,
                "code": "stair_executor_ready" if self._enabled else "stair_executor_disabled",
                "message": "楼梯执行器运行中" if self._is_active() else "楼梯执行器已就绪",
                "execution": dict(self._execution or {}),
                "actions": [],
            }
        )

    def _publish_status(self, result: Dict[str, Any]) -> None:
        execution = result.get("execution")
        if not isinstance(execution, dict):
            execution = self._execution or {}
        payload = {
            "component": "stair_executor",
            "enabled": self._enabled,
            "ready": self._enabled,
            "busy": self._is_active(),
            "ok": bool(result.get("ok")),
            "code": result.get("code"),
            "message": result.get("message"),
            "state": execution.get("state") or "IDLE",
            "status": execution.get("status") or "idle",
            "source_floor": execution.get("source_floor"),
            "target_floor": execution.get("target_floor"),
            "source_map_id": execution.get("source_map_id"),
            "target_map_id": execution.get("target_map_id"),
            "localization_ok": self._localization_ok,
            "request_id": execution.get("request_id"),
            "route_id": execution.get("route_id"),
            "plan_id": execution.get("plan_id"),
            "map_epoch": execution.get("map_epoch"),
        }
        self._status_pub.publish(_json_message(payload))

    @staticmethod
    def _start_error(
        payload: Dict[str, Any],
        code: str,
        message: str,
    ) -> Dict[str, Any]:
        route = payload.get("route") if isinstance(payload.get("route"), dict) else {}
        return {
            "ok": False,
            "code": code,
            "message": message,
            "execution": {
                "request_id": payload.get("request_id"),
                "route_id": route.get("id"),
                "plan_id": payload.get("plan_id"),
                "map_epoch": payload.get("map_epoch"),
                "source_floor": route.get("source_floor"),
                "target_floor": route.get("target_floor"),
                "source_map_id": route.get("source_map_id"),
                "target_map_id": route.get("target_map_id"),
                "state": "FAILED",
                "status": "failed",
            },
            "actions": [],
        }

    @staticmethod
    def _error(code: str, message: str) -> Dict[str, Any]:
        return {"ok": False, "code": code, "message": message, "actions": []}

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def destroy_node(self) -> bool:
        try:
            self._stop_motion()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = StairExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
