#!/usr/bin/env python3
"""Exercise the installed stair executor through real ROS 2 topics.

Run after sourcing the workspace.  Each replay uses private topic names and a
dedicated ROS domain, so no command can reach a robot on the normal domain.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


TIMEOUT_S = 8.0


def _json_message(payload: Dict[str, Any]) -> String:
    message = String()
    message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return message


class Harness(Node):
    def __init__(self, prefix: str) -> None:
        super().__init__(f"stair_executor_smoke_{os.getpid()}")
        self.statuses: List[Dict[str, Any]] = []
        self.goals: List[PoseStamped] = []
        self.gaits: List[str] = []
        self.commands: List[Twist] = []
        self.switches: List[Dict[str, Any]] = []

        self.start_pub = self.create_publisher(String, f"{prefix}/start", 10)
        self.nav_status_pub = self.create_publisher(String, f"{prefix}/nav_status", 10)
        self.floor_result_pub = self.create_publisher(String, f"{prefix}/floor_result", 10)
        self.pose_pub = self.create_publisher(PoseStamped, f"{prefix}/pose", 10)
        self.localization_pub = self.create_publisher(Bool, f"{prefix}/localization", 10)
        self.floor_pub = self.create_publisher(String, f"{prefix}/current_floor", 10)

        self.create_subscription(String, f"{prefix}/status", self._on_status, 10)
        self.create_subscription(PoseStamped, f"{prefix}/goal", self.goals.append, 10)
        self.create_subscription(String, f"{prefix}/gait", self._on_gait, 10)
        self.create_subscription(Twist, f"{prefix}/cmd_vel", self.commands.append, 10)
        self.create_subscription(String, f"{prefix}/floor_request", self._on_switch, 10)

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self.statuses.append(payload)

    def _on_gait(self, message: String) -> None:
        self.gaits.append(str(message.data or ""))

    def _on_switch(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self.switches.append(payload)


def _spin_until(node: Node, condition: Callable[[], bool], label: str) -> None:
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    raise AssertionError(f"timed out waiting for {label}")


def _publish_repeated(node: Node, publisher: Any, message: Any, count: int = 3) -> None:
    for _ in range(count):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.08)


def _pose(x: float, y: float = 0.0) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.w = 1.0
    return message


def _text(value: str) -> String:
    message = String()
    message.data = value
    return message


def _bool(value: bool) -> Bool:
    message = Bool()
    message.data = value
    return message


def _run_direction(direction: str, index: int) -> None:
    source_floor, target_floor = (("F1", "F2") if direction == "up" else ("F2", "F1"))
    prefix = f"/m20pro_stair_smoke_{os.getpid()}_{index}"
    identity = {
        "request_id": f"smoke-{direction}",
        "route_id": f"route-{direction}",
        "plan_id": f"plan-{direction}",
        "map_epoch": index + 1,
    }
    route = {
        "id": identity["route_id"],
        "source_floor": source_floor,
        "target_floor": target_floor,
        "source_map_id": f"map-{source_floor.lower()}",
        "target_map_id": f"map-{target_floor.lower()}",
        "direction": direction,
        "entry": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "source_platform": {"x": 2.0, "y": 0.0, "yaw": 0.0},
        "target_platform": {"x": 0.5, "y": 0.0, "yaw": 0.0},
        "post_exit": {"x": 1.5, "y": 0.0, "yaw": 0.0},
    }
    command = [
        "ros2",
        "run",
        "m20pro_navigation",
        "stair_executor",
        "--ros-args",
        "-p",
        "enabled:=true",
        "-p",
        "gait_settle_s:=0.0",
        "-p",
        "post_switch_goal_delay_s:=0.0",
        "-p",
        "motion_command_hz:=20.0",
        "-p",
        "heartbeat_period_s:=0.1",
    ]
    topic_parameters = {
        "start_topic": f"{prefix}/start",
        "status_topic": f"{prefix}/status",
        "floor_goal_topic": f"{prefix}/goal",
        "floor_switch_request_topic": f"{prefix}/floor_request",
        "floor_switch_result_topic": f"{prefix}/floor_result",
        "current_floor_topic": f"{prefix}/current_floor",
        "stair_status_topic": f"{prefix}/nav_status",
        "stop_task_topic": f"{prefix}/stop",
        "robot_pose_topic": f"{prefix}/pose",
        "localization_ok_topic": f"{prefix}/localization",
        "gait_command_topic": f"{prefix}/gait",
        "cmd_vel_topic": f"{prefix}/cmd_vel",
    }
    for name, value in topic_parameters.items():
        command.extend(["-p", f"{name}:={value}"])

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    node = Harness(prefix)
    try:
        _spin_until(
            node,
            lambda: any(item.get("code") == "stair_executor_ready" for item in node.statuses),
            f"{direction} executor heartbeat",
        )
        _publish_repeated(node, node.localization_pub, _bool(True))
        _publish_repeated(node, node.pose_pub, _pose(0.0))
        node.start_pub.publish(_json_message({**identity, "route": route}))
        _spin_until(node, lambda: len(node.goals) >= 1, f"{direction} entry goal")
        assert node.goals[-1].header.frame_id == source_floor

        _publish_repeated(
            node,
            node.nav_status_pub,
            _text("nav_goal_accepted label=floor_goal goal_seq=1"),
        )
        _publish_repeated(
            node,
            node.nav_status_pub,
            _text("nav_goal_succeeded label=floor_goal goal_seq=1"),
        )
        expected_gait = "stair_up" if direction == "up" else "stair_down"
        _spin_until(node, lambda: expected_gait in node.gaits, f"{direction} stair gait")
        expected_sign = 1.0 if direction == "up" else -1.0
        _spin_until(
            node,
            lambda: any(command.linear.x * expected_sign > 0.01 for command in node.commands),
            f"{direction} signed velocity",
        )

        command_count = len(node.commands)
        _publish_repeated(node, node.localization_pub, _bool(False))
        _publish_repeated(node, node.pose_pub, _pose(1.0))
        _spin_until(
            node,
            lambda: any(
                command.linear.x * expected_sign > 0.01
                for command in node.commands[command_count:]
            ),
            f"{direction} motion with fresh pose during localization flag jitter",
        )

        _publish_repeated(node, node.pose_pub, _pose(2.0))
        _spin_until(node, lambda: bool(node.switches), f"{direction} floor switch request")
        assert all(node.switches[-1].get(key) == value for key, value in identity.items())

        node.floor_result_pub.publish(
            _json_message(
                {
                    **identity,
                    "ok": True,
                    "target_floor": target_floor,
                    "target_map_id": route["target_map_id"],
                }
            )
        )
        wait_deadline = time.monotonic() + 0.3
        while time.monotonic() < wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(node.goals) == 1, "exit goal must wait for target floor context"
        _publish_repeated(node, node.floor_pub, _text(target_floor))
        _spin_until(node, lambda: len(node.goals) >= 2, f"{direction} exit goal")
        assert node.goals[-1].header.frame_id == target_floor

        _publish_repeated(
            node,
            node.nav_status_pub,
            _text("nav_goal_accepted label=floor_goal goal_seq=2"),
        )
        _publish_repeated(
            node,
            node.nav_status_pub,
            _text("nav_goal_succeeded label=floor_goal goal_seq=2"),
        )
        _spin_until(node, lambda: "flat" in node.gaits, f"{direction} flat gait")
        _spin_until(
            node,
            lambda: any(item.get("state") == "COMPLETED" for item in node.statuses),
            f"{direction} completion",
        )
    finally:
        node.destroy_node()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=3.0)
        if process.returncode not in (0, -signal.SIGINT):
            raise RuntimeError(
                f"stair executor process failed for {direction}:\n{output[-2000:]}"
            )


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    if "m20pro_navigation" not in os.environ.get("AMENT_PREFIX_PATH", ""):
        raise SystemExit(
            f"source {workspace / 'install/setup.bash'} before running this smoke test"
        )
    os.environ["ROS_DOMAIN_ID"] = os.environ.get(
        "M20PRO_SMOKE_ROS_DOMAIN_ID", str(220 + os.getpid() % 10)
    )
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    log_dir = Path(
        os.environ.get("M20PRO_SMOKE_ROS_LOG_DIR", "/tmp/m20pro-stair-executor-smoke")
    )
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(log_dir)
    rclpy.init()
    try:
        _run_direction("up", 0)
        _run_direction("down", 1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print("stair executor ROS smoke passed: up and down")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
