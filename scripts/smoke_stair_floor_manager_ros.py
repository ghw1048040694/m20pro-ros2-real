#!/usr/bin/env python3
"""Exercise the installed floor manager and stair executor as one ROS chain."""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String


TIMEOUT_S = 10.0


def _json_message(payload: Dict[str, Any]) -> String:
    message = String()
    message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    return message


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


class Harness(Node):
    def __init__(self, prefix: str, index: int) -> None:
        super().__init__(f"stair_floor_manager_smoke_{os.getpid()}_{index}")
        self.statuses: List[Dict[str, Any]] = []
        self.gaits: List[str] = []
        self.commands: List[Twist] = []
        self.switches: List[Dict[str, Any]] = []
        self.current_floors: List[str] = []
        self.nav_goals: List[PoseStamped] = []
        self.exit_goal_received = threading.Event()
        self.release_exit_goal = threading.Event()

        self.start_pub = self.create_publisher(String, f"{prefix}/start", 10)
        self.floor_result_pub = self.create_publisher(String, f"{prefix}/floor_result", 10)
        self.pose_pub = self.create_publisher(PoseStamped, f"{prefix}/pose", 10)
        self.localization_pub = self.create_publisher(Bool, f"{prefix}/localization", 10)
        self.floor_context_pub = self.create_publisher(String, f"{prefix}/floor_context", 10)

        self.create_subscription(String, f"{prefix}/status", self._on_status, 10)
        self.create_subscription(String, f"{prefix}/gait", self._on_gait, 10)
        self.create_subscription(Twist, f"{prefix}/cmd_vel", self.commands.append, 10)
        self.create_subscription(String, f"{prefix}/floor_request", self._on_switch, 10)
        self.create_subscription(String, f"{prefix}/current_floor", self._on_floor, 10)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            f"{prefix}/navigate_to_pose",
            execute_callback=self._execute_navigation,
        )

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

    def _on_floor(self, message: String) -> None:
        self.current_floors.append(str(message.data or "").strip())

    def _execute_navigation(self, goal_handle: Any) -> NavigateToPose.Result:
        pose = goal_handle.request.pose
        self.nav_goals.append(pose)
        if len(self.nav_goals) == 2:
            self.exit_goal_received.set()
            if not self.release_exit_goal.wait(timeout=TIMEOUT_S):
                goal_handle.abort()
                return NavigateToPose.Result()
        goal_handle.succeed()
        return NavigateToPose.Result()

    def destroy_node(self) -> bool:
        self.release_exit_goal.set()
        self._action_server.destroy()
        return super().destroy_node()


def _wait(condition: Callable[[], bool], label: str) -> None:
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.02)
    raise AssertionError(f"timed out waiting for {label}")


def _publish_repeated(publisher: Any, message: Any, count: int = 3) -> None:
    for _ in range(count):
        publisher.publish(message)
        time.sleep(0.08)


def _stop_process(process: subprocess.Popen[str]) -> str:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=3.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        output, _ = process.communicate(timeout=3.0)
    if process.returncode not in (0, -signal.SIGINT):
        raise RuntimeError(
            f"ROS process exited with {process.returncode}:\n{output[-3000:]}"
        )
    return output


def _run_direction(direction: str, index: int, workspace: Path) -> None:
    source_floor, target_floor = (("F1", "F2") if direction == "up" else ("F2", "F1"))
    prefix = f"/m20pro_floor_chain_smoke_{os.getpid()}_{index}"
    identity = {
        "request_id": f"floor-chain-{direction}",
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
    shared_topics = {
        "floor_goal_topic": f"{prefix}/floor_goal",
        "floor_switch_request_topic": f"{prefix}/floor_request",
        "floor_switch_result_topic": f"{prefix}/floor_result",
        "current_floor_topic": f"{prefix}/current_floor",
        "stair_status_topic": f"{prefix}/nav_status",
        "stop_task_topic": f"{prefix}/stop",
        "robot_pose_topic": f"{prefix}/pose",
        "gait_command_topic": f"{prefix}/gait",
        "cmd_vel_topic": f"{prefix}/cmd_vel",
    }
    floor_manager_command = [
        "ros2",
        "run",
        "m20pro_navigation",
        "floor_manager",
        "--ros-args",
        "-p",
        "enable_rviz_floor_goal_topics:=false",
        "-p",
        "service_timeout_s:=1.0",
        "-p",
        f"navigate_to_pose_action:={prefix}/navigate_to_pose",
        "-p",
        f"floor_context_topic:={prefix}/floor_context",
        "-p",
        f"stair_command_topic:={prefix}/retired_stair",
    ]
    for name in (
        "floor_goal_topic",
        "current_floor_topic",
        "stair_status_topic",
        "stop_task_topic",
        "cmd_vel_topic",
    ):
        value = shared_topics[name]
        floor_manager_command.extend(["-p", f"{name}:={value}"])

    stair_executor_command = [
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
        "-p",
        f"start_topic:={prefix}/start",
        "-p",
        f"status_topic:={prefix}/status",
        "-p",
        f"localization_ok_topic:={prefix}/localization",
    ]
    for name, value in shared_topics.items():
        stair_executor_command.extend(["-p", f"{name}:={value}"])

    processes = [
        subprocess.Popen(
            floor_manager_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        ),
        subprocess.Popen(
            stair_executor_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        ),
    ]
    node = Harness(prefix, index)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        _wait(
            lambda: {"m20pro_floor_manager", "m20pro_stair_executor"}.issubset(
                set(node.get_node_names())
            ),
            f"{direction} installed nodes",
        )
        _wait(
            lambda: any(item.get("code") == "stair_executor_ready" for item in node.statuses),
            f"{direction} executor heartbeat",
        )
        _publish_repeated(node.floor_context_pub, _text(source_floor))
        _wait(lambda: source_floor in node.current_floors, f"{direction} source floor context")
        _publish_repeated(node.localization_pub, _bool(True))
        _publish_repeated(node.pose_pub, _pose(0.0))

        node.start_pub.publish(_json_message({**identity, "route": route}))
        expected_gait = "stair_up" if direction == "up" else "stair_down"
        _wait(lambda: len(node.nav_goals) >= 1, f"{direction} entry Nav2 goal")
        _wait(lambda: expected_gait in node.gaits, f"{direction} stair gait")
        assert "flat" in node.gaits[: node.gaits.index(expected_gait)]
        expected_sign = 1.0 if direction == "up" else -1.0
        _wait(
            lambda: any(command.linear.x * expected_sign > 0.01 for command in node.commands),
            f"{direction} signed stair velocity",
        )

        _publish_repeated(node.pose_pub, _pose(2.0))
        _wait(lambda: bool(node.switches), f"{direction} floor switch request")
        assert all(node.switches[-1].get(key) == value for key, value in identity.items())
        _publish_repeated(node.floor_context_pub, _text(target_floor))
        _wait(lambda: target_floor in node.current_floors, f"{direction} target floor context")
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

        _wait(node.exit_goal_received.is_set, f"{direction} exit Nav2 goal")
        stair_index = node.gaits.index(expected_gait)
        time.sleep(0.4)
        assert "flat" not in node.gaits[stair_index + 1 :], (
            "floor_manager changed to flat before the exit Nav2 goal completed"
        )
        assert len(node.nav_goals) == 2
        assert math.isclose(node.nav_goals[0].pose.position.x, 0.0, abs_tol=1e-6)
        assert math.isclose(node.nav_goals[1].pose.position.x, 1.5, abs_tol=1e-6)

        node.release_exit_goal.set()
        _wait(
            lambda: "flat" in node.gaits[stair_index + 1 :],
            f"{direction} final flat gait",
        )
        _wait(
            lambda: any(item.get("state") == "COMPLETED" for item in node.statuses),
            f"{direction} connector completion",
        )
    finally:
        node.release_exit_goal.set()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(node)
        node.destroy_node()
        errors = []
        for process in processes:
            try:
                _stop_process(process)
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError("\n".join(errors))


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    if "m20pro_navigation" not in os.environ.get("AMENT_PREFIX_PATH", ""):
        raise SystemExit(
            f"source {workspace / 'install/setup.bash'} before running this smoke test"
        )
    os.environ["ROS_DOMAIN_ID"] = os.environ.get(
        "M20PRO_SMOKE_ROS_DOMAIN_ID", str(200 + os.getpid() % 20)
    )
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    log_dir = Path(
        os.environ.get("M20PRO_SMOKE_ROS_LOG_DIR", "/tmp/m20pro-stair-floor-manager-smoke")
    )
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(log_dir)
    rclpy.init()
    try:
        _run_direction("up", 0, workspace)
        _run_direction("down", 1, workspace)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print("stair + floor manager ROS smoke passed: up and down")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
