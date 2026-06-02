from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from .geometry import yaw_to_quaternion
from .map_manifest import load_yaml


class FloorGoalBridge(Node):
    """Publish floor-aware goals from RViz, named waypoints, or simple CLI requests."""

    def __init__(self) -> None:
        super().__init__("m20pro_floor_goal_bridge")
        self.declare_parameter("floor_config", "")
        self.declare_parameter("default_floor", "")
        self.declare_parameter("same_floor_goal_topic", "/m20pro/rviz_goal_current")
        self.declare_parameter("floor_goal_topic", "/m20pro/floor_goal")
        self.declare_parameter("goal_command_topic", "/m20pro/goal_command")
        self.declare_parameter("enable_same_floor_goal_bridge", True)

        self.config = self._load_floor_config()
        self.floors: Dict[str, Dict[str, Any]] = dict(self.config.get("floors", {}))
        self.current_floor = str(self.get_parameter("default_floor").value).strip()
        self.floor_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("floor_goal_topic").value),
            10,
        )
        self.create_subscription(PoseStamped, "/m20pro/floor_goal_raw", self._on_raw_floor_goal, 10)
        self.create_subscription(String, "/m20pro/current_floor", self._on_current_floor, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("goal_command_topic").value),
            self._on_goal_command,
            10,
        )
        if bool(self.get_parameter("enable_same_floor_goal_bridge").value):
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter("same_floor_goal_topic").value),
                self._on_same_floor_goal,
                10,
            )
        self.get_logger().info(
            "floor goal bridge ready; same-floor topic=%s, command topic=%s"
            % (
                str(self.get_parameter("same_floor_goal_topic").value),
                str(self.get_parameter("goal_command_topic").value),
            )
        )

    def _load_floor_config(self) -> Dict[str, Any]:
        path = str(self.get_parameter("floor_config").value).strip()
        if not path:
            return {}
        return load_yaml(path)

    def _on_current_floor(self, msg: Any) -> None:
        value = str(msg.data).strip()
        if value:
            self.current_floor = value

    def _on_same_floor_goal(self, msg: PoseStamped) -> None:
        target_floor = self.current_floor or str(self.get_parameter("default_floor").value).strip()
        if not target_floor:
            self.get_logger().warning("same-floor RViz goal ignored; current floor is unknown")
            return
        self._publish_pose(
            target_floor,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation,
        )

    def _on_raw_floor_goal(self, msg: PoseStamped) -> None:
        self.floor_goal_pub.publish(msg)

    def _on_goal_command(self, msg: Any) -> None:
        command = str(msg.data).strip()
        if not command:
            return
        parsed = self._parse_command(command)
        if parsed is None:
            self.get_logger().warning("ignored bad goal command: %s" % command)
            return
        floor_id, x, y, yaw = parsed
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = floor_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        self.floor_goal_pub.publish(pose)
        self.get_logger().info(
            "published floor goal from command floor=%s x=%.2f y=%.2f yaw=%.2f"
            % (floor_id, x, y, yaw)
        )

    def _publish_pose(self, floor_id: str, x: float, y: float, z: float, orientation: Any) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = floor_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation = orientation
        self.floor_goal_pub.publish(pose)
        self.get_logger().info(
            "forwarded RViz same-floor goal as floor goal floor=%s x=%.2f y=%.2f"
            % (floor_id, pose.pose.position.x, pose.pose.position.y)
        )

    def _parse_command(self, command: str) -> Optional[tuple]:
        parts = command.replace(",", " ").split()
        if not parts:
            return None

        if len(parts) == 1:
            waypoint = self._find_waypoint(parts[0])
            if waypoint is None:
                return None
            return waypoint

        if len(parts) in (3, 4):
            floor_id = parts[0]
            if floor_id not in self.floors:
                return None
            try:
                x = float(parts[1])
                y = float(parts[2])
                yaw = float(parts[3]) if len(parts) == 4 else 0.0
            except ValueError:
                return None
            return floor_id, x, y, yaw
        return None

    def _find_waypoint(self, waypoint_id: str) -> Optional[tuple]:
        for floor_id, floor in self.floors.items():
            for waypoint in floor.get("waypoints") or []:
                if not isinstance(waypoint, dict):
                    continue
                if str(waypoint.get("id") or "") != waypoint_id:
                    continue
                pose = waypoint.get("pose") or {}
                try:
                    return (
                        floor_id,
                        float(pose.get("x", 0.0)),
                        float(pose.get("y", 0.0)),
                        float(pose.get("yaw", 0.0)),
                    )
                except (TypeError, ValueError):
                    return None
        return None


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = FloorGoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
