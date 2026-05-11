import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class DynamicObstacle:
    x1: float
    y1: float
    x2: float
    y2: float
    radius: float
    period: float
    phase: float


class DynamicObstacleSimulator(Node):
    def __init__(self):
        super().__init__("m20pro_dynamic_obstacle_simulator")
        self.declare_parameter(
            "obstacle_specs",
            [
                "-0.5,0.2,1.8,0.2,0.20,8.0,0.0",
                "0.5,-0.6,0.5,1.0,0.18,6.0,0.35",
            ],
        )
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("marker_topic", "/dynamic_obstacle_markers")
        self.declare_parameter("pose_topic", "/dynamic_obstacles")
        self.declare_parameter("alert_topic", "/dynamic_obstacle_active")
        self.declare_parameter("alert_distance", 0.8)
        self.declare_parameter("robot_pose_topic", "/m20pro_tcp_bridge/map_pose")

        self.obstacles = self._parse_specs(self.get_parameter("obstacle_specs").value)
        self.robot_pose: Optional[Pose] = None
        self.start_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseArray, str(self.get_parameter("pose_topic").value), 10)
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 10)
        self.alert_pub = self.create_publisher(Bool, str(self.get_parameter("alert_topic").value), 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("robot_pose_topic").value),
            self._on_robot_pose,
            10,
        )

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("dynamic obstacle simulator ready with %d obstacles" % len(self.obstacles))

    def _on_robot_pose(self, msg) -> None:
        self.robot_pose = msg.pose

    def _tick(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        frame_id = str(self.get_parameter("frame_id").value)
        pose_array = PoseArray()
        pose_array.header.stamp = now.to_msg()
        pose_array.header.frame_id = frame_id

        marker_array = MarkerArray()
        alert = False
        alert_distance = float(self.get_parameter("alert_distance").value)

        for idx, obstacle in enumerate(self.obstacles):
            x, y = self._position_at(obstacle, elapsed)
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            marker = Marker()
            marker.header = pose_array.header
            marker.ns = "dynamic_obstacles"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose
            marker.pose.position.z = 0.2
            marker.scale.x = obstacle.radius * 2.0
            marker.scale.y = obstacle.radius * 2.0
            marker.scale.z = 0.4
            marker.color.r = 1.0
            marker.color.g = 0.3
            marker.color.b = 0.1
            marker.color.a = 0.85
            marker_array.markers.append(marker)

            if self.robot_pose is not None:
                dx = x - self.robot_pose.position.x
                dy = y - self.robot_pose.position.y
                if math.hypot(dx, dy) < alert_distance:
                    alert = True

        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)
        alert_msg = Bool()
        alert_msg.data = alert
        self.alert_pub.publish(alert_msg)

    @staticmethod
    def _position_at(obstacle: DynamicObstacle, elapsed: float) -> Tuple[float, float]:
        if obstacle.period <= 0.01:
            return obstacle.x1, obstacle.y1
        alpha = 0.5 * (1.0 + math.sin(2.0 * math.pi * elapsed / obstacle.period + obstacle.phase * 2.0 * math.pi))
        x = obstacle.x1 + (obstacle.x2 - obstacle.x1) * alpha
        y = obstacle.y1 + (obstacle.y2 - obstacle.y1) * alpha
        return x, y

    @staticmethod
    def _parse_specs(raw_specs) -> List[DynamicObstacle]:
        obstacles: List[DynamicObstacle] = []
        for spec in raw_specs:
            values = [float(item.strip()) for item in str(spec).split(",")]
            if len(values) == 6:
                values.append(0.0)
            if len(values) != 7:
                raise ValueError("dynamic obstacle spec must be x1,y1,x2,y2,radius,period,phase")
            obstacles.append(DynamicObstacle(*values))
        return obstacles


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
