import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .geometry import clamp, quaternion_to_yaw, wrap_angle


class PathFollower(Node):
    """Pure-pursuit style follower that publishes /cmd_vel for the TCP bridge."""

    def __init__(self):
        super().__init__("m20pro_path_follower")
        self.declare_parameter("pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("obstacle_topic", "/m20pro_tcp_bridge/obstacle_active")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("lookahead_m", 0.8)
        self.declare_parameter("goal_tolerance_m", 0.25)
        self.declare_parameter("yaw_tolerance_rad", 0.20)
        self.declare_parameter("linear_speed", 0.25)
        self.declare_parameter("angular_gain", 0.9)
        self.declare_parameter("max_angular_z", 0.7)
        self.declare_parameter("stop_on_obstacle", True)
        self.declare_parameter("dynamic_obstacle_topic", "/dynamic_obstacles")
        self.declare_parameter("enable_local_avoidance", True)
        self.declare_parameter("enable_scan_avoidance", True)
        self.declare_parameter("obstacle_influence_radius", 1.0)
        self.declare_parameter("obstacle_stop_radius", 0.35)
        self.declare_parameter("obstacle_front_angle_rad", 1.2)
        self.declare_parameter("avoidance_gain", 0.9)
        self.declare_parameter("slowdown_gain", 0.7)
        self.declare_parameter("scan_obstacle_max_age", 0.5)
        self.declare_parameter("scan_sample_step", 2)

        self.pose: Optional[PoseStamped] = None
        self.path: Optional[Path] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time = None
        self.obstacle_active = False
        self.dynamic_obstacles: List[Tuple[float, float]] = []

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self._on_pose, 10)
        self.create_subscription(Path, str(self.get_parameter("path_topic").value), self._on_path, 10)
        self.create_subscription(Bool, str(self.get_parameter("obstacle_topic").value), self._on_obstacle, 10)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan, 10)
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("dynamic_obstacle_topic").value),
            self._on_dynamic_obstacles,
            10,
        )
        self.create_timer(0.05, self._tick)
        self.get_logger().info("path follower ready")

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def _on_path(self, msg: Path) -> None:
        self.path = msg
        self.get_logger().info("following new path with %d poses" % len(msg.poses))

    def _on_obstacle(self, msg: Bool) -> None:
        self.obstacle_active = msg.data

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

    def _on_dynamic_obstacles(self, msg: PoseArray) -> None:
        self.dynamic_obstacles = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def _tick(self) -> None:
        if self.pose is None or self.path is None or not self.path.poses:
            return
        if bool(self.get_parameter("stop_on_obstacle").value) and self.obstacle_active:
            self.cmd_pub.publish(Twist())
            return

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        yaw = quaternion_to_yaw(self.pose.pose.orientation)
        goal = self.path.poses[-1]
        goal_dist = math.hypot(goal.pose.position.x - x, goal.pose.position.y - y)
        goal_yaw = quaternion_to_yaw(goal.pose.orientation)
        if goal_dist < float(self.get_parameter("goal_tolerance_m").value):
            if abs(wrap_angle(goal_yaw - yaw)) < float(self.get_parameter("yaw_tolerance_rad").value):
                self.cmd_pub.publish(Twist())
                self.path = None
                self.get_logger().info("path goal reached")
                return

        target = self._select_lookahead(x, y)
        desired_x = target.pose.position.x - x
        desired_y = target.pose.position.y - y
        speed_scale = 1.0
        if bool(self.get_parameter("enable_local_avoidance").value):
            desired_x, desired_y, speed_scale = self._apply_local_avoidance(x, y, yaw, desired_x, desired_y)
        target_yaw = math.atan2(desired_y, desired_x)
        yaw_error = wrap_angle(target_yaw - yaw)

        cmd = Twist()
        cmd.linear.x = (
            float(self.get_parameter("linear_speed").value)
            * speed_scale
            * max(0.15, math.cos(yaw_error))
        )
        cmd.angular.z = clamp(
            float(self.get_parameter("angular_gain").value) * yaw_error,
            -float(self.get_parameter("max_angular_z").value),
            float(self.get_parameter("max_angular_z").value),
        )
        self.cmd_pub.publish(cmd)

    def _select_lookahead(self, x: float, y: float) -> PoseStamped:
        assert self.path is not None
        lookahead = float(self.get_parameter("lookahead_m").value)
        nearest_idx = 0
        nearest_dist = float("inf")
        for idx, pose in enumerate(self.path.poses):
            dist = math.hypot(pose.pose.position.x - x, pose.pose.position.y - y)
            if dist < nearest_dist:
                nearest_idx = idx
                nearest_dist = dist
        for pose in self.path.poses[nearest_idx:]:
            if math.hypot(pose.pose.position.x - x, pose.pose.position.y - y) >= lookahead:
                return pose
        return self.path.poses[-1]

    def _apply_local_avoidance(
        self, x: float, y: float, yaw: float, desired_x: float, desired_y: float
    ) -> Tuple[float, float, float]:
        influence_radius = float(self.get_parameter("obstacle_influence_radius").value)
        stop_radius = float(self.get_parameter("obstacle_stop_radius").value)
        front_angle = float(self.get_parameter("obstacle_front_angle_rad").value)
        avoidance_gain = float(self.get_parameter("avoidance_gain").value)
        slowdown_gain = float(self.get_parameter("slowdown_gain").value)

        repulse_x = 0.0
        repulse_y = 0.0
        speed_scale = 1.0
        min_front_distance = float("inf")

        obstacles = list(self.dynamic_obstacles)
        if bool(self.get_parameter("enable_scan_avoidance").value):
            obstacles.extend(self._scan_obstacles_in_map(x, y, yaw, influence_radius))

        for ox, oy in obstacles:
            dx = ox - x
            dy = oy - y
            distance = math.hypot(dx, dy)
            if distance < 1e-6 or distance > influence_radius:
                continue

            angle = wrap_angle(math.atan2(dy, dx) - yaw)
            if abs(angle) < front_angle:
                min_front_distance = min(min_front_distance, distance)

            strength = avoidance_gain * (1.0 / max(distance, 0.05) - 1.0 / influence_radius)
            strength = max(0.0, strength)
            repulse_x -= (dx / distance) * strength
            repulse_y -= (dy / distance) * strength

        if min_front_distance < stop_radius:
            return desired_x + repulse_x, desired_y + repulse_y, 0.0
        if min_front_distance < influence_radius:
            ratio = (min_front_distance - stop_radius) / max(influence_radius - stop_radius, 1e-3)
            speed_scale = clamp(slowdown_gain * ratio + (1.0 - slowdown_gain), 0.0, 1.0)

        mixed_x = desired_x + repulse_x
        mixed_y = desired_y + repulse_y
        if abs(mixed_x) < 1e-6 and abs(mixed_y) < 1e-6:
            mixed_x = desired_x
            mixed_y = desired_y
        return mixed_x, mixed_y, speed_scale

    def _scan_obstacles_in_map(
        self, x: float, y: float, yaw: float, influence_radius: float
    ) -> List[Tuple[float, float]]:
        if self.latest_scan is None or self.latest_scan_time is None:
            return []

        max_age = float(self.get_parameter("scan_obstacle_max_age").value)
        age = (self.get_clock().now() - self.latest_scan_time).nanoseconds * 1e-9
        if age > max_age:
            return []

        scan = self.latest_scan
        step = max(1, int(self.get_parameter("scan_sample_step").value))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        obstacles: List[Tuple[float, float]] = []

        for idx in range(0, len(scan.ranges), step):
            distance = scan.ranges[idx]
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            if distance > influence_radius:
                continue

            angle = scan.angle_min + idx * scan.angle_increment
            rx = distance * math.cos(angle)
            ry = distance * math.sin(angle)
            wx = x + rx * cos_yaw - ry * sin_yaw
            wy = y + rx * sin_yaw + ry * cos_yaw
            obstacles.append((wx, wy))

        return obstacles


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
