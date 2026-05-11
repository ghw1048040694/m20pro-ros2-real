import math
import struct
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class LidarSimulator(Node):
    """Publish synthetic lidar point clouds from occupancy map and dynamic obstacles."""

    def __init__(self):
        super().__init__("m20pro_lidar_simulator")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("robot_pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("dynamic_obstacle_topic", "/dynamic_obstacles")
        self.declare_parameter("lidar_topic", "/LIDAR/POINTS")
        self.declare_parameter("cloud_nav_topic", "/cloud_nav")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("min_range", 0.2)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("fov_rad", 6.28318530718)
        self.declare_parameter("num_rays", 720)
        self.declare_parameter("ray_step_m", 0.05)
        self.declare_parameter("obstacle_threshold", 65)
        self.declare_parameter("dynamic_obstacle_radius", 0.2)
        self.declare_parameter("point_z", 0.15)

        self.map_msg: Optional[OccupancyGrid] = None
        self.pose_msg: Optional[PoseStamped] = None
        self.dynamic_obstacles: List[Tuple[float, float]] = []

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("robot_pose_topic").value),
            self._on_pose,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("dynamic_obstacle_topic").value),
            self._on_dynamic_obstacles,
            10,
        )

        self.cloud_pub = self.create_publisher(PointCloud2, str(self.get_parameter("lidar_topic").value), 10)
        self.cloud_nav_pub = self.create_publisher(PointCloud2, str(self.get_parameter("cloud_nav_topic").value), 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("lidar simulator ready: publishing /LIDAR/POINTS and /cloud_nav")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.get_logger().info("lidar simulator got map: %dx%d" % (msg.info.width, msg.info.height))

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose_msg = msg

    def _on_dynamic_obstacles(self, msg: PoseArray) -> None:
        self.dynamic_obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def _tick(self) -> None:
        if self.map_msg is None or self.pose_msg is None:
            return
        cloud = self._build_cloud()
        self.cloud_pub.publish(cloud)
        self.cloud_nav_pub.publish(cloud)

    def _build_cloud(self) -> PointCloud2:
        assert self.map_msg is not None
        assert self.pose_msg is not None
        x0 = self.pose_msg.pose.position.x
        y0 = self.pose_msg.pose.position.y
        yaw = self._yaw_from_pose(self.pose_msg)

        min_range = float(self.get_parameter("min_range").value)
        max_range = float(self.get_parameter("max_range").value)
        fov = float(self.get_parameter("fov_rad").value)
        num_rays = max(8, int(self.get_parameter("num_rays").value))
        ray_step = max(0.02, float(self.get_parameter("ray_step_m").value))
        point_z = float(self.get_parameter("point_z").value)

        start_angle = -0.5 * fov
        angle_step = fov / float(num_rays - 1)

        points = []
        for i in range(num_rays):
            local_angle = start_angle + i * angle_step
            world_angle = yaw + local_angle
            hit = self._raycast(x0, y0, world_angle, min_range, max_range, ray_step)
            if hit is None:
                continue
            distance = hit
            lx = distance * math.cos(local_angle)
            ly = distance * math.sin(local_angle)
            points.append((lx, ly, point_z))

        return self._points_to_cloud(points)

    def _raycast(
        self, x0: float, y0: float, world_angle: float, min_range: float, max_range: float, step: float
    ) -> Optional[float]:
        assert self.map_msg is not None
        cos_a = math.cos(world_angle)
        sin_a = math.sin(world_angle)
        dyn_radius = float(self.get_parameter("dynamic_obstacle_radius").value)
        obstacle_threshold = int(self.get_parameter("obstacle_threshold").value)

        distance = min_range
        while distance <= max_range:
            wx = x0 + cos_a * distance
            wy = y0 + sin_a * distance
            if self._is_dynamic_obstacle(wx, wy, dyn_radius):
                return distance
            cell = self._world_to_cell(wx, wy)
            if cell is None:
                return distance
            cx, cy = cell
            value = self.map_msg.data[cy * self.map_msg.info.width + cx]
            if value >= obstacle_threshold and value >= 0:
                return distance
            distance += step
        return None

    def _is_dynamic_obstacle(self, wx: float, wy: float, radius: float) -> bool:
        for ox, oy in self.dynamic_obstacles:
            if (wx - ox) * (wx - ox) + (wy - oy) * (wy - oy) <= radius * radius:
                return True
        return False

    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        assert self.map_msg is not None
        origin = self.map_msg.info.origin.position
        res = self.map_msg.info.resolution
        gx = int((x - origin.x) / res)
        gy = int((y - origin.y) / res)
        if 0 <= gx < self.map_msg.info.width and 0 <= gy < self.map_msg.info.height:
            return gx, gy
        return None

    @staticmethod
    def _yaw_from_pose(pose_msg: PoseStamped) -> float:
        q = pose_msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _points_to_cloud(self, points: List[Tuple[float, float, float]]) -> PointCloud2:
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", x, y, z) for (x, y, z) in points)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
