import math
import numpy as np
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import PointCloud2, PointField


class DualLidarSimulator(Node):
    def __init__(self):
        super().__init__("m20pro_dual_lidar_simulator")
        
        # === 声明参数 ===
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("robot_pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("dynamic_obstacle_topic", "/dynamic_obstacles")
        
        # 前雷达参数
        self.declare_parameter("front_lidar_topic", "/LIDAR/FRONT/POINTS")
        self.declare_parameter("front_lidar_offset_x", 0.3)  
        self.declare_parameter("front_lidar_offset_y", 0.0)
        self.declare_parameter("front_lidar_yaw", 0.0)
        
        # 后雷达参数
        self.declare_parameter("rear_lidar_topic", "/LIDAR/REAR/POINTS")
        self.declare_parameter("rear_lidar_offset_x", -0.3)  
        self.declare_parameter("rear_lidar_offset_y", 0.0)
        self.declare_parameter("rear_lidar_yaw", math.pi)  

        # 导航主链路点云：与真机 AOS DDS /cloud_nav 保持一致
        self.declare_parameter("cloud_nav_topic", "/cloud_nav")
        self.declare_parameter("publish_debug_lidars", False)
        
        # 通用雷达参数
        self.declare_parameter("num_horizontal_rays", 180)
        self.declare_parameter("num_vertical_layers", 8)
        self.declare_parameter("horizontal_stride", 1)
        self.declare_parameter("vertical_stride", 1)
        self.declare_parameter("vertical_fov_deg", 30.0)     
        self.declare_parameter("min_range", 0.2)
        self.declare_parameter("max_range", 15.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ray_step", 0.05)
        
        # 噪声模型参数
        self.declare_parameter("enable_range_noise", True)
        self.declare_parameter("range_noise_sigma_base", 0.02)
        self.declare_parameter("range_noise_sigma_per_meter", 0.01)
        self.declare_parameter("range_bias_factor", 0.015)
        
        # 漏检模型参数
        self.declare_parameter("enable_dropout", True)
        self.declare_parameter("dropout_distance_scale", 20.0)
        
        # 其他参数
        self.declare_parameter("obstacle_threshold", 65)
        self.declare_parameter("dynamic_obstacle_radius", 0.2)
        
        # === 初始化成员变量 ===
        self.map_msg: Optional[OccupancyGrid] = None
        self.pose_msg: Optional[PoseStamped] = None
        self.dynamic_obstacles: List[Tuple[float, float]] = []
        
        # === 创建订阅者 ===
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
        
        # === 创建发布者 ===
        cloud_qos = QoSProfile(depth=1)
        self.front_cloud_pub = self.create_publisher(
            PointCloud2, 
            str(self.get_parameter("front_lidar_topic").value), 
            cloud_qos
        )
        self.rear_cloud_pub = self.create_publisher(
            PointCloud2, 
            str(self.get_parameter("rear_lidar_topic").value), 
            cloud_qos
        )
        self.cloud_nav_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("cloud_nav_topic").value),
            cloud_qos
        )
        
        # === 初始化随机数生成器 ===
        self.rng = np.random.default_rng(seed=42)
        
        # === 创建定时器 ===
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        
        self.get_logger().info("Dual LiDAR simulator initialized")
    
    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.get_logger().info("Map received: %dx%d" % (msg.info.width, msg.info.height))
    
    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose_msg = msg
    
    def _on_dynamic_obstacles(self, msg: PoseArray) -> None:
        self.dynamic_obstacles = [(p.position.x, p.position.y) for p in msg.poses]
    
    def _tick(self) -> None:
        if self.map_msg is None or self.pose_msg is None:
            return

        horizontal_angles, vertical_angles = self._scan_angles()
        
        # 分别生成前后雷达的点云
        front_cloud = self._build_lidar_cloud(
            offset_x=float(self.get_parameter("front_lidar_offset_x").value),
            offset_y=float(self.get_parameter("front_lidar_offset_y").value),
            yaw_offset=float(self.get_parameter("front_lidar_yaw").value),
            horizontal_angles=horizontal_angles,
            vertical_angles=vertical_angles,
        )
        
        rear_cloud = self._build_lidar_cloud(
            offset_x=float(self.get_parameter("rear_lidar_offset_x").value),
            offset_y=float(self.get_parameter("rear_lidar_offset_y").value),
            yaw_offset=float(self.get_parameter("rear_lidar_yaw").value),
            horizontal_angles=horizontal_angles,
            vertical_angles=vertical_angles,
        )
        
        if bool(self.get_parameter("publish_debug_lidars").value):
            self.front_cloud_pub.publish(front_cloud)
            self.rear_cloud_pub.publish(rear_cloud)
        self.cloud_nav_pub.publish(self._merge_clouds(front_cloud, rear_cloud))
    
    def _scan_angles(self) -> Tuple[np.ndarray, np.ndarray]:
        num_horizontal = max(1, int(self.get_parameter("num_horizontal_rays").value))
        num_vertical = max(1, int(self.get_parameter("num_vertical_layers").value))
        horizontal_stride = max(1, int(self.get_parameter("horizontal_stride").value))
        vertical_stride = max(1, int(self.get_parameter("vertical_stride").value))
        vertical_fov = math.radians(float(self.get_parameter("vertical_fov_deg").value))

        horizontal_angles = np.linspace(-math.pi, math.pi, num_horizontal, endpoint=False)
        if num_vertical == 1:
            vertical_angles = np.array([0.0], dtype=np.float64)
        else:
            vertical_angles = np.linspace(-vertical_fov / 2, vertical_fov / 2, num_vertical)

        return horizontal_angles[::horizontal_stride], vertical_angles[::vertical_stride]

    def _build_lidar_cloud(self, offset_x: float, offset_y: float, 
                          yaw_offset: float,
                          horizontal_angles: np.ndarray,
                          vertical_angles: np.ndarray) -> PointCloud2:
        """
        构建单个激光雷达的点云
        
        模拟双雷达 3D 扫描，输出点已经转换到 base_link。
        """
        assert self.map_msg is not None
        assert self.pose_msg is not None
        
        # 雷达在世界坐标系中的位置和姿态
        base_x = self.pose_msg.pose.position.x
        base_y = self.pose_msg.pose.position.y
        base_yaw = self._yaw_from_pose(self.pose_msg)
        
        lidar_x = base_x + offset_x * math.cos(base_yaw) - offset_y * math.sin(base_yaw)
        lidar_y = base_y + offset_x * math.sin(base_yaw) + offset_y * math.cos(base_yaw)
        lidar_yaw = base_yaw + yaw_offset
        
        # 参数读取
        min_range = float(self.get_parameter("min_range").value)
        max_range = float(self.get_parameter("max_range").value)
        points = []
        
        # 对每个垂直层和水平角度进行射线追踪
        for v_angle in vertical_angles:
            for h_angle in horizontal_angles:
                # 计算世界坐标系中的射线方向
                world_h_angle = lidar_yaw + h_angle
                
                # 射线追踪（考虑 3D）
                hit_distance = self._raycast_3d(
                    lidar_x, lidar_y, v_angle,
                    world_h_angle, min_range, max_range
                )
                
                if hit_distance is None:
                    continue
                
                # 应用噪声模型
                if bool(self.get_parameter("enable_range_noise").value):
                    hit_distance = self._add_range_noise(hit_distance)
                
                # 检查是否漏检
                if bool(self.get_parameter("enable_dropout").value):
                    if not self._check_detection_probability(
                        hit_distance, v_angle, h_angle
                    ):
                        continue
                
                # 计算 3D 点坐标（相对于雷达坐标系）
                lx = hit_distance * math.cos(v_angle) * math.cos(h_angle)
                ly = hit_distance * math.cos(v_angle) * math.sin(h_angle)
                lz = hit_distance * math.sin(v_angle)
                
                # 转换到机器人坐标系（base_link）
                cos_yaw = math.cos(yaw_offset)
                sin_yaw = math.sin(yaw_offset)
                
                bl_x = lx * cos_yaw - ly * sin_yaw + offset_x
                bl_y = lx * sin_yaw + ly * cos_yaw + offset_y
                bl_z = lz
                
                points.append((bl_x, bl_y, bl_z))
        
        return self._points_to_cloud(points)
    
    def _raycast_3d(self, x0: float, y0: float, vertical_angle: float,
                   horizontal_angle: float, min_range: float, 
                   max_range: float) -> Optional[float]:
        """
        3D 射线追踪
        
        简化处理：将 3D 射线投影到 2D 地图上进行碰撞检测
        """
        assert self.map_msg is not None
        
        cos_h = math.cos(horizontal_angle)
        sin_h = math.sin(horizontal_angle)
        cos_v = math.cos(vertical_angle)
        
        dyn_radius = float(self.get_parameter("dynamic_obstacle_radius").value)
        obstacle_threshold = int(self.get_parameter("obstacle_threshold").value)
        ray_step = max(0.01, float(self.get_parameter("ray_step").value))
        
        distance = min_range
        while distance <= max_range:
            # 计算世界坐标
            wx = x0 + distance * cos_v * cos_h
            wy = y0 + distance * cos_v * sin_h
            
            # 检查动态障碍物
            if self._is_dynamic_obstacle(wx, wy, dyn_radius):
                return distance
            
            # 检查静态地图
            cell = self._world_to_cell(wx, wy)
            if cell is None:
                return distance
            
            cx, cy = cell
            value = self.map_msg.data[cy * self.map_msg.info.width + cx]
            if value >= obstacle_threshold and value >= 0:
                return distance
            
            distance += ray_step
        
        return None
    
    def _add_range_noise(self, true_distance: float) -> float:
        """添加距离噪声"""
        sigma = (float(self.get_parameter("range_noise_sigma_base").value) +
                float(self.get_parameter("range_noise_sigma_per_meter").value) * true_distance)
        random_noise = self.rng.normal(0.0, sigma)
        systematic_bias = float(self.get_parameter("range_bias_factor").value) * true_distance
        
        noisy_distance = true_distance + random_noise + systematic_bias
        min_range = float(self.get_parameter("min_range").value)
        max_range = float(self.get_parameter("max_range").value)
        
        return max(min_range, min(noisy_distance, max_range))
    
    def _check_detection_probability(self, distance: float, 
                                    vertical_angle: float,
                                    horizontal_angle: float) -> bool:
        """检查是否漏检"""
        base_prob = 0.98
        distance_factor = math.exp(-distance / float(self.get_parameter("dropout_distance_scale").value))
        
        # 垂直角度过大时检测概率下降
        angle_factor = math.cos(vertical_angle)
        
        detection_prob = base_prob * distance_factor * angle_factor
        return self.rng.random() < detection_prob
    
    def _yaw_from_pose(self, pose_msg: PoseStamped) -> float:
        """从四元数提取偏航角"""
        q = pose_msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _is_dynamic_obstacle(self, wx: float, wy: float, radius: float) -> bool:
        """检查某点是否在动态障碍物内"""
        for ox, oy in self.dynamic_obstacles:
            if (wx - ox) * (wx - ox) + (wy - oy) * (wy - oy) <= radius * radius:
                return True
        return False
    
    def _world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """将世界坐标转换为地图栅格坐标"""
        assert self.map_msg is not None
        origin = self.map_msg.info.origin.position
        res = self.map_msg.info.resolution
        gx = int((x - origin.x) / res)
        gy = int((y - origin.y) / res)
        if 0 <= gx < self.map_msg.info.width and 0 <= gy < self.map_msg.info.height:
            return gx, gy
        return None
    
    def _points_to_cloud(self, points: List[Tuple[float, float, float]]) -> PointCloud2:
        """将点列表转换为 PointCloud2 消息"""
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
        msg.data = np.asarray(points, dtype=np.float32).tobytes()
        return msg

    @staticmethod
    def _merge_clouds(front_cloud: PointCloud2, rear_cloud: PointCloud2) -> PointCloud2:
        msg = PointCloud2()
        msg.header = front_cloud.header
        msg.height = 1
        msg.width = front_cloud.width + rear_cloud.width
        msg.fields = front_cloud.fields
        msg.is_bigendian = front_cloud.is_bigendian
        msg.point_step = front_cloud.point_step
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = front_cloud.is_dense and rear_cloud.is_dense
        msg.data = front_cloud.data + rear_cloud.data
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualLidarSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
