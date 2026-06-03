import math
import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener


class PointCloudFusion(Node):
    def __init__(self):
        super().__init__("m20pro_pointcloud_fusion")
        
        # === 声明参数 ===
        self.declare_parameter("input_cloud_topic", "")
        self.declare_parameter("front_lidar_topic", "/LIDAR/FRONT/POINTS")
        self.declare_parameter("rear_lidar_topic", "/LIDAR/REAR/POINTS")
        self.declare_parameter("output_scan_topic", "/scan")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("min_angle", -math.pi)      # 扫描起始角度
        self.declare_parameter("max_angle", math.pi)       # 扫描结束角度
        self.declare_parameter("angle_increment", 0.005)   # 角度分辨率 (约 0.3°)
        self.declare_parameter("min_range", 0.2)
        self.declare_parameter("max_range", 15.0)
        self.declare_parameter("height_min", 0.05)         # 过滤高度范围
        self.declare_parameter("height_max", 0.5)
        self.declare_parameter("robot_radius", 0.25)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("transform_cloud", True)
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("transform_timeout_s", 0.05)
        self.declare_parameter("max_source_age_s", 0.25)
        self.declare_parameter("no_return_as_inf", True)
        self.declare_parameter("cloud_reliability", "reliable")
        self.declare_parameter("scan_reliability", "best_effort")
        self.declare_parameter("max_points_per_cloud", 12000)
        self.declare_parameter("min_cloud_interval_s", 0.05)
        self.declare_parameter("publish_on_cloud_update", False)
        
        # === 初始化成员变量 ===
        self.cloud_ranges: Optional[np.ndarray] = None
        self.front_ranges: Optional[np.ndarray] = None
        self.rear_ranges: Optional[np.ndarray] = None
        self.cloud_received = False
        self.front_received = False
        self.rear_received = False
        self.cloud_stamp = None
        self.front_stamp = None
        self.rear_stamp = None
        self.cloud_update_time = None
        self.front_update_time = None
        self.rear_update_time = None
        self.last_tf_warning_time = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 计算角度数组
        min_angle = float(self.get_parameter("min_angle").value)
        max_angle = float(self.get_parameter("max_angle").value)
        angle_increment = float(self.get_parameter("angle_increment").value)
        
        self.num_readings = int(round((max_angle - min_angle) / angle_increment)) + 1
        self.angle_min = min_angle
        self.angle_increment = angle_increment
        self.angle_max = self.angle_min + (self.num_readings - 1) * self.angle_increment
        
        # 初始化距离数组（填充最大值表示无检测）
        max_range = float(self.get_parameter("max_range").value)
        empty_range = self._empty_range_value()
        self.cloud_ranges = np.full(self.num_readings, empty_range, dtype=np.float32)
        self.front_ranges = np.full(self.num_readings, empty_range, dtype=np.float32)
        self.rear_ranges = np.full(self.num_readings, empty_range, dtype=np.float32)
        
        cloud_qos = QoSProfile(depth=10)
        cloud_qos.history = HistoryPolicy.KEEP_LAST
        cloud_qos.durability = DurabilityPolicy.VOLATILE
        if str(self.get_parameter("cloud_reliability").value).lower() == "best_effort":
            cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            cloud_qos.reliability = ReliabilityPolicy.RELIABLE
        scan_qos = QoSProfile(depth=1)
        scan_qos.history = HistoryPolicy.KEEP_LAST
        scan_qos.durability = DurabilityPolicy.VOLATILE
        if str(self.get_parameter("scan_reliability").value).lower() == "reliable":
            scan_qos.reliability = ReliabilityPolicy.RELIABLE
        else:
            scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.input_cloud_topic = str(self.get_parameter("input_cloud_topic").value)

        # === 创建订阅者 ===
        if self.input_cloud_topic:
            self.create_subscription(
                PointCloud2,
                self.input_cloud_topic,
                self._on_cloud,
                cloud_qos
            )
        else:
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("front_lidar_topic").value),
                self._on_front_lidar,
                cloud_qos
            )

            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("rear_lidar_topic").value),
                self._on_rear_lidar,
                cloud_qos
            )
        
        # === 创建发布者 ===
        self.scan_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter("output_scan_topic").value),
            scan_qos
        )
        
        # === 创建定时器（定期发布融合后的 scan）===
        publish_rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.scan_period = 1.0 / publish_rate
        self.create_timer(self.scan_period, self._publish_scan)
        
        if self.input_cloud_topic:
            source_desc = self.input_cloud_topic
        else:
            source_desc = "%s + %s" % (
                str(self.get_parameter("front_lidar_topic").value),
                str(self.get_parameter("rear_lidar_topic").value),
            )
        self.get_logger().info(
            "PointCloud fusion ready: %s -> %s in %s" % (
                source_desc,
                str(self.get_parameter("output_scan_topic").value),
                self._target_frame(),
            )
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        """处理导航主链路点云。"""
        if not self._should_process_cloud():
            return
        ranges = self._pointcloud_to_ranges(msg)
        if ranges is None:
            return
        self.cloud_ranges = ranges
        self.cloud_stamp = self._output_stamp_for_cloud(msg)
        self.cloud_update_time = self.get_clock().now()
        self.cloud_received = True
        if bool(self.get_parameter("publish_on_cloud_update").value):
            self._publish_scan()
    
    def _on_front_lidar(self, msg: PointCloud2) -> None:
        """处理前雷达点云"""
        ranges = self._pointcloud_to_ranges(msg)
        if ranges is None:
            return
        self.front_ranges = ranges
        self.front_stamp = self._output_stamp_for_cloud(msg)
        self.front_update_time = self.get_clock().now()
        self.front_received = True
    
    def _on_rear_lidar(self, msg: PointCloud2) -> None:
        """处理后雷达点云"""
        ranges = self._pointcloud_to_ranges(msg)
        if ranges is None:
            return
        self.rear_ranges = ranges
        self.rear_stamp = self._output_stamp_for_cloud(msg)
        self.rear_update_time = self.get_clock().now()
        self.rear_received = True
    
    def _pointcloud_to_ranges(self, cloud_msg: PointCloud2) -> Optional[np.ndarray]:
        ranges = np.full(self.num_readings, self._empty_range_value(), dtype=np.float32)
        
        height_min = float(self.get_parameter("height_min").value)
        height_max = float(self.get_parameter("height_max").value)
        min_range = float(self.get_parameter("min_range").value)
        max_range = float(self.get_parameter("max_range").value)
        
        xyz = self._extract_xyz_arrays(cloud_msg)
        if xyz is not None:
            return self._point_arrays_to_ranges(cloud_msg, xyz)

        # Fallback for unusual PointCloud2 layouts. This path is slower and is
        # mainly kept for compatibility with synthetic/test clouds.
        points = pc2.read_points(cloud_msg, field_names=("x", "y", "z"))
        robot_radius = float(self.get_parameter("robot_radius").value)
        transform = self._lookup_cloud_transform(cloud_msg)
        if transform is False:
            return None

        for x, y, z in points:
            if transform is not None:
                x, y, z = self._transform_point(x, y, z, transform)

            # 过滤高度
            if z < height_min or z > height_max:
                continue
            
            # 计算距离和角度
            distance = math.sqrt(x * x + y * y)
            
            # 过滤机器人本体附近的点
            if distance < robot_radius:
                continue
            
            if distance < min_range or distance > max_range:
                continue
            
            angle = math.atan2(y, x)
            
            # 映射到数组索引
            idx = int((angle - self.angle_min) / self.angle_increment)
            
            if idx == self.num_readings and angle <= self.angle_max + 1e-6:
                idx = self.num_readings - 1

            if 0 <= idx < self.num_readings:
                # 取最小距离（最近的障碍物）
                if distance < ranges[idx]:
                    ranges[idx] = distance
        
        return ranges

    def _extract_xyz_arrays(self, cloud_msg: PointCloud2):
        offsets = {}
        for field in cloud_msg.fields:
            if field.name in ("x", "y", "z"):
                offsets[field.name] = field.offset
        if set(offsets) != {"x", "y", "z"}:
            return None
        point_step = int(cloud_msg.point_step)
        if point_step <= 0:
            return None

        endian = ">" if cloud_msg.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [endian + "f4", endian + "f4", endian + "f4"],
                "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                "itemsize": point_step,
            }
        )
        point_count = len(cloud_msg.data) // point_step
        if point_count <= 0:
            return None
        cloud = np.frombuffer(cloud_msg.data, dtype=dtype, count=point_count)

        max_points = int(self.get_parameter("max_points_per_cloud").value)
        if max_points > 0 and point_count > max_points:
            stride = max(1, point_count // max_points)
            cloud = cloud[::stride]

        x = cloud["x"].astype(np.float32, copy=False)
        y = cloud["y"].astype(np.float32, copy=False)
        z = cloud["z"].astype(np.float32, copy=False)
        return x, y, z

    def _point_arrays_to_ranges(self, cloud_msg: PointCloud2, xyz) -> Optional[np.ndarray]:
        x, y, z = xyz
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if not np.any(valid):
            return np.full(self.num_readings, self._empty_range_value(), dtype=np.float32)

        x = x[valid]
        y = y[valid]
        z = z[valid]

        transform = self._lookup_cloud_transform(cloud_msg)
        if transform is False:
            return None
        if transform is not None:
            x, y, z = self._transform_arrays(x, y, z, transform)

        height_min = float(self.get_parameter("height_min").value)
        height_max = float(self.get_parameter("height_max").value)
        min_range = float(self.get_parameter("min_range").value)
        max_range = float(self.get_parameter("max_range").value)
        robot_radius = float(self.get_parameter("robot_radius").value)

        distance = np.hypot(x, y)
        valid = (
            (z >= height_min)
            & (z <= height_max)
            & (distance >= robot_radius)
            & (distance >= min_range)
            & (distance <= max_range)
        )
        if not np.any(valid):
            return np.full(self.num_readings, self._empty_range_value(), dtype=np.float32)

        distance = distance[valid].astype(np.float32, copy=False)
        angle = np.arctan2(y[valid], x[valid])
        idx = np.floor((angle - self.angle_min) / self.angle_increment).astype(np.int64)
        valid_idx = (idx >= 0) & (idx < self.num_readings)

        ranges = np.full(self.num_readings, self._empty_range_value(), dtype=np.float32)
        if np.any(valid_idx):
            np.minimum.at(ranges, idx[valid_idx], distance[valid_idx])
        return ranges

    def _lookup_cloud_transform(self, cloud_msg: PointCloud2):
        """Return None when no transform is needed, False when TF is unavailable."""
        if not bool(self.get_parameter("transform_cloud").value):
            return None
        source_frame = self._clean_frame(cloud_msg.header.frame_id)
        target_frame = self._target_frame()
        if not source_frame or source_frame == target_frame:
            return None

        timeout = Duration(seconds=max(0.0, float(self.get_parameter("transform_timeout_s").value)))
        if bool(self.get_parameter("use_latest_tf").value):
            lookup_time = Time()
        else:
            lookup_time = Time.from_msg(cloud_msg.header.stamp)
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time,
                timeout=timeout,
            )
        except Exception as exc:
            self._warn_tf_throttled(
                "skip cloud: no TF %s -> %s (%s)" % (
                    source_frame,
                    target_frame,
                    exc,
                )
            )
            return False

    def _output_stamp_for_cloud(self, cloud_msg: PointCloud2):
        if not bool(self.get_parameter("transform_cloud").value):
            return cloud_msg.header.stamp
        source_frame = self._clean_frame(cloud_msg.header.frame_id)
        target_frame = self._target_frame()
        if not source_frame or source_frame == target_frame:
            return cloud_msg.header.stamp
        if bool(self.get_parameter("use_latest_tf").value):
            return self.get_clock().now().to_msg()
        return cloud_msg.header.stamp

    def _source_is_fresh(self, update_time) -> bool:
        if update_time is None:
            return False
        max_age = float(self.get_parameter("max_source_age_s").value)
        if max_age <= 0.0:
            return True
        age = (self.get_clock().now() - update_time).nanoseconds * 1e-9
        return age <= max_age

    def _should_process_cloud(self) -> bool:
        min_interval = float(self.get_parameter("min_cloud_interval_s").value)
        if min_interval <= 0.0 or self.cloud_update_time is None:
            return True
        age = (self.get_clock().now() - self.cloud_update_time).nanoseconds * 1e-9
        return age >= min_interval

    def _warn_tf_throttled(self, message: str) -> None:
        now = self.get_clock().now()
        if self.last_tf_warning_time is None:
            self.last_tf_warning_time = now
            self.get_logger().warning(message)
            return
        age = (now - self.last_tf_warning_time).nanoseconds * 1e-9
        if age >= 2.0:
            self.last_tf_warning_time = now
            self.get_logger().warning(message)

    def _target_frame(self) -> str:
        return self._clean_frame(str(self.get_parameter("frame_id").value)) or "base_link"

    def _empty_range_value(self) -> float:
        if bool(self.get_parameter("no_return_as_inf").value):
            return float("inf")
        return float(self.get_parameter("max_range").value)

    @staticmethod
    def _clean_frame(frame_id: str) -> str:
        return frame_id.strip().lstrip("/")

    @staticmethod
    def _transform_point(x: float, y: float, z: float, transform) -> Tuple[float, float, float]:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9:
            return x + translation.x, y + translation.y, z + translation.z
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        tx = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z
        ty = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z
        tz = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z

        return tx + translation.x, ty + translation.y, tz + translation.z

    @staticmethod
    def _transform_arrays(x: np.ndarray, y: np.ndarray, z: np.ndarray, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9:
            return x + translation.x, y + translation.y, z + translation.z
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        tx = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z
        ty = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z
        tz = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z

        return tx + translation.x, ty + translation.y, tz + translation.z
    
    def _publish_scan(self) -> None:
        """发布融合后的 LaserScan"""
        if self.input_cloud_topic:
            if not self.cloud_received or self.cloud_ranges is None:
                return
            if not self._source_is_fresh(self.cloud_update_time):
                return
            fused_ranges = self.cloud_ranges
            stamp = self.cloud_stamp
        elif (
            not self.front_received
            or not self.rear_received
            or self.front_ranges is None
            or self.rear_ranges is None
        ):
            return
        else:
            if (
                not self._source_is_fresh(self.front_update_time)
                or not self._source_is_fresh(self.rear_update_time)
            ):
                return
            # 融合前后雷达数据（取最小值）
            fused_ranges = np.minimum(self.front_ranges, self.rear_ranges)
            stamp = self.front_stamp or self.rear_stamp
        
        # 创建 LaserScan 消息
        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp or self.get_clock().now().to_msg()
        scan_msg.header.frame_id = str(self.get_parameter("frame_id").value)
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = self.scan_period / max(self.num_readings, 1)
        scan_msg.scan_time = self.scan_period
        scan_msg.range_min = float(self.get_parameter("min_range").value)
        scan_msg.range_max = float(self.get_parameter("max_range").value)
        scan_msg.ranges = fused_ranges.tolist()
        scan_msg.intensities = []
        
        self.scan_pub.publish(scan_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
