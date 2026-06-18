import math
import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    import sensor_msgs_py.point_cloud2 as pc2
except ModuleNotFoundError:
    try:
        import sensor_msgs.point_cloud2 as pc2
    except ModuleNotFoundError:
        pc2 = None


class PointCloudFusion(Node):
    def __init__(self):
        super().__init__("m20pro_pointcloud_fusion")
        
        # === 声明参数 ===
        self.declare_parameter("input_cloud_topic", "")
        self.declare_parameter("backup_cloud_topic", "")
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
        self.declare_parameter("diagnostic_topic", "/m20pro/pointcloud_fusion/status")
        self.declare_parameter("diagnostic_period_s", 2.0)

        self.input_cloud_topic = str(self.get_parameter("input_cloud_topic").value)
        self.backup_cloud_topic = str(self.get_parameter("backup_cloud_topic").value)
        self.front_lidar_topic = str(self.get_parameter("front_lidar_topic").value)
        self.rear_lidar_topic = str(self.get_parameter("rear_lidar_topic").value)
        self.output_scan_topic = str(self.get_parameter("output_scan_topic").value)
        self.target_frame = self._clean_frame(str(self.get_parameter("frame_id").value)) or "base_link"
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.height_min = float(self.get_parameter("height_min").value)
        self.height_max = float(self.get_parameter("height_max").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.transform_cloud = bool(self.get_parameter("transform_cloud").value)
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.transform_timeout = Duration(
            seconds=max(0.0, float(self.get_parameter("transform_timeout_s").value))
        )
        self.max_source_age_s = float(self.get_parameter("max_source_age_s").value)
        self.max_points_per_cloud = int(self.get_parameter("max_points_per_cloud").value)
        self.min_cloud_interval_s = float(self.get_parameter("min_cloud_interval_s").value)
        self.publish_on_cloud_update = bool(self.get_parameter("publish_on_cloud_update").value)
        self.diagnostic_topic = str(self.get_parameter("diagnostic_topic").value)
        self.diagnostic_period_s = max(0.5, float(self.get_parameter("diagnostic_period_s").value))
        if bool(self.get_parameter("no_return_as_inf").value):
            self.empty_range_value = float("inf")
        else:
            self.empty_range_value = self.max_range
        
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
        self.cloud_messages = 0
        self.cloud_processed = 0
        self.cloud_skipped_by_interval = 0
        self.cloud_skipped_duplicate = 0
        self.cloud_dropped = 0
        self.scan_published = 0
        self.publish_skipped_no_source = 0
        self.publish_skipped_stale = 0
        self.last_cloud_frame = ""
        self.last_cloud_source_topic = ""
        self.last_processed_cloud_key = None
        self.last_cloud_input_points = 0
        self.last_cloud_sampled_points = 0
        self.last_cloud_finite_points = 0
        self.last_cloud_kept_points = 0
        self.last_cloud_finite_bins = 0
        self.last_drop_reason = "not_started"
        self.last_scan_finite_bins = 0
        self.last_scan_min_range = None
        self.last_scan_source_age_s = None
        self.last_diag_log_time = None
        self.tf_buffer = None
        self.tf_listener = None
        if self.transform_cloud:
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
        self.empty_ranges_template = np.full(
            self.num_readings,
            self.empty_range_value,
            dtype=np.float32,
        )
        self.cloud_ranges = self.empty_ranges_template.copy()
        self.front_ranges = self.empty_ranges_template.copy()
        self.rear_ranges = self.empty_ranges_template.copy()
        
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
        # === 创建订阅者 ===
        if self.input_cloud_topic:
            self.create_subscription(
                PointCloud2,
                self.input_cloud_topic,
                lambda msg: self._on_cloud(msg, self.input_cloud_topic),
                cloud_qos
            )
            if self.backup_cloud_topic and self.backup_cloud_topic != self.input_cloud_topic:
                self.create_subscription(
                    PointCloud2,
                    self.backup_cloud_topic,
                    lambda msg: self._on_cloud(msg, self.backup_cloud_topic),
                    cloud_qos
                )
        else:
            self.create_subscription(
                PointCloud2,
                self.front_lidar_topic,
                self._on_front_lidar,
                cloud_qos
            )

            self.create_subscription(
                PointCloud2,
                self.rear_lidar_topic,
                self._on_rear_lidar,
                cloud_qos
            )
        
        # === 创建发布者 ===
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.output_scan_topic,
            scan_qos
        )
        self.diagnostic_pub = None
        if self.diagnostic_topic:
            self.diagnostic_pub = self.create_publisher(String, self.diagnostic_topic, 10)
        
        # === 创建定时器（定期发布融合后的 scan）===
        publish_rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.scan_period = 1.0 / publish_rate
        self.create_timer(self.scan_period, self._publish_scan)
        self.create_timer(self.diagnostic_period_s, self._publish_diagnostics)
        
        if self.input_cloud_topic:
            if self.backup_cloud_topic and self.backup_cloud_topic != self.input_cloud_topic:
                source_desc = "%s + backup %s" % (
                    self.input_cloud_topic,
                    self.backup_cloud_topic,
                )
            else:
                source_desc = self.input_cloud_topic
        else:
            source_desc = "%s + %s" % (
                self.front_lidar_topic,
                self.rear_lidar_topic,
            )
        self.get_logger().info(
            "PointCloud fusion ready: %s -> %s in %s" % (
                source_desc,
                self.output_scan_topic,
                self._target_frame(),
            )
        )

    def _on_cloud(self, msg: PointCloud2, source_topic: str = "") -> None:
        """处理导航主链路点云。"""
        self.cloud_messages += 1
        self.last_cloud_frame = self._clean_frame(msg.header.frame_id)
        self.last_cloud_source_topic = source_topic
        cloud_key = self._cloud_key(msg)
        if cloud_key == self.last_processed_cloud_key:
            self.cloud_skipped_duplicate += 1
            return
        if not self._should_process_cloud():
            self.cloud_skipped_by_interval += 1
            return
        ranges = self._pointcloud_to_ranges(msg)
        if ranges is None:
            self.cloud_dropped += 1
            return
        self.cloud_ranges = ranges
        self.cloud_stamp = self._output_stamp_for_cloud(msg)
        self.cloud_update_time = self.get_clock().now()
        self.cloud_received = True
        self.cloud_processed += 1
        self.last_processed_cloud_key = cloud_key
        if self.publish_on_cloud_update:
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
        xyz = self._extract_xyz_arrays(cloud_msg)
        if xyz is not None:
            return self._point_arrays_to_ranges(cloud_msg, xyz)

        # Fallback for unusual PointCloud2 layouts. This path is slower and is
        # mainly kept for compatibility with synthetic/test clouds.
        if pc2 is None:
            self.last_drop_reason = "point_cloud2_helper_unavailable"
            return None
        ranges = self.empty_ranges_template.copy()
        points = pc2.read_points(cloud_msg, field_names=("x", "y", "z"))
        transform = self._lookup_cloud_transform(cloud_msg)
        if transform is False:
            return None

        input_points = max(0, int(cloud_msg.width) * max(1, int(cloud_msg.height)))
        self.last_cloud_input_points = input_points
        sampled_points = 0
        finite_points = 0
        kept_points = 0
        for x, y, z in points:
            sampled_points += 1
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            finite_points += 1
            if transform is not None:
                x, y, z = self._transform_point(x, y, z, transform)

            # 过滤高度
            if z < self.height_min or z > self.height_max:
                continue
            
            # 计算距离和角度
            distance = math.sqrt(x * x + y * y)
            
            # 过滤机器人本体附近的点
            if distance < self.robot_radius:
                continue
            
            if distance < self.min_range or distance > self.max_range:
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
                kept_points += 1

        self.last_cloud_sampled_points = sampled_points
        self.last_cloud_finite_points = finite_points
        self.last_cloud_kept_points = kept_points
        self.last_cloud_finite_bins = self._count_finite_ranges(ranges)
        self.last_drop_reason = "ok" if self.last_cloud_finite_bins > 0 else "all_points_filtered"
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
        input_points = max(0, int(cloud_msg.width) * max(1, int(cloud_msg.height)))
        self.last_cloud_input_points = input_points or point_count
        if point_count <= 0:
            self.last_drop_reason = "empty_cloud"
            return None
        cloud = np.frombuffer(cloud_msg.data, dtype=dtype, count=point_count)

        if self.max_points_per_cloud > 0 and point_count > self.max_points_per_cloud:
            stride = max(1, point_count // self.max_points_per_cloud)
            cloud = cloud[::stride]

        x = cloud["x"].astype(np.float32, copy=False)
        y = cloud["y"].astype(np.float32, copy=False)
        z = cloud["z"].astype(np.float32, copy=False)
        self.last_cloud_sampled_points = int(len(x))
        return x, y, z

    def _point_arrays_to_ranges(self, cloud_msg: PointCloud2, xyz) -> Optional[np.ndarray]:
        x, y, z = xyz
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        self.last_cloud_finite_points = int(np.count_nonzero(valid))
        if not np.any(valid):
            self.last_cloud_kept_points = 0
            self.last_cloud_finite_bins = 0
            self.last_drop_reason = "no_finite_xyz"
            return self.empty_ranges_template.copy()

        x = x[valid]
        y = y[valid]
        z = z[valid]

        transform = self._lookup_cloud_transform(cloud_msg)
        if transform is False:
            return None
        if transform is not None:
            x, y, z = self._transform_arrays(x, y, z, transform)

        distance_sq = x * x + y * y
        min_keep_range = max(self.robot_radius, self.min_range)
        valid = (
            (z >= self.height_min)
            & (z <= self.height_max)
            & (distance_sq >= min_keep_range * min_keep_range)
            & (distance_sq <= self.max_range * self.max_range)
        )
        kept_points = int(np.count_nonzero(valid))
        self.last_cloud_kept_points = kept_points
        if not np.any(valid):
            self.last_cloud_finite_bins = 0
            self.last_drop_reason = "all_points_filtered"
            return self.empty_ranges_template.copy()

        distance = np.sqrt(distance_sq[valid]).astype(np.float32, copy=False)
        angle = np.arctan2(y[valid], x[valid])
        idx = np.floor((angle - self.angle_min) / self.angle_increment).astype(np.int64)
        valid_idx = (idx >= 0) & (idx < self.num_readings)

        ranges = self.empty_ranges_template.copy()
        if np.any(valid_idx):
            np.minimum.at(ranges, idx[valid_idx], distance[valid_idx])
        self.last_cloud_finite_bins = self._count_finite_ranges(ranges)
        self.last_drop_reason = "ok" if self.last_cloud_finite_bins > 0 else "no_angle_bins"
        return ranges

    def _lookup_cloud_transform(self, cloud_msg: PointCloud2):
        """Return None when no transform is needed, False when TF is unavailable."""
        if not self.transform_cloud:
            return None
        source_frame = self._clean_frame(cloud_msg.header.frame_id)
        target_frame = self._target_frame()
        if not source_frame or source_frame == target_frame:
            return None

        if self.use_latest_tf:
            lookup_time = Time()
        else:
            lookup_time = Time.from_msg(cloud_msg.header.stamp)
        try:
            if self.tf_buffer is None:
                return False
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time,
                timeout=self.transform_timeout,
            )
        except Exception as exc:
            self.last_drop_reason = "no_tf"
            self._warn_tf_throttled(
                "skip cloud: no TF %s -> %s (%s)" % (
                    source_frame,
                    target_frame,
                    exc,
                )
            )
            return False

    def _output_stamp_for_cloud(self, cloud_msg: PointCloud2):
        if not self.transform_cloud:
            return cloud_msg.header.stamp
        source_frame = self._clean_frame(cloud_msg.header.frame_id)
        target_frame = self._target_frame()
        if not source_frame or source_frame == target_frame:
            return cloud_msg.header.stamp
        if self.use_latest_tf:
            return self.get_clock().now().to_msg()
        return cloud_msg.header.stamp

    def _source_is_fresh(self, update_time) -> bool:
        if update_time is None:
            return False
        if self.max_source_age_s <= 0.0:
            return True
        age = (self.get_clock().now() - update_time).nanoseconds * 1e-9
        self.last_scan_source_age_s = age
        return age <= self.max_source_age_s

    def _should_process_cloud(self) -> bool:
        if self.min_cloud_interval_s <= 0.0 or self.cloud_update_time is None:
            return True
        age = (self.get_clock().now() - self.cloud_update_time).nanoseconds * 1e-9
        return age >= self.min_cloud_interval_s

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
        return self.target_frame

    def _empty_range_value(self) -> float:
        return self.empty_range_value

    def _count_finite_ranges(self, ranges: np.ndarray) -> int:
        return int(np.count_nonzero(np.isfinite(ranges)))

    def _min_finite_range(self, ranges: np.ndarray):
        finite = ranges[np.isfinite(ranges)]
        if finite.size == 0:
            return None
        return float(np.min(finite))

    def _source_age_s(self, update_time) -> Optional[float]:
        if update_time is None:
            return None
        return (self.get_clock().now() - update_time).nanoseconds * 1e-9

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        source_age = self._source_age_s(self.cloud_update_time)
        status = {
            "input": self.input_cloud_topic or "%s+%s" % (self.front_lidar_topic, self.rear_lidar_topic),
            "output": self.output_scan_topic,
            "target_frame": self.target_frame,
            "cloud_messages": self.cloud_messages,
            "cloud_processed": self.cloud_processed,
            "cloud_skipped_by_interval": self.cloud_skipped_by_interval,
            "cloud_skipped_duplicate": self.cloud_skipped_duplicate,
            "cloud_dropped": self.cloud_dropped,
            "scan_published": self.scan_published,
            "publish_skipped_no_source": self.publish_skipped_no_source,
            "publish_skipped_stale": self.publish_skipped_stale,
            "last_cloud_frame": self.last_cloud_frame,
            "last_cloud_source_topic": self.last_cloud_source_topic,
            "last_cloud_input_points": self.last_cloud_input_points,
            "last_cloud_sampled_points": self.last_cloud_sampled_points,
            "last_cloud_finite_points": self.last_cloud_finite_points,
            "last_cloud_kept_points": self.last_cloud_kept_points,
            "last_cloud_finite_bins": self.last_cloud_finite_bins,
            "last_drop_reason": self.last_drop_reason,
            "last_scan_finite_bins": self.last_scan_finite_bins,
            "last_scan_min_range": self.last_scan_min_range,
            "last_source_age_s": source_age,
            "max_source_age_s": self.max_source_age_s,
            "publish_on_cloud_update": self.publish_on_cloud_update,
        }
        text = (
            "cloud=%d processed=%d skipped_interval=%d skipped_duplicate=%d "
            "dropped=%d scan=%d finite_bins=%d source_age=%s source=%s "
            "input_points=%d sampled=%d kept=%d reason=%s"
        ) % (
            self.cloud_messages,
            self.cloud_processed,
            self.cloud_skipped_by_interval,
            self.cloud_skipped_duplicate,
            self.cloud_dropped,
            self.scan_published,
            self.last_scan_finite_bins,
            "%.3f" % source_age if source_age is not None else "none",
            self.last_cloud_source_topic or "none",
            self.last_cloud_input_points,
            self.last_cloud_sampled_points,
            self.last_cloud_kept_points,
            self.last_drop_reason,
        )
        if self.last_diag_log_time is None or (now - self.last_diag_log_time).nanoseconds * 1e-9 >= self.diagnostic_period_s:
            self.last_diag_log_time = now
            if self.scan_published == 0 or self.last_scan_finite_bins == 0:
                self.get_logger().warning("PointCloud fusion status: %s" % text)
            else:
                self.get_logger().info("PointCloud fusion status: %s" % text)
        if self.diagnostic_pub is not None:
            msg = String()
            import json

            msg.data = json.dumps(status, ensure_ascii=True, separators=(",", ":"))
            self.diagnostic_pub.publish(msg)

    @staticmethod
    def _clean_frame(frame_id: str) -> str:
        return frame_id.strip().lstrip("/")

    @staticmethod
    def _cloud_key(cloud_msg: PointCloud2):
        stamp = cloud_msg.header.stamp
        return (
            int(stamp.sec),
            int(stamp.nanosec),
            str(cloud_msg.header.frame_id),
            int(cloud_msg.width),
            int(cloud_msg.height),
            int(cloud_msg.row_step),
        )

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
                self.publish_skipped_no_source += 1
                return
            if not self._source_is_fresh(self.cloud_update_time):
                self.publish_skipped_stale += 1
                return
            fused_ranges = self.cloud_ranges
            stamp = self.cloud_stamp
        elif (
            not self.front_received
            or not self.rear_received
            or self.front_ranges is None
            or self.rear_ranges is None
        ):
            self.publish_skipped_no_source += 1
            return
        else:
            if (
                not self._source_is_fresh(self.front_update_time)
                or not self._source_is_fresh(self.rear_update_time)
            ):
                self.publish_skipped_stale += 1
                return
            # 融合前后雷达数据（取最小值）
            fused_ranges = np.minimum(self.front_ranges, self.rear_ranges)
            stamp = self.front_stamp or self.rear_stamp
        
        # 创建 LaserScan 消息
        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp or self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.target_frame
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = self.scan_period / max(self.num_readings, 1)
        scan_msg.scan_time = self.scan_period
        scan_msg.range_min = self.min_range
        scan_msg.range_max = self.max_range
        scan_msg.ranges = fused_ranges.tolist()
        scan_msg.intensities = []
        
        self.scan_pub.publish(scan_msg)
        self.scan_published += 1
        self.last_scan_finite_bins = self._count_finite_ranges(fused_ranges)
        self.last_scan_min_range = self._min_finite_range(fused_ranges)
        self.last_drop_reason = "published"


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
