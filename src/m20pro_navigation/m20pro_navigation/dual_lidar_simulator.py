import io
import math
import os
import struct
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class DualLidarSimulator(Node):
    """Publish local navigation point clouds by cropping a real PCD map.

    Nav2 still uses the 2D PGM/YAML map for global planning. This node uses the
    matching 3D PCD map to make the simulated local perception closer to the
    robot's factory mapping result, while keeping the existing /cloud_nav API.
    """

    def __init__(self):
        super().__init__("m20pro_dual_lidar_simulator")

        self.declare_parameter("robot_pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("dynamic_obstacle_topic", "/dynamic_obstacles")
        self.declare_parameter("current_floor_topic", "/m20pro/current_floor")
        self.declare_parameter("default_floor", "F20")
        self.declare_parameter("map_topic", "/map")

        self.declare_parameter("front_lidar_topic", "/LIDAR/FRONT/POINTS")
        self.declare_parameter("rear_lidar_topic", "/LIDAR/REAR/POINTS")
        self.declare_parameter("cloud_nav_topic", "/cloud_nav")
        self.declare_parameter("grid_map_3d_topic", "/grid_map_3d")
        self.declare_parameter("publish_debug_lidars", False)
        self.declare_parameter("publish_grid_map_3d", True)

        self.declare_parameter("pcd_map_path", "")
        self.declare_parameter("pcd_voxel_size", 0.08)
        self.declare_parameter("pcd_index_cell_size", 1.0)
        self.declare_parameter("floor_z_ranges", ["F20:-1.0:1.5:0.0"])
        self.declare_parameter("default_z_min", -1.0)
        self.declare_parameter("default_z_max", 1.5)
        self.declare_parameter("default_z_offset", 0.0)
        self.declare_parameter("use_occupancy_filter", True)
        self.declare_parameter("occupancy_filter_threshold", 65)
        self.declare_parameter("occupancy_filter_keep_unknown", False)
        self.declare_parameter("occupancy_filter_padding_m", 0.0)

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("min_range", 0.2)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("local_z_min", -0.35)
        self.declare_parameter("local_z_max", 1.20)
        self.declare_parameter("max_points_per_cloud", 6000)

        self.declare_parameter("enable_range_noise", True)
        self.declare_parameter("range_noise_sigma_base", 0.01)
        self.declare_parameter("range_noise_sigma_per_meter", 0.004)
        self.declare_parameter("range_bias_factor", 0.0)
        self.declare_parameter("enable_dropout", True)
        self.declare_parameter("dropout_distance_scale", 30.0)

        self.declare_parameter("dynamic_obstacle_radius", 0.2)
        self.declare_parameter("dynamic_obstacle_point_count", 48)

        self.pose_msg: Optional[PoseStamped] = None
        self.dynamic_obstacles: List[Tuple[float, float]] = []
        self.current_floor = str(self.get_parameter("default_floor").value).strip()
        self.floor_z_ranges = self._parse_floor_z_ranges(
            list(self.get_parameter("floor_z_ranges").value)
        )
        self.map_msg: Optional[OccupancyGrid] = None
        self.occupancy_mask: Optional[np.ndarray] = None
        self.warned_missing_map = False

        self.rng = np.random.default_rng(seed=42)
        self.map_points = np.empty((0, 3), dtype=np.float32)
        self.sorted_indices = np.empty(0, dtype=np.int64)
        self.cell_slices: Dict[int, Tuple[int, int]] = {}
        self.index_ix_min = 0
        self.index_iy_min = 0
        self.index_ny = 1

        self._load_and_index_pcd()

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
        self.create_subscription(
            String,
            str(self.get_parameter("current_floor_topic").value),
            self._on_current_floor,
            10,
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )

        cloud_qos = QoSProfile(depth=1)
        self.front_cloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("front_lidar_topic").value),
            cloud_qos,
        )
        self.rear_cloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("rear_lidar_topic").value),
            cloud_qos,
        )
        self.cloud_nav_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("cloud_nav_topic").value),
            cloud_qos,
        )
        self.grid_map_3d_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("grid_map_3d_topic").value),
            cloud_qos,
        )

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "PCD local perception simulator ready: %d indexed points -> %s"
            % (
                len(self.map_points),
                str(self.get_parameter("cloud_nav_topic").value),
            )
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose_msg = msg

    def _on_dynamic_obstacles(self, msg: PoseArray) -> None:
        self.dynamic_obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def _on_current_floor(self, msg: String) -> None:
        floor = msg.data.strip()
        if floor:
            self.current_floor = floor

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.occupancy_mask = self._build_occupancy_mask(msg)
        occupied_count = int(np.count_nonzero(self.occupancy_mask))
        self.get_logger().info(
            "occupancy filter map received: %dx%d occupied_or_kept=%d"
            % (msg.info.width, msg.info.height, occupied_count)
        )

    def _tick(self) -> None:
        if self.pose_msg is None or self.map_points.size == 0:
            return

        points = self._local_cloud_from_pcd()
        dynamic_points = self._dynamic_obstacle_points()
        if dynamic_points.size:
            points = np.vstack((points, dynamic_points)) if points.size else dynamic_points

        points = self._apply_sensor_effects(points)
        cloud = self._points_to_cloud(points)
        self.cloud_nav_pub.publish(cloud)
        if bool(self.get_parameter("publish_grid_map_3d").value):
            self.grid_map_3d_pub.publish(cloud)

        if bool(self.get_parameter("publish_debug_lidars").value):
            front = points[points[:, 0] >= 0.0] if points.size else points
            rear = points[points[:, 0] < 0.0] if points.size else points
            self.front_cloud_pub.publish(self._points_to_cloud(front))
            self.rear_cloud_pub.publish(self._points_to_cloud(rear))

    def _local_cloud_from_pcd(self) -> np.ndarray:
        assert self.pose_msg is not None
        base_x = float(self.pose_msg.pose.position.x)
        base_y = float(self.pose_msg.pose.position.y)
        base_yaw = self._yaw_from_pose(self.pose_msg)
        max_range = float(self.get_parameter("max_range").value)
        min_range = float(self.get_parameter("min_range").value)

        candidates = self._query_pcd_xy(base_x, base_y, max_range)
        if candidates.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        candidates = self._filter_static_points_by_occupancy(candidates)
        if candidates.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        z_min, z_max, z_offset = self._active_z_range()
        z_mask = (candidates[:, 2] >= z_min) & (candidates[:, 2] <= z_max)
        candidates = candidates[z_mask]
        if candidates.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        dx = candidates[:, 0] - base_x
        dy = candidates[:, 1] - base_y
        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        local_z = candidates[:, 2] - z_offset

        dist = np.hypot(local_x, local_y)
        local_z_min = float(self.get_parameter("local_z_min").value)
        local_z_max = float(self.get_parameter("local_z_max").value)
        mask = (
            (dist >= min_range)
            & (dist <= max_range)
            & (local_z >= local_z_min)
            & (local_z <= local_z_max)
        )
        if not np.any(mask):
            return np.empty((0, 3), dtype=np.float32)

        points = np.column_stack((local_x[mask], local_y[mask], local_z[mask])).astype(
            np.float32,
            copy=False,
        )
        max_points = int(self.get_parameter("max_points_per_cloud").value)
        if max_points > 0 and len(points) > max_points:
            indices = self.rng.choice(len(points), size=max_points, replace=False)
            points = points[indices]
        return points

    def _filter_static_points_by_occupancy(self, points: np.ndarray) -> np.ndarray:
        if not bool(self.get_parameter("use_occupancy_filter").value):
            return points
        if self.map_msg is None or self.occupancy_mask is None:
            if not self.warned_missing_map:
                self.warned_missing_map = True
                self.get_logger().warning(
                    "waiting for /map before publishing static PCD obstacle points"
                )
            return np.empty((0, 3), dtype=np.float32)

        origin = self.map_msg.info.origin.position
        resolution = float(self.map_msg.info.resolution)
        if resolution <= 0.0:
            return points

        width = int(self.map_msg.info.width)
        height = int(self.map_msg.info.height)
        ix = np.floor((points[:, 0] - origin.x) / resolution).astype(np.int64)
        iy = np.floor((points[:, 1] - origin.y) / resolution).astype(np.int64)
        in_bounds = (ix >= 0) & (iy >= 0) & (ix < width) & (iy < height)
        if not np.any(in_bounds):
            return np.empty((0, 3), dtype=np.float32)

        keep = np.zeros(len(points), dtype=bool)
        bounded_indices = np.nonzero(in_bounds)[0]
        keep[bounded_indices] = self.occupancy_mask[iy[in_bounds], ix[in_bounds]]
        return points[keep]

    def _build_occupancy_mask(self, msg: OccupancyGrid) -> np.ndarray:
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        threshold = int(self.get_parameter("occupancy_filter_threshold").value)
        mask = data >= threshold
        if bool(self.get_parameter("occupancy_filter_keep_unknown").value):
            mask = mask | (data < 0)

        padding_m = max(0.0, float(self.get_parameter("occupancy_filter_padding_m").value))
        resolution = float(msg.info.resolution)
        padding_cells = int(math.ceil(padding_m / resolution)) if resolution > 0.0 else 0
        if padding_cells <= 0:
            return mask

        padded = np.array(mask, copy=True)
        for dy in range(-padding_cells, padding_cells + 1):
            for dx in range(-padding_cells, padding_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                if math.hypot(dx, dy) > padding_cells:
                    continue
                src_y0 = max(0, -dy)
                src_y1 = min(height, height - dy)
                src_x0 = max(0, -dx)
                src_x1 = min(width, width - dx)
                dst_y0 = src_y0 + dy
                dst_y1 = src_y1 + dy
                dst_x0 = src_x0 + dx
                dst_x1 = src_x1 + dx
                padded[dst_y0:dst_y1, dst_x0:dst_x1] |= mask[src_y0:src_y1, src_x0:src_x1]
        return padded

    def _query_pcd_xy(self, x: float, y: float, radius: float) -> np.ndarray:
        if not self.cell_slices:
            return self.map_points

        cell_size = float(self.get_parameter("pcd_index_cell_size").value)
        ix0 = int(math.floor((x - radius) / cell_size))
        ix1 = int(math.floor((x + radius) / cell_size))
        iy0 = int(math.floor((y - radius) / cell_size))
        iy1 = int(math.floor((y + radius) / cell_size))
        chunks = []
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                key = self._cell_key(ix, iy)
                bounds = self.cell_slices.get(key)
                if bounds is None:
                    continue
                start, end = bounds
                chunks.append(self.sorted_indices[start:end])
        if not chunks:
            return np.empty((0, 3), dtype=np.float32)
        indices = np.concatenate(chunks)
        points = self.map_points[indices]
        dist2 = (points[:, 0] - x) ** 2 + (points[:, 1] - y) ** 2
        return points[dist2 <= radius * radius]

    def _dynamic_obstacle_points(self) -> np.ndarray:
        if self.pose_msg is None or not self.dynamic_obstacles:
            return np.empty((0, 3), dtype=np.float32)

        base_x = float(self.pose_msg.pose.position.x)
        base_y = float(self.pose_msg.pose.position.y)
        base_yaw = self._yaw_from_pose(self.pose_msg)
        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)
        radius = float(self.get_parameter("dynamic_obstacle_radius").value)
        max_range = float(self.get_parameter("max_range").value)
        count = max(8, int(self.get_parameter("dynamic_obstacle_point_count").value))
        angles = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
        heights = np.array([0.08, 0.30, 0.52], dtype=np.float32)
        points = []
        for ox, oy in self.dynamic_obstacles:
            dx = ox - base_x
            dy = oy - base_y
            center_x = cos_yaw * dx + sin_yaw * dy
            center_y = -sin_yaw * dx + cos_yaw * dy
            if math.hypot(center_x, center_y) > max_range + radius:
                continue
            for height in heights:
                for angle in angles:
                    points.append(
                        (
                            center_x + radius * math.cos(float(angle)),
                            center_y + radius * math.sin(float(angle)),
                            float(height),
                        )
                    )
        if not points:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def _apply_sensor_effects(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points.astype(np.float32, copy=False)

        result = points.astype(np.float32, copy=True)
        dist = np.hypot(result[:, 0], result[:, 1])

        if bool(self.get_parameter("enable_dropout").value):
            scale = max(1e-6, float(self.get_parameter("dropout_distance_scale").value))
            probability = 0.98 * np.exp(-dist / scale)
            keep = self.rng.random(len(result)) < probability
            result = result[keep]
            dist = dist[keep]
            if result.size == 0:
                return result.reshape((0, 3)).astype(np.float32, copy=False)

        if bool(self.get_parameter("enable_range_noise").value):
            sigma = float(self.get_parameter("range_noise_sigma_base").value) + (
                float(self.get_parameter("range_noise_sigma_per_meter").value) * dist
            )
            noisy_dist = dist + self.rng.normal(0.0, sigma).astype(np.float32)
            noisy_dist += float(self.get_parameter("range_bias_factor").value) * dist
            safe_dist = np.maximum(dist, 1e-6)
            scale = np.clip(noisy_dist / safe_dist, 0.0, 2.0)
            result[:, 0] *= scale
            result[:, 1] *= scale

        return result.astype(np.float32, copy=False)

    def _active_z_range(self) -> Tuple[float, float, float]:
        floor = self.current_floor
        if floor in self.floor_z_ranges:
            return self.floor_z_ranges[floor]
        return (
            float(self.get_parameter("default_z_min").value),
            float(self.get_parameter("default_z_max").value),
            float(self.get_parameter("default_z_offset").value),
        )

    def _parse_floor_z_ranges(self, values: List[str]) -> Dict[str, Tuple[float, float, float]]:
        ranges: Dict[str, Tuple[float, float, float]] = {}
        for value in values:
            parts = [part.strip() for part in str(value).split(":")]
            if len(parts) not in (3, 4):
                self.get_logger().warning("ignored invalid floor_z_ranges entry: %s" % value)
                continue
            floor = parts[0]
            try:
                z_min = float(parts[1])
                z_max = float(parts[2])
                z_offset = float(parts[3]) if len(parts) == 4 else z_min
            except ValueError:
                self.get_logger().warning("ignored invalid floor_z_ranges entry: %s" % value)
                continue
            if floor:
                ranges[floor] = (z_min, z_max, z_offset)
        return ranges

    def _load_and_index_pcd(self) -> None:
        path = self._resolve_path(str(self.get_parameter("pcd_map_path").value))
        if not path:
            self.get_logger().error("pcd_map_path is empty; no simulated local cloud will be published")
            return
        if not os.path.exists(path):
            self.get_logger().error("PCD map does not exist: %s" % path)
            return

        points = self._load_pcd_xyz(path)
        if points.size == 0:
            self.get_logger().error("PCD map has no usable XYZ points: %s" % path)
            return

        finite = np.isfinite(points).all(axis=1)
        points = points[finite].astype(np.float32, copy=False)

        voxel_size = float(self.get_parameter("pcd_voxel_size").value)
        if voxel_size > 0.0 and len(points) > 0:
            points = self._voxel_downsample(points, voxel_size)

        self.map_points = points.astype(np.float32, copy=False)
        self._build_xy_index()
        mins = self.map_points.min(axis=0)
        maxs = self.map_points.max(axis=0)
        self.get_logger().info(
            "loaded PCD map %s points=%d bounds x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]"
            % (path, len(self.map_points), mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2])
        )

    def _resolve_path(self, value: str) -> str:
        path = os.path.expandvars(os.path.expanduser(value.strip()))
        if not path:
            return ""
        if path.startswith("package://"):
            package_and_path = path[len("package://") :]
            package_name, _, relative_path = package_and_path.partition("/")
            if package_name and relative_path:
                return os.path.join(get_package_share_directory(package_name), relative_path)
            return path
        if os.path.isabs(path):
            return path
        candidates = self._workspace_candidates(path)
        candidates.extend(
            [
                Path.cwd() / path,
                Path.cwd().parent / path,
                Path.home() / path,
            ]
        )
        for candidate in candidates:
            if candidate.exists():
                return str(candidate.resolve())
        return str((Path.cwd() / path).resolve())

    @staticmethod
    def _workspace_candidates(relative_path: str) -> List[Path]:
        candidates: List[Path] = []
        for package_name in ("m20pro_bringup", "m20pro_navigation"):
            try:
                share = Path(get_package_share_directory(package_name))
            except Exception:
                continue
            parts = share.parts
            if "install" in parts:
                install_index = parts.index("install")
                if install_index > 0:
                    candidates.append(Path(*parts[:install_index]) / relative_path)
        return candidates

    def _load_pcd_xyz(self, path: str) -> np.ndarray:
        with open(path, "rb") as file:
            header_lines: List[str] = []
            while True:
                line = file.readline()
                if not line:
                    raise RuntimeError("PCD header is missing DATA line: %s" % path)
                decoded = line.decode("utf-8", errors="ignore").strip()
                header_lines.append(decoded)
                if decoded.startswith("DATA"):
                    break
            payload = file.read()

        header = self._parse_pcd_header(header_lines)
        fields = header.get("fields", [])
        sizes = header.get("sizes", [])
        types = header.get("types", [])
        counts = header.get("counts", [1] * len(fields))
        points = int(header.get("points", 0))
        data_mode = str(header.get("data", "")).lower()
        if points <= 0:
            return np.empty((0, 3), dtype=np.float32)

        offsets: Dict[str, int] = {}
        offset = 0
        for field, size, count in zip(fields, sizes, counts):
            offsets[field] = offset
            offset += int(size) * int(count)
        point_step = offset
        for field in ("x", "y", "z"):
            if field not in offsets:
                raise RuntimeError("PCD is missing %s field: %s" % (field, path))

        if data_mode == "binary":
            return self._load_binary_pcd_xyz(payload, points, point_step, offsets, fields, sizes, types)
        if data_mode == "ascii":
            array = np.loadtxt(io.BytesIO(payload), dtype=np.float32)
            if array.ndim == 1:
                array = array.reshape((1, -1))
            columns = [fields.index(field) for field in ("x", "y", "z")]
            return array[:, columns].astype(np.float32, copy=False)
        raise RuntimeError("unsupported PCD DATA mode %s in %s" % (data_mode, path))

    @staticmethod
    def _parse_pcd_header(lines: List[str]) -> Dict[str, object]:
        header: Dict[str, object] = {}
        for line in lines:
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            key = parts[0].upper()
            values = parts[1:]
            if key == "FIELDS":
                header["fields"] = values
            elif key == "SIZE":
                header["sizes"] = [int(value) for value in values]
            elif key == "TYPE":
                header["types"] = values
            elif key == "COUNT":
                header["counts"] = [int(value) for value in values]
            elif key == "POINTS":
                header["points"] = int(values[0])
            elif key == "DATA":
                header["data"] = values[0]
        if "counts" not in header and "fields" in header:
            header["counts"] = [1] * len(header["fields"])
        return header

    @staticmethod
    def _load_binary_pcd_xyz(
        payload: bytes,
        points: int,
        point_step: int,
        offsets: Dict[str, int],
        fields: List[str],
        sizes: List[int],
        types: List[str],
    ) -> np.ndarray:
        field_info = {field: (sizes[idx], types[idx]) for idx, field in enumerate(fields)}
        aligned_float32 = (
            point_step % 4 == 0
            and all(offsets[field] % 4 == 0 for field in ("x", "y", "z"))
            and all(field_info[field] == (4, "F") for field in ("x", "y", "z"))
        )
        expected_bytes = points * point_step
        payload = payload[:expected_bytes]
        if aligned_float32:
            values = np.frombuffer(payload, dtype="<f4")
            columns_per_point = point_step // 4
            rows = values.reshape((-1, columns_per_point))
            columns = [offsets[field] // 4 for field in ("x", "y", "z")]
            return rows[:, columns].astype(np.float32, copy=True)

        xyz = np.empty((points, 3), dtype=np.float32)
        for row in range(points):
            base = row * point_step
            xyz[row, 0] = struct.unpack_from("<f", payload, base + offsets["x"])[0]
            xyz[row, 1] = struct.unpack_from("<f", payload, base + offsets["y"])[0]
            xyz[row, 2] = struct.unpack_from("<f", payload, base + offsets["z"])[0]
        return xyz

    @staticmethod
    def _voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
        keys = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(unique_indices)]

    def _build_xy_index(self) -> None:
        if self.map_points.size == 0:
            return
        cell_size = max(0.2, float(self.get_parameter("pcd_index_cell_size").value))
        ix = np.floor(self.map_points[:, 0] / cell_size).astype(np.int64)
        iy = np.floor(self.map_points[:, 1] / cell_size).astype(np.int64)
        self.index_ix_min = int(ix.min())
        self.index_iy_min = int(iy.min())
        ix_max = int(ix.max())
        iy_max = int(iy.max())
        self.index_ny = max(1, iy_max - self.index_iy_min + 1)
        keys = (ix - self.index_ix_min) * self.index_ny + (iy - self.index_iy_min)
        order = np.argsort(keys, kind="mergesort")
        sorted_keys = keys[order]
        unique_keys, starts = np.unique(sorted_keys, return_index=True)
        ends = np.concatenate((starts[1:], np.array([len(order)], dtype=starts.dtype)))
        self.sorted_indices = order
        self.cell_slices = {
            int(key): (int(start), int(end))
            for key, start, end in zip(unique_keys, starts, ends)
        }

    def _cell_key(self, ix: int, iy: int) -> int:
        return (ix - self.index_ix_min) * self.index_ny + (iy - self.index_iy_min)

    @staticmethod
    def _yaw_from_pose(pose_msg: PoseStamped) -> float:
        q = pose_msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _points_to_cloud(self, points: np.ndarray) -> PointCloud2:
        points = points.reshape((-1, 3)).astype(np.float32, copy=False)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.height = 1
        msg.width = int(len(points))
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualLidarSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
