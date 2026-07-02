import math
import os
import shutil
from typing import Any, Dict, List, Optional, Set

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray


class SystemCheckNode(Node):
    """Compact runtime health check for M20Pro real bringup."""

    def __init__(self) -> None:
        super().__init__("m20pro_system_check")
        self.declare_parameter("mode", "real")
        self.declare_parameter("startup_grace_s", 8.0)
        self.declare_parameter("check_period_s", 2.0)
        self.declare_parameter("cloud_topic", "/cloud_nav")
        self.declare_parameter("require_dynamic_obstacles", False)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("require_costmaps", True)
        self.declare_parameter("require_nav2", True)
        self.declare_parameter("require_map", True)
        self.declare_parameter("require_nodes", True)
        self.declare_parameter("require_floor_manager", True)
        self.declare_parameter("require_cloud_topic", True)
        self.declare_parameter("require_topic_messages", True)
        self.declare_parameter("check_scan_content", False)
        self.declare_parameter("check_local_costmap_content", False)
        self.declare_parameter("check_tf_height", False)
        self.declare_parameter("tf_global_frame", "odom")
        self.declare_parameter("tf_base_frame", "m20pro_base_link")
        self.declare_parameter("max_abs_base_z", 1.0)
        self.declare_parameter("cloud_reliability", "best_effort")
        self.declare_parameter("min_scan_finite_bins", 20)
        self.declare_parameter("min_scan_close_bins", 1)
        self.declare_parameter("scan_close_range_m", 2.0)
        self.declare_parameter("min_local_costmap_marked_cells", 1)
        self.declare_parameter("warn_shm_usage_percent", 90.0)
        self.declare_parameter("expected_nodes", [])
        self.declare_parameter(
            "lifecycle_nodes",
            [
                "/map_server",
                "/controller_server",
                "/planner_server",
                "/bt_navigator",
                "/waypoint_follower",
            ],
        )

        self.mode = str(self.get_parameter("mode").value).strip() or "real"
        self.start_time = self.get_clock().now()
        self.reported_ok = False
        self.seen_topics: Set[str] = set()
        self.lifecycle_states: Dict[str, str] = {}
        self.lifecycle_last_error: Dict[str, str] = {}
        self.scan_stats: Optional[Dict[str, Any]] = None
        self.local_costmap_stats: Optional[Dict[str, int]] = None
        self.tf_buffer = None
        self.tf_listener = None
        if self._bool("check_tf_height"):
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self._subscribe_required_topics()
        self.lifecycle_clients = {
            name: self.create_client(GetState, "%s/get_state" % name)
            for name in self._string_list(self.get_parameter("lifecycle_nodes").value)
            if name
        }

        period = max(1.0, float(self.get_parameter("check_period_s").value))
        self.create_timer(period, self._check)
        self.get_logger().info("system check started mode=%s" % self.mode)

    def _subscribe_required_topics(self) -> None:
        require_messages = self._bool("require_topic_messages")
        need_scan_stats = self._bool("check_scan_content") or self._bool("check_local_costmap_content")
        need_local_costmap_stats = self._bool("check_local_costmap_content")

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        if require_messages and self._bool("require_map"):
            self.create_subscription(OccupancyGrid, "/map", self._mark("/map"), latched_qos)
        if self._bool("require_scan") and (require_messages or need_scan_stats):
            scan_qos = QoSProfile(depth=10)
            scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            scan_qos.durability = DurabilityPolicy.VOLATILE
            self.create_subscription(LaserScan, "/scan", self._on_scan, scan_qos)

        cloud_topic = str(self.get_parameter("cloud_topic").value).strip()
        if require_messages and cloud_topic and self._bool("require_cloud_topic"):
            cloud_qos = QoSProfile(depth=5)
            if str(self.get_parameter("cloud_reliability").value).lower() == "reliable":
                cloud_qos.reliability = ReliabilityPolicy.RELIABLE
            else:
                cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            cloud_qos.durability = DurabilityPolicy.VOLATILE
            self.create_subscription(PointCloud2, cloud_topic, self._mark(cloud_topic), cloud_qos)

        if self._bool("require_costmaps"):
            if require_messages or need_local_costmap_stats:
                self.create_subscription(
                    OccupancyGrid,
                    "/local_costmap/costmap",
                    self._on_local_costmap,
                    10,
                )
            if require_messages:
                self.create_subscription(
                    OccupancyGrid,
                    "/global_costmap/costmap",
                    self._mark("/global_costmap/costmap"),
                    10,
                )

        if require_messages and self._bool("require_floor_manager"):
            self.create_subscription(String, "/m20pro/current_floor", self._mark("/m20pro/current_floor"), 10)
            self.create_subscription(String, "/m20pro/stair_status", self._mark("/m20pro/stair_status"), 10)

        if require_messages and self._bool("require_dynamic_obstacles"):
            self.create_subscription(
                MarkerArray,
                "/dynamic_obstacle_markers",
                self._mark("/dynamic_obstacle_markers"),
                10,
            )

        if require_messages:
            self.create_subscription(Path, "/plan", self._mark_optional("/plan"), 10)

    def _mark(self, topic_name: str):
        def callback(_msg) -> None:
            self.seen_topics.add(topic_name)

        return callback

    def _mark_optional(self, topic_name: str):
        def callback(_msg) -> None:
            self.seen_topics.add(topic_name)

        return callback

    def _on_scan(self, msg: LaserScan) -> None:
        self.seen_topics.add("/scan")
        if not self._bool("check_scan_content") and not self._bool("check_local_costmap_content"):
            return
        finite = [
            value
            for value in msg.ranges
            if math.isfinite(value)
            and value >= msg.range_min
            and value <= msg.range_max
        ]
        close_range = float(self.get_parameter("scan_close_range_m").value)
        self.scan_stats = {
            "finite": len(finite),
            "close": sum(1 for value in finite if value <= close_range),
            "min": min(finite) if finite else None,
            "frame_id": msg.header.frame_id,
        }

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self.seen_topics.add("/local_costmap/costmap")
        if not self._bool("check_local_costmap_content"):
            return
        lethal = 0
        inflated = 0
        unknown = 0
        for value in msg.data:
            if value < 0:
                unknown += 1
            elif value >= 100:
                lethal += 1
            elif value > 0:
                inflated += 1
        self.local_costmap_stats = {
            "lethal": lethal,
            "inflated": inflated,
            "unknown": unknown,
        }

    def _required_topics(self) -> List[str]:
        topics = []
        if self._bool("require_map"):
            topics.append("/map")
        cloud_topic = str(self.get_parameter("cloud_topic").value).strip()
        if cloud_topic and self._bool("require_cloud_topic"):
            topics.append(cloud_topic)
        if self._bool("require_scan"):
            topics.append("/scan")
        if self._bool("require_costmaps"):
            topics.extend(["/local_costmap/costmap", "/global_costmap/costmap"])
        if self._bool("require_floor_manager"):
            topics.append("/m20pro/current_floor")
        if self._bool("require_dynamic_obstacles"):
            topics.append("/dynamic_obstacle_markers")
        return topics

    def _check(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        grace_s = float(self.get_parameter("startup_grace_s").value)
        required_topics = self._required_topics()

        missing_topics = [
            topic
            for topic in required_topics
            if topic not in self.seen_topics and self._publisher_count(topic) == 0
        ]
        waiting_messages = [
            topic
            for topic in required_topics
            if topic not in self.seen_topics and self._publisher_count(topic) > 0
        ]
        missing_nodes = self._missing_nodes(self._expected_nodes())
        inactive = self._inactive_lifecycle_nodes() if self._bool("require_nav2") else []

        problems: List[str] = []
        if missing_nodes:
            problems.append("missing_nodes=%s" % ",".join(missing_nodes))
        if missing_topics:
            problems.append("missing_topics=%s" % ",".join(missing_topics))
        if inactive:
            problems.append("inactive_lifecycle=%s" % ",".join(inactive))
        if elapsed > grace_s and waiting_messages and self._bool("require_topic_messages"):
            problems.append("no_messages_yet=%s" % ",".join(waiting_messages))
        if elapsed > grace_s:
            content_problems = self._content_problems()
            if content_problems:
                problems.extend(content_problems)
            runtime_problems = self._runtime_problems()
            if runtime_problems:
                problems.extend(runtime_problems)

        if problems:
            self.reported_ok = False
            if elapsed >= grace_s:
                self.get_logger().warning(
                    "M20PRO %s WAITING: %s" % (self.mode.upper(), " ".join(problems))
                )
            return

        if not self.reported_ok:
            self.reported_ok = True
            self.get_logger().info(
                "M20PRO %s OK: required topics, nodes, maps and Nav2 are active"
                % self.mode.upper()
            )

    def _expected_nodes(self) -> List[str]:
        if not self._bool("require_nodes"):
            return []
        configured = self._string_list(self.get_parameter("expected_nodes").value)
        if configured:
            return configured

        expected = [
            "m20pro_tcp_bridge",
            "m20pro_pointcloud_fusion",
            "map_server",
            "controller_server",
            "planner_server",
            "bt_navigator",
        ]
        if self._bool("require_floor_manager"):
            expected.append("m20pro_floor_manager")
        return expected

    def _missing_nodes(self, expected: List[str]) -> List[str]:
        node_names = [name.strip("/") for name in self.get_node_names()]
        return [expected_name for expected_name in expected if expected_name not in node_names]

    def _inactive_lifecycle_nodes(self) -> List[str]:
        inactive = []
        for node_name, client in self.lifecycle_clients.items():
            if not client.service_is_ready():
                inactive.append("%s:no_service" % node_name)
                continue
            label = self.lifecycle_states.get(node_name)
            if label != "active":
                detail = self.lifecycle_last_error.get(node_name) or label or "unknown"
                inactive.append("%s:%s" % (node_name, detail))
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda done, lifecycle_name=node_name: self._on_state(done, lifecycle_name)
            )
        return inactive

    def _content_problems(self) -> List[str]:
        problems: List[str] = []
        if self._bool("check_scan_content"):
            if not self.scan_stats:
                problems.append("scan_content=no_scan_stats")
            else:
                finite = int(self.scan_stats.get("finite", 0))
                min_finite = int(self.get_parameter("min_scan_finite_bins").value)
                if finite < min_finite:
                    problems.append(
                        "scan_content=finite:%d/%d,close:%d,min:%s,frame:%s"
                        % (
                            finite,
                            min_finite,
                            int(self.scan_stats.get("close", 0)),
                            self.scan_stats.get("min"),
                            self.scan_stats.get("frame_id"),
                        )
                    )

        if self._bool("check_local_costmap_content"):
            if not self.local_costmap_stats:
                problems.append("local_costmap_content=no_costmap_stats")
            else:
                scan_close = int((self.scan_stats or {}).get("close", 0))
                min_close = int(self.get_parameter("min_scan_close_bins").value)
                marked = int(self.local_costmap_stats.get("lethal", 0)) + int(
                    self.local_costmap_stats.get("inflated", 0)
                )
                min_marked = int(self.get_parameter("min_local_costmap_marked_cells").value)
                if scan_close >= min_close and marked < min_marked:
                    problems.append(
                        "local_costmap_content=marked:%d/%d,lethal:%d,inflated:%d"
                        % (
                            marked,
                            min_marked,
                            int(self.local_costmap_stats.get("lethal", 0)),
                            int(self.local_costmap_stats.get("inflated", 0)),
                        )
                    )

        if self._bool("check_tf_height"):
            transform_problem = self._tf_height_problem()
            if transform_problem:
                problems.append(transform_problem)
        return problems

    def _runtime_problems(self) -> List[str]:
        problems: List[str] = []
        shm_problem = self._shm_problem()
        if shm_problem:
            problems.append(shm_problem)
        return problems

    def _shm_problem(self) -> Optional[str]:
        try:
            usage = shutil.disk_usage("/dev/shm")
        except Exception:
            return None
        total = float(usage.total)
        if total <= 0.0:
            return None
        used_percent = 100.0 * float(usage.used) / total
        warn_percent = float(self.get_parameter("warn_shm_usage_percent").value)
        if used_percent < warn_percent:
            return None
        profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "unset")
        return "shm_usage=%.1f%%,profile=%s" % (used_percent, profile)

    def _tf_height_problem(self) -> Optional[str]:
        global_frame = str(self.get_parameter("tf_global_frame").value).strip() or "odom"
        base_frame = str(self.get_parameter("tf_base_frame").value).strip() or "m20pro_base_link"
        if self.tf_buffer is None:
            return "tf_height=no_tf_listener"
        try:
            transform = self.tf_buffer.lookup_transform(global_frame, base_frame, Time())
        except Exception as exc:
            return "tf_height=no_tf:%s->%s:%s" % (global_frame, base_frame, exc)
        z = float(transform.transform.translation.z)
        max_abs_z = float(self.get_parameter("max_abs_base_z").value)
        if abs(z) > max_abs_z:
            return "tf_height=z:%.3f,max_abs:%.3f" % (z, max_abs_z)
        return None

    def _on_state(self, future, node_name: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.lifecycle_last_error[node_name] = str(exc)
            self.get_logger().warning("lifecycle check failed for %s: %s" % (node_name, exc))
            return
        self.lifecycle_states[node_name] = response.current_state.label
        self.lifecycle_last_error.pop(node_name, None)

    def _publisher_count(self, topic_name: str) -> int:
        return len(self.get_publishers_info_by_topic(topic_name))

    def _bool(self, parameter_name: str) -> bool:
        return self._as_bool(self.get_parameter(parameter_name).value)

    def _as_bool(self, value: Any) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _string_list(self, value: Any) -> List[str]:
        if isinstance(value, (list, tuple)):
            return [str(item).strip() for item in value if str(item).strip()]
        if value is None:
            return []
        text = str(value).strip()
        if not text:
            return []
        return [item.strip() for item in text.split(",") if item.strip()]


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SystemCheckNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
