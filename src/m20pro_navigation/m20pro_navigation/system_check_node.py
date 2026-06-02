from typing import Any, Dict, List, Optional, Set

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


class SystemCheckNode(Node):
    """Compact runtime health check for sim and real bringup."""

    def __init__(self) -> None:
        super().__init__("m20pro_system_check")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("startup_grace_s", 8.0)
        self.declare_parameter("check_period_s", 2.0)
        self.declare_parameter("cloud_topic", "/cloud_nav")
        self.declare_parameter("require_dynamic_obstacles", False)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("require_costmaps", True)
        self.declare_parameter("require_nav2", True)
        self.declare_parameter("require_map", True)
        self.declare_parameter("require_robot_model", True)
        self.declare_parameter("require_nodes", True)
        self.declare_parameter("require_floor_manager", True)
        self.declare_parameter("expected_nodes", [])
        self.declare_parameter(
            "lifecycle_nodes",
            [
                "/map_server",
                "/controller_server",
                "/planner_server",
                "/bt_navigator",
                "/waypoint_follower",
                "/velocity_smoother",
            ],
        )

        self.mode = str(self.get_parameter("mode").value).strip() or "sim"
        self.start_time = self.get_clock().now()
        self.reported_ok = False
        self.seen_topics: Set[str] = set()
        self.lifecycle_states: Dict[str, str] = {}

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
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        if self._bool("require_robot_model"):
            self.create_subscription(
                String,
                "/robot_description",
                self._mark("/robot_description"),
                latched_qos,
            )
        if self._bool("require_map"):
            self.create_subscription(OccupancyGrid, "/map", self._mark("/map"), latched_qos)
        if self._bool("require_scan"):
            self.create_subscription(LaserScan, "/scan", self._mark("/scan"), 10)

        cloud_topic = str(self.get_parameter("cloud_topic").value).strip()
        if cloud_topic:
            self.create_subscription(PointCloud2, cloud_topic, self._mark(cloud_topic), 10)

        if self._bool("require_costmaps"):
            self.create_subscription(
                OccupancyGrid,
                "/local_costmap/costmap",
                self._mark("/local_costmap/costmap"),
                10,
            )
            self.create_subscription(
                OccupancyGrid,
                "/global_costmap/costmap",
                self._mark("/global_costmap/costmap"),
                10,
            )

        if self._bool("require_floor_manager"):
            self.create_subscription(String, "/m20pro/current_floor", self._mark("/m20pro/current_floor"), 10)
            self.create_subscription(String, "/m20pro/stair_status", self._mark("/m20pro/stair_status"), 10)

        if self._bool("require_dynamic_obstacles"):
            self.create_subscription(
                MarkerArray,
                "/dynamic_obstacle_markers",
                self._mark("/dynamic_obstacle_markers"),
                10,
            )

        self.create_subscription(Path, "/plan", self._mark_optional("/plan"), 10)

    def _mark(self, topic_name: str):
        def callback(_msg) -> None:
            self.seen_topics.add(topic_name)

        return callback

    def _mark_optional(self, topic_name: str):
        def callback(_msg) -> None:
            self.seen_topics.add(topic_name)

        return callback

    def _required_topics(self) -> List[str]:
        topics = []
        if self._bool("require_robot_model"):
            topics.append("/robot_description")
        if self._bool("require_map"):
            topics.append("/map")
        cloud_topic = str(self.get_parameter("cloud_topic").value).strip()
        if cloud_topic:
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
        if elapsed > grace_s and waiting_messages:
            problems.append("no_messages_yet=%s" % ",".join(waiting_messages))

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
            "robot_state_publisher",
            "m20pro_tcp_bridge",
            "m20pro_pointcloud_fusion",
            "map_server",
            "controller_server",
            "planner_server",
            "bt_navigator",
        ]
        if self.mode == "sim":
            expected.append("m20pro_dual_lidar_simulator")
        if self._bool("require_floor_manager"):
            expected.append("m20pro_floor_manager")
        if self._bool("require_dynamic_obstacles"):
            expected.append("m20pro_dynamic_obstacle_simulator")
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
                inactive.append("%s:%s" % (node_name, label or "unknown"))
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda done, lifecycle_name=node_name: self._on_state(done, lifecycle_name)
            )
        return inactive

    def _on_state(self, future, node_name: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning("lifecycle check failed for %s: %s" % (node_name, exc))
            return
        self.lifecycle_states[node_name] = response.current_state.label

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
