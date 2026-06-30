from typing import Any, List, Set

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


class SimHealthMonitor(Node):
    """Report common sim startup failures in one place."""

    def __init__(self) -> None:
        super().__init__("m20pro_sim_health_monitor")
        self.declare_parameter("startup_grace_s", 8.0)
        self.declare_parameter("check_period_s", 2.0)
        self.declare_parameter("require_dynamic_obstacles", True)
        self.require_dynamic_obstacles = self._as_bool(
            self.get_parameter("require_dynamic_obstacles").value
        )

        self.start_time = self.get_clock().now()
        self.reported_ok = False
        self.seen_topics: Set[str] = set()
        self.lifecycle_states = {}

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            String,
            "/robot_description",
            self._mark("/robot_description"),
            latched_qos,
        )
        self.create_subscription(OccupancyGrid, "/map", self._mark("/map"), latched_qos)
        self.create_subscription(LaserScan, "/scan", self._mark("/scan"), 10)
        self.create_subscription(PointCloud2, "/cloud_nav", self._mark("/cloud_nav"), 10)
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
        self.create_subscription(
            MarkerArray,
            "/dynamic_obstacle_markers",
            self._mark("/dynamic_obstacle_markers"),
            10,
        )

        self.lifecycle_clients = {
            name: self.create_client(GetState, "%s/get_state" % name)
            for name in [
                "/map_server",
                "/controller_server",
                "/planner_server",
                "/bt_navigator",
                "/waypoint_follower",
                "/velocity_smoother",
            ]
        }

        period = max(1.0, float(self.get_parameter("check_period_s").value))
        self.create_timer(period, self._check)
        self.get_logger().info("sim health monitor started")

    def _mark(self, topic_name: str):
        def callback(_msg) -> None:
            self.seen_topics.add(topic_name)

        return callback

    def _check(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        grace_s = float(self.get_parameter("startup_grace_s").value)
        required_topics = [
            "/robot_description",
            "/map",
            "/cloud_nav",
            "/scan",
            "/local_costmap/costmap",
            "/global_costmap/costmap",
        ]
        if self.require_dynamic_obstacles:
            required_topics.append("/dynamic_obstacle_markers")

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

        missing_nodes = self._missing_nodes(
            [
                "robot_state_publisher",
                "m20pro_tcp_bridge",
                "m20pro_dual_lidar_simulator",
                "m20pro_pointcloud_fusion",
                "map_server",
                "controller_server",
                "planner_server",
                "bt_navigator",
            ]
        )
        inactive = self._inactive_lifecycle_nodes()

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
                self.get_logger().warning("SIM HEALTH WAITING: %s" % " ".join(problems))
            return

        if not self.reported_ok:
            self.reported_ok = True
            self.get_logger().info(
                "SIM HEALTH OK: map, robot model, scan, cloud_nav, costmaps and Nav2 are active"
            )

    def _publisher_count(self, topic_name: str) -> int:
        return len(self.get_publishers_info_by_topic(topic_name))

    def _as_bool(self, value: Any) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _missing_nodes(self, expected: List[str]) -> List[str]:
        node_names = [name.strip("/") for name in self.get_node_names()]
        missing = []
        for expected_name in expected:
            if expected_name not in node_names:
                missing.append(expected_name)
        return missing

    def _on_state(self, future, node_name: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning("lifecycle check failed for %s: %s" % (node_name, exc))
            return
        label = response.current_state.label
        self.lifecycle_states[node_name] = label

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimHealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
