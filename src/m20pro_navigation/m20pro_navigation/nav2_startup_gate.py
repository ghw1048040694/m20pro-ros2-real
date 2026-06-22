import math
import time
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class Nav2StartupGate(Node):
    """Start Nav2 lifecycle after real robot prerequisites are present."""

    def __init__(self) -> None:
        super().__init__("m20pro_nav2_startup_gate")
        self.declare_parameter("enabled", True)
        self.declare_parameter("localization_topic", "/m20pro_tcp_bridge/localization_ok")
        self.declare_parameter("pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter(
            "lifecycle_manager_service",
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self.declare_parameter(
            "lifecycle_nodes",
            [
                "/controller_server",
                "/planner_server",
                "/bt_navigator",
                "/waypoint_follower",
            ],
        )
        self.declare_parameter("tf_global_frame", "map")
        self.declare_parameter("tf_base_frame", "m20pro_base_link")
        self.declare_parameter("required_fresh_age_s", 3.0)
        self.declare_parameter("startup_retry_s", 10.0)
        self.declare_parameter("startup_timeout_s", 20.0)
        self.declare_parameter("timer_period_s", 1.0)

        self.enabled = self._as_bool(self.get_parameter("enabled").value)
        self.required_fresh_age_s = max(
            0.5,
            float(self.get_parameter("required_fresh_age_s").value),
        )
        self.startup_retry_s = max(2.0, float(self.get_parameter("startup_retry_s").value))
        self.startup_timeout_s = max(5.0, float(self.get_parameter("startup_timeout_s").value))
        self.lifecycle_nodes = self._string_list(self.get_parameter("lifecycle_nodes").value)
        self.lifecycle_states: Dict[str, str] = {}
        self.pending_state_futures: Dict[str, Any] = {}

        self.localization_ok: Optional[bool] = None
        self.localization_time: Optional[float] = None
        self.pose_time: Optional[float] = None
        self.pose_valid = False
        self.scan_time: Optional[float] = None
        self.scan_finite = 0
        self.map_seen = False

        self.startup_future = None
        self.startup_started_at: Optional[float] = None
        self.next_attempt_at = 0.0
        self.last_wait_reason = ""
        self.reported_active = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.manager_client = self.create_client(
            ManageLifecycleNodes,
            str(self.get_parameter("lifecycle_manager_service").value),
        )
        self.state_clients = {
            name: self.create_client(GetState, "%s/get_state" % name)
            for name in self.lifecycle_nodes
            if name
        }

        self.create_subscription(
            Bool,
            str(self.get_parameter("localization_topic").value),
            self._on_localization,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            10,
        )
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        scan_qos.durability = DurabilityPolicy.VOLATILE
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            scan_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )

        period = max(0.5, float(self.get_parameter("timer_period_s").value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "Nav2 startup gate enabled=%s nodes=%s"
            % (self.enabled, ",".join(self.lifecycle_nodes))
        )

    def _on_localization(self, msg: Bool) -> None:
        self.localization_ok = bool(msg.data)
        self.localization_time = time.time()

    def _on_pose(self, msg: PoseStamped) -> None:
        position = msg.pose.position
        orientation = msg.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.pose_valid = all(math.isfinite(float(value)) for value in values)
        self.pose_time = time.time()

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan_time = time.time()
        self.scan_finite = sum(
            1
            for value in msg.ranges
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max
        )

    def _on_map(self, _msg: OccupancyGrid) -> None:
        self.map_seen = True

    def _tick(self) -> None:
        if not self.enabled:
            return
        self._query_lifecycle_states()
        if self._all_nav2_active():
            if not self.reported_active:
                self.reported_active = True
                self.get_logger().info("Nav2 lifecycle is active")
            return
        self.reported_active = False

        now = time.time()
        if self.startup_future is not None:
            if self.startup_future.done():
                try:
                    response = self.startup_future.result()
                    if response and response.success:
                        self.get_logger().info("Nav2 lifecycle startup request accepted")
                    else:
                        self.get_logger().warning("Nav2 lifecycle startup request returned failure")
                except Exception as exc:
                    self.get_logger().warning("Nav2 lifecycle startup request failed: %s" % exc)
                self.startup_future = None
                self.startup_started_at = None
                self.next_attempt_at = now + self.startup_retry_s
            elif self.startup_started_at and now - self.startup_started_at > self.startup_timeout_s:
                self.get_logger().warning(
                    "Nav2 lifecycle startup request timed out after %.1fs"
                    % self.startup_timeout_s
                )
                self.startup_future = None
                self.startup_started_at = None
                self.next_attempt_at = now + self.startup_retry_s
            return

        ready, reason = self._prerequisites_ready()
        if not ready:
            if reason != self.last_wait_reason:
                self.last_wait_reason = reason
                self.get_logger().info("Nav2 startup gate waiting: %s" % reason)
            return

        if now < self.next_attempt_at:
            return
        if not self.manager_client.service_is_ready():
            self.manager_client.wait_for_service(timeout_sec=0.1)
        if not self.manager_client.service_is_ready():
            self.get_logger().warning("Nav2 lifecycle manager service is not ready")
            self.next_attempt_at = now + self.startup_retry_s
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.get_logger().info("Nav2 prerequisites ready; requesting lifecycle startup")
        self.startup_future = self.manager_client.call_async(request)
        self.startup_started_at = now

    def _query_lifecycle_states(self) -> None:
        for node_name, future in list(self.pending_state_futures.items()):
            if not future.done():
                continue
            try:
                response = future.result()
                self.lifecycle_states[node_name] = str(response.current_state.label)
            except Exception as exc:
                self.lifecycle_states[node_name] = "error:%s" % exc
            self.pending_state_futures.pop(node_name, None)

        for node_name, client in self.state_clients.items():
            if node_name in self.pending_state_futures:
                continue
            if not client.service_is_ready():
                continue
            self.pending_state_futures[node_name] = client.call_async(GetState.Request())

    def _all_nav2_active(self) -> bool:
        if not self.lifecycle_nodes:
            return False
        return all(self.lifecycle_states.get(name) == "active" for name in self.lifecycle_nodes)

    def _prerequisites_ready(self):
        now = time.time()
        if not self.map_seen:
            return False, "waiting /map"
        if not self._fresh(self.scan_time, now) or self.scan_finite <= 0:
            return False, "waiting fresh /scan"
        if self.localization_ok is not True or not self._fresh(self.localization_time, now):
            return False, "waiting localization_ok=true"
        if not self.pose_valid or not self._fresh(self.pose_time, now):
            return False, "waiting valid map_pose"

        global_frame = str(self.get_parameter("tf_global_frame").value).strip() or "map"
        base_frame = str(self.get_parameter("tf_base_frame").value).strip() or "m20pro_base_link"
        try:
            self.tf_buffer.lookup_transform(global_frame, base_frame, Time())
        except Exception as exc:
            return False, "waiting TF %s->%s: %s" % (global_frame, base_frame, exc)
        return True, "ready"

    def _fresh(self, stamp: Optional[float], now: float) -> bool:
        return stamp is not None and 0.0 <= now - stamp <= self.required_fresh_age_s

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2StartupGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
