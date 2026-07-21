import json
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class NavigationScanSelector(Node):
    """Select the flat or stair-safe scan without changing the public /scan."""

    def __init__(self) -> None:
        super().__init__("m20pro_navigation_scan_selector")
        self.declare_parameter("normal_scan_topic", "/scan")
        self.declare_parameter("stair_scan_topic", "/m20pro/stair_obstacle_scan")
        self.declare_parameter("mode_topic", "/m20pro/stair_perception_mode")
        self.declare_parameter("output_topic", "/m20pro/navigation_scan")
        self.declare_parameter("status_topic", "/m20pro/navigation_scan_status")
        self.declare_parameter("mode_timeout_s", 0.0)
        self.declare_parameter("field_profile_name", "")
        self.declare_parameter("field_profile_hash", "")

        self.field_profile_name = str(self.get_parameter("field_profile_name").value).strip()
        self.field_profile_hash = str(self.get_parameter("field_profile_hash").value).strip()
        if (
            not self.field_profile_name
            or len(self.field_profile_hash) != 64
            or any(ch not in "0123456789abcdef" for ch in self.field_profile_hash)
            or float(self.get_parameter("mode_timeout_s").value) <= 0.0
        ):
            raise RuntimeError("navigation_scan_selector requires a validated canonical field profile")

        self.stair_active = False
        self.session_id = ""
        self.mode_profile_name = ""
        self.mode_profile_hash = ""
        self.rejected_mode_profile_signature: Optional[tuple] = None
        self.last_normal_monotonic = 0.0
        self.last_stair_monotonic = 0.0
        self.last_output_source = "none"
        self.last_mode_monotonic = 0.0

        scan_qos = QoSProfile(depth=5)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        scan_qos.durability = DurabilityPolicy.VOLATILE
        # The selector is the single navigation-scan boundary.  Keep its
        # input sensor-compatible, but publish reliably so Nav2, rosbag2 and
        # the recording mirror cannot silently negotiate different streams.
        output_qos = QoSProfile(depth=10)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.VOLATILE
        self.scan_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter("output_topic").value),
            output_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("normal_scan_topic").value),
            self._on_normal_scan,
            scan_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("stair_scan_topic").value),
            self._on_stair_scan,
            scan_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mode_topic").value),
            self._on_mode,
            10,
        )
        self.create_timer(0.1, self._expire_stair_mode)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "navigation scan selector ready in flat mode; field_profile=%s hash=%s"
            % (self.field_profile_name, self.field_profile_hash)
        )

    def _on_mode(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        active = bool(payload.get("active")) if isinstance(payload, dict) else False
        session_id = str(payload.get("session_id") or "") if isinstance(payload, dict) else ""
        profile_name = str(payload.get("profile_name") or "") if isinstance(payload, dict) else ""
        profile_hash = str(payload.get("profile_hash") or "") if isinstance(payload, dict) else ""
        profile_matches = (
            profile_name == self.field_profile_name and profile_hash == self.field_profile_hash
        )
        rejected_signature = (profile_name, profile_hash)
        if (
            active
            and not profile_matches
            and self.rejected_mode_profile_signature != rejected_signature
        ):
            self.get_logger().error(
                "rejected stair scan mode due to field profile mismatch expected=%s received=%s"
                % (self.field_profile_hash, profile_hash or "missing")
            )
            self.rejected_mode_profile_signature = rejected_signature
        elif not active or profile_matches:
            self.rejected_mode_profile_signature = None
        active = active and bool(session_id) and profile_matches
        changed = active != self.stair_active or (
            active and session_id != self.session_id
        )
        self.stair_active = active
        self.session_id = session_id if active else ""
        self.mode_profile_name = profile_name
        self.mode_profile_hash = profile_hash
        self.last_mode_monotonic = time.monotonic() if active else 0.0
        if changed:
            self.last_stair_monotonic = 0.0
            self.get_logger().info(
                "navigation scan source requested: %s session=%s"
                % ("stair_obstacle_scan" if active else "normal_scan", self.session_id or "-")
            )

    def _expire_stair_mode(self) -> None:
        if not self.stair_active:
            return
        timeout_s = max(0.5, float(self.get_parameter("mode_timeout_s").value))
        if time.monotonic() - self.last_mode_monotonic <= timeout_s:
            return
        expired_session = self.session_id
        self.stair_active = False
        self.session_id = ""
        self.last_mode_monotonic = 0.0
        self.get_logger().error(
            "stair perception mode lease expired; returning to normal scan session=%s"
            % expired_session
        )

    def _on_normal_scan(self, msg: LaserScan) -> None:
        self.last_normal_monotonic = time.monotonic()
        if self.stair_active:
            return
        self.scan_pub.publish(msg)
        self.last_output_source = "normal_scan"

    def _on_stair_scan(self, msg: LaserScan) -> None:
        self.last_stair_monotonic = time.monotonic()
        if not self.stair_active:
            return
        self.scan_pub.publish(msg)
        self.last_output_source = "stair_obstacle_scan"

    @staticmethod
    def _age(last_monotonic: float) -> Optional[float]:
        if last_monotonic <= 0.0:
            return None
        return max(0.0, time.monotonic() - last_monotonic)

    def _publish_status(self) -> None:
        payload: Dict[str, Any] = {
            "active": self.stair_active,
            "session_id": self.session_id or None,
            "field_profile_name": self.field_profile_name,
            "field_profile_hash": self.field_profile_hash,
            "mode_profile_name": self.mode_profile_name or None,
            "mode_profile_hash": self.mode_profile_hash or None,
            "source": self.last_output_source,
            "normal_scan_age_s": self._age(self.last_normal_monotonic),
            "stair_scan_age_s": self._age(self.last_stair_monotonic),
            "mode_age_s": self._age(self.last_mode_monotonic),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = NavigationScanSelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
