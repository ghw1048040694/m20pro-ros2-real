import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class LidarRelayNode(Node):
    """Minimal long-lived relay for the factory pointcloud topic."""

    def __init__(self) -> None:
        super().__init__("m20pro_lidar_relay")
        self.declare_parameter("input_topic", "/LIDAR/POINTS")
        self.declare_parameter("output_topic", "/m20pro/lidar_points_relay")
        self.declare_parameter("status_topic", "/m20pro/lidar_relay/status")
        self.declare_parameter("cloud_reliability", "reliable")
        self.declare_parameter("status_period_s", 2.0)
        self.declare_parameter("log_period_s", 5.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.status_period_s = max(0.5, float(self.get_parameter("status_period_s").value))
        self.log_period_s = max(1.0, float(self.get_parameter("log_period_s").value))

        cloud_qos = QoSProfile(depth=4)
        cloud_qos.history = HistoryPolicy.KEEP_LAST
        cloud_qos.durability = DurabilityPolicy.VOLATILE
        if str(self.get_parameter("cloud_reliability").value).lower() == "best_effort":
            cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            cloud_qos.reliability = ReliabilityPolicy.RELIABLE

        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, cloud_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(PointCloud2, self.input_topic, self._on_cloud, cloud_qos)
        self.create_timer(self.status_period_s, self._publish_status)

        self.messages = 0
        self.bytes_relayed = 0
        self.last_update = None
        self.last_log_time = 0.0
        self.last_frame_id = ""
        self.last_width = 0
        self.last_height = 0
        self.last_stamp = None

        self.get_logger().info(
            "LIDAR relay ready: %s -> %s" % (self.input_topic, self.output_topic)
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.messages += 1
        self.bytes_relayed += len(msg.data)
        self.last_update = time.time()
        self.last_frame_id = msg.header.frame_id
        self.last_width = int(msg.width)
        self.last_height = int(msg.height)
        self.last_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.cloud_pub.publish(msg)

        now = time.time()
        if now - self.last_log_time >= self.log_period_s:
            self.last_log_time = now
            self.get_logger().info(
                "LIDAR relay sample OK: frame=%s points=%d messages=%d"
                % (
                    self.last_frame_id or "unknown",
                    self.last_width * max(1, self.last_height),
                    self.messages,
                )
            )

    def _publish_status(self) -> None:
        msg = String()
        now = time.time()
        age = None if self.last_update is None else max(0.0, now - self.last_update)
        msg.data = json.dumps(
            {
                "input_topic": self.input_topic,
                "output_topic": self.output_topic,
                "messages": self.messages,
                "last_update": self.last_update,
                "age_sec": age,
                "frame_id": self.last_frame_id,
                "width": self.last_width,
                "height": self.last_height,
                "stamp": self.last_stamp,
                "bytes_relayed": self.bytes_relayed,
            },
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
