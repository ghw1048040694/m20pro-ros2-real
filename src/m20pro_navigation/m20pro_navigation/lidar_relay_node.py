import json
import math
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
        self.declare_parameter("max_output_points", 12000)
        self.declare_parameter("min_publish_interval_s", 0.1)
        self.declare_parameter("status_period_s", 2.0)
        self.declare_parameter("log_period_s", 5.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.max_output_points = max(0, int(self.get_parameter("max_output_points").value))
        self.min_publish_interval_s = max(
            0.0,
            float(self.get_parameter("min_publish_interval_s").value),
        )
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
        self.messages_published = 0
        self.messages_skipped_interval = 0
        self.bytes_relayed = 0
        self.bytes_published = 0
        self.last_update = None
        self.last_publish = None
        self.last_log_time = 0.0
        self.last_frame_id = ""
        self.last_width = 0
        self.last_height = 0
        self.last_output_width = 0
        self.last_output_height = 0
        self.last_output_stride = 1
        self.last_stamp = None

        self.get_logger().info(
            "LIDAR relay ready: %s -> %s max_output_points=%d min_publish_interval_s=%.3f"
            % (
                self.input_topic,
                self.output_topic,
                self.max_output_points,
                self.min_publish_interval_s,
            )
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self.messages += 1
        self.last_update = time.time()
        self.last_frame_id = msg.header.frame_id
        self.last_width = int(msg.width)
        self.last_height = int(msg.height)
        self.last_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        now = time.time()
        if (
            self.min_publish_interval_s > 0.0
            and self.last_publish is not None
            and now - self.last_publish < self.min_publish_interval_s
        ):
            self.messages_skipped_interval += 1
            return

        out_msg = self._downsample_cloud(msg)
        self.cloud_pub.publish(out_msg)
        self.messages_published += 1
        self.last_publish = now
        self.bytes_relayed += len(msg.data)
        self.bytes_published += len(out_msg.data)
        self.last_output_width = int(out_msg.width)
        self.last_output_height = int(out_msg.height)

        if now - self.last_log_time >= self.log_period_s:
            self.last_log_time = now
            self.get_logger().info(
                "LIDAR relay sample OK: frame=%s input_points=%d output_points=%d stride=%d messages=%d published=%d skipped_interval=%d"
                % (
                    self.last_frame_id or "unknown",
                    self.last_width * max(1, self.last_height),
                    self.last_output_width * max(1, self.last_output_height),
                    self.last_output_stride,
                    self.messages,
                    self.messages_published,
                    self.messages_skipped_interval,
                )
            )

    def _downsample_cloud(self, msg: PointCloud2) -> PointCloud2:
        point_step = int(msg.point_step)
        point_count = int(msg.width) * max(1, int(msg.height))
        if (
            self.max_output_points <= 0
            or point_step <= 0
            or point_count <= self.max_output_points
        ):
            self.last_output_stride = 1
            return msg

        stride = max(1, int(math.ceil(point_count / float(self.max_output_points))))
        data = msg.data
        sampled = bytearray()
        limit = min(len(data), point_count * point_step)
        for offset in range(0, limit, point_step * stride):
            sampled.extend(data[offset : offset + point_step])

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(sampled) // point_step
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.width * point_step
        out.data = bytes(sampled)
        out.is_dense = msg.is_dense
        self.last_output_stride = stride
        return out

    def _publish_status(self) -> None:
        msg = String()
        now = time.time()
        age = None if self.last_update is None else max(0.0, now - self.last_update)
        msg.data = json.dumps(
            {
                "input_topic": self.input_topic,
                "output_topic": self.output_topic,
                "messages": self.messages,
                "messages_published": self.messages_published,
                "messages_skipped_interval": self.messages_skipped_interval,
                "last_update": self.last_update,
                "age_sec": age,
                "frame_id": self.last_frame_id,
                "width": self.last_width,
                "height": self.last_height,
                "output_width": self.last_output_width,
                "output_height": self.last_output_height,
                "output_stride": self.last_output_stride,
                "max_output_points": self.max_output_points,
                "min_publish_interval_s": self.min_publish_interval_s,
                "stamp": self.last_stamp,
                "bytes_relayed": self.bytes_relayed,
                "bytes_published": self.bytes_published,
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
