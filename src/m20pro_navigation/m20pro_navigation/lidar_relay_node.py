import json
import math
import time

import numpy as np
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
        self.declare_parameter("cloud_reliability", "auto")
        self.declare_parameter("max_output_points", 6000)
        self.declare_parameter("min_publish_interval_s", 0.2)
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

        self.cloud_reliability = str(self.get_parameter("cloud_reliability").value).lower()
        self.publish_qos = self._cloud_qos("best_effort")
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, self.publish_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.cloud_subscriptions = []
        self.subscription_modes = []
        for mode in self._subscription_reliability_modes(self.cloud_reliability):
            self.cloud_subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    self.input_topic,
                    lambda msg, source_mode=mode: self._on_cloud(msg, source_mode),
                    self._cloud_qos(mode),
                )
            )
            self.subscription_modes.append(mode)
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
        self.last_downsample_method = "none"
        self.last_stamp = None
        self.first_update = None
        self.first_publish = None
        self.last_input_publisher_count = 0
        self.duplicate_messages = 0
        self.last_cloud_key = None
        self.last_subscription_mode = ""

        self.get_logger().info(
            "LIDAR relay ready: %s -> %s reliability=%s subscriptions=%s max_output_points=%d min_publish_interval_s=%.3f"
            % (
                self.input_topic,
                self.output_topic,
                self.cloud_reliability,
                ",".join(self.subscription_modes),
                self.max_output_points,
                self.min_publish_interval_s,
            )
        )

    def _cloud_qos(self, reliability: str) -> QoSProfile:
        cloud_qos = QoSProfile(depth=10)
        cloud_qos.history = HistoryPolicy.KEEP_LAST
        cloud_qos.durability = DurabilityPolicy.VOLATILE
        if reliability == "reliable":
            cloud_qos.reliability = ReliabilityPolicy.RELIABLE
        else:
            cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        return cloud_qos

    def _subscription_reliability_modes(self, reliability: str):
        if reliability in ("auto", "both"):
            return ("best_effort", "reliable")
        if reliability == "reliable":
            return ("reliable",)
        return ("best_effort",)

    def _cloud_key(self, msg: PointCloud2):
        stamp = msg.header.stamp
        return (
            int(stamp.sec),
            int(stamp.nanosec),
            str(msg.header.frame_id),
            int(msg.width),
            int(msg.height),
            int(msg.row_step),
        )

    def _on_cloud(self, msg: PointCloud2, subscription_mode: str = "") -> None:
        cloud_key = self._cloud_key(msg)
        if cloud_key == self.last_cloud_key:
            self.duplicate_messages += 1
            return
        self.last_cloud_key = cloud_key
        self.last_subscription_mode = subscription_mode
        self.messages += 1
        now = time.time()
        if self.first_update is None:
            self.first_update = now
        self.last_update = now
        self.last_frame_id = msg.header.frame_id
        self.last_width = int(msg.width)
        self.last_height = int(msg.height)
        self.last_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

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
        if self.first_publish is None:
            self.first_publish = now
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
            self.last_downsample_method = "passthrough"
            return msg

        stride = max(1, int(math.ceil(point_count / float(self.max_output_points))))
        data = msg.data
        limit = min(len(data), point_count * point_step)
        sampled = self._sample_data_bytes(data, point_step, stride, limit)

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

    def _sample_data_bytes(self, data: bytes, point_step: int, stride: int, limit: int) -> bytes:
        if limit <= 0:
            self.last_downsample_method = "empty"
            return b""
        bounded = memoryview(data)[:limit]
        try:
            rows = np.frombuffer(bounded, dtype=np.uint8).reshape((-1, point_step))
            self.last_downsample_method = "numpy_stride"
            return rows[::stride].copy().tobytes()
        except Exception as exc:
            self.last_downsample_method = "python_loop"
            now = time.monotonic()
            if now - self.last_log_time >= self.log_period_s:
                self.get_logger().warning(
                    "LIDAR relay numpy downsample fallback: %s" % exc
                )
            sampled = bytearray()
            for offset in range(0, limit, point_step * stride):
                sampled.extend(data[offset : offset + point_step])
            return bytes(sampled)

    def _publish_status(self) -> None:
        msg = String()
        now = time.time()
        age = None if self.last_update is None else max(0.0, now - self.last_update)
        input_elapsed = max(0.0, now - self.first_update) if self.first_update is not None else 0.0
        publish_elapsed = max(0.0, now - self.first_publish) if self.first_publish is not None else 0.0
        input_rate_hz = None
        publish_rate_hz = None
        if input_elapsed > 0.0:
            input_rate_hz = self.messages / input_elapsed
        if publish_elapsed > 0.0:
            publish_rate_hz = self.messages_published / publish_elapsed
        skip_ratio = 0.0
        if self.messages > 0:
            skip_ratio = self.messages_skipped_interval / float(self.messages)
        try:
            self.last_input_publisher_count = self.count_publishers(self.input_topic)
        except Exception:
            self.last_input_publisher_count = -1
        msg.data = json.dumps(
            {
                "input_topic": self.input_topic,
                "output_topic": self.output_topic,
                "cloud_reliability": self.cloud_reliability,
                "subscription_modes": self.subscription_modes,
                "last_subscription_mode": self.last_subscription_mode,
                "input_publisher_count": self.last_input_publisher_count,
                "messages": self.messages,
                "messages_published": self.messages_published,
                "messages_skipped_interval": self.messages_skipped_interval,
                "duplicate_messages": self.duplicate_messages,
                "last_update": self.last_update,
                "age_sec": age,
                "frame_id": self.last_frame_id,
                "width": self.last_width,
                "height": self.last_height,
                "output_width": self.last_output_width,
                "output_height": self.last_output_height,
                "output_stride": self.last_output_stride,
                "downsample_method": self.last_downsample_method,
                "input_rate_hz": input_rate_hz,
                "publish_rate_hz": publish_rate_hz,
                "skip_ratio": skip_ratio,
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
