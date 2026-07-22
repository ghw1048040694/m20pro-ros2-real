#!/usr/bin/env python3
"""Keep a ROS-native scan publisher alive so late rosbag readers receive data."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRecordingRelay(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_scan_recording_relay")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/m20pro/recording_scan")
        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(LaserScan, output_topic, output_qos)
        input_topic = str(self.get_parameter("input_topic").value)
        self._subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.get_logger().info("recording scan relay ready: %s -> %s" % (input_topic, output_topic))

    def _on_scan(self, msg: LaserScan) -> None:
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRecordingRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
