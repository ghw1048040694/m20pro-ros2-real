import xml.etree.ElementTree as ET
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ZeroJointStatePublisher(Node):
    def __init__(self):
        super().__init__("zero_joint_state_publisher")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("publish_rate_hz", 20.0)

        robot_description = str(self.get_parameter("robot_description").value)
        self.joint_names = self._extract_joint_names(robot_description)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.message = JointState()
        self.message.name = list(self.joint_names)
        self.message.position = [0.0] * len(self.joint_names)
        self.message.velocity = [0.0] * len(self.joint_names)
        self.message.effort = [0.0] * len(self.joint_names)

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_joint_states)
        self.get_logger().info(
            "publishing zero joint states for %d joints" % len(self.joint_names)
        )

    def _publish_joint_states(self) -> None:
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)

    @staticmethod
    def _extract_joint_names(robot_description: str) -> List[str]:
        if not robot_description.strip():
            return []
        root = ET.fromstring(robot_description)
        names: List[str] = []
        for joint in root.findall("joint"):
            joint_type = joint.attrib.get("type", "")
            if joint_type != "fixed":
                name = joint.attrib.get("name")
                if name:
                    names.append(name)
        return names


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ZeroJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
