import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from .geometry import quaternion_to_yaw, yaw_to_quaternion


class M20SimBridge(Node):
    """A tiny 2D kinematic simulator that mimics the TCP bridge topics.

    It lets the planner and follower run on a laptop without the 103/106 hosts.
    """

    def __init__(self):
        super().__init__("m20pro_tcp_bridge")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)
        self.latest_cmd = Twist()
        self.last_time = self.get_clock().now()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(PoseStamped, "~/map_pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.loc_pub = self.create_publisher(Bool, "~/localization_ok", 10)
        self.obs_pub = self.create_publisher(Bool, "~/obstacle_active", 10)
        self.status_pub = self.create_publisher(String, "~/navigation_status", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.create_timer(period, self._tick)
        self.get_logger().info("M20 Pro sim bridge publishing fake map_pose/odom; no robot connection is used")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.get_logger().info("simulation pose reset to x=%.2f y=%.2f yaw=%.2f" % (self.x, self.y, self.yaw))

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        vx = self.latest_cmd.linear.x
        vy = self.latest_cmd.linear.y
        wz = self.latest_cmd.angular.z
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        self._publish_state(now)

    def _publish_state(self, stamp) -> None:
        map_frame = str(self.get_parameter("map_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        quat = yaw_to_quaternion(self.yaw)

        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = map_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation = quat
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header.stamp = pose.header.stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose = pose.pose
        odom.twist.twist = self.latest_cmd
        self.odom_pub.publish(odom)

        ok = Bool()
        ok.data = True
        self.loc_pub.publish(ok)
        obs = Bool()
        obs.data = False
        self.obs_pub.publish(obs)
        status = String()
        status.data = "simulation location=0 obstacle=0"
        self.status_pub.publish(status)

        if bool(self.get_parameter("publish_tf").value):
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = pose.header.stamp
            map_to_odom.header.frame_id = map_frame
            map_to_odom.child_frame_id = odom_frame
            map_to_odom.transform.rotation.w = 1.0

            transform = TransformStamped()
            transform.header.stamp = pose.header.stamp
            transform.header.frame_id = odom_frame
            transform.child_frame_id = base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = quat
            self.tf_broadcaster.sendTransform([map_to_odom, transform])


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = M20SimBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
