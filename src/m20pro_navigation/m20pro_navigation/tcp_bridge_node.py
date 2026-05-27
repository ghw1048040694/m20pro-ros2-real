from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from .geometry import quaternion_to_yaw, yaw_to_quaternion
from .tcp_protocol import M20TcpClient, patrol_items


class M20TcpBridge(Node):
    def __init__(self):
        super().__init__("m20pro_tcp_bridge")
        self.declare_parameter("robot_ip", "10.21.31.103")
        self.declare_parameter("tcp_port", 30001)
        self.declare_parameter("poll_rate_hz", 5.0)
        self.declare_parameter("cmd_vel_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("max_linear_x", 0.8)
        self.declare_parameter("max_linear_y", 0.5)
        self.declare_parameter("max_angular_z", 1.0)
        self.declare_parameter("linear_x_sign", 1.0)
        self.declare_parameter("linear_y_sign", 1.0)
        self.declare_parameter("angular_z_sign", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("enable_native_goal_bridge", False)
        self.declare_parameter("enable_axis_command", True)
        self.declare_parameter("enable_gait_command", True)
        self.declare_parameter("gait_command_topic", "/m20pro/gait_command")
        self.declare_parameter("gait_flat_param", 1)
        self.declare_parameter("gait_stair_param", 14)
        self.declare_parameter("enable_initialpose_relocalization", True)
        self.declare_parameter("enable_initialpose_3d_relocalization", True)
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("initialpose_3d_topic", "/m20pro/initialpose_3d")
        self.declare_parameter("relocalization_response_timeout_s", 2.0)
        self.declare_parameter("send_heartbeat", False)

        self.client = M20TcpClient(
            self.get_parameter("robot_ip").value,
            int(self.get_parameter("tcp_port").value),
            timeout=2.0,
        )
        self.latest_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.connected = False
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(PoseStamped, "~/map_pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.loc_pub = self.create_publisher(Bool, "~/localization_ok", 10)
        self.obs_pub = self.create_publisher(Bool, "~/obstacle_active", 10)
        self.status_pub = self.create_publisher(String, "~/navigation_status", 10)
        self.raw_pub = self.create_publisher(String, "~/raw_status_json", 10)
        self.relocalization_pub = self.create_publisher(String, "~/relocalization_result", 10)
        self.gait_result_pub = self.create_publisher(String, "~/gait_result", 10)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        if bool(self.get_parameter("enable_gait_command").value):
            self.create_subscription(
                String,
                str(self.get_parameter("gait_command_topic").value),
                self._on_gait_command,
                10,
            )
        if bool(self.get_parameter("enable_native_goal_bridge").value):
            self.create_subscription(PoseStamped, "/goal_pose", self._on_goal_pose, 10)
        if bool(self.get_parameter("enable_initialpose_relocalization").value):
            self.create_subscription(
                PoseWithCovarianceStamped,
                str(self.get_parameter("initialpose_topic").value),
                self._on_initial_pose,
                10,
            )
        if bool(self.get_parameter("enable_initialpose_3d_relocalization").value):
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter("initialpose_3d_topic").value),
                self._on_initial_pose_3d,
                10,
            )

        poll_period = 1.0 / max(0.5, float(self.get_parameter("poll_rate_hz").value))
        cmd_period = 1.0 / max(1.0, float(self.get_parameter("cmd_vel_rate_hz").value))
        self.create_timer(poll_period, self._poll_robot)
        if bool(self.get_parameter("enable_axis_command").value):
            self.create_timer(cmd_period, self._send_axis_command)
            command_mode = "axis command enabled"
        else:
            command_mode = "shadow mode; axis command disabled"
        self.get_logger().info(
            "M20 TCP bridge ready; target 103 host is %s:%s; %s"
            % (self.client.ip, self.client.port, command_mode)
        )

    def destroy_node(self):
        self.client.close()
        super().destroy_node()

    def _ensure_connected(self) -> bool:
        if self.client.is_connected():
            return True
        try:
            self.client.connect()
            self.connected = True
            self.get_logger().info("connected to M20 body protocol")
            return True
        except OSError as exc:
            if self.connected:
                self.get_logger().warning("lost M20 TCP connection: %s" % exc)
            self.connected = False
            return False

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_gait_command(self, msg: String) -> None:
        gait_param = self._resolve_gait_param(msg.data)
        if gait_param is None:
            text = "failed: unknown gait command '%s'" % msg.data
            self.get_logger().warning(text)
            self._publish_gait_result(text)
            return
        if not self._ensure_connected():
            self._publish_gait_result("failed: tcp connection unavailable")
            return

        try:
            self.client.request(2, 23, {"GaitParam": gait_param}, wait_response=False)
        except OSError as exc:
            text = "failed: %s" % exc
            self.get_logger().warning("gait command failed: %s" % exc)
            self._publish_gait_result(text)
            return

        text = "sent: label=%s GaitParam=%d" % (msg.data.strip(), gait_param)
        self.get_logger().info("vendor gait command sent: %s" % text)
        self._publish_gait_result(text)

    def _publish_gait_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.gait_result_pub.publish(msg)

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Forward RViz 2D Pose Estimate to the vendor localization reset API."""
        self._send_relocalization_pose(msg.header.frame_id, msg.pose.pose)

    def _on_initial_pose_3d(self, msg: PoseStamped) -> None:
        """Forward a z-aware initial pose to the vendor localization reset API."""
        self._send_relocalization_pose(msg.header.frame_id, msg.pose)

    def _send_relocalization_pose(self, frame_id: str, pose: object) -> None:
        if not self._ensure_connected():
            self._publish_relocalization_result("failed: tcp connection unavailable")
            return

        map_frame = str(self.get_parameter("map_frame").value)
        if frame_id and frame_id != map_frame:
            self.get_logger().warning(
                "initial pose frame is '%s', expected '%s'; sending raw coordinates anyway"
                % (frame_id, map_frame)
            )

        yaw = quaternion_to_yaw(pose.orientation)
        items = {
            "PosX": float(pose.position.x),
            "PosY": float(pose.position.y),
            "PosZ": float(pose.position.z),
            "Yaw": float(yaw),
        }
        timeout_s = max(0.1, float(self.get_parameter("relocalization_response_timeout_s").value))
        try:
            response = self.client.request(2101, 1, items, response_timeout=timeout_s)
            result = patrol_items(response)
            error_code = int(result.get("ErrorCode", -1))
        except Exception as exc:
            text = "failed: %s" % exc
            self.get_logger().warning("vendor relocalization request failed: %s" % exc)
            self._publish_relocalization_result(text)
            return

        if error_code == 0:
            text = "success: x=%.3f y=%.3f z=%.3f yaw=%.3f" % (
                items["PosX"], items["PosY"], items["PosZ"], items["Yaw"]
            )
            self.get_logger().info("vendor relocalization reset accepted: %s" % text)
            self._publish_map_pose()
            self._publish_navigation_status()
        else:
            text = "failed: ErrorCode=0x%04X x=%.3f y=%.3f yaw=%.3f" % (
                error_code & 0xFFFF, items["PosX"], items["PosY"], items["Yaw"]
            )
            self.get_logger().warning("vendor relocalization reset rejected: %s" % text)
        self._publish_relocalization_result(text)

    def _publish_relocalization_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.relocalization_pub.publish(msg)

    def _send_axis_command(self) -> None:
        if not self._ensure_connected():
            return
        max_x = float(self.get_parameter("max_linear_x").value)
        max_y = float(self.get_parameter("max_linear_y").value)
        max_yaw = float(self.get_parameter("max_angular_z").value)
        timeout_s = max(0.0, float(self.get_parameter("cmd_vel_timeout_s").value))
        age_s = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        cmd = self.latest_cmd if age_s <= timeout_s else Twist()
        linear_x_sign = float(self.get_parameter("linear_x_sign").value)
        linear_y_sign = float(self.get_parameter("linear_y_sign").value)
        angular_z_sign = float(self.get_parameter("angular_z_sign").value)
        items = {
            "X": self._norm(cmd.linear.x * linear_x_sign, max_x),
            "Y": self._norm(cmd.linear.y * linear_y_sign, max_y),
            "Z": 0.0,
            "Roll": 0.0,
            "Pitch": 0.0,
            "Yaw": self._norm(cmd.angular.z * angular_z_sign, max_yaw),
        }
        try:
            self.client.request(2, 21, items, wait_response=False)
        except OSError as exc:
            self.get_logger().warning("axis command failed: %s" % exc)

    def _poll_robot(self) -> None:
        if not self._ensure_connected():
            return
        if bool(self.get_parameter("send_heartbeat").value):
            try:
                self.client.request(100, 100, {}, wait_response=False)
            except OSError:
                return
        self._publish_map_pose()
        self._publish_navigation_status()

    def _publish_map_pose(self) -> None:
        try:
            items = patrol_items(self.client.request(1007, 2, {}, response_timeout=1.0))
        except Exception as exc:
            self.get_logger().debug("map pose query failed: %s" % exc)
            return
        if not items:
            return
        now = self.get_clock().now().to_msg()
        map_frame = str(self.get_parameter("map_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        x = float(items.get("PosX", 0.0))
        y = float(items.get("PosY", 0.0))
        z = float(items.get("PosZ", 0.0))
        yaw = float(items.get("Yaw", 0.0))
        quat = yaw_to_quaternion(yaw)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = quat
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose = pose.pose
        self.odom_pub.publish(odom)

        if bool(self.get_parameter("publish_tf").value):
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = now
            map_to_odom.header.frame_id = map_frame
            map_to_odom.child_frame_id = odom_frame
            map_to_odom.transform.rotation.w = 1.0

            odom_to_base = TransformStamped()
            odom_to_base.header.stamp = now
            odom_to_base.header.frame_id = odom_frame
            odom_to_base.child_frame_id = base_frame
            odom_to_base.transform.translation.x = x
            odom_to_base.transform.translation.y = y
            odom_to_base.transform.translation.z = z
            odom_to_base.transform.rotation = quat
            self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

        localization_ok = Bool()
        localization_ok.data = int(items.get("Location", 1)) == 0
        self.loc_pub.publish(localization_ok)

    def _publish_navigation_status(self) -> None:
        try:
            items = patrol_items(self.client.request(2002, 1, {}, response_timeout=1.0))
        except Exception as exc:
            self.get_logger().debug("navigation status query failed: %s" % exc)
            return
        if not items:
            return
        obs = Bool()
        obs.data = int(items.get("ObsState", 0)) == 1
        self.obs_pub.publish(obs)
        loc = Bool()
        loc.data = int(items.get("Location", 1)) == 0
        self.loc_pub.publish(loc)
        status = String()
        status.data = "location=%s obstacle=%s" % (items.get("Location"), items.get("ObsState"))
        self.status_pub.publish(status)
        raw = String()
        raw.data = str(items)
        self.raw_pub.publish(raw)

    def _on_goal_pose(self, goal: PoseStamped) -> None:
        if not self._ensure_connected():
            return
        yaw = quaternion_to_yaw(goal.pose.orientation)
        items = {
            "Value": 1,
            "MapID": 0,
            "PosX": goal.pose.position.x,
            "PosY": goal.pose.position.y,
            "PosZ": goal.pose.position.z,
            "AngleYaw": yaw,
            "PointInfo": 1,
            "Gait": 12,
            "Speed": 1,
            "Manner": 0,
            "ObsMode": 0,
            "NavMode": 1,
        }
        try:
            self.client.request(1003, 1, items, wait_response=False)
            self.get_logger().info("native M20 navigation goal sent to 103 host")
        except Exception as exc:
            self.get_logger().warning("failed to send native navigation goal: %s" % exc)

    @staticmethod
    def _norm(value: float, scale: float) -> float:
        if scale <= 0.0:
            return 0.0
        return max(-1.0, min(1.0, float(value) / scale))

    def _resolve_gait_param(self, command: str) -> Optional[int]:
        text = command.strip()
        if not text:
            return None
        normalized = text.lower().replace("-", "_").replace(" ", "_")
        if normalized.startswith("gaitparam="):
            normalized = normalized.split("=", 1)[1].strip()
        try:
            return int(normalized, 0)
        except ValueError:
            pass

        flat = int(self.get_parameter("gait_flat_param").value)
        stair = int(self.get_parameter("gait_stair_param").value)
        mapping = {
            "flat": flat,
            "base": flat,
            "basic": flat,
            "normal": flat,
            "stand": flat,
            "stair": stair,
            "stairs": stair,
            "stair_up": stair,
            "stair_down": stair,
            "upstairs": stair,
            "downstairs": stair,
        }
        return mapping.get(normalized)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = M20TcpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
