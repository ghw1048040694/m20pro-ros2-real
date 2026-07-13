import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from .geometry import quaternion_to_yaw, yaw_to_quaternion
from .odom_alignment_contract import odom_alignment_update
from .pose_stability_contract import stable_jump_decision
from .tcp_protocol import M20TcpClient, patrol_items


class M20TcpBridge(Node):
    def __init__(self):
        super().__init__("m20pro_tcp_bridge")
        self.declare_parameter("robot_ip", "10.21.31.103")
        self.declare_parameter("tcp_port", 30001)
        self.declare_parameter("poll_rate_hz", 5.0)
        self.declare_parameter("cmd_vel_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("send_idle_zero_commands", False)
        self.declare_parameter("max_linear_x", 0.8)
        self.declare_parameter("max_linear_y", 0.5)
        self.declare_parameter("max_angular_z", 1.0)
        self.declare_parameter("linear_x_sign", 1.0)
        self.declare_parameter("linear_y_sign", 1.0)
        self.declare_parameter("angular_z_sign", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("vendor_position_scale", 1.0)
        self.declare_parameter("vendor_position_offset_x", 0.0)
        self.declare_parameter("vendor_position_offset_y", 0.0)
        self.declare_parameter("vendor_position_offset_z", 0.0)
        self.declare_parameter("max_abs_map_position_m", 10000.0)
        self.declare_parameter("pose_source", "tcp_1007")
        self.declare_parameter("reject_origin_placeholder_pose", True)
        self.declare_parameter("origin_placeholder_radius_m", 0.05)
        self.declare_parameter("pose_jump_reject_m", 0.6)
        self.declare_parameter("pose_jump_accept_after_s", 8.0)
        self.declare_parameter("pose_jump_candidate_radius_m", 0.30)
        self.declare_parameter("pose_jump_candidate_yaw_tolerance_rad", 0.35)
        self.declare_parameter("pose_filter_hold_last_good_s", 1.0)
        self.declare_parameter("pose_relocalization_jump_grace_s", 3.0)
        self.declare_parameter("pose_relocalization_jump_grace_radius_m", 1.0)
        self.declare_parameter("odom_rebase_jump_m", 0.6)
        self.declare_parameter("odom_rebase_jump_yaw_rad", 0.75)
        self.declare_parameter("enable_tf_pose_fallback", True)
        self.declare_parameter("tf_pose_fallback_topic", "/tf")
        self.declare_parameter("tf_pose_fallback_map_frame", "map")
        self.declare_parameter("tf_pose_fallback_base_frame", "base_link")
        self.declare_parameter("tf_pose_fallback_max_age_s", 1.0)
        self.declare_parameter("tf_pose_fallback_require_location_ok", True)
        self.declare_parameter("flatten_odom_z", False)
        self.declare_parameter("odom_z", 0.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("enable_native_goal_bridge", False)
        self.declare_parameter("enable_axis_command", True)
        self.declare_parameter("enable_gait_command", True)
        self.declare_parameter("gait_command_topic", "/m20pro/gait_command")
        self.declare_parameter("gait_flat_param", 1)
        self.declare_parameter("gait_assist_param", 12)
        self.declare_parameter("gait_stair_param", 14)
        self.declare_parameter("enable_usage_mode_command", False)
        self.declare_parameter("usage_mode_command_topic", "/m20pro/usage_mode_command")
        self.declare_parameter("enable_initialpose_relocalization", True)
        self.declare_parameter("enable_initialpose_3d_relocalization", True)
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("initialpose_3d_topic", "/m20pro/initialpose_3d")
        self.declare_parameter("relocalization_response_timeout_s", 2.0)
        self.declare_parameter("relocalization_duplicate_suppression_s", 1.5)
        self.declare_parameter("relocalization_duplicate_tolerance_m", 0.05)
        self.declare_parameter("relocalization_duplicate_yaw_tolerance_rad", 0.05)
        self.declare_parameter("send_heartbeat", False)

        self.client = M20TcpClient(
            self.get_parameter("robot_ip").value,
            int(self.get_parameter("tcp_port").value),
            timeout=2.0,
        )
        self.latest_cmd = Twist()
        self.last_cmd_time = None
        self.axis_command_active = False
        self.idle_zero_sent = False
        self.connected = False
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_pose_warning_time = None
        self.last_status_warning_time = None
        self.last_invalid_pose_warning_time = None
        self.last_good_map_pose = None
        self.pending_jump_pose = None
        self.last_relocalization_request_time = None
        self.last_relocalization_target = None
        self.last_relocalization_attempt_monotonic = 0.0
        self.last_relocalization_attempt_target = None
        self.last_tf_pose_fallback = None
        self.last_navigation_location_ok = None
        self.last_navigation_status_time = None
        self.official_tf_pose_unlocked = False
        self.last_published_map_pose = None
        self.last_published_odom_pose = None
        self.map_to_odom_pose = None
        self.pending_odom_rebase = False

        self.pose_pub = self.create_publisher(PoseStamped, "~/map_pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.loc_pub = self.create_publisher(Bool, "~/localization_ok", 10)
        self.obs_pub = self.create_publisher(Bool, "~/obstacle_active", 10)
        self.status_pub = self.create_publisher(String, "~/navigation_status", 10)
        self.raw_pub = self.create_publisher(String, "~/raw_status_json", 10)
        self.relocalization_pub = self.create_publisher(String, "~/relocalization_result", 10)
        self.gait_result_pub = self.create_publisher(String, "~/gait_result", 10)
        self.usage_mode_result_pub = self.create_publisher(String, "~/usage_mode_result", 10)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        if bool(self.get_parameter("enable_gait_command").value):
            self.create_subscription(
                String,
                str(self.get_parameter("gait_command_topic").value),
                self._on_gait_command,
                10,
            )
        if bool(self.get_parameter("enable_usage_mode_command").value):
            self.create_subscription(
                String,
                str(self.get_parameter("usage_mode_command_topic").value),
                self._on_usage_mode_command,
                10,
            )
        if bool(self.get_parameter("enable_tf_pose_fallback").value):
            self.create_subscription(
                TFMessage,
                str(self.get_parameter("tf_pose_fallback_topic").value),
                self._on_tf_pose_fallback,
                50,
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
        idle_mode = (
            "idle zero enabled"
            if bool(self.get_parameter("send_idle_zero_commands").value)
            else "idle zero disabled"
        )
        self.get_logger().info(
            "M20 TCP bridge ready; target 103 host is %s:%s; %s; %s"
            % (self.client.ip, self.client.port, command_mode, idle_mode)
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
        self.idle_zero_sent = False

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

    def _on_usage_mode_command(self, msg: String) -> None:
        mode = self._resolve_usage_mode(msg.data)
        if mode is None:
            text = "failed: unknown usage mode command '%s'" % msg.data
            self.get_logger().warning(text)
            self._publish_usage_mode_result(text)
            return
        if not self._ensure_connected():
            self._publish_usage_mode_result("failed: tcp connection unavailable")
            return

        try:
            self.client.request(1101, 5, {"Mode": mode}, wait_response=False)
        except OSError as exc:
            text = "failed: %s" % exc
            self.get_logger().warning("usage mode command failed: %s" % exc)
            self._publish_usage_mode_result(text)
            return

        text = "sent: label=%s Mode=%d" % (msg.data.strip(), mode)
        self.get_logger().info("vendor usage mode command sent: %s" % text)
        self._publish_usage_mode_result(text)

    def _publish_usage_mode_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.usage_mode_result_pub.publish(msg)

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Forward RViz 2D Pose Estimate to the vendor localization reset API."""
        self._send_relocalization_pose(msg.header.frame_id, msg.pose.pose)

    def _on_initial_pose_3d(self, msg: PoseStamped) -> None:
        """Forward a z-aware initial pose to the vendor localization reset API."""
        self._send_relocalization_pose(msg.header.frame_id, msg.pose)

    def _on_tf_pose_fallback(self, msg: TFMessage) -> None:
        map_frame = str(self.get_parameter("tf_pose_fallback_map_frame").value)
        base_frame = str(self.get_parameter("tf_pose_fallback_base_frame").value)
        for transform in msg.transforms:
            if transform.header.frame_id != map_frame or transform.child_frame_id != base_frame:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = quaternion_to_yaw(rotation)
            values = (translation.x, translation.y, translation.z, yaw)
            if not all(math.isfinite(float(value)) for value in values):
                continue
            self.last_tf_pose_fallback = {
                "x": float(translation.x),
                "y": float(translation.y),
                "z": float(translation.z),
                "yaw": float(yaw),
                "stamp": self.get_clock().now(),
                "source_frame": map_frame,
                "source_child_frame": base_frame,
            }
            if self._pose_source() == "official_tf":
                self._publish_official_tf_pose("official_tf")
            return

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
        # The M20 Pro developer manual defines 2101/1 PosX/PosY/PosZ in map-frame
        # meters. Do not apply the 1007 pose scale used by some firmware builds.
        vendor_x = float(pose.position.x)
        vendor_y = float(pose.position.y)
        vendor_z = float(pose.position.z)
        items = {
            "PosX": vendor_x,
            "PosY": vendor_y,
            "PosZ": vendor_z,
            "Yaw": float(yaw),
        }
        duplicate = self._duplicate_relocalization_request(items)
        if duplicate:
            self._warn_throttled(
                "relocalization_duplicate",
                "ignored duplicate relocalization request: x=%.3f y=%.3f z=%.3f yaw=%.3f"
                % (items["PosX"], items["PosY"], items["PosZ"], items["Yaw"]),
                1.0,
            )
            return
        self.last_relocalization_attempt_monotonic = time.monotonic()
        self.last_relocalization_attempt_target = dict(items)
        # Arm pose verification before waiting for the reply. Firmware on this
        # robot can execute 2101/1 while returning ErrorCode=0xFFFF.
        self.last_relocalization_request_time = self.get_clock().now()
        self.last_relocalization_target = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw": float(items["Yaw"]),
        }
        self.pending_jump_pose = None
        self.pending_odom_rebase = True
        timeout_s = max(0.1, float(self.get_parameter("relocalization_response_timeout_s").value))
        try:
            response = self.client.request(2101, 1, items, response_timeout=timeout_s)
            result = patrol_items(response)
            error_code = int(result.get("ErrorCode", -1))
        except Exception as exc:
            self.last_relocalization_request_time = None
            self.last_relocalization_target = None
            self.pending_odom_rebase = False
            text = "failed: %s" % exc
            self.get_logger().warning("vendor relocalization request failed: %s" % exc)
            self._publish_relocalization_result(text)
            return

        if error_code == 0:
            text = "success: x=%.3f y=%.3f z=%.3f yaw=%.3f" % (
                pose.position.x, pose.position.y, pose.position.z, items["Yaw"]
            )
            self.get_logger().info("vendor relocalization reset accepted: %s" % text)
            self._publish_map_pose()
            self._publish_navigation_status()
        elif (error_code & 0xFFFF) == 0xFFFF:
            text = (
                "pending_verification: ErrorCode=0xFFFF firmware reply ambiguous "
                "x=%.3f y=%.3f z=%.3f yaw=%.3f"
                % (items["PosX"], items["PosY"], items["PosZ"], items["Yaw"])
            )
            self.get_logger().warning(
                "vendor relocalization reply is ambiguous; verifying official pose: %s" % text
            )
            self._publish_map_pose()
            self._publish_navigation_status()
        else:
            self.last_relocalization_request_time = None
            self.last_relocalization_target = None
            self.pending_odom_rebase = False
            meaning = "初始化定位失败" if error_code == 1 else "原厂返回错误"
            text = "failed: ErrorCode=0x%04X %s x=%.3f y=%.3f z=%.3f yaw=%.3f" % (
                error_code & 0xFFFF,
                meaning,
                items["PosX"],
                items["PosY"],
                items["PosZ"],
                items["Yaw"],
            )
            self.get_logger().warning("vendor relocalization reset rejected: %s" % text)
        self._publish_relocalization_result(text)

    def _publish_relocalization_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.relocalization_pub.publish(msg)

    def _duplicate_relocalization_request(self, items: dict) -> bool:
        last = self.last_relocalization_attempt_target
        if not isinstance(last, dict):
            return False
        window_s = max(0.0, float(self.get_parameter("relocalization_duplicate_suppression_s").value))
        if window_s <= 0.0:
            return False
        if time.monotonic() - float(self.last_relocalization_attempt_monotonic or 0.0) > window_s:
            return False
        pos_tolerance = max(0.0, float(self.get_parameter("relocalization_duplicate_tolerance_m").value))
        yaw_tolerance = max(
            0.0,
            float(self.get_parameter("relocalization_duplicate_yaw_tolerance_rad").value),
        )
        try:
            distance = math.sqrt(
                (float(items["PosX"]) - float(last["PosX"])) ** 2
                + (float(items["PosY"]) - float(last["PosY"])) ** 2
                + (float(items["PosZ"]) - float(last["PosZ"])) ** 2
            )
            yaw_error = abs(self._wrap_angle(float(items["Yaw"]) - float(last["Yaw"])))
        except (KeyError, TypeError, ValueError):
            return False
        return distance <= pos_tolerance and yaw_error <= yaw_tolerance

    def _send_axis_command(self) -> None:
        timeout_s = max(0.0, float(self.get_parameter("cmd_vel_timeout_s").value))
        send_idle_zero = bool(self.get_parameter("send_idle_zero_commands").value)
        if self.last_cmd_time is None:
            if not send_idle_zero:
                return
            cmd = Twist()
        else:
            age_s = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age_s <= timeout_s:
                cmd = self.latest_cmd
            elif send_idle_zero:
                cmd = Twist()
            else:
                if self.idle_zero_sent or not self.axis_command_active:
                    return
                cmd = Twist()

        if not self._ensure_connected():
            return
        max_x = float(self.get_parameter("max_linear_x").value)
        max_y = float(self.get_parameter("max_linear_y").value)
        max_yaw = float(self.get_parameter("max_angular_z").value)
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
            return

        nonzero = (
            abs(items["X"]) > 1e-6
            or abs(items["Y"]) > 1e-6
            or abs(items["Yaw"]) > 1e-6
        )
        if nonzero:
            self.axis_command_active = True
            self.idle_zero_sent = False
        else:
            self.axis_command_active = False
            if self.last_cmd_time is not None:
                age_s = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
                if age_s > timeout_s and not send_idle_zero:
                    self.idle_zero_sent = True

    def _poll_robot(self) -> None:
        if not self._ensure_connected():
            return
        if bool(self.get_parameter("send_heartbeat").value):
            try:
                self.client.request(100, 100, {}, wait_response=False)
            except OSError:
                return
        self._publish_navigation_status()
        self._publish_map_pose()

    def _publish_map_pose(self) -> None:
        pose_source = self._pose_source()
        if pose_source == "official_tf":
            if not self._publish_official_tf_pose("official_tf_primary"):
                self._publish_localization_ok(False)
            return
        items = {}
        try:
            items = patrol_items(self.client.request(1007, 2, {}, response_timeout=1.0))
        except Exception as exc:
            self._warn_throttled("pose", "map pose query failed: %s" % exc, 5.0)
            if pose_source == "auto" and self._publish_tf_pose_fallback("vendor_query_failed", items):
                return
            self._publish_localization_ok(False)
            return
        if not items:
            if pose_source == "auto" and self._publish_tf_pose_fallback("vendor_pose_empty", items):
                return
            self._publish_localization_ok(False)
            return

        vendor_pose = self._vendor_pose_from_items(items)
        if not vendor_pose["ok"]:
            if pose_source == "auto" and self._publish_tf_pose_fallback(str(vendor_pose["reason"]), items):
                return
            self._warn_throttled(
                "invalid_pose",
                "ignored invalid or unlocalized vendor pose: %s Location=%s PosX=%s PosY=%s PosZ=%s Yaw=%s"
                % (
                    vendor_pose["reason"],
                    items.get("Location"),
                    items.get("PosX"),
                    items.get("PosY"),
                    items.get("PosZ"),
                    items.get("Yaw"),
                ),
                5.0,
            )
            self._publish_localization_ok(False)
            return

        x = float(vendor_pose["x"])
        y = float(vendor_pose["y"])
        z = float(vendor_pose["z"])
        yaw = float(vendor_pose["yaw"])
        max_abs_position = max(1.0, float(self.get_parameter("max_abs_map_position_m").value))
        pose_values_finite = all(math.isfinite(value) for value in (x, y, z, yaw))
        pose_values_plausible = (
            pose_values_finite
            and abs(x) <= max_abs_position
            and abs(y) <= max_abs_position
            and abs(z) <= max_abs_position
        )
        if not pose_values_plausible:
            if pose_source == "auto" and self._publish_tf_pose_fallback("vendor_pose_implausible", items):
                return
            self._warn_throttled(
                "invalid_pose",
                "ignored invalid vendor pose: Location=%s PosX=%s PosY=%s PosZ=%s Yaw=%s"
                % (
                    items.get("Location"),
                    items.get("PosX"),
                    items.get("PosY"),
                    items.get("PosZ"),
                    items.get("Yaw"),
                ),
                5.0,
            )
            localization_ok = Bool()
            localization_ok.data = False
            self.loc_pub.publish(localization_ok)
            return
        valid_by_filter, filter_reason = self._map_pose_passes_stability_filter(x, y, z, yaw)
        if not valid_by_filter:
            if pose_source == "auto" and self._publish_tf_pose_fallback(filter_reason, items):
                return
            self._warn_throttled(
                "invalid_pose",
                "ignored unstable vendor pose: %s Location=%s PosX=%s PosY=%s PosZ=%s Yaw=%s"
                % (
                    filter_reason,
                    items.get("Location"),
                    items.get("PosX"),
                    items.get("PosY"),
                    items.get("PosZ"),
                    items.get("Yaw"),
                ),
                2.0,
            )
            self._publish_localization_ok(self._has_recent_good_map_pose())
            return
        self._publish_map_pose_values(x, y, z, yaw)

    def _pose_source(self) -> str:
        value = str(self.get_parameter("pose_source").value or "").strip().lower()
        if value in ("official_tf", "tf", "factory_tf"):
            return "official_tf"
        if value in ("auto", "tf_fallback"):
            return "auto"
        return "tcp_1007"

    def _vendor_pose_from_items(self, items: dict) -> dict:
        try:
            location_ok = int(items.get("Location", 1)) == 0
        except (TypeError, ValueError):
            location_ok = False
        if not location_ok:
            return {"ok": False, "reason": "location_not_ok"}
        required = ("PosX", "PosY", "PosZ", "Yaw")
        missing = [key for key in required if key not in items or items.get(key) is None]
        if missing:
            return {"ok": False, "reason": "missing_pose_fields:%s" % ",".join(missing)}
        try:
            x = float(items["PosX"])
            y = float(items["PosY"])
            z = float(items["PosZ"])
            yaw = float(items["Yaw"])
        except (TypeError, ValueError):
            return {"ok": False, "reason": "non_numeric_pose_fields"}
        x, y, z = self._vendor_position_to_ros(x, y, z)
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            return {"ok": False, "reason": "non_finite_pose_fields"}
        return {"ok": True, "x": x, "y": y, "z": z, "yaw": yaw}

    def _publish_tf_pose_fallback(self, reason: str, vendor_items: dict) -> bool:
        fallback = self._tf_pose_fallback_payload(vendor_items)
        if fallback is None:
            return False
        self._warn_throttled(
            "pose_fallback",
            "using TF pose fallback %s->%s because %s; x=%.3f y=%.3f z=%.3f yaw=%.3f"
            % (
                fallback["source_frame"],
                fallback["source_child_frame"],
                reason,
                fallback["x"],
                fallback["y"],
                fallback["z"],
                fallback["yaw"],
            ),
            2.0,
        )
        self.last_good_map_pose = {
            "x": fallback["x"],
            "y": fallback["y"],
            "z": fallback["z"],
            "yaw": fallback["yaw"],
            "stamp": self.get_clock().now(),
        }
        self.pending_jump_pose = None
        self._publish_map_pose_values(
            fallback["x"],
            fallback["y"],
            fallback["z"],
            fallback["yaw"],
        )
        return True

    def _publish_official_tf_pose(self, reason: str) -> bool:
        fallback = self._tf_pose_fallback_payload({}, require_location_ok=False)
        if fallback is None:
            self._warn_throttled(
                "pose_fallback",
                "official TF pose source is not ready; waiting for fresh %s->%s"
                % (
                    self.get_parameter("tf_pose_fallback_map_frame").value,
                    self.get_parameter("tf_pose_fallback_base_frame").value,
                ),
                2.0,
            )
            return False
        location_ok = self._recent_navigation_location_ok()
        if not self.official_tf_pose_unlocked:
            if not location_ok:
                self._warn_throttled(
                    "pose_fallback",
                    "official TF pose source has fresh data but is locked until Location=0; x=%.3f y=%.3f z=%.3f"
                    % (fallback["x"], fallback["y"], fallback["z"]),
                    2.0,
                )
                return False
            self.official_tf_pose_unlocked = True

        valid_by_filter, filter_reason = self._map_pose_passes_stability_filter(
            fallback["x"],
            fallback["y"],
            fallback["z"],
            fallback["yaw"],
            allow_stable_recovery=location_ok,
        )
        if not valid_by_filter:
            self._warn_throttled(
                "pose_fallback",
                "ignored unstable official TF pose: %s x=%.3f y=%.3f z=%.3f yaw=%.3f"
                % (
                    filter_reason,
                    fallback["x"],
                    fallback["y"],
                    fallback["z"],
                    fallback["yaw"],
                ),
                2.0,
            )
            # Location=0 without an accepted fresh map pose is not navigation-ready.
            self._publish_localization_ok(False)
            return False
        self.last_good_map_pose = {
            "x": fallback["x"],
            "y": fallback["y"],
            "z": fallback["z"],
            "yaw": fallback["yaw"],
            "stamp": self.get_clock().now(),
        }
        self.pending_jump_pose = None
        self._publish_map_pose_values(
            fallback["x"],
            fallback["y"],
            fallback["z"],
            fallback["yaw"],
            localization_ok=location_ok,
        )
        self._info_throttled(
            "pose_fallback",
            "using official TF pose source %s->%s reason=%s; x=%.3f y=%.3f z=%.3f yaw=%.3f"
            % (
                fallback["source_frame"],
                fallback["source_child_frame"],
                reason,
                fallback["x"],
                fallback["y"],
                fallback["z"],
                fallback["yaw"],
            ),
            30.0,
        )
        return True

    def _tf_pose_fallback_payload(self, vendor_items: dict, require_location_ok: Optional[bool] = None):
        if not bool(self.get_parameter("enable_tf_pose_fallback").value):
            return None
        fallback = self.last_tf_pose_fallback
        if not isinstance(fallback, dict):
            return None
        stamp = fallback.get("stamp")
        if stamp is None:
            return None
        max_age_s = max(0.0, float(self.get_parameter("tf_pose_fallback_max_age_s").value))
        if max_age_s > 0.0 and (self.get_clock().now() - stamp).nanoseconds * 1e-9 > max_age_s:
            return None
        if require_location_ok is None:
            require_location_ok = bool(self.get_parameter("tf_pose_fallback_require_location_ok").value)
        if bool(require_location_ok):
            location_ok = False
            try:
                location_ok = int(vendor_items.get("Location", 1)) == 0
            except (TypeError, ValueError, AttributeError):
                location_ok = False
            if not location_ok and self.last_navigation_location_ok is True:
                status_stamp = self.last_navigation_status_time
                if status_stamp is not None:
                    location_ok = (self.get_clock().now() - status_stamp).nanoseconds * 1e-9 <= 2.0
            if not location_ok:
                return None
        x = float(fallback["x"])
        y = float(fallback["y"])
        z = float(fallback["z"])
        yaw = float(fallback["yaw"])
        max_abs_position = max(1.0, float(self.get_parameter("max_abs_map_position_m").value))
        if (
            not all(math.isfinite(value) for value in (x, y, z, yaw))
            or abs(x) > max_abs_position
            or abs(y) > max_abs_position
            or abs(z) > max_abs_position
        ):
            return None
        return {
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "source_frame": str(fallback.get("source_frame") or ""),
            "source_child_frame": str(fallback.get("source_child_frame") or ""),
        }

    def _recent_navigation_location_ok(self) -> bool:
        if self.last_navigation_location_ok is not True:
            return False
        status_stamp = self.last_navigation_status_time
        if status_stamp is None:
            return False
        age_s = (self.get_clock().now() - status_stamp).nanoseconds * 1e-9
        return age_s <= 2.0

    def _publish_map_pose_values(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        localization_ok: bool = True,
    ) -> None:
        clock_now = self.get_clock().now()
        now = clock_now.to_msg()
        map_frame = str(self.get_parameter("map_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        map_quat = yaw_to_quaternion(yaw)
        force_odom_rebase = bool(
            self.pending_odom_rebase
            and self._within_relocalization_jump_grace(clock_now, x, y)
        )
        alignment = odom_alignment_update(
            map_pose={"x": x, "y": y, "yaw": yaw},
            previous_map_pose=self.last_published_map_pose,
            previous_odom_pose=self.last_published_odom_pose,
            map_to_odom=self.map_to_odom_pose,
            force_rebase=force_odom_rebase,
            jump_threshold_m=max(
                0.0,
                float(self.get_parameter("odom_rebase_jump_m").value),
            ),
            yaw_threshold_rad=max(
                0.0,
                float(self.get_parameter("odom_rebase_jump_yaw_rad").value),
            ),
        )
        odom_pose = dict(alignment["odom_pose"])
        map_to_odom_pose = dict(alignment["map_to_odom"])
        self.last_published_map_pose = dict(alignment["map_pose"])
        self.last_published_odom_pose = odom_pose
        self.map_to_odom_pose = map_to_odom_pose
        if force_odom_rebase:
            self.pending_odom_rebase = False
        if alignment.get("rebased"):
            self.get_logger().warning(
                "rebased map->odom after %s; map_step=%.3fm yaw_step=%.3frad"
                % (
                    alignment.get("reason"),
                    float(alignment.get("map_step_distance_m", 0.0)),
                    float(alignment.get("map_step_yaw_rad", 0.0)),
                )
            )
        odom_quat = yaw_to_quaternion(float(odom_pose["yaw"]))
        map_to_odom_quat = yaw_to_quaternion(float(map_to_odom_pose["yaw"]))

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = map_quat
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom_z = self._odom_z(z)
        odom.pose.pose.position.x = float(odom_pose["x"])
        odom.pose.pose.position.y = float(odom_pose["y"])
        odom.pose.pose.position.z = odom_z
        odom.pose.pose.orientation = odom_quat
        self.odom_pub.publish(odom)

        if bool(self.get_parameter("publish_tf").value):
            map_to_odom_tf = TransformStamped()
            map_to_odom_tf.header.stamp = now
            map_to_odom_tf.header.frame_id = map_frame
            map_to_odom_tf.child_frame_id = odom_frame
            map_to_odom_tf.transform.translation.x = float(map_to_odom_pose["x"])
            map_to_odom_tf.transform.translation.y = float(map_to_odom_pose["y"])
            map_to_odom_tf.transform.rotation = map_to_odom_quat

            odom_to_base = TransformStamped()
            odom_to_base.header.stamp = now
            odom_to_base.header.frame_id = odom_frame
            odom_to_base.child_frame_id = base_frame
            odom_to_base.transform.translation.x = float(odom_pose["x"])
            odom_to_base.transform.translation.y = float(odom_pose["y"])
            odom_to_base.transform.translation.z = odom_z
            odom_to_base.transform.rotation = odom_quat
            self.tf_broadcaster.sendTransform([map_to_odom_tf, odom_to_base])

        self._publish_localization_ok(bool(localization_ok))

    def _map_pose_passes_stability_filter(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        allow_stable_recovery: bool = True,
    ):
        now = self.get_clock().now()
        last = self.last_good_map_pose

        if last is not None and bool(self.get_parameter("reject_origin_placeholder_pose").value):
            origin_radius = max(0.0, float(self.get_parameter("origin_placeholder_radius_m").value))
            last_radius = math.hypot(float(last["x"]), float(last["y"]))
            if (
                math.hypot(x, y) <= origin_radius
                and last_radius > max(0.5, origin_radius * 4.0)
                and not self._within_relocalization_jump_grace(now, x, y)
            ):
                return False, "origin_placeholder_after_good_pose"

        jump_limit = max(0.0, float(self.get_parameter("pose_jump_reject_m").value))
        if last is not None and jump_limit > 0.0 and not self._within_relocalization_jump_grace(now, x, y):
            decision = stable_jump_decision(
                last_pose=last,
                pending_pose=self.pending_jump_pose,
                candidate={"x": x, "y": y, "z": z, "yaw": yaw},
                now_s=now.nanoseconds * 1e-9,
                jump_limit_m=jump_limit,
                accept_after_s=max(
                    0.0,
                    float(self.get_parameter("pose_jump_accept_after_s").value),
                ),
                candidate_radius_m=max(
                    0.01,
                    float(self.get_parameter("pose_jump_candidate_radius_m").value),
                ),
                candidate_yaw_tolerance_rad=max(
                    0.01,
                    float(
                        self.get_parameter("pose_jump_candidate_yaw_tolerance_rad").value
                    ),
                ),
                allow_stable_recovery=allow_stable_recovery,
            )
            self.pending_jump_pose = decision.get("pending_pose")
            if not decision["accept"]:
                return False, str(decision["reason"])
            if decision.get("reason"):
                self.get_logger().warning(str(decision["reason"]))

        self.last_good_map_pose = {"x": x, "y": y, "z": z, "yaw": yaw, "stamp": now}
        self.pending_jump_pose = None
        return True, ""

    def _has_recent_good_map_pose(self) -> bool:
        if self.last_good_map_pose is None:
            return False
        stamp = self.last_good_map_pose.get("stamp")
        if stamp is None:
            return False
        hold_s = max(0.0, float(self.get_parameter("pose_filter_hold_last_good_s").value))
        if hold_s <= 0.0:
            return False
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9 <= hold_s

    def _within_relocalization_jump_grace(self, now, x: float, y: float) -> bool:
        if self.last_relocalization_request_time is None or self.last_relocalization_target is None:
            return False
        grace_s = max(0.0, float(self.get_parameter("pose_relocalization_jump_grace_s").value))
        if grace_s <= 0.0:
            return False
        if (now - self.last_relocalization_request_time).nanoseconds * 1e-9 > grace_s:
            return False
        radius_m = max(0.0, float(self.get_parameter("pose_relocalization_jump_grace_radius_m").value))
        target = self.last_relocalization_target
        return math.hypot(x - float(target["x"]), y - float(target["y"])) <= radius_m

    def _publish_localization_ok(self, ok: bool) -> None:
        localization_ok = Bool()
        localization_ok.data = bool(ok)
        self.loc_pub.publish(localization_ok)

    def _publish_navigation_status(self) -> None:
        try:
            items = patrol_items(self.client.request(2002, 1, {}, response_timeout=1.0))
        except Exception as exc:
            self._warn_throttled("status", "navigation status query failed: %s" % exc, 5.0)
            return
        if not items:
            return
        obs = Bool()
        obs.data = int(items.get("ObsState", 0)) == 1
        self.obs_pub.publish(obs)
        try:
            self.last_navigation_location_ok = int(items.get("Location", 1)) == 0
            self.last_navigation_status_time = self.get_clock().now()
        except (TypeError, ValueError):
            self.last_navigation_location_ok = False
            self.last_navigation_status_time = self.get_clock().now()
        # Keep localization_ok owned by _publish_map_pose(): it requires a fresh,
        # plausible map pose and avoids this status poll racing that decision.
        status = String()
        status.data = "location=%s obstacle=%s usage_mode=%s ooa=%s" % (
            items.get("Location"),
            items.get("ObsState"),
            items.get("ControlUsageMode"),
            items.get("OOA"),
        )
        self.status_pub.publish(status)
        raw = String()
        raw.data = str(items)
        self.raw_pub.publish(raw)

    def _vendor_position_to_ros(self, x: float, y: float, z: float):
        scale = float(self.get_parameter("vendor_position_scale").value)
        return (
            x * scale + float(self.get_parameter("vendor_position_offset_x").value),
            y * scale + float(self.get_parameter("vendor_position_offset_y").value),
            z * scale + float(self.get_parameter("vendor_position_offset_z").value),
        )

    def _odom_z(self, z: float) -> float:
        if bool(self.get_parameter("flatten_odom_z").value):
            return float(self.get_parameter("odom_z").value)
        return z

    def _warn_throttled(self, key: str, message: str, period_s: float) -> None:
        attr = "last_%s_warning_time" % key
        now = self.get_clock().now()
        last = getattr(self, attr, None)
        if last is None or (now - last).nanoseconds * 1e-9 >= period_s:
            setattr(self, attr, now)
            self.get_logger().warning(message)

    def _info_throttled(self, key: str, message: str, period_s: float) -> None:
        attr = "last_%s_info_time" % key
        now = self.get_clock().now()
        last = getattr(self, attr, None)
        if last is None or (now - last).nanoseconds * 1e-9 >= period_s:
            setattr(self, attr, now)
            self.get_logger().info(message)

    @staticmethod
    def _wrap_angle(value: float) -> float:
        while value > math.pi:
            value -= math.tau
        while value <= -math.pi:
            value += math.tau
        return value

    def _ros_position_to_vendor(self, x: float, y: float, z: float):
        scale = float(self.get_parameter("vendor_position_scale").value)
        if abs(scale) < 1e-12:
            scale = 1.0
        return (
            (x - float(self.get_parameter("vendor_position_offset_x").value)) / scale,
            (y - float(self.get_parameter("vendor_position_offset_y").value)) / scale,
            (z - float(self.get_parameter("vendor_position_offset_z").value)) / scale,
        )

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
        assist = int(self.get_parameter("gait_assist_param").value)
        stair = int(self.get_parameter("gait_stair_param").value)
        mapping = {
            "flat": flat,
            "base": flat,
            "basic": flat,
            "normal": flat,
            "stand": flat,
            "assist": assist,
            "assisted": assist,
            "agile": assist,
            "rl": assist,
            "rl_gait": assist,
            "reinforcement": assist,
            "terrain": assist,
            "all_terrain": assist,
            "stair": stair,
            "stairs": stair,
            "stair_up": stair,
            "stair_down": stair,
            "upstairs": stair,
            "downstairs": stair,
        }
        return mapping.get(normalized)

    def _resolve_usage_mode(self, command: str) -> Optional[int]:
        text = command.strip()
        if not text:
            return None
        normalized = text.lower().replace("-", "_").replace(" ", "_")
        for prefix in ("mode=", "usage_mode=", "controlusagemode="):
            if normalized.startswith(prefix):
                normalized = normalized.split("=", 1)[1].strip()
                break
        try:
            return int(normalized, 0)
        except ValueError:
            pass
        mapping = {
            "normal": 0,
            "regular": 0,
            "manual": 0,
            "axis": 0,
            "teleop": 0,
            "navigation": 1,
            "nav": 1,
            "task": 1,
            "assist": 2,
            "assisted": 2,
            "aux": 2,
            "auxiliary": 2,
            "rl": 2,
            "rl_gait": 2,
            "terrain": 2,
            "all_terrain": 2,
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
