import math
import time
from functools import partial
from typing import Any, List, Optional, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from .geometry import quaternion_to_yaw


class FloorManager(Node):
    """Route same-floor goals to Nav2 while tracking the selected floor."""

    def __init__(self) -> None:
        super().__init__("m20pro_floor_manager")
        self.declare_parameter("initial_floor", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("stair_command_topic", "/m20pro/use_stairs")
        self.declare_parameter("floor_goal_topic", "/m20pro/floor_goal")
        self.declare_parameter("stop_task_topic", "/m20pro/stop_task")
        self.declare_parameter("enable_rviz_floor_goal_topics", True)
        self.declare_parameter("rviz_floor_goal_topics", [])
        self.declare_parameter("current_floor_topic", "/m20pro/current_floor")
        self.declare_parameter("stair_status_topic", "/m20pro/stair_status")
        self.declare_parameter("floor_context_topic", "/m20pro/set_current_floor")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("duplicate_goal_tolerance_m", 0.08)
        self.declare_parameter("duplicate_goal_yaw_tolerance_rad", 0.12)
        self.declare_parameter("nav_feedback_status_period_s", 1.0)

        self.map_frame = (
            str(self.get_parameter("map_frame").value).strip() or "map"
        )

        self.current_floor = ""
        self.active_floor_mission = False
        self.active_nav_goal_handle: Optional[Any] = None
        self.active_nav_goal_label = ""
        self.active_nav_goal_pose: Optional[PoseStamped] = None
        self.active_nav_goal_sequence = 0
        self.nav_goal_sequence = 0
        self.stop_nav_goal_sequence = 0
        self.last_nav_feedback_publish_monotonic = 0.0
        self.cancelled_nav_goal_handle_ids: Set[int] = set()
        self.rviz_floor_goal_subscriptions: List[Any] = []

        self.navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value),
        )
        self.current_floor_pub = self.create_publisher(
            String,
            str(self.get_parameter("current_floor_topic").value),
            10,
        )
        self.stair_status_pub = self.create_publisher(
            String,
            str(self.get_parameter("stair_status_topic").value),
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )

        self.create_subscription(
            String,
            str(self.get_parameter("floor_context_topic").value),
            self._on_floor_context,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stair_command_topic").value),
            self._on_stair_command,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stop_task_topic").value),
            self._on_stop_task,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("floor_goal_topic").value),
            self._on_floor_goal,
            10,
        )
        if bool(self.get_parameter("enable_rviz_floor_goal_topics").value):
            self._create_rviz_floor_goal_subscriptions()
        self.create_timer(1.0, self._publish_current_floor)

        initial_floor = str(self.get_parameter("initial_floor").value).strip()
        if initial_floor:
            self.current_floor = initial_floor
            self.get_logger().info(
                "assuming runtime map label %s without reloading map"
                % initial_floor
            )

        self.get_logger().info(
            "floor manager ready; map_frame=%s" % self.map_frame
        )

    def _create_rviz_floor_goal_subscriptions(self) -> None:
        routes = self._parse_floor_goal_routes(
            list(self.get_parameter("rviz_floor_goal_topics").value),
        )
        if not routes:
            self.get_logger().info(
                "rviz floor goal topics disabled; ordinary map mode"
            )
            return
        for floor_id, topic in routes:
            self.rviz_floor_goal_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    partial(
                        self._on_rviz_floor_goal,
                        floor_id=floor_id,
                        topic=topic,
                    ),
                    10,
                )
            )
        route_text = ", ".join(
            "%s<=%s" % (floor, topic) for floor, topic in routes
        )
        self.get_logger().info(
            "rviz floor goal topics enabled: %s" % route_text
        )

    def _parse_floor_goal_routes(
        self, values: List[str]
    ) -> List[Tuple[str, str]]:
        routes: List[Tuple[str, str]] = []
        for value in values:
            floor_id, separator, topic = str(value).partition(":")
            floor_id = floor_id.strip()
            topic = topic.strip()
            if not separator or not floor_id or not topic.startswith("/"):
                self.get_logger().warning(
                    "ignored invalid rviz_floor_goal_topics entry: %s" % value
                )
                continue
            routes.append((floor_id, topic))
        return routes

    def _on_floor_context(self, msg: String) -> None:
        floor = str(msg.data or "").strip()
        if not floor or self.active_floor_mission:
            return
        self.current_floor = floor
        self._publish_current_floor()
        self.get_logger().info(
            "current floor synchronized from selected map: %s" % floor
        )

    def _on_stair_command(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            self.get_logger().warning("ignored empty stair command")
            return
        self.use_stairs(command)

    def _on_stop_task(self, msg: String) -> None:
        reason = msg.data.strip() or "stop_task"
        self.get_logger().warning(
            "received stop task request: %s" % reason
        )
        self._publish_stair_status("stopped reason=%s" % reason)
        self._clear_floor_mission_if_needed("floor_goal")
        self._cancel_active_nav_goal(reason)

    def _on_floor_goal(self, msg: PoseStamped) -> None:
        target_floor = self._normalize_floor_id(msg.header.frame_id)
        if not target_floor:
            target_floor = self.current_floor
        if not target_floor:
            self.get_logger().error(
                "floor goal ignored; current floor is unknown"
            )
            self._publish_stair_status(
                "error reason=no_current_floor_for_goal label=floor_goal"
            )
            return
        if not self.current_floor:
            self.current_floor = target_floor

        goal = self._pose_in_map_frame(msg)
        if self.active_floor_mission:
            if self._is_duplicate_active_floor_goal(target_floor, goal):
                self.get_logger().info(
                    "duplicate active floor goal ignored"
                )
                self._publish_stair_status(
                    "ignored reason=duplicate_floor_goal target_floor=%s"
                    % target_floor
                )
                return
            if self._can_replace_active_floor_goal(target_floor):
                self.get_logger().warning(
                    "replacing active same-floor goal with a new floor goal"
                )
                self._publish_stair_status(
                    "replacing_active_floor_goal target_floor=%s"
                    % target_floor
                )
                self._cancel_active_nav_goal("replace_floor_goal")
            else:
                self.get_logger().warning(
                    "floor goal ignored; another floor mission is active"
                )
                self._publish_stair_status(
                    "ignored reason=floor_mission_active"
                )
                return

        self.get_logger().info(
            "received floor goal target_floor=%s x=%.2f y=%.2f"
            % (
                target_floor,
                goal.pose.position.x,
                goal.pose.position.y,
            )
        )
        if target_floor == self.current_floor:
            self.active_floor_mission = True
            self._publish_stair_status(
                "same_floor_goal target_floor=%s" % target_floor
            )
            self._send_nav_goal(goal, "floor_goal")
            return

        self.get_logger().error(
            "cross-floor goal rejected: stair_executor owns connector motion"
        )
        self._publish_stair_status(
            "error reason=stair_execution_retired current=%s "
            "target=%s label=floor_goal"
            % (self.current_floor, target_floor)
        )

    def _on_rviz_floor_goal(
        self, msg: PoseStamped, floor_id: str, topic: str
    ) -> None:
        goal = PoseStamped()
        goal.header = msg.header
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = floor_id
        goal.pose = msg.pose
        self.get_logger().info(
            "rviz goal from %s -> floor=%s x=%.2f y=%.2f"
            % (
                topic,
                floor_id,
                goal.pose.position.x,
                goal.pose.position.y,
            )
        )
        self._on_floor_goal(goal)

    def _publish_current_floor(self) -> None:
        if not self.current_floor:
            return
        msg = String()
        msg.data = self.current_floor
        self.current_floor_pub.publish(msg)

    def _send_nav_goal(self, pose: PoseStamped, label: str) -> None:
        if not self.navigate_to_pose_client.wait_for_server(
            timeout_sec=float(self.get_parameter("service_timeout_s").value)
        ):
            self.get_logger().error(
                "navigate_to_pose action server is not available"
            )
            self._publish_stair_status(
                "error reason=navigate_action_unavailable label=%s" % label
            )
            self._clear_floor_mission_if_needed(label)
            return

        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            "sending Nav2 goal label=%s x=%.2f y=%.2f"
            % (label, pose.pose.position.x, pose.pose.position.y)
        )
        self.nav_goal_sequence += 1
        goal_sequence = self.nav_goal_sequence
        future = self.navigate_to_pose_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_nav_feedback(
                feedback, label, goal_sequence
            ),
        )
        future.add_done_callback(
            lambda done: self._on_nav_goal_response(
                done, label, goal_sequence, pose
            )
        )

    def _on_nav_goal_response(
        self,
        future: Any,
        label: str,
        goal_sequence: int,
        pose: PoseStamped,
    ) -> None:
        stale_after_stop = goal_sequence <= self.stop_nav_goal_sequence
        status_suffix = self._goal_status_suffix(
            label, goal_sequence, pose
        )
        try:
            goal_handle = future.result()
        except Exception as exc:
            if stale_after_stop:
                self.get_logger().warning(
                    "ignored stale Nav2 goal request failure after stop "
                    "for %s: %s" % (label, exc)
                )
                self._publish_stair_status(
                    "ignored reason=stale_nav_goal_request %s"
                    % status_suffix
                )
                return
            self.get_logger().error(
                "Nav2 goal request failed for %s: %s" % (label, exc)
            )
            self._publish_stair_status(
                "error reason=nav_goal_request_failed %s" % status_suffix
            )
            self._clear_floor_mission_if_needed(label)
            return
        if not goal_handle.accepted:
            if stale_after_stop:
                self.get_logger().warning(
                    "ignored stale Nav2 goal rejection after stop: %s"
                    % label
                )
                self._publish_stair_status(
                    "ignored reason=stale_nav_goal_rejected %s"
                    % status_suffix
                )
                return
            self.get_logger().error("Nav2 goal rejected: %s" % label)
            self._publish_stair_status(
                "error reason=nav_goal_rejected %s" % status_suffix
            )
            self._clear_floor_mission_if_needed(label)
            return
        if stale_after_stop:
            self.get_logger().warning(
                "cancelling stale Nav2 goal after stop request: %s" % label
            )
            self.cancelled_nav_goal_handle_ids.add(id(goal_handle))
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done: self._on_nav_result(
                    done,
                    label,
                    goal_handle,
                    goal_sequence,
                    pose,
                )
            )
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda done: self._on_cancel_done(
                        done, label, "stale_goal_after_stop"
                    )
                )
            except Exception as exc:
                self.get_logger().error(
                    "failed to cancel stale Nav2 goal %s: %s"
                    % (label, exc)
                )
                self._publish_stair_status(
                    "error reason=nav_cancel_failed label=%s" % label
                )
            self._publish_zero_cmd()
            return
        self.active_nav_goal_handle = goal_handle
        self.active_nav_goal_label = label
        self.active_nav_goal_pose = pose
        self.active_nav_goal_sequence = goal_sequence
        self._publish_stair_status(
            "nav_goal_accepted %s" % status_suffix
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._on_nav_result(
                done,
                label,
                goal_handle,
                goal_sequence,
                pose,
            )
        )

    def _on_nav_feedback(
        self,
        feedback_msg: Any,
        label: str,
        goal_sequence: int,
    ) -> None:
        if goal_sequence <= self.stop_nav_goal_sequence:
            return
        if (
            self.active_nav_goal_sequence
            and self.active_nav_goal_sequence != goal_sequence
        ):
            return
        if self.active_nav_goal_label and self.active_nav_goal_label != label:
            return
        now_monotonic = time.monotonic()
        period_s = max(
            0.2,
            float(
                self.get_parameter(
                    "nav_feedback_status_period_s"
                ).value
            ),
        )
        if (
            now_monotonic - self.last_nav_feedback_publish_monotonic
            < period_s
        ):
            return
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return
        self.last_nav_feedback_publish_monotonic = now_monotonic
        pose_text = self._nav_feedback_pose_text(
            getattr(feedback, "current_pose", None)
        )
        status_suffix = self._goal_status_suffix(
            label,
            goal_sequence,
            self.active_nav_goal_pose,
        )
        self._publish_stair_status(
            "nav_goal_feedback %s distance_remaining=%.3f "
            "navigation_time=%.2f estimated_time_remaining=%.2f "
            "recoveries=%d%s"
            % (
                status_suffix,
                float(getattr(feedback, "distance_remaining", 0.0)),
                self._duration_to_sec(
                    getattr(feedback, "navigation_time", None)
                ),
                self._duration_to_sec(
                    getattr(feedback, "estimated_time_remaining", None)
                ),
                int(
                    getattr(feedback, "number_of_recoveries", 0) or 0
                ),
                pose_text,
            )
        )

    def _on_nav_result(
        self,
        future: Any,
        label: str,
        goal_handle: Any,
        goal_sequence: int,
        pose: PoseStamped,
    ) -> None:
        was_current_goal = self.active_nav_goal_handle is goal_handle
        status_suffix = self._goal_status_suffix(
            label, goal_sequence, pose
        )
        try:
            result = future.result()
        except Exception as exc:
            if not was_current_goal:
                self.get_logger().warning(
                    "ignored stale Nav2 result failure label=%s: %s"
                    % (label, exc)
                )
                self._publish_stair_status(
                    "ignored reason=stale_nav_result_failed %s"
                    % status_suffix
                )
                return
            self.get_logger().error(
                "Nav2 result failed for %s: %s" % (label, exc)
            )
            self._clear_active_nav_goal(goal_handle)
            self._publish_stair_status(
                "error reason=nav_result_failed %s" % status_suffix
            )
            self._clear_floor_mission_if_needed(label)
            return

        status = int(
            getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
        )
        was_cancel_requested = (
            id(goal_handle) in self.cancelled_nav_goal_handle_ids
        )
        self.cancelled_nav_goal_handle_ids.discard(id(goal_handle))
        self._clear_active_nav_goal(goal_handle)
        if (
            status == GoalStatus.STATUS_CANCELED
            and was_cancel_requested
        ):
            self.get_logger().warning(
                "Nav2 goal %s finished as cancelled" % label
            )
            self._publish_stair_status(
                "nav_goal_cancelled_result %s" % status_suffix
            )
            if was_current_goal:
                self._clear_floor_mission_if_needed(label)
            else:
                self.get_logger().info(
                    "ignored stale cancelled Nav2 result for %s"
                    % label
                )
            return
        if not was_current_goal:
            self.get_logger().warning(
                "ignored stale Nav2 result label=%s status=%d"
                % (label, status)
            )
            self._publish_stair_status(
                "ignored reason=stale_nav_result %s status=%d"
                % (status_suffix, status)
            )
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                "Nav2 goal %s finished with status %d"
                % (label, status)
            )
            self._publish_stair_status(
                "error reason=nav_goal_failed %s status=%d"
                % (status_suffix, status)
            )
            self._clear_floor_mission_if_needed(label)
            return

        self._publish_stair_status(
            "nav_goal_succeeded %s" % status_suffix
        )
        if label == "floor_goal":
            self.active_floor_mission = False

    def _can_replace_active_floor_goal(
        self, target_floor: str
    ) -> bool:
        return (
            target_floor == self.current_floor
            and self.active_nav_goal_label == "floor_goal"
        )

    def _cancel_active_nav_goal(self, reason: str) -> None:
        self.stop_nav_goal_sequence = self.nav_goal_sequence
        goal_handle = self.active_nav_goal_handle
        label = self.active_nav_goal_label
        self.active_nav_goal_handle = None
        self.active_nav_goal_label = ""
        self.active_nav_goal_pose = None
        self.active_nav_goal_sequence = 0
        self._publish_zero_cmd()
        if goal_handle is None:
            self.get_logger().info(
                "no active Nav2 goal to cancel for stop request"
            )
            return
        self.cancelled_nav_goal_handle_ids.add(id(goal_handle))
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda done: self._on_cancel_done(
                    done, label, reason
                )
            )
        except Exception as exc:
            self.get_logger().error(
                "failed to cancel Nav2 goal %s: %s" % (label, exc)
            )
            self._publish_stair_status(
                "error reason=nav_cancel_failed label=%s" % label
            )

    def _on_cancel_done(
        self, future: Any, label: str, reason: str
    ) -> None:
        try:
            result = future.result()
            count = len(
                getattr(result, "goals_canceling", []) or []
            )
        except Exception as exc:
            self.get_logger().error(
                "Nav2 cancel result failed for %s: %s"
                % (label, exc)
            )
            self._publish_stair_status(
                "error reason=nav_cancel_result_failed label=%s"
                % label
            )
            return
        self._publish_zero_cmd()
        self.get_logger().warning(
            "cancelled Nav2 goal label=%s count=%d reason=%s"
            % (label, count, reason)
        )
        self._publish_stair_status(
            "nav_goal_cancelled label=%s reason=%s count=%d"
            % (label, reason, count)
        )

    def _clear_active_nav_goal(self, goal_handle: Any) -> None:
        if self.active_nav_goal_handle is goal_handle:
            self.active_nav_goal_handle = None
            self.active_nav_goal_label = ""
            self.active_nav_goal_pose = None
            self.active_nav_goal_sequence = 0

    def _publish_zero_cmd(self, samples: int = 5) -> None:
        count = max(1, int(samples))
        for index in range(count):
            self.cmd_vel_pub.publish(Twist())
            if index + 1 < count:
                time.sleep(0.03)

    def _is_floor_mission_label(self, label: str) -> bool:
        return label == "floor_goal"

    def _is_duplicate_active_floor_goal(
        self,
        target_floor: str,
        goal: PoseStamped,
    ) -> bool:
        if target_floor != self.current_floor:
            return False
        if (
            self.active_nav_goal_label != "floor_goal"
            or self.active_nav_goal_pose is None
        ):
            return False
        active_pose = self.active_nav_goal_pose.pose
        dx = active_pose.position.x - goal.pose.position.x
        dy = active_pose.position.y - goal.pose.position.y
        distance = math.hypot(dx, dy)
        tolerance = max(
            0.0,
            float(
                self.get_parameter(
                    "duplicate_goal_tolerance_m"
                ).value
            ),
        )
        if distance > tolerance:
            return False
        active_yaw = quaternion_to_yaw(active_pose.orientation)
        goal_yaw = quaternion_to_yaw(goal.pose.orientation)
        yaw_error = abs(self._wrap_angle(active_yaw - goal_yaw))
        yaw_tolerance = max(
            0.0,
            float(
                self.get_parameter(
                    "duplicate_goal_yaw_tolerance_rad"
                ).value
            ),
        )
        return yaw_error <= yaw_tolerance

    @staticmethod
    def _wrap_angle(value: float) -> float:
        while value > math.pi:
            value -= math.tau
        while value <= -math.pi:
            value += math.tau
        return value

    @staticmethod
    def _duration_to_sec(value: Any) -> float:
        if value is None:
            return 0.0
        try:
            return float(getattr(value, "sec", 0)) + float(
                getattr(value, "nanosec", 0)
            ) * 1e-9
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _nav_feedback_pose_text(current_pose: Any) -> str:
        pose = getattr(current_pose, "pose", None)
        if pose is None:
            return ""
        try:
            return " pose_x=%.3f pose_y=%.3f pose_yaw=%.4f" % (
                float(pose.position.x),
                float(pose.position.y),
                quaternion_to_yaw(pose.orientation),
            )
        except (TypeError, ValueError):
            return ""

    @staticmethod
    def _goal_status_suffix(
        label: str,
        goal_sequence: int,
        pose: Optional[PoseStamped],
    ) -> str:
        parts = [
            "label=%s" % label,
            "goal_seq=%d" % int(goal_sequence),
        ]
        if pose is None:
            return " ".join(parts)
        try:
            parts.extend(
                [
                    "goal_frame=%s"
                    % (
                        str(
                            pose.header.frame_id or "map"
                        ).replace(" ", "_")
                    ),
                    "goal_x=%.3f" % float(pose.pose.position.x),
                    "goal_y=%.3f" % float(pose.pose.position.y),
                    "goal_z=%.3f" % float(pose.pose.position.z),
                    "goal_yaw=%.4f"
                    % quaternion_to_yaw(pose.pose.orientation),
                ]
            )
        except (TypeError, ValueError):
            pass
        return " ".join(parts)

    def _clear_floor_mission_if_needed(self, label: str) -> None:
        if self._is_floor_mission_label(label):
            self.active_floor_mission = False

    def _pose_in_map_frame(self, msg: PoseStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose = msg.pose
        return pose

    def _normalize_floor_id(self, frame_id: str) -> str:
        value = frame_id.strip()
        if not value or value == self.map_frame:
            return ""
        if value.startswith("floor:"):
            value = value.split(":", 1)[1]
        return value

    def use_stairs(self, command: str) -> None:
        self.get_logger().error(
            "stair command rejected: stair_executor owns connector motion"
        )
        self._publish_stair_status(
            "error reason=stair_execution_retired command=%s"
            % command.strip()
        )

    def _publish_stair_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.stair_status_pub.publish(msg)
        self.get_logger().info("stair status: %s" % status)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = FloorManager()
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
