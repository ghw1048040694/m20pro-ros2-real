import json
import math
import os
import time
from functools import partial
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .geometry import quaternion_to_yaw, yaw_to_quaternion
from .terrain_segment_contract import (
    terrain_entry_gait,
    terrain_segment_at_pose,
    terrain_segments_from_config,
)


class FloorManager(Node):
    """Switch Nav2 floor maps through configured stair-platform transition points."""

    def __init__(self) -> None:
        super().__init__("m20pro_floor_manager")
        self.declare_parameter("config_file", "")
        self.declare_parameter("initial_floor", "")
        self.declare_parameter("load_initial_floor", False)
        self.declare_parameter("switch_floor_topic", "/m20pro/switch_floor")
        self.declare_parameter("stair_command_topic", "/m20pro/use_stairs")
        self.declare_parameter("floor_goal_topic", "/m20pro/floor_goal")
        self.declare_parameter("stop_task_topic", "/m20pro/stop_task")
        self.declare_parameter("enable_rviz_floor_goal_topics", True)
        self.declare_parameter(
            "rviz_floor_goal_topics",
            [],
        )
        self.declare_parameter("current_floor_topic", "/m20pro/current_floor")
        self.declare_parameter("stair_status_topic", "/m20pro/stair_status")
        self.declare_parameter("gait_command_topic", "/m20pro/gait_command")
        self.declare_parameter("stair_zones_topic", "/m20pro/stair_zones")
        self.declare_parameter("floor_route_config_topic", "/m20pro/floor_route_config")
        self.declare_parameter("floor_switch_request_topic", "/m20pro/floor_switch_request")
        self.declare_parameter("floor_switch_result_topic", "/m20pro/floor_switch_result")
        self.declare_parameter("floor_context_topic", "/m20pro/set_current_floor")
        # These defaults preserve the field-tested transition behavior.  The
        # shared profile can override them, but metadata/hash must never be a
        # runtime startup gate for the legacy manager.
        self.declare_parameter("floor_switch_timeout_s", 110.0)
        self.declare_parameter("field_profile_name", "")
        self.declare_parameter("field_profile_hash", "")
        self.declare_parameter(
            "stair_behavior_tree",
            "package://m20pro_bringup/behavior_trees/m20pro_stair_traverse_foxy.xml",
        )
        self.declare_parameter("robot_pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("map_server_load_map_service", "/map_server/load_map")
        self.declare_parameter(
            "clear_costmap_services",
            [
                "/global_costmap/clear_entirely_global_costmap",
                "/local_costmap/clear_entirely_local_costmap",
            ],
        )
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("publish_initialpose_repeats", 3)
        self.declare_parameter("initialpose_covariance_xy", 0.25)
        self.declare_parameter("initialpose_covariance_yaw", 0.0685)
        self.declare_parameter("require_near_stair_entry", True)
        self.declare_parameter("stair_entry_tolerance_m", 0.80)
        self.declare_parameter("flat_gait_label", "flat")
        self.declare_parameter("publish_flat_gait_before_nav", True)
        self.declare_parameter("stair_up_gait_label", "stair_up")
        self.declare_parameter("stair_down_gait_label", "stair_down")
        self.declare_parameter("post_switch_goal_delay_s", 1.0)
        self.declare_parameter("duplicate_goal_tolerance_m", 0.08)
        self.declare_parameter("duplicate_goal_yaw_tolerance_rad", 0.12)
        self.declare_parameter("nav_feedback_status_period_s", 1.0)

        self.field_profile_name = str(self.get_parameter("field_profile_name").value).strip()
        self.field_profile_hash = str(self.get_parameter("field_profile_hash").value).strip()

        self.config_file = self._resolve_path(str(self.get_parameter("config_file").value))
        self.config = self._load_config(self.config_file)
        self.floors: Dict[str, Dict[str, Any]] = dict(self.config.get("floors", {}))
        self.route_configured = self._has_stair_routes(self.floors)

        self.current_floor = ""
        self.pending_floor: Optional[str] = None
        self.pending_pose_override: Optional[Dict[str, Any]] = None
        self.pending_flat_gait_after_switch = False
        self.pending_stair_transition: Optional[Dict[str, Any]] = None
        self.pending_floor_goal: Optional[PoseStamped] = None
        self.pending_stair_after_nav: Optional[Dict[str, Any]] = None
        self.pending_post_switch_goal: Optional[PoseStamped] = None
        self.pending_post_switch_goal_floor: Optional[str] = None
        self.pending_post_exit_pose: Optional[Dict[str, Any]] = None
        self.pending_floor_switch: Optional[Dict[str, Any]] = None
        self.active_floor_mission = False
        self.active_nav_goal_handle: Optional[Any] = None
        self.active_nav_goal_label = ""
        self.active_nav_goal_pose: Optional[PoseStamped] = None
        self.active_nav_goal_sequence = 0
        self.nav_goal_sequence = 0
        self.stop_nav_goal_sequence = 0
        self.last_nav_feedback_publish_monotonic = 0.0
        self.cancelled_nav_goal_handle_ids: Set[int] = set()
        self.stair_timer = None
        self.post_exit_goal_timer = None
        self.post_switch_goal_timer = None
        self.robot_pose: Optional[PoseStamped] = None
        self.stair_zones_by_floor: Dict[str, List[Dict[str, Any]]] = {}
        self.terrain_segments_by_floor = terrain_segments_from_config(self.config)
        self.active_terrain_segment: Optional[Dict[str, Any]] = None
        self.initialpose_repeats_left = 0
        self.pending_initialpose: Optional[PoseWithCovarianceStamped] = None
        self.initial_floor_timer = None
        self.floor_switch_request_started_monotonic = 0.0
        self.rviz_floor_goal_subscriptions: List[Any] = []
        self._validate_floor_config()

        self.load_map_client = self.create_client(
            LoadMap,
            str(self.get_parameter("map_server_load_map_service").value),
        )
        self.navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value),
        )
        self.clear_costmap_clients = [
            self.create_client(ClearEntireCostmap, str(service_name))
            for service_name in self.get_parameter("clear_costmap_services").value
        ]

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            10,
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
        self.gait_command_pub = self.create_publisher(
            String,
            str(self.get_parameter("gait_command_topic").value),
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.floor_switch_request_pub = self.create_publisher(
            String,
            str(self.get_parameter("floor_switch_request_topic").value),
            10,
        )
        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            str(self.get_parameter("floor_route_config_topic").value),
            self._on_floor_route_config,
            route_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("floor_switch_result_topic").value),
            self._on_floor_switch_result,
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
            str(self.get_parameter("switch_floor_topic").value),
            self._on_switch_floor,
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
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("robot_pose_topic").value),
            self._on_robot_pose,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stair_zones_topic").value),
            self._on_stair_zones,
            10,
        )
        self.create_timer(0.2, self._republish_initialpose)
        self.create_timer(1.0, self._publish_current_floor)
        self.create_timer(0.5, self._check_floor_switch_timeout)

        initial_floor = str(self.get_parameter("initial_floor").value).strip()
        if initial_floor:
            if bool(self.get_parameter("load_initial_floor").value) and self.route_configured:
                self.initial_floor_timer = self.create_timer(
                    1.0,
                    lambda: self._switch_initial_floor(initial_floor),
                )
            elif not self.route_configured or initial_floor in self.floors:
                self.current_floor = initial_floor
                self.get_logger().info(
                    "assuming runtime map label %s without reloading map" % initial_floor
                )
            else:
                self.get_logger().warning("unknown initial_floor: %s" % initial_floor)

        self.get_logger().info(
            "floor manager ready; field_profile=%s hash=%s route_configured=%s floors: %s"
            % (
                self.field_profile_name,
                self.field_profile_hash,
                self.route_configured,
                ", ".join(sorted(self.floors.keys())) or "(ordinary maps)",
            )
        )

    def _create_rviz_floor_goal_subscriptions(self) -> None:
        routes = self._parse_floor_goal_routes(
            list(self.get_parameter("rviz_floor_goal_topics").value),
        )
        if not routes:
            self.get_logger().info("rviz floor goal topics disabled; ordinary map mode")
            return
        for floor_id, topic in routes:
            self.rviz_floor_goal_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    partial(self._on_rviz_floor_goal, floor_id=floor_id, topic=topic),
                    10,
                )
            )

        route_text = ", ".join("%s<=%s" % (floor, topic) for floor, topic in routes)
        self.get_logger().info("rviz floor goal topics enabled: %s" % route_text)

    def _parse_floor_goal_routes(self, values: List[str]) -> List[Tuple[str, str]]:
        routes: List[Tuple[str, str]] = []
        for value in values:
            floor_id, sep, topic = str(value).partition(":")
            floor_id = floor_id.strip()
            topic = topic.strip()
            if not sep or not floor_id or not topic.startswith("/"):
                self.get_logger().warning(
                    "ignored invalid rviz_floor_goal_topics entry: %s" % value
                )
                continue
            routes.append((floor_id, topic))
        return routes

    def _switch_initial_floor(self, initial_floor: str) -> None:
        if self.initial_floor_timer is not None:
            self.initial_floor_timer.cancel()
            self.initial_floor_timer = None
        self.switch_floor(initial_floor)

    def _load_config(self, config_file: str) -> Dict[str, Any]:
        if not config_file:
            raise RuntimeError("config_file parameter is required")
        with open(config_file, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}
        if not isinstance(data, dict):
            raise RuntimeError("floor manager config must be a YAML mapping")
        return data

    @staticmethod
    def _has_stair_routes(floors: Dict[str, Dict[str, Any]]) -> bool:
        return any(
            isinstance(floor, dict)
            and isinstance(floor.get("stairs"), dict)
            and any(isinstance(route, dict) and route.get("target_floor") for route in floor["stairs"].values())
            for floor in floors.values()
        )

    def _on_floor_route_config(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            config = payload.get("config") if isinstance(payload, dict) else None
        except Exception as exc:
            self.get_logger().warning("ignored invalid runtime floor route config: %s" % exc)
            return
        if not isinstance(config, dict) or not isinstance(config.get("floors"), dict):
            self.get_logger().warning("ignored runtime floor route config without floors")
            return
        if self.active_floor_mission:
            self.get_logger().warning("ignored runtime floor route update while a floor mission is active")
            return
        self.config = dict(config)
        self.floors = dict(config.get("floors") or {})
        self.route_configured = self._has_stair_routes(self.floors)
        self.terrain_segments_by_floor = terrain_segments_from_config(self.config)
        self._validate_floor_config()
        self.get_logger().info(
            "runtime floor routes updated; configured=%s floors=%s"
            % (self.route_configured, ",".join(sorted(self.floors)) or "(none)")
        )

    def _validate_floor_config(self) -> None:
        for floor_id, floor in self.floors.items():
            if not isinstance(floor, dict):
                self.get_logger().warning("floor %s config is not a mapping" % floor_id)
                continue
            stairs = floor.get("stairs") or {}
            if not isinstance(stairs, dict):
                self.get_logger().warning("floor %s stairs config is not a mapping" % floor_id)
                continue
            for stair_name, stair in stairs.items():
                if not isinstance(stair, dict):
                    continue
                target_floor = self._stair_target_floor(stair_name, stair)
                if not target_floor:
                    continue
                if target_floor not in self.floors:
                    self.get_logger().warning(
                        "stair route %s/%s targets unknown floor %s"
                        % (floor_id, stair_name, target_floor)
                    )
                if not self._resolve_stair_entry_pose(stair):
                    self.get_logger().warning(
                        "stair route %s/%s has no entry pose" % (floor_id, stair_name)
                    )
                metadata = self._resolve_stair_transition_metadata(stair)
                if not self._resolve_stair_traverse_pose(stair):
                    self.get_logger().warning(
                        "stair route %s/%s has no source platform pose"
                        % (floor_id, stair_name)
                    )
                if not self._resolve_stair_exit_pose(stair, target_floor):
                    self.get_logger().warning(
                        "stair route %s/%s has no target platform/exit pose"
                        % (floor_id, stair_name)
                    )
                if not self._resolve_stair_post_exit_pose(stair):
                    self.get_logger().warning(
                        "stair route %s/%s has no post_exit pose; flat gait will be restored immediately after map switch"
                        % (floor_id, stair_name)
                    )
                if metadata["model"] != "shared_platform":
                    self.get_logger().warning(
                        "stair route %s/%s uses transition model %s; current M20Pro logic is tuned for shared_platform"
                        % (floor_id, stair_name, metadata["model"])
                    )
                margin = metadata["entry_margin_m"]
                if 0.0 < margin < 0.8:
                    self.get_logger().warning(
                        "stair route %s/%s entry_margin_m %.2f is below the X30 recommended 0.8m"
                        % (floor_id, stair_name, margin)
                    )
            for segment in self.terrain_segments_by_floor.get(str(floor_id), []):
                if not segment.get("configured"):
                    self.get_logger().warning(
                        "terrain segment %s is invalid: %s"
                        % (segment.get("id"), segment.get("error"))
                    )

    def _resolve_path(self, value: str) -> str:
        path = os.path.expandvars(os.path.expanduser(value))
        if path.startswith("package://"):
            package_and_path = path[len("package://") :]
            package_name, _, relative_path = package_and_path.partition("/")
            if not package_name or not relative_path:
                raise RuntimeError("invalid package path: %s" % value)
            return os.path.join(get_package_share_directory(package_name), relative_path)
        if os.path.isabs(path):
            return path
        if path:
            return str((Path.cwd() / path).resolve())
        return path

    def _resolve_config_relative_path(self, value: str) -> str:
        path = os.path.expandvars(os.path.expanduser(value))
        if path.startswith("package://"):
            return self._resolve_path(path)
        if os.path.isabs(path):
            return path
        return str((Path(self.config_file).parent / path).resolve())

    def _on_switch_floor(self, msg: String) -> None:
        target_floor = msg.data.strip()
        if not target_floor:
            self.get_logger().warning("ignored empty floor switch request")
            return
        self.switch_floor(target_floor)

    def _on_floor_context(self, msg: String) -> None:
        floor = str(msg.data or "").strip()
        if not floor or self.active_floor_mission or self.pending_floor_switch is not None:
            return
        if self.route_configured and floor not in self.floors:
            self.get_logger().warning("ignored floor context outside configured route profile: %s" % floor)
            return
        self.current_floor = floor
        self._publish_current_floor()
        self.get_logger().info("current floor synchronized from selected map: %s" % floor)

    def _on_stair_command(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            self.get_logger().warning("ignored empty stair command")
            return
        self.use_stairs(command)

    def _on_floor_switch_result(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning("ignored invalid floor switch result: %s" % exc)
            return
        pending = self.pending_floor_switch
        if not isinstance(result, dict) or pending is None:
            return
        if str(result.get("request_id") or "") != str(pending.get("request_id") or ""):
            return
        self.pending_floor_switch = None
        self.floor_switch_request_started_monotonic = 0.0
        if not result.get("ok"):
            self.get_logger().error("coordinated floor switch failed: %s" % result.get("message"))
            self._publish_zero_cmd()
            if bool(result.get("state_uncertain")):
                self.current_floor = ""
                self._publish_current_floor()
            self._publish_stair_status(
                "error reason=coordinated_floor_switch_failed source_floor=%s target_floor=%s code=%s"
                % (
                    pending.get("source_floor"),
                    pending.get("target_floor"),
                    result.get("code") or "unknown",
                )
            )
            self._clear_floor_mission_if_needed("floor_goal")
            return
        target_floor = str(pending.get("target_floor") or "")
        self.current_floor = target_floor
        self.pending_flat_gait_after_switch = True
        self.pending_post_exit_pose = dict(pending.get("post_exit") or {})
        self._publish_current_floor()
        self._publish_stair_status(
            "coordinated_floor_switch_confirmed target_floor=%s map_id=%s"
            % (target_floor, result.get("target_map_id") or pending.get("target_map_id") or "")
        )
        if self.pending_post_exit_pose:
            self._schedule_post_exit_goal()
        else:
            self.pending_flat_gait_after_switch = False
            self._publish_gait(str(self.get_parameter("flat_gait_label").value))
            self._publish_stair_status("complete target_floor=%s" % target_floor)
            if self.pending_post_switch_goal is not None:
                self._schedule_post_switch_goal()

    def _check_floor_switch_timeout(self) -> None:
        if self.pending_floor_switch is None or self.floor_switch_request_started_monotonic <= 0.0:
            return
        timeout_s = max(5.0, float(self.get_parameter("floor_switch_timeout_s").value))
        if time.monotonic() - self.floor_switch_request_started_monotonic <= timeout_s:
            return
        pending = dict(self.pending_floor_switch)
        self.pending_floor_switch = None
        self.floor_switch_request_started_monotonic = 0.0
        self._publish_zero_cmd()
        self._publish_stair_status(
            "error reason=coordinated_floor_switch_timeout source_floor=%s target_floor=%s"
            % (pending.get("source_floor"), pending.get("target_floor"))
        )
        self._clear_floor_mission_if_needed("floor_goal")

    def _on_stop_task(self, msg: String) -> None:
        reason = msg.data.strip() or "stop_task"
        self.get_logger().warning("received stop task request: %s" % reason)
        self._publish_stair_status("stopped reason=%s" % reason)
        self._clear_floor_mission_if_needed("floor_goal")
        self._cancel_active_nav_goal(reason)

    def _on_floor_goal(self, msg: PoseStamped) -> None:
        target_floor = self._normalize_floor_id(msg.header.frame_id)
        if not target_floor:
            target_floor = self.current_floor
        if self.route_configured and not target_floor:
            self.get_logger().error("floor goal ignored; current floor is unknown")
            self._publish_stair_status("error reason=no_current_floor_for_goal label=floor_goal")
            return
        if self.route_configured and target_floor not in self.floors:
            self.get_logger().error("floor goal has unknown target floor: %s" % target_floor)
            self._publish_stair_status(
                "error reason=unknown_goal_floor floor=%s label=floor_goal" % target_floor
            )
            return
        if not self.route_configured and target_floor and self.current_floor and target_floor != self.current_floor:
            self.get_logger().error(
                "ordinary map mode cannot switch maps from a floor goal; select the target map first"
            )
            self._publish_stair_status(
                "error reason=ordinary_map_floor_mismatch current=%s target=%s label=floor_goal"
                % (self.current_floor, target_floor)
            )
            return
        if not self.route_configured and target_floor and not self.current_floor:
            self.current_floor = target_floor

        goal = self._pose_in_map_frame(msg)

        if self.active_floor_mission:
            if self._is_duplicate_active_floor_goal(target_floor, goal):
                self.get_logger().info("duplicate active floor goal ignored")
                self._publish_stair_status("ignored reason=duplicate_floor_goal target_floor=%s" % target_floor)
                return
            if self._can_replace_active_floor_goal(target_floor):
                self.get_logger().warning("replacing active same-floor goal with a new floor goal")
                self._publish_stair_status("replacing_active_floor_goal target_floor=%s" % target_floor)
                self._cancel_active_nav_goal("replace_floor_goal")
            else:
                self.get_logger().warning("floor goal ignored; another floor mission is active")
                self._publish_stair_status("ignored reason=floor_mission_active")
                return

        self.get_logger().info(
            "received floor goal target_floor=%s x=%.2f y=%.2f"
            % (target_floor, goal.pose.position.x, goal.pose.position.y)
        )

        if target_floor == self.current_floor:
            self.active_floor_mission = True
            self._publish_stair_status("same_floor_goal target_floor=%s" % target_floor)
            self._publish_flat_gait("same_floor_goal")
            self._send_nav_goal(goal, "floor_goal")
            return

        self.get_logger().error(
            "cross-floor goal rejected: the retired stair execution chain has been removed"
        )
        self._publish_stair_status(
            "error reason=stair_execution_retired current=%s target=%s label=floor_goal"
            % (self.current_floor, target_floor)
        )

    def _on_rviz_floor_goal(self, msg: PoseStamped, floor_id: str, topic: str) -> None:
        goal = PoseStamped()
        goal.header = msg.header
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = floor_id
        goal.pose = msg.pose
        self.get_logger().info(
            "rviz goal from %s -> floor=%s x=%.2f y=%.2f"
            % (topic, floor_id, goal.pose.position.x, goal.pose.position.y)
        )
        self._on_floor_goal(goal)

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        self.robot_pose = msg
        self._update_terrain_segment_gait(msg)

    def _update_terrain_segment_gait(self, pose: PoseStamped) -> None:
        if not self.current_floor:
            return
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        segment = terrain_segment_at_pose(self.terrain_segments_by_floor, self.current_floor, x, y)
        active_id = str((self.active_terrain_segment or {}).get("id") or "")
        segment_id = str((segment or {}).get("id") or "")
        if active_id == segment_id:
            return
        self._leave_active_terrain_segment("pose_exit")
        if segment is None:
            return
        gait, direction = terrain_entry_gait(segment, x, y)
        self.active_terrain_segment = dict(segment)
        self._publish_gait(gait)
        self._publish_stair_status(
            "terrain_enter floor=%s segment=%s terrain=%s direction=%s gait=%s"
            % (self.current_floor, segment.get("name"), segment.get("terrain"), direction, gait)
        )

    def _leave_active_terrain_segment(self, reason: str) -> None:
        if self.active_terrain_segment is None:
            return
        previous = self.active_terrain_segment
        self.active_terrain_segment = None
        exit_gait = str(previous.get("exit_gait") or self.get_parameter("flat_gait_label").value)
        self._publish_gait(exit_gait)
        self._publish_stair_status(
            "terrain_exit floor=%s segment=%s terrain=%s gait=%s reason=%s"
            % (self.current_floor, previous.get("name"), previous.get("terrain"), exit_gait, reason)
        )

    def _on_stair_zones(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warning("ignored invalid stair zones JSON: %s" % exc)
            return
        if not isinstance(payload, dict):
            return
        zones = payload.get("zones") or []
        if not isinstance(zones, list):
            return
        payload_floor = str(payload.get("floor") or "").strip()
        if payload_floor and not zones:
            self.stair_zones_by_floor[payload_floor] = []
            self.get_logger().debug("cleared stair zones for floor %s" % payload_floor)
            return
        grouped: Dict[str, List[Dict[str, Any]]] = {}
        for zone in zones:
            if not isinstance(zone, dict):
                continue
            if not bool(zone.get("trigger_gait", False)):
                continue
            floor = str(zone.get("source_floor") or zone.get("floor") or "").strip()
            if not floor:
                continue
            grouped.setdefault(floor, []).append(zone)
        if not grouped:
            return
        self.stair_zones_by_floor.update(grouped)
        self.get_logger().debug(
            "updated stair zones: %s"
            % ", ".join("%s=%d" % (floor, len(items)) for floor, items in grouped.items())
        )

    def switch_floor(
        self,
        target_floor: str,
        pose_override: Optional[Dict[str, Any]] = None,
    ) -> None:
        if self.pending_floor:
            self.get_logger().warning(
                "floor switch to %s ignored; %s is still pending"
                % (target_floor, self.pending_floor)
            )
            return
        if target_floor == self.current_floor:
            self.get_logger().info("already on floor %s" % target_floor)
            return
        if not self.route_configured:
            self.get_logger().error(
                "floor switch rejected: no cross-floor route profile is configured; select an ordinary map instead"
            )
            self._publish_stair_status("error reason=no_cross_floor_route_profile")
            return
        floor = self.floors.get(target_floor)
        if floor is None:
            self.get_logger().error(
                "unknown floor %s; known floors: %s"
                % (target_floor, ", ".join(sorted(self.floors.keys())))
            )
            return

        map_yaml = str(floor.get("map_yaml", "")).strip()
        if not map_yaml:
            self.get_logger().error("floor %s has empty map_yaml" % target_floor)
            return
        map_yaml = self._resolve_config_relative_path(map_yaml)
        if not os.path.exists(map_yaml):
            self.get_logger().error("floor %s map does not exist: %s" % (target_floor, map_yaml))
            return

        if not self._wait_for_service(self.load_map_client):
            self.get_logger().error("map server load_map service is not available")
            return

        self.pending_floor = target_floor
        self.pending_pose_override = pose_override
        request = LoadMap.Request()
        request.map_url = map_yaml
        self.get_logger().info("loading floor %s map: %s" % (target_floor, map_yaml))
        future = self.load_map_client.call_async(request)
        future.add_done_callback(lambda done: self._on_load_map_done(done, target_floor, floor))

    def _on_load_map_done(self, future: Any, target_floor: str, floor: Dict[str, Any]) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.pending_floor = None
            self.get_logger().error("load_map call failed: %s" % exc)
            return

        result = int(getattr(response, "result", 0))
        if result != 0:
            self.pending_floor = None
            self.get_logger().error("load_map returned result code %d" % result)
            return

        self._leave_active_terrain_segment("floor_switch")
        self.current_floor = target_floor
        self.pending_floor = None
        pose_override = self.pending_pose_override
        self.pending_pose_override = None
        self.get_logger().info("floor %s map loaded" % target_floor)
        self._clear_costmaps()
        self._publish_initialpose_for_floor(target_floor, floor, pose_override)
        self._publish_current_floor()
        if self.pending_flat_gait_after_switch:
            if self.pending_post_exit_pose:
                self._publish_stair_status(
                    "navigating_from_platform_to_flat target_floor=%s" % target_floor
                )
                self._schedule_post_exit_goal()
                return
            self.pending_flat_gait_after_switch = False
            self._publish_gait(str(self.get_parameter("flat_gait_label").value))
            self._publish_stair_status("complete target_floor=%s" % target_floor)
        if self.pending_post_switch_goal is not None and self.current_floor == target_floor:
            self._schedule_post_switch_goal()

    def _wait_for_service(self, client: Any) -> bool:
        timeout_s = float(self.get_parameter("service_timeout_s").value)
        return client.wait_for_service(timeout_sec=timeout_s)

    def _clear_costmaps(self) -> None:
        for client in self.clear_costmap_clients:
            if not self._wait_for_service(client):
                self.get_logger().warning(
                    "costmap clear service unavailable: %s" % client.srv_name
                )
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            future.add_done_callback(
                lambda done, service_name=client.srv_name: self._on_clear_costmap_done(
                    done,
                    service_name,
                )
            )

    def _on_clear_costmap_done(self, future: Any, service_name: str) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warning("costmap clear failed for %s: %s" % (service_name, exc))

    def _publish_initialpose_for_floor(
        self,
        target_floor: str,
        floor: Dict[str, Any],
        pose_override: Optional[Dict[str, Any]] = None,
    ) -> None:
        pose_data = pose_override or floor.get("initial_pose") or floor.get("entry_pose") or {}
        if not pose_data:
            self.get_logger().warning(
                "floor %s has no initial_pose; map switched without pose reset" % target_floor
            )
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = str(self.config.get("mission", {}).get("frame_id", "map"))
        msg.pose.pose.position.x = float(pose_data.get("x", 0.0))
        msg.pose.pose.position.y = float(pose_data.get("y", 0.0))
        msg.pose.pose.position.z = float(pose_data.get("z", 0.0))
        msg.pose.pose.orientation = yaw_to_quaternion(float(pose_data.get("yaw", 0.0)))
        xy_cov = float(self.get_parameter("initialpose_covariance_xy").value)
        yaw_cov = float(self.get_parameter("initialpose_covariance_yaw").value)
        msg.pose.covariance[0] = xy_cov
        msg.pose.covariance[7] = xy_cov
        msg.pose.covariance[35] = yaw_cov

        self.pending_initialpose = msg
        self.initialpose_repeats_left = max(
            1,
            int(self.get_parameter("publish_initialpose_repeats").value),
        )
        self.get_logger().info(
            "resetting pose for floor %s to x=%.2f y=%.2f z=%.2f yaw=%.2f"
            % (
                target_floor,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                float(pose_data.get("yaw", 0.0)),
            )
        )

    def _republish_initialpose(self) -> None:
        if self.pending_initialpose is None or self.initialpose_repeats_left <= 0:
            return
        self.pending_initialpose.header.stamp = self.get_clock().now().to_msg()
        self.initialpose_pub.publish(self.pending_initialpose)
        self.initialpose_repeats_left -= 1
        if self.initialpose_repeats_left <= 0:
            self.pending_initialpose = None

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
            self.get_logger().error("navigate_to_pose action server is not available")
            self._publish_stair_status("error reason=navigate_action_unavailable label=%s" % label)
            self._clear_floor_mission_if_needed(label)
            return

        pose.header.frame_id = str(self.config.get("mission", {}).get("frame_id", "map"))
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        if label in ("stair_traverse", "stair_exit"):
            behavior_tree = self._resolve_path(
                str(self.get_parameter("stair_behavior_tree").value)
            )
            if not behavior_tree or not os.path.isfile(behavior_tree):
                self.get_logger().error("stair behavior tree is unavailable: %s" % behavior_tree)
                self._publish_stair_status("error reason=stair_behavior_tree_missing")
                self._clear_floor_mission_if_needed(label)
                return
            goal.behavior_tree = behavior_tree
        self.get_logger().info(
            "sending Nav2 goal label=%s x=%.2f y=%.2f"
            % (label, pose.pose.position.x, pose.pose.position.y)
        )
        self.nav_goal_sequence += 1
        goal_sequence = self.nav_goal_sequence
        future = self.navigate_to_pose_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_nav_feedback(feedback, label, goal_sequence),
        )
        future.add_done_callback(
            lambda done: self._on_nav_goal_response(done, label, goal_sequence, pose)
        )

    def _on_nav_goal_response(
        self,
        future: Any,
        label: str,
        goal_sequence: int,
        pose: PoseStamped,
    ) -> None:
        stale_after_stop = goal_sequence <= self.stop_nav_goal_sequence
        status_suffix = self._goal_status_suffix(label, goal_sequence, pose)
        try:
            goal_handle = future.result()
        except Exception as exc:
            if stale_after_stop:
                self.get_logger().warning(
                    "ignored stale Nav2 goal request failure after stop for %s: %s" % (label, exc)
                )
                self._publish_stair_status("ignored reason=stale_nav_goal_request %s" % status_suffix)
                return
            self.get_logger().error("Nav2 goal request failed for %s: %s" % (label, exc))
            self._publish_stair_status("error reason=nav_goal_request_failed %s" % status_suffix)
            self._clear_floor_mission_if_needed(label)
            return
        if not goal_handle.accepted:
            if stale_after_stop:
                self.get_logger().warning("ignored stale Nav2 goal rejection after stop: %s" % label)
                self._publish_stair_status("ignored reason=stale_nav_goal_rejected %s" % status_suffix)
                return
            self.get_logger().error("Nav2 goal rejected: %s" % label)
            self._publish_stair_status("error reason=nav_goal_rejected %s" % status_suffix)
            self._clear_floor_mission_if_needed(label)
            return
        if stale_after_stop:
            self.get_logger().warning("cancelling stale Nav2 goal after stop request: %s" % label)
            self.cancelled_nav_goal_handle_ids.add(id(goal_handle))
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done: self._on_nav_result(done, label, goal_handle, goal_sequence, pose)
            )
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda done: self._on_cancel_done(done, label, "stale_goal_after_stop")
                )
            except Exception as exc:
                self.get_logger().error("failed to cancel stale Nav2 goal %s: %s" % (label, exc))
                self._publish_stair_status("error reason=nav_cancel_failed label=%s" % label)
            self._publish_zero_cmd()
            return
        self.active_nav_goal_handle = goal_handle
        self.active_nav_goal_label = label
        self.active_nav_goal_pose = pose
        self.active_nav_goal_sequence = goal_sequence
        self._publish_stair_status("nav_goal_accepted %s" % status_suffix)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._on_nav_result(done, label, goal_handle, goal_sequence, pose)
        )

    def _on_nav_feedback(self, feedback_msg: Any, label: str, goal_sequence: int) -> None:
        if goal_sequence <= self.stop_nav_goal_sequence:
            return
        if self.active_nav_goal_sequence and self.active_nav_goal_sequence != goal_sequence:
            return
        if self.active_nav_goal_label and self.active_nav_goal_label != label:
            return
        now_monotonic = time.monotonic()
        period_s = max(0.2, float(self.get_parameter("nav_feedback_status_period_s").value))
        if now_monotonic - self.last_nav_feedback_publish_monotonic < period_s:
            return
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return
        self.last_nav_feedback_publish_monotonic = now_monotonic
        pose_text = self._nav_feedback_pose_text(getattr(feedback, "current_pose", None))
        status_suffix = self._goal_status_suffix(label, goal_sequence, self.active_nav_goal_pose)
        self._publish_stair_status(
            "nav_goal_feedback %s distance_remaining=%.3f navigation_time=%.2f "
            "estimated_time_remaining=%.2f recoveries=%d%s"
            % (
                status_suffix,
                float(getattr(feedback, "distance_remaining", 0.0)),
                self._duration_to_sec(getattr(feedback, "navigation_time", None)),
                self._duration_to_sec(getattr(feedback, "estimated_time_remaining", None)),
                int(getattr(feedback, "number_of_recoveries", 0) or 0),
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
        status_suffix = self._goal_status_suffix(label, goal_sequence, pose)
        try:
            result = future.result()
        except Exception as exc:
            if not was_current_goal:
                self.get_logger().warning(
                    "ignored stale Nav2 result failure label=%s: %s" % (label, exc)
                )
                self._publish_stair_status("ignored reason=stale_nav_result_failed %s" % status_suffix)
                return
            self.get_logger().error("Nav2 result failed for %s: %s" % (label, exc))
            self._clear_active_nav_goal(goal_handle)
            self._publish_stair_status("error reason=nav_result_failed %s" % status_suffix)
            self._clear_floor_mission_if_needed(label)
            return

        status = int(getattr(result, "status", GoalStatus.STATUS_UNKNOWN))
        was_cancel_requested = id(goal_handle) in self.cancelled_nav_goal_handle_ids
        self.cancelled_nav_goal_handle_ids.discard(id(goal_handle))
        self._clear_active_nav_goal(goal_handle)
        if status == GoalStatus.STATUS_CANCELED and was_cancel_requested:
            self.get_logger().warning("Nav2 goal %s finished as cancelled" % label)
            self._publish_stair_status("nav_goal_cancelled_result %s" % status_suffix)
            if was_current_goal:
                self._clear_floor_mission_if_needed(label)
            else:
                self.get_logger().info("ignored stale cancelled Nav2 result for %s" % label)
            return
        if not was_current_goal:
            self.get_logger().warning(
                "ignored stale Nav2 result label=%s status=%d" % (label, status)
            )
            self._publish_stair_status("ignored reason=stale_nav_result %s status=%d" % (status_suffix, status))
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning("Nav2 goal %s finished with status %d" % (label, status))
            self._publish_stair_status(
                "error reason=nav_goal_failed %s status=%d" % (status_suffix, status)
            )
            self._clear_floor_mission_if_needed(label)
            return

        self._publish_stair_status("nav_goal_succeeded %s" % status_suffix)
        if label == "stair_entry":
            self._start_stair_after_entry()
        elif label == "stair_traverse":
            self._finish_pending_stair_transition()
        elif label == "stair_exit":
            self.pending_flat_gait_after_switch = False
            self.pending_post_exit_pose = None
            self._publish_gait(str(self.get_parameter("flat_gait_label").value))
            self._publish_stair_status("complete target_floor=%s" % self.current_floor)
            if self.pending_post_switch_goal is not None:
                self._schedule_post_switch_goal()
            else:
                self.active_floor_mission = False
        elif label == "floor_goal":
            self.active_floor_mission = False
            self.pending_post_switch_goal_floor = None

    def _can_replace_active_floor_goal(self, target_floor: str) -> bool:
        if target_floor != self.current_floor:
            return False
        if self.active_nav_goal_label != "floor_goal":
            return False
        return (
            self.pending_floor is None
            and self.pending_stair_transition is None
            and self.pending_floor_goal is None
            and self.pending_stair_after_nav is None
            and self.pending_post_switch_goal is None
            and self.pending_post_switch_goal_floor is None
            and self.pending_post_exit_pose is None
            and not self.pending_flat_gait_after_switch
            and self.stair_timer is None
            and self.post_exit_goal_timer is None
            and self.post_switch_goal_timer is None
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
            self.get_logger().info("no active Nav2 goal to cancel for stop request")
            return
        self.cancelled_nav_goal_handle_ids.add(id(goal_handle))
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(lambda done: self._on_cancel_done(done, label, reason))
        except Exception as exc:
            self.get_logger().error("failed to cancel Nav2 goal %s: %s" % (label, exc))
            self._publish_stair_status("error reason=nav_cancel_failed label=%s" % label)

    def _on_cancel_done(self, future: Any, label: str, reason: str) -> None:
        try:
            result = future.result()
            count = len(getattr(result, "goals_canceling", []) or [])
        except Exception as exc:
            self.get_logger().error("Nav2 cancel result failed for %s: %s" % (label, exc))
            self._publish_stair_status("error reason=nav_cancel_result_failed label=%s" % label)
            return
        self._publish_zero_cmd()
        self.get_logger().warning(
            "cancelled Nav2 goal label=%s count=%d reason=%s" % (label, count, reason)
        )
        self._publish_stair_status("nav_goal_cancelled label=%s reason=%s count=%d" % (label, reason, count))

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
        return label in ("stair_entry", "stair_traverse", "stair_exit", "floor_goal")

    def _is_duplicate_active_floor_goal(self, target_floor: str, goal: PoseStamped) -> bool:
        if target_floor != self.current_floor:
            return False
        if self.active_nav_goal_label != "floor_goal" or self.active_nav_goal_pose is None:
            return False
        active_pose = self.active_nav_goal_pose.pose
        dx = active_pose.position.x - goal.pose.position.x
        dy = active_pose.position.y - goal.pose.position.y
        distance = math.hypot(dx, dy)
        tolerance = max(0.0, float(self.get_parameter("duplicate_goal_tolerance_m").value))
        if distance > tolerance:
            return False
        active_yaw = quaternion_to_yaw(active_pose.orientation)
        goal_yaw = quaternion_to_yaw(goal.pose.orientation)
        yaw_error = abs(self._wrap_angle(active_yaw - goal_yaw))
        yaw_tolerance = max(0.0, float(self.get_parameter("duplicate_goal_yaw_tolerance_rad").value))
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
            return float(getattr(value, "sec", 0)) + float(getattr(value, "nanosec", 0)) * 1e-9
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
    def _goal_status_suffix(label: str, goal_sequence: int, pose: Optional[PoseStamped]) -> str:
        parts = ["label=%s" % label, "goal_seq=%d" % int(goal_sequence)]
        if pose is None:
            return " ".join(parts)
        try:
            parts.extend(
                [
                    "goal_frame=%s" % (str(pose.header.frame_id or "map").replace(" ", "_")),
                    "goal_x=%.3f" % float(pose.pose.position.x),
                    "goal_y=%.3f" % float(pose.pose.position.y),
                    "goal_z=%.3f" % float(pose.pose.position.z),
                    "goal_yaw=%.4f" % quaternion_to_yaw(pose.pose.orientation),
                ]
            )
        except (TypeError, ValueError):
            pass
        return " ".join(parts)

    def _start_stair_after_entry(self) -> None:
        route = self.pending_stair_after_nav
        goal = self.pending_floor_goal
        self.pending_stair_after_nav = None
        self.pending_floor_goal = None
        if route is None or goal is None:
            self.get_logger().error("missing pending floor goal after stair entry")
            self._publish_stair_status("error reason=missing_pending_floor_goal")
            self.active_floor_mission = False
            return
        self.pending_post_switch_goal = goal
        self.use_stairs(str(route["target_floor"]))

    def _schedule_post_switch_goal(self) -> None:
        if self.post_switch_goal_timer is not None:
            self.post_switch_goal_timer.cancel()
            self.post_switch_goal_timer = None
        delay = max(0.1, float(self.get_parameter("post_switch_goal_delay_s").value))
        self.post_switch_goal_timer = self.create_timer(delay, self._send_pending_post_switch_goal)

    def _schedule_post_exit_goal(self) -> None:
        if self.post_exit_goal_timer is not None:
            self.post_exit_goal_timer.cancel()
            self.post_exit_goal_timer = None
        delay = max(0.1, float(self.get_parameter("post_switch_goal_delay_s").value))
        self.post_exit_goal_timer = self.create_timer(delay, self._send_pending_post_exit_goal)

    def _send_pending_post_exit_goal(self) -> None:
        if self.post_exit_goal_timer is not None:
            self.post_exit_goal_timer.cancel()
            self.post_exit_goal_timer = None
        post_exit = self.pending_post_exit_pose
        if not post_exit:
            self.pending_flat_gait_after_switch = False
            self._publish_gait(str(self.get_parameter("flat_gait_label").value))
            self._publish_stair_status("complete target_floor=%s" % self.current_floor)
            if self.pending_post_switch_goal is not None:
                self._schedule_post_switch_goal()
            return
        self._send_nav_goal(self._pose_from_xy_yaw(post_exit), "stair_exit")

    def _send_pending_post_switch_goal(self) -> None:
        if self.post_switch_goal_timer is not None:
            self.post_switch_goal_timer.cancel()
            self.post_switch_goal_timer = None
        goal = self.pending_post_switch_goal
        self.pending_post_switch_goal = None
        if goal is None:
            return
        target_floor = self.pending_post_switch_goal_floor
        if target_floor and target_floor != self.current_floor:
            route = self._resolve_next_stair_route(target_floor)
            if route is None:
                self._clear_floor_mission_if_needed("floor_goal")
                return
            self.pending_floor_goal = goal
            if not self._start_stair_route_to_floor(target_floor, route):
                self._clear_floor_mission_if_needed("floor_goal")
            return
        self._publish_stair_status("navigating_to_floor_goal floor=%s" % self.current_floor)
        self._publish_flat_gait("floor_goal")
        self._send_nav_goal(goal, "floor_goal")

    def _clear_floor_mission_if_needed(self, label: str) -> None:
        if not self._is_floor_mission_label(label):
            return
        self.active_floor_mission = False
        self.pending_floor_goal = None
        self.pending_stair_after_nav = None
        self.pending_post_switch_goal = None
        self.pending_post_switch_goal_floor = None
        self.pending_post_exit_pose = None
        self.pending_flat_gait_after_switch = False
        self.pending_stair_transition = None
        self.pending_floor_switch = None
        self.floor_switch_request_started_monotonic = 0.0
        if self.stair_timer is not None:
            self.stair_timer.cancel()
            self.stair_timer = None
        if self.post_exit_goal_timer is not None:
            self.post_exit_goal_timer.cancel()
            self.post_exit_goal_timer = None
        if self.post_switch_goal_timer is not None:
            self.post_switch_goal_timer.cancel()
            self.post_switch_goal_timer = None

    def _pose_from_xy_yaw(self, pose_data: Dict[str, Any]) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = str(self.config.get("mission", {}).get("frame_id", "map"))
        msg.pose.position.x = float(pose_data.get("x", 0.0))
        msg.pose.position.y = float(pose_data.get("y", 0.0))
        msg.pose.position.z = float(pose_data.get("z", 0.0))
        msg.pose.orientation = yaw_to_quaternion(float(pose_data.get("yaw", 0.0)))
        return msg

    def _pose_in_map_frame(self, msg: PoseStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.config.get("mission", {}).get("frame_id", "map"))
        pose.pose = msg.pose
        return pose

    def _normalize_floor_id(self, frame_id: str) -> str:
        value = frame_id.strip()
        if not value or value == str(self.config.get("mission", {}).get("frame_id", "map")):
            return ""
        if value.startswith("floor:"):
            value = value.split(":", 1)[1]
        return value

    def use_stairs(self, command: str) -> None:
        self.get_logger().error(
            "stair command rejected: the retired stair execution chain has been removed"
        )
        self._publish_stair_status(
            "error reason=stair_execution_retired command=%s" % command.strip()
        )

    def _resolve_stair_route(
        self,
        command: str,
    ) -> Optional[Tuple[str, str, str, Dict[str, Any]]]:
        current_floor = self.current_floor
        target_floor = self._resolve_target_floor(command)
        if target_floor is None:
            self.get_logger().error("cannot resolve stair command: %s" % command)
            self._publish_stair_status("error reason=bad_command command=%s" % command)
            return None
        if target_floor == current_floor:
            self.get_logger().warning("stair command resolves to current floor: %s" % target_floor)
            self._publish_stair_status("ignored reason=same_floor floor=%s" % target_floor)
            return None

        direction = self._infer_direction(current_floor, target_floor, command)
        floor = self.floors.get(current_floor, {})
        stairs = floor.get("stairs") or {}
        for stair_name, stair in stairs.items():
            if not isinstance(stair, dict):
                continue
            stair_target = self._stair_target_floor(stair_name, stair)
            if stair_target != target_floor:
                continue
            stair_direction = self._stair_direction(stair_name, stair, current_floor, target_floor)
            if stair_direction == direction:
                return target_floor, direction, stair_name, stair

        self.get_logger().error(
            "no stair route from %s to %s (%s)" % (current_floor, target_floor, direction)
        )
        self._publish_stair_status(
            "error reason=no_route source_floor=%s target_floor=%s direction=%s"
            % (current_floor, target_floor, direction)
        )
        return None

    def _resolve_next_stair_route(
        self,
        final_target_floor: str,
    ) -> Optional[Tuple[str, str, str, Dict[str, Any]]]:
        next_floor = self._next_floor_towards(final_target_floor)
        if next_floor is None:
            self.get_logger().error(
                "cannot resolve next floor from %s to %s"
                % (self.current_floor, final_target_floor)
            )
            self._publish_stair_status(
                "error reason=no_next_floor source_floor=%s target_floor=%s"
                % (self.current_floor, final_target_floor)
            )
            return None
        return self._resolve_stair_route(next_floor)

    def _start_stair_route_to_floor(
        self,
        final_target_floor: str,
        route: Tuple[str, str, str, Dict[str, Any]],
    ) -> bool:
        route_target_floor, direction, stair_name, stair = route
        entry = self._resolve_stair_entry_pose(stair)
        if not entry:
            self.get_logger().error("stair %s has no entry pose" % stair_name)
            self._publish_stair_status("error reason=no_stair_entry stair=%s" % stair_name)
            return False

        self.pending_stair_after_nav = {
            "target_floor": route_target_floor,
            "direction": direction,
            "stair_name": stair_name,
            "stair": stair,
            "metadata": self._resolve_stair_transition_metadata(stair),
        }
        metadata = self.pending_stair_after_nav["metadata"]
        stair_goal = self._pose_from_xy_yaw(entry)
        self._publish_stair_status(
            "navigating_to_stair_entry source_floor=%s next_floor=%s final_floor=%s direction=%s stair=%s model=%s"
            % (
                self.current_floor,
                route_target_floor,
                final_target_floor,
                direction,
                stair_name,
                metadata["model"],
            )
        )
        self._publish_flat_gait("stair_entry")
        self._send_nav_goal(stair_goal, "stair_entry")
        return True

    def _resolve_target_floor(self, command: str) -> Optional[str]:
        raw = command.strip()
        normalized = raw.lower().replace(" ", "")
        if raw in self.floors:
            return raw
        if normalized in ("up", "f+1", "+1", "next"):
            return self._adjacent_floor(+1)
        if normalized in ("down", "f-1", "-1", "prev", "previous"):
            return self._adjacent_floor(-1)
        return None

    def _adjacent_floor(self, step: int) -> Optional[str]:
        current_level = self._floor_level(self.current_floor)
        if current_level is None:
            return None
        candidates: List[Tuple[float, str]] = []
        for floor_id in self.floors.keys():
            level = self._floor_level(floor_id)
            if level is None:
                continue
            if step > 0 and level > current_level:
                candidates.append((level, floor_id))
            elif step < 0 and level < current_level:
                candidates.append((level, floor_id))
        if not candidates:
            return None
        candidates.sort()
        return candidates[0][1] if step > 0 else candidates[-1][1]

    def _next_floor_towards(self, target_floor: str) -> Optional[str]:
        if target_floor == self.current_floor:
            return target_floor
        current_level = self._floor_level(self.current_floor)
        target_level = self._floor_level(target_floor)
        if current_level is None or target_level is None:
            return target_floor

        step = 1 if target_level > current_level else -1
        candidates: List[Tuple[float, str]] = []
        for floor_id in self.floors.keys():
            level = self._floor_level(floor_id)
            if level is None:
                continue
            if step > 0 and current_level < level <= target_level:
                candidates.append((level, floor_id))
            elif step < 0 and target_level <= level < current_level:
                candidates.append((level, floor_id))
        if not candidates:
            return target_floor
        candidates.sort()
        return candidates[0][1] if step > 0 else candidates[-1][1]

    def _floor_level(self, floor_id: str) -> Optional[float]:
        floor = self.floors.get(floor_id, {})
        if "level" in floor:
            try:
                return float(floor["level"])
            except (TypeError, ValueError):
                return None
        text = floor_id.upper()
        if text.startswith("B") and text[1:].isdigit():
            return -float(text[1:])
        if text.startswith("F") and text[1:].isdigit():
            return float(text[1:])
        return None

    def _infer_direction(self, source_floor: str, target_floor: str, command: str) -> str:
        normalized = command.strip().lower().replace(" ", "")
        if normalized in ("up", "f+1", "+1", "next"):
            return "up"
        if normalized in ("down", "f-1", "-1", "prev", "previous"):
            return "down"
        source_level = self._floor_level(source_floor)
        target_level = self._floor_level(target_floor)
        if source_level is not None and target_level is not None:
            return "up" if target_level > source_level else "down"
        return "up"

    def _stair_target_floor(self, stair_name: str, stair: Dict[str, Any]) -> str:
        target = str(stair.get("target_floor") or stair.get("exit_floor") or "").strip()
        if target:
            return target
        marker = "_to_"
        if marker in stair_name:
            return stair_name.split(marker, 1)[1]
        return ""

    def _stair_direction(
        self,
        stair_name: str,
        stair: Dict[str, Any],
        source_floor: str,
        target_floor: str,
    ) -> str:
        direction = str(stair.get("direction", "")).strip().lower()
        if direction in ("up", "down"):
            return direction
        name = stair_name.lower()
        if "up" in name:
            return "up"
        if "down" in name:
            return "down"
        return self._infer_direction(source_floor, target_floor, target_floor)

    def _resolve_stair_transition_metadata(self, stair: Dict[str, Any]) -> Dict[str, Any]:
        defaults = self.config.get("mission", {}).get("stair_transition_defaults") or {}
        transition = stair.get("transition") or {}
        metadata: Dict[str, Any] = {}
        if isinstance(defaults, dict):
            metadata.update(defaults)
        if isinstance(transition, dict):
            metadata.update(transition)

        return {
            "model": str(metadata.get("model") or "shared_platform"),
            "point_type": str(metadata.get("point_type") or "transition"),
            "terrain": str(metadata.get("terrain") or "stairs"),
            "nav_mode": str(metadata.get("nav_mode") or "straight"),
            "direction_mode": str(metadata.get("direction_mode") or "forward"),
            "speed": str(metadata.get("speed") or "low"),
            "obstacle_policy": str(metadata.get("obstacle_policy") or "stop_only"),
            "entry_margin_m": self._optional_float(metadata.get("entry_margin_m"), 0.0),
        }

    def _resolve_stair_entry_pose(self, stair: Dict[str, Any]) -> Dict[str, Any]:
        transition = stair.get("transition") if isinstance(stair.get("transition"), dict) else {}
        entry = (
            transition.get("entry")
            or transition.get("approach")
            or stair.get("entry")
            or stair.get("approach")
        )
        return entry if isinstance(entry, dict) else {}

    def _resolve_stair_exit_pose(
        self,
        stair: Dict[str, Any],
        target_floor: str,
    ) -> Dict[str, Any]:
        transition = stair.get("transition") if isinstance(stair.get("transition"), dict) else {}
        target_exit = (
            transition.get("target_platform")
            or transition.get("target_exit")
            or transition.get("exit")
            or stair.get("target_platform")
            or stair.get("target_exit")
            or stair.get("exit")
        )
        if isinstance(target_exit, dict):
            return target_exit

        target_floor_data = self.floors.get(target_floor, {})
        target_stairs = target_floor_data.get("stairs") or {}
        for target_stair in target_stairs.values():
            if not isinstance(target_stair, dict):
                continue
            source = str(target_stair.get("source_floor", "")).strip()
            if source == self.current_floor and isinstance(target_stair.get("exit"), dict):
                return target_stair["exit"]
        initial_pose = target_floor_data.get("initial_pose")
        return initial_pose if isinstance(initial_pose, dict) else {}

    def _resolve_stair_traverse_pose(self, stair: Dict[str, Any]) -> Dict[str, Any]:
        transition = stair.get("transition") if isinstance(stair.get("transition"), dict) else {}
        source_platform = (
            transition.get("source_platform")
            or transition.get("platform_entry")
            or transition.get("platform_switch")
            or transition.get("traverse_to")
            or stair.get("source_platform")
            or stair.get("platform_entry")
            or stair.get("platform_switch")
            or stair.get("traverse_to")
            or stair.get("stair_map_exit")
        )
        return source_platform if isinstance(source_platform, dict) else {}

    def _resolve_stair_post_exit_pose(self, stair: Dict[str, Any]) -> Dict[str, Any]:
        transition = stair.get("transition") if isinstance(stair.get("transition"), dict) else {}
        post_exit = (
            transition.get("post_exit")
            or transition.get("flat_transition")
            or transition.get("flat_entry")
            or stair.get("post_exit")
            or stair.get("flat_transition")
            or stair.get("flat_entry")
            or stair.get("stair_exit")
        )
        return post_exit if isinstance(post_exit, dict) else {}

    def _optional_float(self, value: Any, default: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _is_robot_near_pose(self, pose_data: Dict[str, Any]) -> bool:
        if self.robot_pose is None:
            self.get_logger().warning("robot pose is not available; cannot verify stair entry")
            self._publish_stair_status("error reason=no_robot_pose")
            return False
        dx = self.robot_pose.pose.position.x - float(pose_data.get("x", 0.0))
        dy = self.robot_pose.pose.position.y - float(pose_data.get("y", 0.0))
        distance = math.hypot(dx, dy)
        tolerance = float(self.get_parameter("stair_entry_tolerance_m").value)
        if distance > tolerance:
            zone = self._active_stair_zone_at_robot()
            if zone is not None:
                self.get_logger().warning(
                    "robot is %.2fm from stair entry but inside semantic stair zone %s"
                    % (distance, zone.get("id") or zone.get("name") or "unknown")
                )
                self._publish_stair_status(
                    "entry_confirmed_by_zone zone=%s distance=%.2f"
                    % (zone.get("id") or zone.get("name") or "unknown", distance)
                )
                return True
            self.get_logger().warning(
                "robot is %.2fm from stair entry; tolerance is %.2fm" % (distance, tolerance)
            )
            self._publish_stair_status(
                "blocked reason=not_at_entry distance=%.2f tolerance=%.2f" % (distance, tolerance)
            )
            return False
        return True

    def _active_stair_zone_at_robot(self) -> Optional[Dict[str, Any]]:
        if self.robot_pose is None or not self.current_floor:
            return None
        x = float(self.robot_pose.pose.position.x)
        y = float(self.robot_pose.pose.position.y)
        for zone in self.stair_zones_by_floor.get(self.current_floor, []):
            polygon = zone.get("polygon") or []
            if self._point_in_polygon(x, y, polygon):
                return zone
        return None

    @staticmethod
    def _point_in_polygon(x: float, y: float, polygon: Any) -> bool:
        if not isinstance(polygon, list) or len(polygon) < 3:
            return False
        inside = False
        previous = polygon[-1]
        for current in polygon:
            try:
                xi = float(current.get("x"))
                yi = float(current.get("y"))
                xj = float(previous.get("x"))
                yj = float(previous.get("y"))
            except (AttributeError, TypeError, ValueError):
                previous = current
                continue
            intersects = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi
            )
            if intersects:
                inside = not inside
            previous = current
        return inside

    def _finish_pending_stair_transition(self) -> None:
        if self.stair_timer is not None:
            self.stair_timer.cancel()
            self.stair_timer = None
        transition = self.pending_stair_transition
        if transition is None:
            return
        self.pending_stair_transition = None
        metadata = transition.get("metadata") or {}
        self._publish_stair_status(
            "requesting_coordinated_floor_switch source_floor=%s target_floor=%s direction=%s stair=%s model=%s"
            % (
                transition["source_floor"],
                transition["target_floor"],
                transition["direction"],
                transition["stair_name"],
                metadata.get("model", "shared_platform"),
            )
        )
        self._request_coordinated_floor_switch(transition)

    def _request_coordinated_floor_switch(self, transition: Dict[str, Any]) -> None:
        if self.pending_floor_switch is not None:
            self._publish_stair_status("error reason=floor_switch_request_already_pending")
            self._clear_floor_mission_if_needed("floor_goal")
            return
        request_id = "floor-switch-%d" % int(time.time() * 1000.0)
        request = {
            "request_id": request_id,
            "route_id": str(transition.get("route_id") or transition.get("stair_name") or ""),
            "source_floor": str(transition.get("source_floor") or self.current_floor),
            "target_floor": str(transition.get("target_floor") or ""),
            "target_map_id": str(transition.get("target_map_id") or ""),
            "target_pose": dict(transition.get("target_platform") or {}),
            "post_exit": dict(transition.get("post_exit") or {}),
        }
        self.pending_floor_switch = dict(request)
        self.floor_switch_request_started_monotonic = time.monotonic()
        message = String()
        message.data = json.dumps(request, separators=(",", ":"))
        self.floor_switch_request_pub.publish(message)
        self._publish_stair_status(
            "floor_switch_request_sent request_id=%s source_floor=%s target_floor=%s map_id=%s"
            % (
                request_id,
                request["source_floor"],
                request["target_floor"],
                request["target_map_id"],
            )
        )

    def _publish_gait(self, label: str) -> None:
        msg = String()
        msg.data = label
        self.gait_command_pub.publish(msg)
        self.get_logger().info("gait command: %s" % label)

    def _publish_flat_gait(self, reason: str) -> None:
        if not bool(self.get_parameter("publish_flat_gait_before_nav").value):
            return
        label = str(self.get_parameter("flat_gait_label").value).strip()
        if not label:
            return
        self._publish_gait(label)
        self._publish_stair_status("flat_gait_requested label=%s reason=%s" % (label, reason))

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
