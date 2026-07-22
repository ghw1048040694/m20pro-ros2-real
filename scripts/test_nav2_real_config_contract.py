#!/usr/bin/env python3
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_navigation"))

from m20pro_navigation.field_profile_contract import (  # noqa: E402
    edge_environment,
    floor_manager_field_parameters,
    load_field_profile,
    render_m20pro_parameters,
    render_nav2_parameters,
    tcp_bridge_parameters,
    web_navigation_field_parameters,
)


def field_placeholders(value: object) -> list:
    if isinstance(value, dict):
        return [item for nested in value.values() for item in field_placeholders(nested)]
    if isinstance(value, str) and value.startswith("__FIELD_PROFILE_"):
        return [value]
    return []


def main() -> None:
    config_path = ROOT / "src" / "m20pro_bringup" / "config" / "nav2_params_real.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    field_profile = load_field_profile(
        ROOT / "src/m20pro_bringup/config/m20pro_field_profile.yaml"
    )
    navigation_profile = field_profile["navigation"]
    localization_profile = field_profile["localization"]
    real_config_path = ROOT / "src" / "m20pro_bringup" / "config" / "m20pro_real.yaml"
    real_config = yaml.safe_load(real_config_path.read_text(encoding="utf-8"))
    bridge_config = real_config["m20pro_tcp_bridge"]["ros__parameters"]
    assert bridge_config["pose_source"] == "auto"
    assert bridge_config["send_heartbeat"] is True
    assert float(bridge_config["pose_jump_accept_after_s"]) == 0.0
    assert bridge_config["pose_stationary_drift_reject_m"] == "__FIELD_PROFILE_STATIONARY_DRIFT_REJECT__"
    assert bridge_config["pose_motion_command_hold_s"] == "__FIELD_PROFILE_MOTION_COMMAND_HOLD__"
    assert bridge_config["handle_steer_motion_hold_s"] == "__FIELD_PROFILE_HANDLE_STEER_HOLD__"
    assert bridge_config["posture_transition_hold_s"] == "__FIELD_PROFILE_POSTURE_HOLD__"
    assert localization_profile["stationary_drift_reject_m"] > 0.0
    assert localization_profile["motion_command_hold_s"] > 0.0
    controller = config["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    controller_profile = navigation_profile["controller"]
    goal_profile = navigation_profile["goal"]
    progress_profile = navigation_profile["progress"]
    local_planner_profile = navigation_profile["local_planner"]
    costmap_profile = navigation_profile["costmap"]
    planner_profile = navigation_profile["global_planner"]
    web_navigation = web_navigation_field_parameters(field_profile)
    bridge_rewrites = tcp_bridge_parameters(field_profile)
    floor_rewrites = floor_manager_field_parameters(field_profile)

    assert controller["controller_frequency"] == "__FIELD_PROFILE_CONTROLLER_FREQUENCY__"
    assert controller_profile["frequency_hz"] <= 10.0
    assert controller["progress_checker"]["required_movement_radius"] == "__FIELD_PROFILE_PROGRESS_RADIUS__"
    assert controller["progress_checker"]["movement_time_allowance"] == "__FIELD_PROFILE_PROGRESS_TIME__"
    assert progress_profile["movement_time_allowance_s"] >= 12.0
    assert controller["goal_checker"]["stateful"] is False
    assert controller["goal_checker"]["xy_goal_tolerance"] == "__FIELD_PROFILE_XY_GOAL_TOLERANCE__"
    assert controller["goal_checker"]["yaw_goal_tolerance"] == "__FIELD_PROFILE_YAW_GOAL_TOLERANCE__"
    assert follow["xy_goal_tolerance"] == "__FIELD_PROFILE_XY_GOAL_TOLERANCE__"
    assert abs(goal_profile["xy_tolerance_m"] - 0.35) < 1e-6
    assert web_navigation["goal_reached_tolerance_m"] == goal_profile["xy_tolerance_m"]
    assert abs(goal_profile["yaw_tolerance_rad"] - 0.35) < 1e-6
    assert "ObstacleFootprint" in follow["critics"]
    assert "BaseObstacle" not in follow["critics"]
    assert follow["sim_time"] == "__FIELD_PROFILE_SIMULATION_TIME__"
    assert local_planner_profile["simulation_time_s"] >= 1.5
    assert follow["ObstacleFootprint.scale"] == "__FIELD_PROFILE_OBSTACLE_CRITIC_SCALE__"
    assert local_planner_profile["obstacle_critic_scale"] >= 1.0
    assert follow["max_vel_x"] == "__FIELD_PROFILE_MAX_LINEAR_SPEED__"
    assert follow["max_speed_xy"] == "__FIELD_PROFILE_MAX_LINEAR_SPEED__"
    assert follow["max_vel_theta"] == "__FIELD_PROFILE_MAX_ANGULAR_SPEED__"
    assert follow["publish_local_plan"] is True
    assert follow["stateful"] is False
    assert local["always_send_full_costmap"] is True
    assert global_costmap["always_send_full_costmap"] is False
    for costmap in (local, global_costmap):
        assert "footprint" in costmap
        assert "robot_radius" not in costmap
        assert costmap["inflation_layer"]["inflation_radius"] == "__FIELD_PROFILE_INFLATION_RADIUS__"
        assert costmap_profile["inflation_radius_m"] >= 0.60
        assert costmap["inflation_layer"]["cost_scaling_factor"] == "__FIELD_PROFILE_INFLATION_COST_SCALING__"
        assert costmap["obstacle_layer"]["scan"]["topic"] == "/scan"
        assert costmap["obstacle_layer"]["scan"]["inf_is_valid"] is True
        assert costmap["obstacle_layer"]["scan"]["obstacle_range"] == "__FIELD_PROFILE_OBSTACLE_RANGE__"
        assert costmap["obstacle_layer"]["scan"]["raytrace_range"] == "__FIELD_PROFILE_RAYTRACE_RANGE__"
    assert local["update_frequency"] == "__FIELD_PROFILE_LOCAL_COSTMAP_UPDATE_FREQUENCY__"
    assert global_costmap["update_frequency"] == "__FIELD_PROFILE_GLOBAL_COSTMAP_UPDATE_FREQUENCY__"
    planner = config["planner_server"]["ros__parameters"]
    assert planner["expected_planner_frequency"] == "__FIELD_PROFILE_PLANNER_FREQUENCY__"
    assert planner["GridBased"]["tolerance"] == "__FIELD_PROFILE_PLANNER_GOAL_TOLERANCE__"
    assert planner_profile["goal_tolerance_m"] >= goal_profile["xy_tolerance_m"]
    assert len(bridge_rewrites) == 18
    assert len(floor_rewrites) == 7
    for parameter_name in bridge_rewrites:
        assert bridge_config[parameter_name].startswith("__FIELD_PROFILE_")
    rendered_nav2 = render_nav2_parameters(config, field_profile)
    rendered_real = render_m20pro_parameters(
        real_config, field_profile, enable_axis_command=False
    )
    assert field_placeholders(rendered_nav2) == []
    assert field_placeholders(rendered_real) == []
    rendered_controller = rendered_nav2["controller_server"]["ros__parameters"]
    assert rendered_controller["FollowPath"]["decel_lim_x"] == -2.2
    assert type(rendered_controller["FollowPath"]["decel_lim_x"]) is float
    assert type(rendered_controller["FollowPath"]["decel_lim_theta"]) is float
    assert type(rendered_controller["FollowPath"]["vx_samples"]) is int
    assert type(rendered_controller["FollowPath"]["vtheta_samples"]) is int
    assert type(rendered_controller["controller_frequency"]) is float
    assert type(rendered_controller["progress_checker"]["movement_time_allowance"]) is float
    assert rendered_nav2["local_costmap"]["local_costmap"]["ros__parameters"][
        "update_frequency"
    ] == 8.0
    assert rendered_nav2["global_costmap"]["global_costmap"]["ros__parameters"][
        "update_frequency"
    ] == 1.0
    assert type(
        rendered_nav2["planner_server"]["ros__parameters"][
            "expected_planner_frequency"
        ]
    ) is float
    rendered_bridge = rendered_real["m20pro_tcp_bridge"]["ros__parameters"]
    for parameter_name, expected in bridge_rewrites.items():
        assert rendered_bridge[parameter_name] == expected
        assert type(rendered_bridge[parameter_name]) is float
    assert rendered_bridge["enable_axis_command"] is False
    assert rendered_bridge["enable_initialpose_relocalization"] is True
    assert rendered_bridge["enable_initialpose_3d_relocalization"] is False

    tree_path = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "behavior_trees"
        / "m20pro_navigate_to_pose_foxy.xml"
    )
    root = ET.parse(tree_path).getroot()
    assert root.find(".//ReactiveFallback[@name='FollowPathFallback']") is not None
    assert root.find(".//BackUp") is not None
    navigate_recovery = root.find(".//RecoveryNode[@name='NavigateRecovery']")
    follow_recovery = root.find(".//RecoveryNode[@name='FollowPath']")
    assert navigate_recovery is not None
    assert follow_recovery is not None
    assert int(navigate_recovery.attrib["number_of_retries"]) >= 4
    assert int(follow_recovery.attrib["number_of_retries"]) >= 8
    controller_actions = root.find(".//RoundRobin[@name='ControllerRecoveryActions']")
    navigation_actions = root.find(".//RoundRobin[@name='NavigationRecoveryActions']")
    assert controller_actions is not None
    assert navigation_actions is not None
    controller_children = list(controller_actions)
    navigation_children = list(navigation_actions)
    assert controller_children[0].tag == "Wait"
    assert navigation_children[0].tag == "Wait"
    assert int(controller_children[0].attrib["wait_duration"]) >= 5
    assert int(navigation_children[0].attrib["wait_duration"]) >= 5
    timed_bt_tags = {
        "ComputePathToPose",
        "FollowPath",
        "ClearEntireCostmap",
        "BackUp",
        "Spin",
        "Wait",
    }
    timed_nodes = [node for node in root.iter() if node.tag in timed_bt_tags]
    assert timed_nodes
    for node in timed_nodes:
        assert int(node.attrib.get("server_timeout", "0")) >= 500

    stair_tree_path = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "behavior_trees"
        / "m20pro_stair_traverse_foxy.xml"
    )
    stair_root = ET.parse(stair_tree_path).getroot()
    assert stair_root.find(".//ComputePathToPose") is not None
    assert stair_root.find(".//FollowPath") is not None
    for forbidden_tag in ("BackUp", "Spin", "ClearEntireCostmap", "RecoveryNode"):
        assert stair_root.find(f".//{forbidden_tag}") is None

    dashboard = (
        ROOT
        / "src"
        / "m20pro_cloud_bridge"
        / "m20pro_cloud_bridge"
        / "web_dashboard_node.py"
    ).read_text(encoding="utf-8")
    tcp_bridge = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "tcp_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "accept_after_s=0.0" in tcp_bridge
    assert "allow_stable_recovery=location_ok" not in tcp_bridge
    # The posture-transition branch must use the declared canonical parameter
    # names; a legacy unprefixed lookup kills the bridge during TF callbacks.
    assert 'get_parameter("stationary_drift_reject_m")' not in tcp_bridge
    assert 'get_parameter("stationary_drift_reject_yaw_rad")' not in tcp_bridge
    assert 'get_parameter("pose_stationary_drift_reject_m")' in tcp_bridge
    assert 'get_parameter("pose_stationary_drift_reject_yaw_rad")' in tcp_bridge
    assert "fallback = self._aligned_tf_fallback_pose(fallback)" in tcp_bridge
    assert 'self.active_pose_source = "tcp_1007"' in tcp_bridge
    assert 'self.active_pose_source = "official_tf_fallback"' in tcp_bridge
    assert "set_unsolicited_response_handler" in tcp_bridge
    assert 'self.create_publisher(Int32, "~/motion_state", 10)' in tcp_bridge
    assert 'now - self.last_heartbeat_monotonic >= 1.0' in tcp_bridge
    assert "OccupancyGridUpdate" in dashboard
    assert '"local_costmap_updates_topic"' in dashboard
    assert '"global_costmap_updates_topic"' in dashboard
    assert '"local_plan_topic"' in dashboard
    assert '"odom_topic", "/odom"' in dashboard
    assert "local_costmap_odom_alignment_payload" in dashboard

    recorder = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "scripts"
        / "m20pro_record_real.sh"
    ).read_text(encoding="utf-8")
    assert "/local_plan" in recorder

    real_launch = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "launch"
        / "m20pro_real.launch.py"
    ).read_text(encoding="utf-8")
    nav_launch = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "launch"
        / "nav2_navigation_real_foxy.launch.py"
    ).read_text(encoding="utf-8")
    startup_gate = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "nav2_startup_gate.py"
    ).read_text(encoding="utf-8")
    system_check = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "system_check_node.py"
    ).read_text(encoding="utf-8")
    assert "nav2_navigation_real_foxy.launch.py" in real_launch
    assert 'executable="waypoint_follower"' not in nav_launch
    assert '"waypoint_follower"' not in startup_gate
    assert '"/waypoint_follower"' not in system_check
    assert '"/waypoint_follower"' not in dashboard
    assert real_launch.count('"scan_topic": scan_topic') >= 3
    assert '"scan_timeout_s": 3.0' in real_launch
    assert '"check_scan_content": True' in real_launch
    assert '"web_scan_topic"' not in real_launch
    assert 'executable="navigation_scan_selector"' not in real_launch
    assert '"input_topic": scan_topic' in real_launch
    assert "/m20pro/navigation_scan" not in real_launch
    assert "load_field_profile(default_field_profile)" in real_launch
    assert "nav2_parameter_rewrites" not in nav_launch
    assert "__FIELD_PROFILE_" not in nav_launch
    assert "local_costmap.local_costmap.ros__parameters" not in nav_launch
    assert 'LaunchConfiguration("controller_frequency")' not in nav_launch
    assert "localization_parameters" not in real_launch
    assert "**floor_manager_parameters" in real_launch
    assert 'executable="command_mux"' in real_launch
    assert real_launch.count('"cmd_vel_topic": "/cmd_vel_nav"') >= 2
    assert '"teleop_cmd_vel_topic": "/cmd_vel_teleop"' in real_launch
    assert 'LaunchConfiguration("cmd_vel_topic")' in nav_launch
    assert '("cmd_vel", cmd_vel_topic)' in nav_launch

    command_mux = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "command_mux_node.py"
    ).read_text(encoding="utf-8")
    assert 'self.declare_parameter("navigation_cmd_vel_topic", "/cmd_vel_nav")' in command_mux
    assert 'self.declare_parameter("teleop_cmd_vel_topic", "/cmd_vel_teleop")' in command_mux
    assert 'self.declare_parameter("output_cmd_vel_topic", "/cmd_vel")' in command_mux
    assert 'self.declare_parameter("initial_mode", "navigation")' in command_mux
    assert '"locked", "navigation", "teleop"' in command_mux
    assert "self._watchdog_timer.cancel()" in command_mux
    assert "self._watchdog_timer.reset()" in command_mux

    scan_selector = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "navigation_scan_selector.py"
    )
    floor_manager = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "floor_manager.py"
    ).read_text(encoding="utf-8")
    assert not scan_selector.exists()
    startup_gate_source = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "nav2_startup_gate.py"
    ).read_text(encoding="utf-8")
    system_check_source = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "system_check_node.py"
    ).read_text(encoding="utf-8")
    assert 'self.declare_parameter("scan_topic", "/scan")' in startup_gate_source
    assert 'self.declare_parameter("scan_topic", "/scan")' in system_check_source
    assert 'self.declare_parameter("scan_timeout_s", 3.0)' in system_check_source
    assert 'self.scan_received_monotonic = 0.0' in system_check_source
    assert 'scan_stale=age:' in system_check_source
    assert 'self.create_subscription(LaserScan, self.scan_topic' in system_check_source
    assert 'label in ("stair_traverse", "stair_exit")' in floor_manager
    assert "goal.behavior_tree = behavior_tree" in floor_manager
    assert "stair_execution_retired" in floor_manager
    assert "stair_clearance" not in floor_manager
    assert "stair_perception" not in floor_manager
    assert 'self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")' in floor_manager

    fastdds_profile = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "config"
        / "m20pro_fastdds_udp.xml"
    ).read_text(encoding="utf-8")
    assert "<type>UDPv4</type>" in fastdds_profile
    assert "<type>SHM</type>" not in fastdds_profile
    assert "m20pro_shm_transport" not in fastdds_profile

    real_start = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "scripts"
        / "m20pro_real_full.sh"
    ).read_text(encoding="utf-8")
    assert "export PYTHONDONTWRITEBYTECODE=1" in real_start
    assert "render-real-yaml" in real_start
    assert "render-nav2-yaml" in real_start
    assert 'real_nav2_params_file:="${RUNTIME_NAV2_PARAMS}"' in real_start
    assert "cleanup_runtime_params" in real_start

    edge_env = edge_environment(field_profile)
    assert float(edge_env["HEIGHT_MIN"]) <= -0.25
    assert float(edge_env["HEIGHT_MAX"]) >= 0.60
    assert int(edge_env["MAX_POINTS"]) == 0
    assert 0.5 <= float(edge_env["BIN_HOLD_S"]) <= 1.0
    assert not any(key.startswith("STAIR_") for key in edge_env)
    assert edge_env["FIELD_PROFILE_HASH"] == field_profile["profile_hash"]
    assert not (
        ROOT
        / "tools/edge_scan_feasibility/service/m20pro-edge-scan-106.env.edge_scan"
    ).exists()

    edge_unit = (
        ROOT
        / "tools"
        / "edge_scan_feasibility"
        / "service"
        / "m20pro-edge-scan-106.service.example"
    ).read_text(encoding="utf-8")
    for variable in ("FIELD_PROFILE_NAME", "FIELD_PROFILE_HASH"):
        assert "${%s}" % variable in edge_unit
    assert "STAIR_" not in edge_unit

    edge_source = (
        ROOT
        / "tools"
        / "edge_scan_feasibility"
        / "drdds_edge_scan_demo.cpp"
    ).read_text(encoding="utf-8")
    assert "if (argc != 18)" in edge_source
    assert "stair" not in edge_source.lower()
    assert "field_profile_hash" in edge_source

    assert "/scan" in recorder
    for topic in (
        "/m20pro/navigation_scan",
        "/m20pro/stair_obstacle_scan",
        "/m20pro/stair_clearance",
        "/m20pro/stair_perception_mode",
        "/m20pro/navigation_scan_status",
    ):
        assert topic not in recorder

    print("real Nav2 configuration contract tests passed")


if __name__ == "__main__":
    main()
