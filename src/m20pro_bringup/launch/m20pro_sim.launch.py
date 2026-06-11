import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")
    desc_share = get_package_share_directory("m20pro_description")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(bringup_share, "config", "m20pro.yaml")
    default_nav2_params = os.path.join(bringup_share, "config", "nav2_params_sim.yaml")
    default_floor_config = os.path.join(bringup_share, "config", "inspection_waypoints.yaml")
    default_map_manifest = os.path.join(bringup_share, "config", "map_manifest.yaml")
    default_urdf = os.path.join(desc_share, "urdf", "M20.urdf")
    default_map = os.path.join(bringup_share, "maps", "F20", "occ_grid.yaml")
    default_rviz = os.path.join(bringup_share, "rviz", "m20pro_sim.rviz")

    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    floor_config = LaunchConfiguration("floor_config")
    map_manifest = LaunchConfiguration("map_manifest")
    map_yaml = LaunchConfiguration("map")
    enable_floor_manager = LaunchConfiguration("enable_floor_manager")
    enable_floor_goal_bridge = LaunchConfiguration("enable_floor_goal_bridge")
    enable_dynamic_obstacles = LaunchConfiguration("enable_dynamic_obstacles")
    enable_system_check = LaunchConfiguration("enable_system_check")
    enable_config_audit = LaunchConfiguration("enable_config_audit")
    enable_web_dashboard = LaunchConfiguration("enable_web_dashboard")
    web_dashboard_port = LaunchConfiguration("web_dashboard_port")
    web_dashboard_data_dir = LaunchConfiguration("web_dashboard_data_dir")
    web_dashboard_map_archive_dir = LaunchConfiguration("web_dashboard_map_archive_dir")
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    relocalization_result_topic = LaunchConfiguration("relocalization_result_topic")
    factory_host = LaunchConfiguration("factory_host")
    factory_user = LaunchConfiguration("factory_user")
    factory_active_map = LaunchConfiguration("factory_active_map")
    factory_mapping_start_command = LaunchConfiguration("factory_mapping_start_command")
    factory_mapping_finish_command = LaunchConfiguration("factory_mapping_finish_command")
    factory_mapping_cancel_command = LaunchConfiguration("factory_mapping_cancel_command")
    initial_floor = LaunchConfiguration("initial_floor")
    use_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_delay_s = LaunchConfiguration("rviz_delay_s")

    with open(default_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument("floor_config", default_value=default_floor_config),
        DeclareLaunchArgument("map_manifest", default_value=default_map_manifest),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("enable_floor_manager", default_value="true"),
        DeclareLaunchArgument("enable_floor_goal_bridge", default_value="true"),
        DeclareLaunchArgument("enable_dynamic_obstacles", default_value="true"),
        DeclareLaunchArgument("enable_system_check", default_value="true"),
        DeclareLaunchArgument("enable_config_audit", default_value="true"),
        DeclareLaunchArgument("enable_web_dashboard", default_value="true"),
        DeclareLaunchArgument("web_dashboard_port", default_value="8080"),
        DeclareLaunchArgument("web_dashboard_data_dir", default_value="~/.m20pro_web"),
        DeclareLaunchArgument("web_dashboard_map_archive_dir", default_value="~/m20pro_maps"),
        DeclareLaunchArgument("initialpose_topic", default_value="/initialpose"),
        DeclareLaunchArgument(
            "relocalization_result_topic",
            default_value="/m20pro_tcp_bridge/relocalization_result",
        ),
        DeclareLaunchArgument("factory_host", default_value="10.21.31.106"),
        DeclareLaunchArgument("factory_user", default_value="user"),
        DeclareLaunchArgument("factory_active_map", default_value="/var/opt/robot/data/maps/active"),
        DeclareLaunchArgument(
            "factory_mapping_start_command",
            default_value=(
                "ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "
                "\"nohup sudo -n drmap mapping -s -n {map_name} > "
                "/tmp/m20pro_drmap_mapping_{session_id}.log 2>&1 &\""
            ),
        ),
        DeclareLaunchArgument(
            "factory_mapping_finish_command",
            default_value=(
                "ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "
                "\"sudo -n drmap stop_mapping\""
            ),
        ),
        DeclareLaunchArgument(
            "factory_mapping_cancel_command",
            default_value=(
                "ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "
                "\"sudo -n drmap stop_mapping\""
            ),
        ),
        DeclareLaunchArgument("initial_floor", default_value="F20"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("rviz_delay_s", default_value="5.0"),

        Node(
            package="m20pro_navigation",
            executable="config_audit",
            name="m20pro_config_audit",
            output="screen",
            parameters=[
                {
                    "map_manifest": map_manifest,
                    "floor_config": floor_config,
                    "fail_on_error": False,
                }
            ],
            condition=IfCondition(enable_config_audit),
        ),
        Node(
            package="m20pro_navigation",
            executable="zero_joint_state_publisher",
            name="zero_joint_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="m20pro_nav_base_to_urdf_base",
            output="screen",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                "m20pro_base_link",
                "base_link",
            ],
        ),
        Node(
            package="m20pro_navigation",
            executable="sim_bridge",
            name="m20pro_tcp_bridge",
            output="screen",
            parameters=[params_file, {"base_frame": "m20pro_base_link"}],
        ),
        Node(
            package="m20pro_navigation",
            executable="dual_lidar_simulator",
            name="m20pro_dual_lidar_simulator",
            output="screen",
            parameters=[
                params_file,
                {
                    "map_manifest": map_manifest,
                    "frame_id": "m20pro_base_link",
                },
            ],
        ),
        Node(
            package="m20pro_navigation",
            executable="pointcloud_fusion",
            name="m20pro_pointcloud_fusion",
            output="screen",
            parameters=[
                params_file,
                {
                    # The recorded factory PCD contains low ground/leg/person
                    # remnants. Filter them more aggressively in sim so the
                    # local costmap does not turn narrow corridors into a blue
                    # inflated sheet.
                    "input_cloud_topic": "/cloud_nav",
                    "output_scan_topic": "/scan",
                    "frame_id": "m20pro_base_link",
                    "publish_rate_hz": 10.0,
                    "height_min": 0.05,
                    "height_max": 0.85,
                    "robot_radius": 0.45,
                    "cloud_reliability": "reliable",
                    "scan_reliability": "best_effort",
                    "max_points_per_cloud": 12000,
                    "min_cloud_interval_s": 0.05,
                    "publish_on_cloud_update": False,
                },
            ],
        ),
        Node(
            package="m20pro_navigation",
            executable="dynamic_obstacle_simulator",
            name="m20pro_dynamic_obstacle_simulator",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(enable_dynamic_obstacles),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"use_sim_time": False}, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"node_names": ["map_server"]},
            ],
        ),
        Node(
            package="m20pro_navigation",
            executable="floor_manager",
            name="m20pro_floor_manager",
            output="screen",
            parameters=[
                {
                    "config_file": floor_config,
                    "initial_floor": initial_floor,
                }
            ],
            condition=IfCondition(enable_floor_manager),
        ),
        Node(
            package="m20pro_navigation",
            executable="floor_goal_bridge",
            name="m20pro_floor_goal_bridge",
            output="screen",
            parameters=[
                {
                    "floor_config": floor_config,
                    "default_floor": initial_floor,
                    "enable_same_floor_goal_bridge": True,
                }
            ],
            condition=IfCondition(enable_floor_goal_bridge),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params_file,
                "use_sim_time": "False",
                "use_composition": "False",
                "map_subscribe_transient_local": "True",
            }.items(),
        ),
        Node(
            package="m20pro_navigation",
            executable="system_check",
            name="m20pro_system_check",
            output="screen",
            parameters=[
                {
                    "mode": "sim",
                    "cloud_topic": "/cloud_nav",
                    "require_dynamic_obstacles": enable_dynamic_obstacles,
                    "require_floor_manager": enable_floor_manager,
                    "check_scan_content": True,
                    "check_local_costmap_content": True,
                    "check_tf_height": True,
                    "tf_global_frame": "odom",
                    "tf_base_frame": "m20pro_base_link",
                    "max_abs_base_z": 0.5,
                    "min_scan_finite_bins": 20,
                    "min_scan_close_bins": 1,
                    "scan_close_range_m": 2.0,
                    "min_local_costmap_marked_cells": 1,
                }
            ],
            condition=IfCondition(enable_system_check),
        ),
        Node(
            package="m20pro_cloud_bridge",
            executable="web_dashboard",
            name="m20pro_web_dashboard",
            output="screen",
            parameters=[
                {
                    "port": web_dashboard_port,
                    "data_dir": web_dashboard_data_dir,
                    "map_archive_dir": web_dashboard_map_archive_dir,
                    "map_manifest": map_manifest,
                    "initialpose_topic": initialpose_topic,
                    "relocalization_result_topic": relocalization_result_topic,
                    "factory_host": factory_host,
                    "factory_user": factory_user,
                    "factory_active_map": factory_active_map,
                    "factory_mapping_start_command": factory_mapping_start_command,
                    "factory_mapping_finish_command": factory_mapping_finish_command,
                    "factory_mapping_cancel_command": factory_mapping_cancel_command,
                }
            ],
            condition=IfCondition(enable_web_dashboard),
        ),
        TimerAction(
            period=rviz_delay_s,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=["-d", rviz_config],
                    condition=IfCondition(use_rviz),
                )
            ],
        ),
    ])
