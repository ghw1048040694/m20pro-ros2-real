import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from m20pro_navigation.field_profile_contract import (
    command_mux_field_parameters,
    floor_manager_field_parameters,
    load_field_profile,
    stair_executor_field_parameters,
    web_navigation_field_parameters,
    web_teleoperation_field_parameters,
)


def has_package(package_name: str) -> bool:
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")
    inspection_share = get_package_share_directory("m20pro_inspection")
    radar_share = get_package_share_directory("m20pro_radar_inspection")
    nav2_stack_available = (
        has_package("nav2_bringup")
        and has_package("nav2_map_server")
        and has_package("nav2_lifecycle_manager")
    )
    default_params = os.path.join(bringup_share, "config", "m20pro_real.yaml")
    default_nav2_params = os.path.join(bringup_share, "config", "nav2_params_real.yaml")
    default_field_profile = os.path.join(
        bringup_share, "config", "m20pro_field_profile.yaml"
    )
    field_profile = load_field_profile(default_field_profile)
    floor_manager_parameters = floor_manager_field_parameters(field_profile)
    command_mux_parameters = command_mux_field_parameters(field_profile)
    stair_executor_parameters = stair_executor_field_parameters(field_profile)
    web_navigation_parameters = web_navigation_field_parameters(field_profile)
    web_teleoperation_parameters = web_teleoperation_field_parameters(field_profile)
    default_floor_config = os.path.join(bringup_share, "config", "runtime_navigation.yaml")
    default_map_manifest = os.path.join(bringup_share, "config", "map_manifest.yaml")
    default_map = os.path.join(bringup_share, "maps", "F20", "occ_grid.yaml")
    default_rviz = os.path.join(bringup_share, "rviz", "m20pro_navigation.rviz")
    inspection_launch = os.path.join(inspection_share, "launch", "m20pro_inspection.launch.py")
    default_inspection_model = os.path.join(inspection_share, "models", "best_rk3588_fp16.rknn")
    default_inspection_classes = os.path.join(inspection_share, "models", "labels_zh.txt")
    radar_launch = os.path.join(radar_share, "launch", "m20pro_radar_inspection.launch.py")

    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    floor_config = LaunchConfiguration("floor_config")
    map_manifest = LaunchConfiguration("map_manifest")
    map_yaml = LaunchConfiguration("map")
    scan_topic = LaunchConfiguration("scan_topic")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    enable_floor_manager = LaunchConfiguration("enable_floor_manager")
    enable_floor_goal_bridge = LaunchConfiguration("enable_floor_goal_bridge")
    enable_stair_connector = LaunchConfiguration("enable_stair_connector")
    enable_system_check = LaunchConfiguration("enable_system_check")
    enable_config_audit = LaunchConfiguration("enable_config_audit")
    initial_floor = LaunchConfiguration("initial_floor")
    enable_initialpose_3d_adapter = LaunchConfiguration("enable_initialpose_3d_adapter")
    initialpose_3d_z = LaunchConfiguration("initialpose_3d_z")
    enable_web_dashboard = LaunchConfiguration("enable_web_dashboard")
    enable_nav2_startup_gate = LaunchConfiguration("enable_nav2_startup_gate")
    web_dashboard_port = LaunchConfiguration("web_dashboard_port")
    web_dashboard_data_dir = LaunchConfiguration("web_dashboard_data_dir")
    web_dashboard_map_archive_dir = LaunchConfiguration("web_dashboard_map_archive_dir")
    robot_pose_display_yaw_offset_rad = LaunchConfiguration("robot_pose_display_yaw_offset_rad")
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    relocalization_result_topic = LaunchConfiguration("relocalization_result_topic")
    factory_host = LaunchConfiguration("factory_host")
    factory_user = LaunchConfiguration("factory_user")
    factory_active_map = LaunchConfiguration("factory_active_map")
    factory_mapping_start_command = LaunchConfiguration("factory_mapping_start_command")
    factory_mapping_finish_command = LaunchConfiguration("factory_mapping_finish_command")
    factory_mapping_cancel_command = LaunchConfiguration("factory_mapping_cancel_command")
    enable_stair_zone_postprocess = LaunchConfiguration("enable_stair_zone_postprocess")
    stair_zones_topic = LaunchConfiguration("stair_zones_topic")
    enable_camera_proxy = LaunchConfiguration("enable_camera_proxy")
    front_camera_url = LaunchConfiguration("front_camera_url")
    rear_camera_url = LaunchConfiguration("rear_camera_url")
    camera_proxy_backend = LaunchConfiguration("camera_proxy_backend")
    camera_proxy_fps = LaunchConfiguration("camera_proxy_fps")
    camera_proxy_jpeg_quality = LaunchConfiguration("camera_proxy_jpeg_quality")
    camera_proxy_ffmpeg_mjpeg_qscale = LaunchConfiguration("camera_proxy_ffmpeg_mjpeg_qscale")
    camera_proxy_max_width = LaunchConfiguration("camera_proxy_max_width")
    camera_proxy_transport = LaunchConfiguration("camera_proxy_transport")
    enable_inspection = LaunchConfiguration("enable_inspection")
    inspection_backend = LaunchConfiguration("inspection_backend")
    inspection_source_type = LaunchConfiguration("inspection_source_type")
    inspection_rtsp_url = LaunchConfiguration("inspection_rtsp_url")
    inspection_camera_name = LaunchConfiguration("inspection_camera_name")
    inspection_model_path = LaunchConfiguration("inspection_model_path")
    inspection_class_names_path = LaunchConfiguration("inspection_class_names_path")
    enable_radar_inspection = LaunchConfiguration("enable_radar_inspection")
    radar_backend = LaunchConfiguration("radar_backend")
    radar_device_url = LaunchConfiguration("radar_device_url")
    radar_request_timeout_s = LaunchConfiguration("radar_request_timeout_s")
    radar_poll_interval_s = LaunchConfiguration("radar_poll_interval_s")
    radar_max_wait_s = LaunchConfiguration("radar_max_wait_s")
    radar_dry_run_duration_s = LaunchConfiguration("radar_dry_run_duration_s")
    radar_scan_mode = LaunchConfiguration("radar_scan_mode")
    radar_scan_density = LaunchConfiguration("radar_scan_density")
    radar_release_on_analysis = LaunchConfiguration("radar_release_on_analysis")
    radar_start_retry_timeout_s = LaunchConfiguration("radar_start_retry_timeout_s")
    radar_start_retry_interval_s = LaunchConfiguration("radar_start_retry_interval_s")
    radar_result_retry_count = LaunchConfiguration("radar_result_retry_count")
    radar_result_retry_interval_s = LaunchConfiguration("radar_result_retry_interval_s")
    radar_query_error_timeout_s = LaunchConfiguration("radar_query_error_timeout_s")
    radar_modeling_scene = LaunchConfiguration("radar_modeling_scene")
    radar_modeling_enable_camera = LaunchConfiguration("radar_modeling_enable_camera")
    radar_inspection_timeout_s = LaunchConfiguration("radar_inspection_timeout_s")
    radar_output_dir = LaunchConfiguration("radar_output_dir")
    use_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_delay_s = LaunchConfiguration("rviz_delay_s")
    enable_axis_command = LaunchConfiguration("enable_axis_command")
    enable_initialpose_relocalization = LaunchConfiguration("enable_initialpose_relocalization")

    return LaunchDescription([
        LogInfo(
            msg=(
                "M20Pro field profile: %s hash=%s"
                % (field_profile["profile_name"], field_profile["profile_hash"])
            )
        ),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument("floor_config", default_value=default_floor_config),
        DeclareLaunchArgument("map_manifest", default_value=default_map_manifest),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "enable_nav2",
            default_value="true" if nav2_stack_available else "false",
            description="Start Nav2 map server and navigation stack when available.",
        ),
        DeclareLaunchArgument("enable_floor_manager", default_value="true"),
        DeclareLaunchArgument("enable_floor_goal_bridge", default_value="true"),
        DeclareLaunchArgument(
            "enable_stair_connector",
            default_value="false",
            description="Start the single unified stair connector executor.",
        ),
        DeclareLaunchArgument("enable_system_check", default_value="true"),
        DeclareLaunchArgument("enable_config_audit", default_value="true"),
        DeclareLaunchArgument(
            "initial_floor",
            default_value="",
            description="Optional runtime map label; empty means ordinary map navigation without a floor registry.",
        ),
        DeclareLaunchArgument("enable_initialpose_3d_adapter", default_value="false"),
        DeclareLaunchArgument("initialpose_3d_z", default_value="0.0"),
        DeclareLaunchArgument("enable_web_dashboard", default_value="false"),
        DeclareLaunchArgument(
            "enable_nav2_startup_gate",
            default_value="true",
            description="Delay Nav2 lifecycle startup until map, scan, localization and TF are ready.",
        ),
        DeclareLaunchArgument("web_dashboard_port", default_value="8080"),
        DeclareLaunchArgument("web_dashboard_data_dir", default_value="~/.m20pro_web"),
        DeclareLaunchArgument("web_dashboard_map_archive_dir", default_value="~/m20pro_maps"),
        DeclareLaunchArgument("robot_pose_display_yaw_offset_rad", default_value="0.0"),
        DeclareLaunchArgument("initialpose_topic", default_value="/initialpose"),
        DeclareLaunchArgument(
            "relocalization_result_topic",
            default_value="/m20pro_tcp_bridge/relocalization_result",
        ),
        DeclareLaunchArgument("factory_host", default_value="10.21.31.106"),
        DeclareLaunchArgument("factory_user", default_value="user"),
        DeclareLaunchArgument("factory_active_map", default_value="/var/opt/robot/data/maps/active"),
        DeclareLaunchArgument("enable_camera_proxy", default_value="false"),
        DeclareLaunchArgument("front_camera_url", default_value="rtsp://10.21.31.103:8554/video1"),
        DeclareLaunchArgument("rear_camera_url", default_value="rtsp://10.21.31.103:8554/video2"),
        DeclareLaunchArgument("camera_proxy_backend", default_value="ffmpeg_mjpeg"),
        DeclareLaunchArgument("camera_proxy_fps", default_value="10.0"),
        DeclareLaunchArgument("camera_proxy_jpeg_quality", default_value="45"),
        DeclareLaunchArgument("camera_proxy_ffmpeg_mjpeg_qscale", default_value="5"),
        DeclareLaunchArgument("camera_proxy_max_width", default_value="480"),
        DeclareLaunchArgument("camera_proxy_transport", default_value="tcp"),
        DeclareLaunchArgument("enable_inspection", default_value="false"),
        DeclareLaunchArgument("inspection_backend", default_value="auto"),
        DeclareLaunchArgument("inspection_source_type", default_value="rtsp"),
        DeclareLaunchArgument("inspection_rtsp_url", default_value="rtsp://10.21.31.103:8554/video1"),
        DeclareLaunchArgument("inspection_camera_name", default_value="front_wide"),
        DeclareLaunchArgument("inspection_model_path", default_value=default_inspection_model),
        DeclareLaunchArgument("inspection_class_names_path", default_value=default_inspection_classes),
        DeclareLaunchArgument("enable_radar_inspection", default_value="false"),
        DeclareLaunchArgument("radar_backend", default_value="dry_run"),
        DeclareLaunchArgument("radar_device_url", default_value="http://192.168.107.72:8080"),
        DeclareLaunchArgument("radar_request_timeout_s", default_value="10.0"),
        DeclareLaunchArgument("radar_poll_interval_s", default_value="2.0"),
        DeclareLaunchArgument("radar_max_wait_s", default_value="1800.0"),
        DeclareLaunchArgument("radar_dry_run_duration_s", default_value="2.0"),
        DeclareLaunchArgument("radar_scan_mode", default_value="measuring"),
        DeclareLaunchArgument("radar_scan_density", default_value="low"),
        DeclareLaunchArgument("radar_release_on_analysis", default_value="true"),
        DeclareLaunchArgument("radar_start_retry_timeout_s", default_value="120.0"),
        DeclareLaunchArgument("radar_start_retry_interval_s", default_value="5.0"),
        DeclareLaunchArgument("radar_result_retry_count", default_value="5"),
        DeclareLaunchArgument("radar_result_retry_interval_s", default_value="2.0"),
        DeclareLaunchArgument("radar_query_error_timeout_s", default_value="120.0"),
        DeclareLaunchArgument("radar_modeling_scene", default_value="modeling"),
        DeclareLaunchArgument("radar_modeling_enable_camera", default_value="false"),
        DeclareLaunchArgument("radar_inspection_timeout_s", default_value="1800.0"),
        DeclareLaunchArgument("radar_output_dir", default_value=""),
        DeclareLaunchArgument(
            "factory_mapping_start_command",
            default_value=(
                "ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "
                "\"nohup sudo -n drmap mapping -b -s -n {map_name} > "
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
        DeclareLaunchArgument("enable_stair_zone_postprocess", default_value="false"),
        DeclareLaunchArgument("stair_zones_topic", default_value="/m20pro/stair_zones"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("rviz_delay_s", default_value="5.0"),
        DeclareLaunchArgument(
            "enable_axis_command",
            default_value="false",
            description="Set true only when 104 is allowed to send /cmd_vel axis commands to 103.",
        ),
        DeclareLaunchArgument(
            "enable_initialpose_relocalization",
            default_value="false",
            description="Diagnostic only: forward /initialpose to the 103 TCP 2101/1 reset API.",
        ),

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
            executable="command_mux",
            name="m20pro_command_mux",
            output="screen",
            parameters=[command_mux_parameters],
        ),
        Node(
            package="m20pro_navigation",
            executable="stair_executor",
            name="m20pro_stair_executor",
            output="screen",
            parameters=[{"enabled": True, **stair_executor_parameters}],
            condition=IfCondition(
                enable_stair_connector if nav2_stack_available else "false"
            ),
        ),
        Node(
            package="m20pro_navigation",
            executable="tcp_bridge",
            name="m20pro_tcp_bridge",
            output="screen",
            parameters=[
                params_file,
                {
                    "publish_tf": True,
                    "base_frame": "m20pro_base_link",
                    "enable_native_goal_bridge": False,
                    "enable_initialpose_relocalization": ParameterValue(
                        enable_initialpose_relocalization,
                        value_type=bool,
                    ),
                    "enable_initialpose_3d_relocalization": False,
                    "enable_axis_command": ParameterValue(enable_axis_command, value_type=bool),
                },
            ],
            condition=UnlessCondition(enable_initialpose_3d_adapter),
        ),
        Node(
            package="m20pro_navigation",
            executable="tcp_bridge",
            name="m20pro_tcp_bridge",
            output="screen",
            parameters=[
                params_file,
                {
                    "publish_tf": True,
                    "base_frame": "m20pro_base_link",
                    "enable_native_goal_bridge": False,
                    "enable_axis_command": ParameterValue(enable_axis_command, value_type=bool),
                    "enable_initialpose_relocalization": False,
                    "enable_initialpose_3d_relocalization": ParameterValue(
                        enable_initialpose_relocalization, value_type=bool
                    ),
                },
            ],
            condition=IfCondition(enable_initialpose_3d_adapter),
        ),
        *(
            [
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    parameters=[{"use_sim_time": False}, {"yaml_filename": map_yaml}],
                    condition=IfCondition(enable_nav2),
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
                    condition=IfCondition(enable_nav2),
                ),
            ]
            if nav2_stack_available
            else [
                LogInfo(
                    msg=(
                        "Nav2 packages are not installed; starting M20Pro real "
                        "bringup in observation mode without map_server/navigation."
                    )
                )
            ]
        ),
        Node(
            package="m20pro_navigation",
            executable="scan_recording_relay",
            name="m20pro_scan_recording_relay",
            output="screen",
            parameters=[
                {
                    # Mirror the same canonical /scan stream consumed by Nav2.
                    "input_topic": scan_topic,
                    "output_topic": "/m20pro/recording_scan",
                }
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
                    "cmd_vel_topic": "/cmd_vel_nav",
                    **floor_manager_parameters,
                }
            ],
            condition=IfCondition(enable_floor_manager if nav2_stack_available else "false"),
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
            condition=IfCondition(enable_floor_goal_bridge if nav2_stack_available else "false"),
        ),
        Node(
            package="m20pro_navigation",
            executable="initialpose_3d_adapter",
            name="m20pro_initialpose_3d_adapter",
            output="screen",
            parameters=[
                {
                    "enabled": True,
                    "z": ParameterValue(initialpose_3d_z, value_type=float),
                    "config_file": floor_config,
                }
            ],
            condition=IfCondition(enable_initialpose_3d_adapter),
        ),
        *(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_share, "launch", "nav2_navigation_real_foxy.launch.py")
                    ),
                    launch_arguments={
                        "params_file": nav2_params_file,
                        "use_sim_time": "False",
                        "use_composition": "False",
                        "map_subscribe_transient_local": "True",
                        "autostart": "False",
                        "cmd_vel_topic": "/cmd_vel_nav",
                    }.items(),
                    condition=IfCondition(enable_nav2),
                )
            ]
            if nav2_stack_available
            else []
        ),
        Node(
            package="m20pro_navigation",
            executable="nav2_startup_gate",
            name="m20pro_nav2_startup_gate",
            output="screen",
            parameters=[
                {
                    "enabled": ParameterValue(enable_nav2_startup_gate, value_type=bool),
                    "scan_topic": scan_topic,
                    "required_fresh_age_s": 3.0,
                    "startup_retry_s": 10.0,
                    "startup_timeout_s": 20.0,
                    "tf_global_frame": "map",
                    "tf_base_frame": "m20pro_base_link",
                }
            ],
            condition=IfCondition(enable_nav2 if nav2_stack_available else "false"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(inspection_launch),
            launch_arguments={
                "enabled": enable_inspection,
                "backend": inspection_backend,
                "source_type": inspection_source_type,
                "rtsp_url": inspection_rtsp_url,
                "camera_name": inspection_camera_name,
                "model_path": inspection_model_path,
                "class_names_path": inspection_class_names_path,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(radar_launch),
            launch_arguments={
                "radar_backend": radar_backend,
                "radar_device_url": radar_device_url,
                "radar_request_timeout_s": radar_request_timeout_s,
                "radar_poll_interval_s": radar_poll_interval_s,
                "radar_max_wait_s": radar_max_wait_s,
                "radar_dry_run_duration_s": radar_dry_run_duration_s,
                "radar_scan_mode": radar_scan_mode,
                "radar_scan_density": radar_scan_density,
                "radar_release_on_analysis": radar_release_on_analysis,
                "radar_start_retry_timeout_s": radar_start_retry_timeout_s,
                "radar_start_retry_interval_s": radar_start_retry_interval_s,
                "radar_result_retry_count": radar_result_retry_count,
                "radar_result_retry_interval_s": radar_result_retry_interval_s,
                "radar_query_error_timeout_s": radar_query_error_timeout_s,
                "radar_modeling_scene": radar_modeling_scene,
                "radar_modeling_enable_camera": radar_modeling_enable_camera,
                "radar_output_dir": radar_output_dir,
            }.items(),
            condition=IfCondition(enable_radar_inspection),
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
                    "robot_pose_display_yaw_offset_rad": ParameterValue(
                        robot_pose_display_yaw_offset_rad,
                        value_type=float,
                    ),
                    "map_manifest": map_manifest,
                    "floor_config": floor_config,
                    "initialpose_topic": initialpose_topic,
                    "relocalization_result_topic": relocalization_result_topic,
                    "factory_host": factory_host,
                    "factory_user": factory_user,
                    "factory_active_map": factory_active_map,
                    "factory_mapping_start_command": factory_mapping_start_command,
                    "factory_mapping_finish_command": factory_mapping_finish_command,
                    "factory_mapping_cancel_command": factory_mapping_cancel_command,
                    "enable_stair_zone_postprocess": ParameterValue(
                        enable_stair_zone_postprocess,
                        value_type=bool,
                    ),
                    "stair_zones_topic": stair_zones_topic,
                    # The Web node observes the exact /scan stream consumed by Nav2.
                    "scan_topic": scan_topic,
                    "startup_sync_selected_map_delay_s": 6.0,
                    "enable_camera_proxy": ParameterValue(enable_camera_proxy, value_type=bool),
                    "front_camera_url": front_camera_url,
                    "rear_camera_url": rear_camera_url,
                    "camera_proxy_backend": camera_proxy_backend,
                    "camera_proxy_fps": ParameterValue(camera_proxy_fps, value_type=float),
                    "camera_proxy_jpeg_quality": ParameterValue(
                        camera_proxy_jpeg_quality,
                        value_type=int,
                    ),
                    "camera_proxy_ffmpeg_mjpeg_qscale": ParameterValue(
                        camera_proxy_ffmpeg_mjpeg_qscale,
                        value_type=int,
                    ),
                    "camera_proxy_max_width": ParameterValue(
                        camera_proxy_max_width,
                        value_type=int,
                    ),
                    "camera_proxy_transport": camera_proxy_transport,
                    "wait_for_radar_inspection": ParameterValue(
                        enable_radar_inspection,
                        value_type=bool,
                    ),
                    "radar_inspection_timeout_s": ParameterValue(
                        radar_inspection_timeout_s,
                        value_type=float,
                    ),
                    "radar_results_dir": radar_output_dir,
                    "cmd_vel_topic": "/cmd_vel_nav",
                    "teleop_cmd_vel_topic": "/cmd_vel_teleop",
                    **web_navigation_parameters,
                    **web_teleoperation_parameters,
                }
            ],
            condition=IfCondition(enable_web_dashboard),
        ),
        Node(
            package="m20pro_navigation",
            executable="system_check",
            name="m20pro_system_check",
            output="screen",
            parameters=[
                {
                    "mode": "real",
                    "scan_topic": scan_topic,
                    "scan_timeout_s": 3.0,
                    "check_period_s": 5.0,
                    "require_topic_messages": False,
                    "require_nav2": nav2_stack_available,
                    "require_costmaps": nav2_stack_available,
                    "require_map": nav2_stack_available,
                    "require_nodes": nav2_stack_available,
                    "require_scan": nav2_stack_available,
                    "require_dynamic_obstacles": False,
                    "require_floor_manager": nav2_stack_available,
                    "check_scan_content": True,
                    "check_local_costmap_content": False,
                    "check_tf_height": False,
                    "tf_global_frame": "odom",
                    "tf_base_frame": "m20pro_base_link",
                    "max_abs_base_z": 1.0,
                    "min_scan_finite_bins": 20,
                    "min_scan_close_bins": 1,
                    "scan_close_range_m": 2.0,
                    "min_local_costmap_marked_cells": 1,
                }
            ],
            condition=IfCondition(enable_system_check),
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
