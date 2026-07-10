import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")
    inspection_share = get_package_share_directory("m20pro_inspection")
    real_launch = os.path.join(bringup_share, "launch", "m20pro_real.launch.py")

    mode = LaunchConfiguration("mode")
    rviz = LaunchConfiguration("rviz")
    initial_floor = LaunchConfiguration("initial_floor")
    map_yaml = LaunchConfiguration("map")
    floor_config = LaunchConfiguration("floor_config")
    map_manifest = LaunchConfiguration("map_manifest")
    real_params_file = LaunchConfiguration("real_params_file")
    real_nav2_params_file = LaunchConfiguration("real_nav2_params_file")
    enable_axis_command = LaunchConfiguration("enable_axis_command")
    enable_initialpose_relocalization = LaunchConfiguration("enable_initialpose_relocalization")
    enable_initialpose_3d_adapter = LaunchConfiguration("enable_initialpose_3d_adapter")
    initialpose_3d_z = LaunchConfiguration("initialpose_3d_z")
    enable_web_dashboard = LaunchConfiguration("enable_web_dashboard")
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="real",
            description="Real-only workspace compatibility argument; use real.",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("initial_floor", default_value="F20"),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(bringup_share, "maps", "F20", "occ_grid.yaml"),
        ),
        DeclareLaunchArgument(
            "floor_config",
            default_value=os.path.join(bringup_share, "config", "inspection_waypoints.yaml"),
        ),
        DeclareLaunchArgument(
            "map_manifest",
            default_value=os.path.join(bringup_share, "config", "map_manifest.yaml"),
        ),
        DeclareLaunchArgument(
            "real_params_file",
            default_value=os.path.join(bringup_share, "config", "m20pro_real.yaml"),
        ),
        DeclareLaunchArgument(
            "real_nav2_params_file",
            default_value=os.path.join(bringup_share, "config", "nav2_params_real.yaml"),
        ),
        DeclareLaunchArgument("enable_axis_command", default_value="false"),
        DeclareLaunchArgument(
            "enable_initialpose_relocalization",
            default_value="false",
            description="Diagnostic only: let m20pro_tcp_bridge consume /initialpose and forward it to 103 TCP 2101/1.",
        ),
        DeclareLaunchArgument("enable_initialpose_3d_adapter", default_value="false"),
        DeclareLaunchArgument("initialpose_3d_z", default_value="0.0"),
        DeclareLaunchArgument("enable_web_dashboard", default_value="false"),
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
        DeclareLaunchArgument(
            "inspection_model_path",
            default_value=os.path.join(inspection_share, "models", "best_rk3588_fp16.rknn"),
        ),
        DeclareLaunchArgument(
            "inspection_class_names_path",
            default_value=os.path.join(inspection_share, "models", "labels_zh.txt"),
        ),
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
        DeclareLaunchArgument("enable_stair_zone_postprocess", default_value="true"),

        LogInfo(
            msg="m20pro_bringup is real-only; non-real mode is unsupported in this workspace.",
            condition=IfCondition(PythonExpression(["'", mode, "' != 'real'"])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                "rviz": rviz,
                "initial_floor": initial_floor,
                "map": map_yaml,
                "floor_config": floor_config,
                "map_manifest": map_manifest,
                "params_file": real_params_file,
                "nav2_params_file": real_nav2_params_file,
                "enable_axis_command": enable_axis_command,
                "enable_initialpose_relocalization": enable_initialpose_relocalization,
                "enable_initialpose_3d_adapter": enable_initialpose_3d_adapter,
                "initialpose_3d_z": initialpose_3d_z,
                "enable_web_dashboard": enable_web_dashboard,
                "web_dashboard_port": web_dashboard_port,
                "web_dashboard_data_dir": web_dashboard_data_dir,
                "web_dashboard_map_archive_dir": web_dashboard_map_archive_dir,
                "robot_pose_display_yaw_offset_rad": robot_pose_display_yaw_offset_rad,
                "initialpose_topic": initialpose_topic,
                "relocalization_result_topic": relocalization_result_topic,
                "factory_host": factory_host,
                "factory_user": factory_user,
                "factory_active_map": factory_active_map,
                "factory_mapping_start_command": factory_mapping_start_command,
                "factory_mapping_finish_command": factory_mapping_finish_command,
                "factory_mapping_cancel_command": factory_mapping_cancel_command,
                "enable_stair_zone_postprocess": enable_stair_zone_postprocess,
                "enable_camera_proxy": enable_camera_proxy,
                "front_camera_url": front_camera_url,
                "rear_camera_url": rear_camera_url,
                "camera_proxy_backend": camera_proxy_backend,
                "camera_proxy_fps": camera_proxy_fps,
                "camera_proxy_jpeg_quality": camera_proxy_jpeg_quality,
                "camera_proxy_ffmpeg_mjpeg_qscale": camera_proxy_ffmpeg_mjpeg_qscale,
                "camera_proxy_max_width": camera_proxy_max_width,
                "camera_proxy_transport": camera_proxy_transport,
                "enable_inspection": enable_inspection,
                "inspection_backend": inspection_backend,
                "inspection_source_type": inspection_source_type,
                "inspection_rtsp_url": inspection_rtsp_url,
                "inspection_camera_name": inspection_camera_name,
                "inspection_model_path": inspection_model_path,
                "inspection_class_names_path": inspection_class_names_path,
            }.items(),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
        ),
    ])
