import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")
    inspection_share = get_package_share_directory("m20pro_inspection")
    sim_launch = os.path.join(bringup_share, "launch", "m20pro_sim.launch.py")
    real_launch = os.path.join(bringup_share, "launch", "m20pro_real.launch.py")

    mode = LaunchConfiguration("mode")
    rviz = LaunchConfiguration("rviz")
    initial_floor = LaunchConfiguration("initial_floor")
    map_yaml = LaunchConfiguration("map")
    floor_config = LaunchConfiguration("floor_config")
    map_manifest = LaunchConfiguration("map_manifest")
    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    real_nav2_params_file = LaunchConfiguration("real_nav2_params_file")
    cloud_topic = LaunchConfiguration("cloud_topic")
    enable_axis_command = LaunchConfiguration("enable_axis_command")
    enable_initialpose_3d_adapter = LaunchConfiguration("enable_initialpose_3d_adapter")
    initialpose_3d_z = LaunchConfiguration("initialpose_3d_z")
    enable_dynamic_obstacles = LaunchConfiguration("enable_dynamic_obstacles")
    enable_web_dashboard = LaunchConfiguration("enable_web_dashboard")
    web_dashboard_port = LaunchConfiguration("web_dashboard_port")
    web_dashboard_data_dir = LaunchConfiguration("web_dashboard_data_dir")
    web_dashboard_map_archive_dir = LaunchConfiguration("web_dashboard_map_archive_dir")
    factory_host = LaunchConfiguration("factory_host")
    factory_user = LaunchConfiguration("factory_user")
    factory_active_map = LaunchConfiguration("factory_active_map")
    factory_mapping_start_command = LaunchConfiguration("factory_mapping_start_command")
    factory_mapping_finish_command = LaunchConfiguration("factory_mapping_finish_command")
    factory_mapping_cancel_command = LaunchConfiguration("factory_mapping_cancel_command")
    enable_camera_proxy = LaunchConfiguration("enable_camera_proxy")
    front_camera_url = LaunchConfiguration("front_camera_url")
    rear_camera_url = LaunchConfiguration("rear_camera_url")
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
            default_value="sim",
            description="sim or real. This is the unified M20Pro startup entry.",
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
            "params_file",
            default_value=os.path.join(bringup_share, "config", "m20pro.yaml"),
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(bringup_share, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument(
            "real_nav2_params_file",
            default_value=os.path.join(bringup_share, "config", "nav2_params_foxy.yaml"),
        ),
        DeclareLaunchArgument("cloud_topic", default_value="/LIDAR/POINTS"),
        DeclareLaunchArgument("enable_axis_command", default_value="false"),
        DeclareLaunchArgument("enable_initialpose_3d_adapter", default_value="false"),
        DeclareLaunchArgument("initialpose_3d_z", default_value="0.0"),
        DeclareLaunchArgument("enable_dynamic_obstacles", default_value="true"),
        DeclareLaunchArgument("enable_web_dashboard", default_value="true"),
        DeclareLaunchArgument("web_dashboard_port", default_value="8080"),
        DeclareLaunchArgument("web_dashboard_data_dir", default_value="~/.m20pro_web"),
        DeclareLaunchArgument("web_dashboard_map_archive_dir", default_value="~/m20pro_maps"),
        DeclareLaunchArgument("factory_host", default_value="10.21.31.106"),
        DeclareLaunchArgument("factory_user", default_value="user"),
        DeclareLaunchArgument("factory_active_map", default_value="/var/opt/robot/data/maps/active"),
        DeclareLaunchArgument("enable_camera_proxy", default_value="true"),
        DeclareLaunchArgument("front_camera_url", default_value="rtsp://10.21.31.103:8554/video1"),
        DeclareLaunchArgument("rear_camera_url", default_value="rtsp://10.21.31.103:8554/video2"),
        DeclareLaunchArgument("enable_inspection", default_value="true"),
        DeclareLaunchArgument("inspection_backend", default_value="dry_run"),
        DeclareLaunchArgument("inspection_source_type", default_value="rtsp"),
        DeclareLaunchArgument("inspection_rtsp_url", default_value="rtsp://10.21.31.103:8554/video1"),
        DeclareLaunchArgument("inspection_camera_name", default_value="front_wide"),
        DeclareLaunchArgument(
            "inspection_model_path",
            default_value=os.path.join(inspection_share, "models", "playphone_bg_best_rk3588_int8.rknn"),
        ),
        DeclareLaunchArgument(
            "inspection_class_names_path",
            default_value=os.path.join(inspection_share, "models", "labels_zh.txt"),
        ),
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "rviz": rviz,
                "initial_floor": initial_floor,
                "map": map_yaml,
                "floor_config": floor_config,
                "map_manifest": map_manifest,
                "params_file": params_file,
                "nav2_params_file": nav2_params_file,
                "enable_dynamic_obstacles": enable_dynamic_obstacles,
                "enable_web_dashboard": enable_web_dashboard,
                "web_dashboard_port": web_dashboard_port,
                "web_dashboard_data_dir": web_dashboard_data_dir,
                "web_dashboard_map_archive_dir": web_dashboard_map_archive_dir,
                "factory_host": factory_host,
                "factory_user": factory_user,
                "factory_active_map": factory_active_map,
                "factory_mapping_start_command": factory_mapping_start_command,
                "factory_mapping_finish_command": factory_mapping_finish_command,
                "factory_mapping_cancel_command": factory_mapping_cancel_command,
                "enable_camera_proxy": enable_camera_proxy,
                "front_camera_url": front_camera_url,
                "rear_camera_url": rear_camera_url,
                "enable_inspection": "false",
            }.items(),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                "rviz": rviz,
                "initial_floor": initial_floor,
                "map": map_yaml,
                "floor_config": floor_config,
                "map_manifest": map_manifest,
                "params_file": params_file,
                "nav2_params_file": real_nav2_params_file,
                "cloud_topic": cloud_topic,
                "enable_axis_command": enable_axis_command,
                "enable_initialpose_3d_adapter": enable_initialpose_3d_adapter,
                "initialpose_3d_z": initialpose_3d_z,
                "enable_web_dashboard": enable_web_dashboard,
                "web_dashboard_port": web_dashboard_port,
                "web_dashboard_data_dir": web_dashboard_data_dir,
                "web_dashboard_map_archive_dir": web_dashboard_map_archive_dir,
                "factory_host": factory_host,
                "factory_user": factory_user,
                "factory_active_map": factory_active_map,
                "factory_mapping_start_command": factory_mapping_start_command,
                "factory_mapping_finish_command": factory_mapping_finish_command,
                "factory_mapping_cancel_command": factory_mapping_cancel_command,
                "enable_camera_proxy": enable_camera_proxy,
                "front_camera_url": front_camera_url,
                "rear_camera_url": rear_camera_url,
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
