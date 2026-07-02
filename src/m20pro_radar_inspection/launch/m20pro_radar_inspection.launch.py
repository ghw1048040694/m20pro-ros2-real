import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    radar_share = get_package_share_directory("m20pro_radar_inspection")
    default_config = os.path.join(radar_share, "config", "radar_inspection.yaml")

    config_file = LaunchConfiguration("config_file")
    backend = LaunchConfiguration("backend")
    device_url = LaunchConfiguration("device_url")
    request_timeout_s = LaunchConfiguration("request_timeout_s")
    poll_interval_s = LaunchConfiguration("poll_interval_s")
    max_wait_s = LaunchConfiguration("max_wait_s")
    dry_run_duration_s = LaunchConfiguration("dry_run_duration_s")
    scan_mode = LaunchConfiguration("scan_mode")
    scan_density = LaunchConfiguration("scan_density")
    release_on_analysis = LaunchConfiguration("release_on_analysis")
    start_retry_timeout_s = LaunchConfiguration("start_retry_timeout_s")
    start_retry_interval_s = LaunchConfiguration("start_retry_interval_s")
    modeling_scene = LaunchConfiguration("modeling_scene")
    modeling_enable_camera = LaunchConfiguration("modeling_enable_camera")
    output_dir = LaunchConfiguration("output_dir")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("backend", default_value="dry_run"),
        DeclareLaunchArgument("device_url", default_value="http://192.168.107.72:8080"),
        DeclareLaunchArgument("request_timeout_s", default_value="10.0"),
        DeclareLaunchArgument("poll_interval_s", default_value="2.0"),
        DeclareLaunchArgument("max_wait_s", default_value="1800.0"),
        DeclareLaunchArgument("dry_run_duration_s", default_value="2.0"),
        DeclareLaunchArgument("scan_mode", default_value="measuring"),
        DeclareLaunchArgument("scan_density", default_value="low"),
        DeclareLaunchArgument("release_on_analysis", default_value="true"),
        DeclareLaunchArgument("start_retry_timeout_s", default_value="120.0"),
        DeclareLaunchArgument("start_retry_interval_s", default_value="5.0"),
        DeclareLaunchArgument("modeling_scene", default_value="modeling"),
        DeclareLaunchArgument("modeling_enable_camera", default_value="false"),
        DeclareLaunchArgument("output_dir", default_value="~/.m20pro_radar_results"),
        Node(
            package="m20pro_radar_inspection",
            executable="radar_inspection",
            name="m20pro_radar_inspection",
            output="screen",
            parameters=[
                config_file,
                {
                    "backend": backend,
                    "device_url": device_url,
                    "request_timeout_s": ParameterValue(request_timeout_s, value_type=float),
                    "poll_interval_s": ParameterValue(poll_interval_s, value_type=float),
                    "max_wait_s": ParameterValue(max_wait_s, value_type=float),
                    "dry_run_duration_s": ParameterValue(dry_run_duration_s, value_type=float),
                    "scan_mode": scan_mode,
                    "scan_density": scan_density,
                    "release_on_analysis": ParameterValue(release_on_analysis, value_type=bool),
                    "start_retry_timeout_s": ParameterValue(start_retry_timeout_s, value_type=float),
                    "start_retry_interval_s": ParameterValue(start_retry_interval_s, value_type=float),
                    "modeling_scene": modeling_scene,
                    "modeling_enable_camera": ParameterValue(
                        modeling_enable_camera,
                        value_type=bool,
                    ),
                    "output_dir": output_dir,
                },
            ],
        ),
    ])
