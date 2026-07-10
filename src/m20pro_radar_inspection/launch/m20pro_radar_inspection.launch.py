from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    backend = LaunchConfiguration("radar_backend")
    device_url = LaunchConfiguration("radar_device_url")
    request_timeout_s = LaunchConfiguration("radar_request_timeout_s")
    poll_interval_s = LaunchConfiguration("radar_poll_interval_s")
    max_wait_s = LaunchConfiguration("radar_max_wait_s")
    dry_run_duration_s = LaunchConfiguration("radar_dry_run_duration_s")
    scan_mode = LaunchConfiguration("radar_scan_mode")
    scan_density = LaunchConfiguration("radar_scan_density")
    release_on_analysis = LaunchConfiguration("radar_release_on_analysis")
    start_retry_timeout_s = LaunchConfiguration("radar_start_retry_timeout_s")
    start_retry_interval_s = LaunchConfiguration("radar_start_retry_interval_s")
    result_retry_count = LaunchConfiguration("radar_result_retry_count")
    result_retry_interval_s = LaunchConfiguration("radar_result_retry_interval_s")
    query_error_timeout_s = LaunchConfiguration("radar_query_error_timeout_s")
    modeling_scene = LaunchConfiguration("radar_modeling_scene")
    modeling_enable_camera = LaunchConfiguration("radar_modeling_enable_camera")
    output_dir = LaunchConfiguration("radar_output_dir")

    return LaunchDescription([
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
        DeclareLaunchArgument("radar_output_dir", default_value="~/.m20pro_radar_results"),
        Node(
            package="m20pro_radar_inspection",
            executable="radar_inspection",
            name="m20pro_radar_inspection",
            output="screen",
            parameters=[
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
                    "result_retry_count": ParameterValue(result_retry_count, value_type=int),
                    "result_retry_interval_s": ParameterValue(result_retry_interval_s, value_type=float),
                    "query_error_timeout_s": ParameterValue(query_error_timeout_s, value_type=float),
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
