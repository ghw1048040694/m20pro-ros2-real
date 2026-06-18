import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")

    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    data_dir = LaunchConfiguration("data_dir")
    map_archive_dir = LaunchConfiguration("map_archive_dir")
    robot_pose_display_yaw_offset_rad = LaunchConfiguration("robot_pose_display_yaw_offset_rad")
    map_manifest = LaunchConfiguration("map_manifest")
    factory_host = LaunchConfiguration("factory_host")
    factory_user = LaunchConfiguration("factory_user")
    factory_active_map = LaunchConfiguration("factory_active_map")
    factory_mapping_start_command = LaunchConfiguration("factory_mapping_start_command")
    factory_mapping_finish_command = LaunchConfiguration("factory_mapping_finish_command")
    factory_mapping_cancel_command = LaunchConfiguration("factory_mapping_cancel_command")
    enable_map_pcd_postprocess = LaunchConfiguration("enable_map_pcd_postprocess")
    pcd_terrain_cell_size = LaunchConfiguration("pcd_terrain_cell_size")
    stair_zones_topic = LaunchConfiguration("stair_zones_topic")
    enable_camera_proxy = LaunchConfiguration("enable_camera_proxy")
    front_camera_url = LaunchConfiguration("front_camera_url")
    rear_camera_url = LaunchConfiguration("rear_camera_url")
    camera_proxy_fps = LaunchConfiguration("camera_proxy_fps")
    camera_proxy_jpeg_quality = LaunchConfiguration("camera_proxy_jpeg_quality")
    camera_proxy_max_width = LaunchConfiguration("camera_proxy_max_width")
    camera_proxy_transport = LaunchConfiguration("camera_proxy_transport")
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    relocalization_result_topic = LaunchConfiguration("relocalization_result_topic")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("data_dir", default_value="~/.m20pro_web"),
        DeclareLaunchArgument("map_archive_dir", default_value="~/m20pro_maps"),
        DeclareLaunchArgument("robot_pose_display_yaw_offset_rad", default_value="0.0"),
        DeclareLaunchArgument(
            "map_manifest",
            default_value=os.path.join(bringup_share, "config", "map_manifest.yaml"),
        ),
        DeclareLaunchArgument("factory_host", default_value="10.21.31.106"),
        DeclareLaunchArgument("factory_user", default_value="user"),
        DeclareLaunchArgument(
            "factory_active_map",
            default_value="/var/opt/robot/data/maps/active",
        ),
        DeclareLaunchArgument(
            "factory_mapping_start_command",
            default_value=(
                "ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "
                "\"nohup sudo -n drmap mapping -s -n {map_name} > "
                "/tmp/m20pro_drmap_mapping_{session_id}.log 2>&1 &\""
            ),
            description="Shell command template for starting drmap mapping on 106.",
        ),
        DeclareLaunchArgument(
            "factory_mapping_finish_command",
            default_value=(
                "ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "
                "\"sudo -n drmap stop_mapping\""
            ),
            description="Shell command template for stopping/saving drmap mapping on 106.",
        ),
        DeclareLaunchArgument(
            "factory_mapping_cancel_command",
            default_value=(
                "ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "
                "\"sudo -n drmap stop_mapping\""
            ),
            description="Shell command template for cancelling drmap mapping on 106.",
        ),
        DeclareLaunchArgument("enable_map_pcd_postprocess", default_value="true"),
        DeclareLaunchArgument("pcd_terrain_cell_size", default_value="0.25"),
        DeclareLaunchArgument("stair_zones_topic", default_value="/m20pro/stair_zones"),
        DeclareLaunchArgument("enable_camera_proxy", default_value="false"),
        DeclareLaunchArgument("front_camera_url", default_value="rtsp://10.21.31.103:8554/video1"),
        DeclareLaunchArgument("rear_camera_url", default_value="rtsp://10.21.31.103:8554/video2"),
        DeclareLaunchArgument("camera_proxy_fps", default_value="3.0"),
        DeclareLaunchArgument("camera_proxy_jpeg_quality", default_value="55"),
        DeclareLaunchArgument("camera_proxy_max_width", default_value="480"),
        DeclareLaunchArgument("camera_proxy_transport", default_value="tcp"),
        DeclareLaunchArgument("initialpose_topic", default_value="/initialpose"),
        DeclareLaunchArgument(
            "relocalization_result_topic",
            default_value="/m20pro_tcp_bridge/relocalization_result",
        ),
        Node(
            package="m20pro_cloud_bridge",
            executable="web_dashboard",
            name="m20pro_web_dashboard",
            output="screen",
            parameters=[
                {
                    "host": host,
                    "port": port,
                    "data_dir": data_dir,
                    "map_archive_dir": map_archive_dir,
                    "robot_pose_display_yaw_offset_rad": ParameterValue(
                        robot_pose_display_yaw_offset_rad,
                        value_type=float,
                    ),
                    "map_manifest": map_manifest,
                    "factory_host": factory_host,
                    "factory_user": factory_user,
                    "factory_active_map": factory_active_map,
                    "factory_mapping_start_command": factory_mapping_start_command,
                    "factory_mapping_finish_command": factory_mapping_finish_command,
                    "factory_mapping_cancel_command": factory_mapping_cancel_command,
                    "enable_map_pcd_postprocess": ParameterValue(
                        enable_map_pcd_postprocess,
                        value_type=bool,
                    ),
                    "pcd_terrain_cell_size": ParameterValue(
                        pcd_terrain_cell_size,
                        value_type=float,
                    ),
                    "stair_zones_topic": stair_zones_topic,
                    "enable_camera_proxy": ParameterValue(enable_camera_proxy, value_type=bool),
                    "front_camera_url": front_camera_url,
                    "rear_camera_url": rear_camera_url,
                    "camera_proxy_fps": ParameterValue(camera_proxy_fps, value_type=float),
                    "camera_proxy_jpeg_quality": ParameterValue(
                        camera_proxy_jpeg_quality,
                        value_type=int,
                    ),
                    "camera_proxy_max_width": ParameterValue(
                        camera_proxy_max_width,
                        value_type=int,
                    ),
                    "camera_proxy_transport": camera_proxy_transport,
                    "initialpose_topic": initialpose_topic,
                    "relocalization_result_topic": relocalization_result_topic,
                }
            ],
        ),
    ])
