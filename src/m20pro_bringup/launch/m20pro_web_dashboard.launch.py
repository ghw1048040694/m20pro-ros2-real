from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    data_dir = LaunchConfiguration("data_dir")
    map_archive_dir = LaunchConfiguration("map_archive_dir")
    factory_host = LaunchConfiguration("factory_host")
    factory_user = LaunchConfiguration("factory_user")
    factory_active_map = LaunchConfiguration("factory_active_map")
    factory_mapping_start_command = LaunchConfiguration("factory_mapping_start_command")
    factory_mapping_finish_command = LaunchConfiguration("factory_mapping_finish_command")
    factory_mapping_cancel_command = LaunchConfiguration("factory_mapping_cancel_command")

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("data_dir", default_value="~/.m20pro_web"),
        DeclareLaunchArgument("map_archive_dir", default_value="~/m20pro_maps"),
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
                    "factory_host": factory_host,
                    "factory_user": factory_user,
                    "factory_active_map": factory_active_map,
                    "factory_mapping_start_command": factory_mapping_start_command,
                    "factory_mapping_finish_command": factory_mapping_finish_command,
                    "factory_mapping_cancel_command": factory_mapping_cancel_command,
                }
            ],
        ),
    ])
