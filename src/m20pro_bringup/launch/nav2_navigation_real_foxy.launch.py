import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from m20pro_navigation.field_profile_contract import (
    load_field_profile,
    nav2_parameter_rewrites,
)


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")
    field_profile = load_field_profile(
        os.path.join(bringup_share, "config", "m20pro_field_profile.yaml")
    )
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    map_subscribe_transient_local = LaunchConfiguration("map_subscribe_transient_local")

    # Real tasks sequence NavigateToPose actions in the project task manager.
    # Keeping waypoint_follower in this lifecycle makes an unused component a
    # single point of failure for all navigation startup on Foxy.
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "recoveries_server",
        "bt_navigator",
    ]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            "use_sim_time": use_sim_time,
            "default_bt_xml_filename": default_bt_xml_filename,
            "autostart": autostart,
            "map_subscribe_transient_local": map_subscribe_transient_local,
            **nav2_parameter_rewrites(field_profile),
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2_params_real.yaml"),
            ),
            DeclareLaunchArgument(
                "default_bt_xml_filename",
                default_value=os.path.join(
                    bringup_share,
                    "behavior_trees",
                    "m20pro_navigate_to_pose_foxy.xml",
                ),
            ),
            DeclareLaunchArgument("map_subscribe_transient_local", default_value="true"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_recoveries",
                executable="recoveries_server",
                name="recoveries_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
