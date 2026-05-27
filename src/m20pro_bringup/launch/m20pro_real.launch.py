import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")
    desc_share = get_package_share_directory("m20pro_description")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(bringup_share, "config", "m20pro.yaml")
    default_nav2_params = os.path.join(bringup_share, "config", "nav2_params_foxy.yaml")
    default_floor_config = os.path.join(bringup_share, "config", "inspection_waypoints.yaml")
    default_urdf = os.path.join(desc_share, "urdf", "M20.urdf")
    default_map = os.path.join(bringup_share, "maps", "F20", "occ_grid.yaml")
    default_rviz = os.path.join(bringup_share, "rviz", "m20pro_sim.rviz")

    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    floor_config = LaunchConfiguration("floor_config")
    map_yaml = LaunchConfiguration("map")
    cloud_topic = LaunchConfiguration("cloud_topic")
    use_fusion = LaunchConfiguration("fusion")
    enable_floor_manager = LaunchConfiguration("enable_floor_manager")
    initial_floor = LaunchConfiguration("initial_floor")
    load_initial_floor = LaunchConfiguration("load_initial_floor")
    enable_initialpose_3d_adapter = LaunchConfiguration("enable_initialpose_3d_adapter")
    initialpose_3d_z = LaunchConfiguration("initialpose_3d_z")
    use_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_axis_command = LaunchConfiguration("enable_axis_command")
    enable_initialpose_relocalization = LaunchConfiguration("enable_initialpose_relocalization")

    with open(default_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument("floor_config", default_value=default_floor_config),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_nav"),
        DeclareLaunchArgument("fusion", default_value="true"),
        DeclareLaunchArgument("enable_floor_manager", default_value="true"),
        DeclareLaunchArgument("initial_floor", default_value="F20"),
        DeclareLaunchArgument("load_initial_floor", default_value="false"),
        DeclareLaunchArgument("enable_initialpose_3d_adapter", default_value="false"),
        DeclareLaunchArgument("initialpose_3d_z", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument(
            "enable_axis_command",
            default_value="false",
            description="Set true only when 104 is allowed to send /cmd_vel axis commands to 103.",
        ),
        DeclareLaunchArgument(
            "enable_initialpose_relocalization",
            default_value="true",
            description="Forward RViz /initialpose to the vendor localization reset API.",
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
            package="m20pro_navigation",
            executable="tcp_bridge",
            name="m20pro_tcp_bridge",
            output="screen",
            parameters=[
                params_file,
                {
                    "publish_tf": True,
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
        Node(
            package="m20pro_navigation",
            executable="pointcloud_fusion",
            name="m20pro_pointcloud_fusion",
            output="screen",
            parameters=[
                params_file,
                {
                    "input_cloud_topic": cloud_topic,
                },
            ],
            condition=IfCondition(use_fusion),
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
                    "load_initial_floor": ParameterValue(load_initial_floor, value_type=bool),
                }
            ],
            condition=IfCondition(enable_floor_manager),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params_file,
                "use_sim_time": "False",
                "use_composition": "False",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
