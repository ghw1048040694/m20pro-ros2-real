import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")
    desc_share = get_package_share_directory("m20pro_description")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(bringup_share, "config", "m20pro.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_urdf = os.path.join(desc_share, "urdf", "M20.urdf")
    default_map = os.path.join(
        bringup_share, "maps", "working_1-20260429-162852_edited3", "occ_grid.yaml"
    )
    default_rviz = os.path.join(bringup_share, "rviz", "m20pro_sim.rviz")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    with open(default_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),

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
            executable="sim_bridge",
            name="m20pro_tcp_bridge",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="m20pro_navigation",
            executable="dual_lidar_simulator",
            name="m20pro_dual_lidar_simulator",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="m20pro_navigation",
            executable="pointcloud_fusion",
            name="m20pro_pointcloud_fusion",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="m20pro_navigation",
            executable="dynamic_obstacle_simulator",
            name="m20pro_dynamic_obstacle_simulator",
            output="screen",
            parameters=[params_file],
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params,
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
