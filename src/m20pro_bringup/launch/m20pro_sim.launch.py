import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("m20pro_bringup")
    desc_share = get_package_share_directory("m20pro_description")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(bringup_share, "config", "m20pro.yaml")
    default_nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")
    default_floor_config = os.path.join(bringup_share, "config", "inspection_waypoints.yaml")
    default_urdf = os.path.join(desc_share, "urdf", "M20.urdf")
    default_map = os.path.join(bringup_share, "maps", "F20", "occ_grid.yaml")
    default_rviz = os.path.join(bringup_share, "rviz", "m20pro_sim.rviz")

    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    floor_config = LaunchConfiguration("floor_config")
    map_yaml = LaunchConfiguration("map")
    enable_floor_manager = LaunchConfiguration("enable_floor_manager")
    enable_dynamic_obstacles = LaunchConfiguration("enable_dynamic_obstacles")
    enable_health_monitor = LaunchConfiguration("enable_health_monitor")
    enable_web_dashboard = LaunchConfiguration("enable_web_dashboard")
    web_dashboard_port = LaunchConfiguration("web_dashboard_port")
    initial_floor = LaunchConfiguration("initial_floor")
    use_rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_delay_s = LaunchConfiguration("rviz_delay_s")

    with open(default_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument("floor_config", default_value=default_floor_config),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("enable_floor_manager", default_value="true"),
        DeclareLaunchArgument("enable_dynamic_obstacles", default_value="true"),
        DeclareLaunchArgument("enable_health_monitor", default_value="true"),
        DeclareLaunchArgument("enable_web_dashboard", default_value="true"),
        DeclareLaunchArgument("web_dashboard_port", default_value="8080"),
        DeclareLaunchArgument("initial_floor", default_value="F20"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("rviz_delay_s", default_value="5.0"),

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
            parameters=[
                params_file,
                {
                    # The recorded factory PCD contains low ground/leg/person
                    # remnants. Filter them more aggressively in sim so the
                    # local costmap does not turn narrow corridors into a blue
                    # inflated sheet.
                    "height_min": 0.05,
                    "height_max": 0.85,
                    "robot_radius": 0.45,
                },
            ],
        ),
        Node(
            package="m20pro_navigation",
            executable="dynamic_obstacle_simulator",
            name="m20pro_dynamic_obstacle_simulator",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(enable_dynamic_obstacles),
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
                }
            ],
            condition=IfCondition(enable_floor_manager),
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
            package="m20pro_navigation",
            executable="sim_health_monitor",
            name="m20pro_sim_health_monitor",
            output="screen",
            parameters=[{"require_dynamic_obstacles": enable_dynamic_obstacles}],
            condition=IfCondition(enable_health_monitor),
        ),
        Node(
            package="m20pro_cloud_bridge",
            executable="web_dashboard",
            name="m20pro_web_dashboard",
            output="screen",
            parameters=[{"port": web_dashboard_port}],
            condition=IfCondition(enable_web_dashboard),
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
