from setuptools import setup

package_name = "m20pro_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="M20Pro Developer",
    maintainer_email="user@example.com",
    description="ROS 2 navigation bridge and lightweight planner for DEEP Robotics M20 Pro.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tcp_bridge = m20pro_navigation.tcp_bridge_node:main",
            "sim_bridge = m20pro_navigation.sim_bridge_node:main",
            "grid_planner = m20pro_navigation.grid_planner_node:main",
            "path_follower = m20pro_navigation.path_follower_node:main",
            "control_gui = m20pro_navigation.control_gui:main",
            "zero_joint_state_publisher = m20pro_navigation.zero_joint_state_publisher:main",
            "map_editor = m20pro_navigation.map_editor:main",
            "floor_manager = m20pro_navigation.floor_manager:main",
            "floor_goal_bridge = m20pro_navigation.floor_goal_bridge:main",
            "initialpose_3d_adapter = m20pro_navigation.initialpose_3d_adapter:main",
            "dynamic_obstacle_simulator = m20pro_navigation.dynamic_obstacle_simulator:main",
            "dual_lidar_simulator = m20pro_navigation.dual_lidar_simulator:main",
            "pointcloud_fusion = m20pro_navigation.pointcloud_fusion:main",
            "sim_health_monitor = m20pro_navigation.sim_health_monitor:main",
            "system_check = m20pro_navigation.system_check_node:main",
            "config_audit = m20pro_navigation.config_audit_node:main",
        ],
    },
)
