cmake_minimum_required(VERSION 3.5)
project(m20pro_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/m20pro_real_nav.sh
  scripts/m20pro_real_full.sh
  scripts/m20pro_real_web.sh
  scripts/m20pro_record_real.sh
  scripts/m20pro_lidar_guard.sh
  scripts/m20pro_lidar_relay_guard.sh
  scripts/m20pro_runtime_snapshot.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
