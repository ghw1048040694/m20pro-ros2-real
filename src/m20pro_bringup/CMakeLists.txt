cmake_minimum_required(VERSION 3.5)
project(m20pro_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
