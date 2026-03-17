cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  bringup/demo_scenario_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
