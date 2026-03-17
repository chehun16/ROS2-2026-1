cmake_minimum_required(VERSION 3.8)
project(semantic)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  semantic/semantic_costmap_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
