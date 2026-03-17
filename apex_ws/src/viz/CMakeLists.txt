cmake_minimum_required(VERSION 3.8)
project(viz)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  viz/viz_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
