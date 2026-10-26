cmake_minimum_required(VERSION 3.5)
project(qbo_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY qbo_navigation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()