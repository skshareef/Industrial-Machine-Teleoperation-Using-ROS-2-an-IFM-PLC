cmake_minimum_required(VERSION 3.8)
project(trench_roller_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  PROGRAMS
    src/joy_teleop_node.py
    src/tcp_handler_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
