cmake_minimum_required(VERSION 3.5)
project(check_robot_moving VERSION 0.0.0 LANGUAGES CXX)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(waypoint_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add executable
add_executable(check_robot_moving_node
  src/check_robot_moving_node.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(check_robot_moving_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  waypoint_manager_msgs
  Eigen3
)

# Install the executable
install(TARGETS check_robot_moving_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
