cmake_minimum_required(VERSION 3.5)
project(waypoint_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(goal_event_handler REQUIRED)
find_package(waypoint_manager_msgs REQUIRED)
find_package(waypoint_server REQUIRED)
find_package(check_robot_moving REQUIRED)
find_package(waypoint_reconfigure REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${goal_event_handler_INCLUDE_DIRS}
  ${waypoint_manager_msgs_INCLUDE_DIRS}
  ${waypoint_server_INCLUDE_DIRS}
  ${check_robot_moving_INCLUDE_DIRS}
  ${waypoint_reconfigure_INCLUDE_DIRS}
)

# Declare a C++ library
# add_library(${PROJECT_NAME} src/waypoint_manager.cpp)

# Add executable
add_executable(${PROJECT_NAME}_node src/waypoint_manager_node.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  goal_event_handler
  waypoint_manager_msgs
  waypoint_server
  check_robot_moving
  waypoint_reconfigure
)

# Install executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install include headers (if any)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
