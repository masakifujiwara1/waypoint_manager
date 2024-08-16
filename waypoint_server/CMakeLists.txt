cmake_minimum_required(VERSION 3.5)
project(waypoint_server VERSION 0.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(waypoint_manager_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/map.cpp
  src/${PROJECT_NAME}/route.cpp
)

# Declare C++ executables
add_executable(waypoint_server_node
  src/waypoint_server_node.cpp
)

add_executable(waypoint_to_posestamped_node
  src/waypoint_to_posestamped_node.cpp
)

# Specify libraries to link against
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

target_link_libraries(waypoint_server_node
  ${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

target_link_libraries(waypoint_to_posestamped_node
  ${rclcpp_LIBRARIES}
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  waypoint_server_node
  waypoint_to_posestamped_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  launch/
  rviz/
  DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
