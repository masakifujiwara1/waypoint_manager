cmake_minimum_required(VERSION 3.8)
project(check_robot_moving VERSION 0.0.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(waypoint_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(check_robot_moving_node
  src/check_robot_moving_node.cpp
)

target_include_directories(check_robot_moving_node
  PRIVATE
    include
)

target_compile_features(check_robot_moving_node PRIVATE cxx_std_14)

ament_target_dependencies(check_robot_moving_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  waypoint_manager_msgs
  std_msgs
  std_srvs
)

target_link_libraries(check_robot_moving_node
  Eigen3::Eigen
)

# Install
install(TARGETS
  check_robot_moving_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
