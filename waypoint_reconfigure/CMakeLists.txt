cmake_minimum_required(VERSION 3.8)
project(waypoint_reconfigure VERSION 0.0.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(waypoint_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Add executable
add_executable(waypoint_reconfigure_node
  src/waypoint_reconfigure_node.cpp
)

target_include_directories(waypoint_reconfigure_node
  PRIVATE
    include
)

target_compile_features(waypoint_reconfigure_node PRIVATE cxx_std_14)

ament_target_dependencies(waypoint_reconfigure_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  std_msgs
  std_srvs
  waypoint_manager_msgs
  ament_index_cpp
)

target_link_libraries(waypoint_reconfigure_node
  Eigen3::Eigen
  yaml-cpp
)

# Install
install(TARGETS
  waypoint_reconfigure_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
