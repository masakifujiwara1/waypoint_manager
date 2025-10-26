cmake_minimum_required(VERSION 3.8)
project(waypoint_server VERSION 0.0.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(waypoint_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/map.cpp
  src/${PROJECT_NAME}/route.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  waypoint_manager_msgs
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
)

# Declare C++ executables
add_executable(waypoint_server_node
  src/waypoint_server_node.cpp
)

target_include_directories(waypoint_server_node
  PRIVATE
    include
)

target_compile_features(waypoint_server_node PRIVATE cxx_std_17)

ament_target_dependencies(waypoint_server_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  waypoint_manager_msgs
)

target_link_libraries(waypoint_server_node
  ${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
)

add_executable(waypoint_to_posestamped_node
  src/waypoint_to_posestamped_node.cpp
)

target_compile_features(waypoint_to_posestamped_node PRIVATE cxx_std_17)

ament_target_dependencies(waypoint_to_posestamped_node
  rclcpp
  std_msgs
  geometry_msgs
  waypoint_manager_msgs
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  waypoint_server_node
  waypoint_to_posestamped_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  waypoint_manager_msgs
)

ament_package()

