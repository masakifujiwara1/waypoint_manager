cmake_minimum_required(VERSION 3.8)
project(waypoint_visualization VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(waypoint_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)

set(waypoint_rviz_panel_headers_to_moc
  src/rviz/waypoint_operator_panel.h
)

foreach(header "${waypoint_rviz_panel_headers_to_moc}")
  qt5_wrap_cpp(waypoint_rviz_panel_moc_files "${header}")
endforeach()

add_library(waypoint_rviz_panel SHARED
  ${waypoint_rviz_panel_moc_files}
  src/rviz/waypoint_operator_panel.cpp
)

target_include_directories(waypoint_rviz_panel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(waypoint_rviz_panel PRIVATE cxx_std_14)

ament_target_dependencies(waypoint_rviz_panel
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  pluginlib
  geometry_msgs
  std_srvs
  waypoint_manager_msgs
)

target_link_libraries(waypoint_rviz_panel
  Qt5::Core
  Qt5::Widgets
)

add_executable(waypoint_visualization_node
  src/waypoint_visualization_node.cpp
)

target_include_directories(waypoint_visualization_node
  PRIVATE
    include
)

target_compile_features(waypoint_visualization_node PRIVATE cxx_std_17)

ament_target_dependencies(waypoint_visualization_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interactive_markers
  geometry_msgs
  visualization_msgs
  waypoint_manager_msgs
)

install(TARGETS
  waypoint_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  waypoint_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  rviz_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  rviz_common
  pluginlib
  geometry_msgs
  std_srvs
  waypoint_manager_msgs
)

ament_package()
