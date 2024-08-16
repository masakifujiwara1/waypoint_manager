cmake_minimum_required(VERSION 3.5)
project(waypoint_visualization VERSION 0.1.0 LANGUAGES CXX)

# C++11およびC++17を使用する設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ROS2のパッケージを検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(waypoint_manager_msgs REQUIRED)

# Qtのバージョンを確認して設定
if(NOT rviz_QT_VERSION)
    set(rviz_QT_VERSION 5)
endif()

if(rviz_QT_VERSION VERSION_LESS 5)
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED
        COMPONENTS
            QtCore
            QtGui
    )
else()
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
        COMPONENTS
            Core
            Widgets
    )
endif()

# インクルードディレクトリの設定
include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${interactive_markers_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_default_plugins_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${waypoint_manager_msgs_INCLUDE_DIRS}
)

# Qt関連の処理
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt_wrap_cpp(waypoint_rviz_panel_MOC
    src/rviz/waypoint_operator_panel.h
)

# ライブラリの設定
add_library(waypoint_rviz_panel SHARED
    ${waypoint_rviz_panel_MOC}
    src/rviz/waypoint_operator_panel.cpp
)

# 実行ファイルの設定
add_executable(waypoint_visualization_node
    src/waypoint_visualization_node.cpp
)

# ターゲットプロパティの設定
set_target_properties(waypoint_rviz_panel
    PROPERTIES
        OUTPUT_NAME waypoint_rviz_panel
        VERSION ${PROJECT_VERSION}
)

set_target_properties(waypoint_visualization_node
    PROPERTIES
        OUTPUT_NAME waypoint_visualization_node
        VERSION ${PROJECT_VERSION}
)

# ライブラリのリンク設定
if(rviz_QT_VERSION VERSION_LESS 5)
    target_link_libraries(waypoint_rviz_panel
        ${rclcpp_LIBRARIES}
        ${Qt4_LIBRARIES}
        ${rviz_common_LIBRARIES}
    )
else()
    target_link_libraries(waypoint_rviz_panel
        ${rclcpp_LIBRARIES}
        Qt5::Widgets
        ${rviz_common_LIBRARIES}
    )
endif()

target_link_libraries(waypoint_visualization_node
    ${rclcpp_LIBRARIES}
    ${rviz_default_plugins_LIBRARIES}
)

# インストール設定
install(TARGETS waypoint_rviz_panel waypoint_visualization_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    FILES
        rviz_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

# パッケージのエクスポート設定
ament_export_dependencies(rclcpp tf2 tf2_ros interactive_markers pluginlib rviz_common rviz_default_plugins geometry_msgs visualization_msgs waypoint_manager_msgs)
ament_package()
