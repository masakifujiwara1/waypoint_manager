cmake_minimum_required(VERSION 3.5)
project(goal_event_handler VERSION 0.0.0 LANGUAGES CXX)

# C++14を使用する設定（ROS2の標準）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 依存関係の検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(waypoint_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ヘッダファイルのディレクトリを指定
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 実行可能ファイルの設定
add_executable(radius_node
  src/radius_node.cpp
)

# ライブラリのリンク
ament_target_dependencies(radius_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  waypoint_manager_msgs
  Eigen3
)

# 実行ファイルのプロパティ設定
set_target_properties(radius_node
  PROPERTIES
    OUTPUT_NAME radius_node
    VERSION ${PROJECT_VERSION}
)

# インストールターゲットの設定
install(TARGETS radius_node
  DESTINATION lib/${PROJECT_NAME}
)

# パッケージのエクスポート設定
ament_package()
