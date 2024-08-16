cmake_minimum_required(VERSION 3.5)
project(waypoint_manager_msgs)

# メッセージの生成に必要な依存関係を指定
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## メッセージの宣言とビルド                   ##
################################################

# 'msg'フォルダ内のメッセージファイルを指定
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Property.msg"
  "msg/Waypoint.msg"
  "msg/WaypointStamped.msg"
  "msg/WaypointsInfo.msg"
  "msg/Waypoints.msg"
  "msg/Route.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

###################################
## パッケージのエクスポート設定 ##
###################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()
