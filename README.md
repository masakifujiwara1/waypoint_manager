# waypoint_manager (ROS 2 Humble)

<img width="736" alt="Screen Shot 2022-01-19 at 18 19 39" src="https://user-images.githubusercontent.com/18626482/150101087-60d64f9e-ca0b-46d5-82cf-49a279f38a61.png">

## Overview

This is a ROS 2 Humble port of the waypoint_manager package. This package provides waypoint management functionality for autonomous navigation.

## Releases
https://github.com/masakifujiwara1/waypoint_manager/releases

## Usage

```shell
ros2 launch waypoint_server waypoint_server.launch.py
```

### With custom configuration

```shell
ros2 launch waypoint_server waypoint_server.launch.py config_file:=/path/to/your/config.yaml
```

### Launch Arguments

- `output`: Output destination for node logs (default: `screen`)
- `goal_topic`: Goal topic for navigation (default: `/goal_pose`)
- `regist_goal_topic`: Topic for registering goals (default: `/clicked_point`)
- `config_file`: Path to configuration file (default: package's `config/waypoint_server.yaml`)

## Migration Status

This package has been migrated from ROS 1 to ROS 2 Humble. The following changes have been made:

### Completed

- ✅ **waypoint_manager_msgs**: Converted to ROS 2 message package using `rosidl_default_generators`
- ✅ **waypoint_server**: Build files (package.xml, CMakeLists.txt) converted to ROS 2 format
- ✅ **check_robot_moving**: Build files converted to ROS 2 format
- ✅ **goal_event_handler**: Build files converted to ROS 2 format
- ✅ **waypoint_reconfigure**: Build files converted to ROS 2 format (dynamic_reconfigure → parameters)
- ✅ **waypoint_visualization**: Build files converted to ROS 2 format (RViz plugin)
- ✅ **Launch files**: Converted to Python format

### Requires Manual Migration

⚠️ **Important**: The C++ source code in the following packages requires manual migration to ROS 2 APIs:

- **waypoint_server** (`src/waypoint_server_node.cpp`, `src/waypoint_to_posestamped_node.cpp`, library files)
- **check_robot_moving** (`src/check_robot_moving_node.cpp`)
- **goal_event_handler** (`src/radius_node.cpp`)
- **waypoint_reconfigure** (`src/waypoint_reconfigure_node.cpp`)
- **waypoint_visualization** (`src/waypoint_visualization_node.cpp`, RViz plugin)

### Key API Changes Required

The following ROS 1 → ROS 2 API changes need to be applied to the source code:

1. **Node initialization**: `ros::NodeHandle` → `rclcpp::Node`
2. **Publishers**: `ros::Publisher` → `rclcpp::Publisher<T>::SharedPtr`
3. **Subscribers**: `ros::Subscriber` → `rclcpp::Subscription<T>::SharedPtr`
4. **Services**: `ros::ServiceServer` → `rclcpp::Service<T>::SharedPtr`
5. **Service Clients**: `ros::ServiceClient` → `rclcpp::Client<T>::SharedPtr`
6. **Parameters**: `nh.param()` → `node->declare_parameter()` / `node->get_parameter()`
7. **Timers**: `ros::Timer` → `rclcpp::TimerBase::SharedPtr`
8. **TF**: `tf::TransformListener` → `tf2_ros::Buffer` and `tf2_ros::TransformListener`
9. **Spin**: `ros::spin()` → `rclcpp::spin(node)`
10. **Logging**: `ROS_INFO()` → `RCLCPP_INFO()`

### Example Migration Pattern

```cpp
// ROS 1
ros::NodeHandle nh;
ros::Publisher pub = nh.advertise<std_msgs::String>("topic", 10);
ros::spin();

// ROS 2
auto node = std::make_shared<rclcpp::Node>("node_name");
auto pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
rclcpp::spin(node);
```

## Build Instructions

```shell
cd ~/ros2_ws
colcon build --packages-select waypoint_manager_msgs waypoint_server check_robot_moving goal_event_handler waypoint_reconfigure waypoint_visualization
source install/setup.bash
```

## TODO

+ Complete C++ source code migration to ROS 2 APIs
+ Add an edit panel for waypoint properties
+ Support action servers (replaces actionlib)
+ Refactor code
+ Test with actual navigation stack

## Notes

- This migration targets ROS 2 Humble
- The build files have been updated to use `ament_cmake`
- Dynamic reconfigure has been replaced with ROS 2 parameters
- RViz plugins require RViz 2 APIs
- Launch files now use Python format
