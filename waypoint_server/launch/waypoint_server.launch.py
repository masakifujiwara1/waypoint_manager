#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Package directories
    waypoint_server_dir = get_package_share_directory('waypoint_server')
    
    # Launch arguments
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output destination for node logs'
    )
    
    goal_topic_arg = DeclareLaunchArgument(
        'goal_topic',
        default_value='/goal_pose',
        description='Goal topic for navigation'
    )
    
    regist_goal_topic_arg = DeclareLaunchArgument(
        'regist_goal_topic',
        default_value='/clicked_point',
        description='Topic for registering goals via clicked points'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(waypoint_server_dir, 'config', 'waypoint_server.yaml'),
        description='Path to waypoint server configuration file'
    )
    
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(waypoint_server_dir, 'config', 'waypoints', 'test.yaml'),
        description='Path to waypoints file'
    )
    
    route_file_arg = DeclareLaunchArgument(
        'route_file',
        default_value=os.path.join(waypoint_server_dir, 'config', 'routes', 'test.yaml'),
        description='Path to route file'
    )
    
    # Launch configurations
    output = LaunchConfiguration('output')
    goal_topic = LaunchConfiguration('goal_topic')
    regist_goal_topic = LaunchConfiguration('regist_goal_topic')
    config_file = LaunchConfiguration('config_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    route_file = LaunchConfiguration('route_file')
    
    # Nodes within waypoint_manager namespace
    waypoint_manager_group = GroupAction([
        PushRosNamespace('waypoint_manager'),
        
        Node(
            package='waypoint_server',
            executable='waypoint_server_node',
            name='waypoint_server',
            output=output,
            parameters=[
                config_file,
                {
                    'clear_costmap_srv': '/clear_costmap',
                    'waypoints_file': waypoints_file,
                    'route_file': route_file,
                }
            ],
            remappings=[
                ('waypoint/regist_point', regist_goal_topic),
            ],
        ),
        
        Node(
            package='waypoint_server',
            executable='waypoint_to_posestamped_node',
            name='waypoint_to_posestamped',
            output=output,
            parameters=[config_file],
            remappings=[
                ('move_base_simple/goal', goal_topic),
            ],
        ),
        
        Node(
            package='waypoint_visualization',
            executable='waypoint_visualization_node',
            name='waypoint_visualization',
            output=output,
            parameters=[config_file],
            remappings=[
                ('waypoint/regist_point', regist_goal_topic),
            ],
        ),
        
        Node(
            package='goal_event_handler',
            executable='radius_node',
            name='radius_node',
            output=output,
            parameters=[config_file],
        ),
    ])
    
    return LaunchDescription([
        output_arg,
        goal_topic_arg,
        regist_goal_topic_arg,
        config_file_arg,
        waypoints_file_arg,
        route_file_arg,
        waypoint_manager_group,
    ])
