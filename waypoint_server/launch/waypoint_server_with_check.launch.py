from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'output',
            default_value='screen'
        ),
        DeclareLaunchArgument(
            'goal_topic',
            default_value='/move_base_simple/goal'
        ),
        DeclareLaunchArgument(
            'regist_goal_topic',
            default_value='/clicked_point'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=[FindPackageShare('waypoint_server'), '/config/waypoint_server.yaml']
        ),

        # Group for namespace 'waypoint_manager'
        GroupAction([
            PushRosNamespace('waypoint_manager'),

            # waypoint_server_node
            Node(
                package='waypoint_server',
                executable='waypoint_server_node',
                name='waypoint_server',
                output=LaunchConfiguration('output'),
                parameters=[LaunchConfiguration('config_file')],
                remappings=[
                    ('waypoint/regist_point', LaunchConfiguration('regist_goal_topic'))
                ],
                additional_parameters={'clear_costmap_srv': '/move_base/clear_costmaps'}
            ),

            # waypoint_to_posestamped_node
            Node(
                package='waypoint_server',
                executable='waypoint_to_posestamped_node',
                name='waypoint_to_posestamped',
                output=LaunchConfiguration('output'),
                parameters=[LaunchConfiguration('config_file')],
                remappings=[
                    ('move_base_simple/goal', LaunchConfiguration('goal_topic'))
                ]
            ),

            # waypoint_visualization_node
            Node(
                package='waypoint_visualization',
                executable='waypoint_visualization_node',
                name='waypoint_visualization',
                output=LaunchConfiguration('output'),
                parameters=[LaunchConfiguration('config_file')],
                remappings=[
                    ('waypoint/regist_point', LaunchConfiguration('regist_goal_topic'))
                ]
            ),

            # radius_node (goal_event_handler)
            Node(
                package='goal_event_handler',
                executable='radius_node',
                name='radius_node',
                output=LaunchConfiguration('output'),
                parameters=[LaunchConfiguration('config_file')]
            ),

            # check_robot_moving_node
            Node(
                package='check_robot_moving',
                executable='check_robot_moving_node',
                name='check_robot_moving_node',
                output=LaunchConfiguration('output'),
                parameters=[LaunchConfiguration('config_file')],
                additional_parameters={
                    'mcl_pose_topic': '/mcl_pose',
                    'cmd_vel_topic': '/icart_mini/cmd_vel',
                    'clear_costmap_srv': '/move_base/clear_costmaps'
                }
            )
        ])
    ])
