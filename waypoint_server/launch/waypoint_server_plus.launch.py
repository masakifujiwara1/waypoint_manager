from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
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
        DeclareLaunchArgument(
            'reconfig_file',
            default_value=[FindPackageShare('waypoint_reconfigure'), '/config/reconfigure_list.yaml']
        ),

        GroupAction([
            PushRosNamespace('waypoint_manager'),

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

            Node(
                package='goal_event_handler',
                executable='radius_node',
                name='radius_node',
                output=LaunchConfiguration('output'),
                parameters=[LaunchConfiguration('config_file')]
            ),

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
            ),

            Node(
                package='waypoint_reconfigure',
                executable='waypoint_reconfigure_node',
                name='waypoint_reconfigure_node',
                output=LaunchConfiguration('output'),
                parameters=[LaunchConfiguration('config_file')],
                additional_parameters={
                    'file_path': LaunchConfiguration('reconfig_file')
                }
            )
        ])
    ])
