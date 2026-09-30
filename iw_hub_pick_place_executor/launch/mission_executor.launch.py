import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('iw_hub_pick_place_executor')
    default_goals_file = os.path.join(pkg_dir, 'config', 'pick_place_goals.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('goals_file', default_value=default_goals_file),
        DeclareLaunchArgument('lift_cmd_topic', default_value='/lift_position_cmd'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('lift_joint_name', default_value='lift_joint'),
        DeclareLaunchArgument('lift_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument('lift_tolerance', default_value='0.002'),
        DeclareLaunchArgument('nav_action_name', default_value='navigate_to_pose'),
        DeclareLaunchArgument('nav_timeout_sec', default_value='300.0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/chassis/odom'),
        DeclareLaunchArgument('retreat_distance', default_value='1.5'),
        DeclareLaunchArgument('retreat_speed', default_value='0.8'),
        DeclareLaunchArgument('retreat_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('manual_linear_speed', default_value='0.5'),
        DeclareLaunchArgument('manual_angular_speed', default_value='0.3'),
        DeclareLaunchArgument('manual_max_distance', default_value='10.0'),
        DeclareLaunchArgument('manual_timeout_sec', default_value='60.0'),

        Node(
            package='iw_hub_pick_place_executor',
            executable='mission_executor_node',
            name='mission_executor_node',
            output='screen',
            parameters=[{
                'goals_file': LaunchConfiguration('goals_file'),
                'lift_cmd_topic': LaunchConfiguration('lift_cmd_topic'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'lift_joint_name': LaunchConfiguration('lift_joint_name'),
                'lift_timeout_sec': LaunchConfiguration('lift_timeout_sec'),
                'lift_tolerance': LaunchConfiguration('lift_tolerance'),
                'nav_action_name': LaunchConfiguration('nav_action_name'),
                'nav_timeout_sec': LaunchConfiguration('nav_timeout_sec'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'retreat_distance': LaunchConfiguration('retreat_distance'),
                'retreat_speed': LaunchConfiguration('retreat_speed'),
                'retreat_timeout_sec': LaunchConfiguration('retreat_timeout_sec'),
                'manual_linear_speed': LaunchConfiguration('manual_linear_speed'),
                'manual_angular_speed': LaunchConfiguration('manual_angular_speed'),
                'manual_max_distance': LaunchConfiguration('manual_max_distance'),
                'manual_timeout_sec': LaunchConfiguration('manual_timeout_sec'),
            }],
        ),
    ])