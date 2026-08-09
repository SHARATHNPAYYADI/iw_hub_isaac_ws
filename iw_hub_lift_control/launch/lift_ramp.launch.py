from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joint_name', default_value='lift_joint'),
        DeclareLaunchArgument('command_topic', default_value='/joint_command'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('lift_cmd_topic', default_value='/lift_position_cmd'),
        DeclareLaunchArgument('max_velocity', default_value='0.01'),
        DeclareLaunchArgument('max_acceleration', default_value='0.01'),
        DeclareLaunchArgument('publish_rate', default_value='50.0'),
        DeclareLaunchArgument('effort', default_value='10.0'),
        DeclareLaunchArgument('lower_limit', default_value='0.0'),
        DeclareLaunchArgument('upper_limit', default_value='0.1'),

        Node(
            package='iw_hub_lift_control',
            executable='lift_ramp_node',
            name='lift_ramp_node',
            output='screen',
            parameters=[{
                'joint_name': LaunchConfiguration('joint_name'),
                'command_topic': LaunchConfiguration('command_topic'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'lift_cmd_topic': LaunchConfiguration('lift_cmd_topic'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'max_acceleration': LaunchConfiguration('max_acceleration'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'effort': LaunchConfiguration('effort'),
                'lower_limit': LaunchConfiguration('lower_limit'),
                'upper_limit': LaunchConfiguration('upper_limit'),
            }],
        ),
    ])