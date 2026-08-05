"""
Single entry point for the full iw_hub stack - equivalent to running:

  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  ros2 launch iw_hub_navigation isaac_nav2_bringup.launch.py
  ros2 launch iw_hub_lift_control lift_ramp.launch.py
  ros2 launch iw_hub_pick_place_executor mission_executor.launch.py

all in one terminal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_dir = get_package_share_directory('iw_hub_navigation')
    lift_dir = get_package_share_directory('iw_hub_lift_control')
    executor_dir = get_package_share_directory('iw_hub_pick_place_executor')
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    use_sim_time = LaunchConfiguration('use_sim_time')
    goals_file = LaunchConfiguration('goals_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Isaac Sim /clock as ROS time')

    declare_goals_file_cmd = DeclareLaunchArgument(
        'goals_file',
        default_value=os.path.join(executor_dir, 'config', 'pick_place_goals.yaml'),
        description='Full path to pick_place_goals.yaml')

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')))

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'isaac_nav2_bringup.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    lift_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lift_dir, 'launch', 'lift_ramp.launch.py')))

    mission_executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(executor_dir, 'launch', 'mission_executor.launch.py')),
        launch_arguments={'goals_file': goals_file}.items())

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_goals_file_cmd,
        rosbridge_launch,
        nav2_launch,
        lift_launch,
        mission_executor_launch,
    ])