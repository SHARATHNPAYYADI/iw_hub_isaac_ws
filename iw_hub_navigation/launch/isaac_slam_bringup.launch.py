"""
Run this first (with teleop or a keyboard driver) to build iw_hub_map.yaml
via SLAM Toolbox, then save it with:
  ros2 run nav2_map_server map_saver_cli -f <pkg>/maps/iw_hub_map --ros-args -p use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    return LaunchDescription([
        declare_use_sim_time_cmd,
        slam_launch,
    ])
