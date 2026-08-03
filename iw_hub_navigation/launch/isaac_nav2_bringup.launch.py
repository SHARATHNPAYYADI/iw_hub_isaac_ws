"""
Nav2 bringup for the idealworks iw_hub in Isaac Sim.

Assumes Isaac Sim is already running with:
  - the iw_hub_ROS payload loaded (differential_drive OGN graph -> /cmd_vel in, /odom + TF out)
  - a ROS2 Clock publisher action graph node bridging sim time to /clock
  - the two RTX Lidar (RPLidar_S2E front/rear) publishers active

Check your actual topic names in the Isaac action graph (Window > Graph Editor)
before running this - the defaults below assume /cmd_vel, /odom, /scan_front, /scan_rear.
If your graph publishes a single combined /scan instead, just drop the "_rear"
observation source in nav2_params.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('iw_hub_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'iw_hub_map.yaml'),
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 params file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Isaac Sim /clock as ROS time')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start the nav2 stack lifecycle')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz2')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items())

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz))

    return LaunchDescription([
        declare_map_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_use_rviz_cmd,
        nav2_bringup_launch,
        rviz_launch,
    ])
