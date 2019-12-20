#!/usr/bin/env python3


"""Asterix launcher."""


import os
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    urdf = os.path.join(get_package_share_directory('asterix'), 'robot', 'asterix.urdf')
    params = os.path.join(get_package_share_directory('asterix'), 'param', 'asterix.yml')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=params,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(package='drive', node_executable='drive', output='screen', parameters=[params]),

        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),

        Node(package='tf2_ros', node_executable='static_transform_publisher', output='screen', arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2_bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params': params}.items(),
        )
    ])
