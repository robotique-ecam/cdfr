#!/usr/bin/env python3


"""Assurancetourix launcher."""


import os
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    params = os.path.join(get_package_share_directory('assurancetourix'), 'param', 'assurancetourix.yml')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=params,
            description='Full path to param file to load'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='transformix',
            executable='transformix',
            output='screen',
            arguments=[]
        ),

        Node(
            package='panoramix',
            executable='panoramix',
            output='screen',
            arguments=[]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['1.4', '2', '1', '-2.35619449', '0', '3.141592654', 'map', 'assurancetourix']
        ),

        Node(
            package='assurancetourix',
            executable='assurancetourix',
            output='screen',
            parameters=[params]
        ),

    ])
