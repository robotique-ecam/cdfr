#!/usr/bin/env python3


"""Titan supervisor launchfile. Webots interfaces for all robots."""


import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from robot import interfaces

robots = ['asterix']


def generate_launch_description():
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory(robot), '/launch/interfaces.py']),
            launch_arguments={
                'namespace': robot,
                'params_fileba': os.path.join(get_package_share_directory(robot), 'param', f'{robot}.yml')
            }.items(),
        ) for robot in robots
    ] + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('assurancetourix'), '/launch/launch.py']),
            launch_arguments={
                'namespace': 'assurancetourix',
                'params_file': os.path.join(get_package_share_directory('assurancetourix'), 'param', 'assurancetourix.yml')
            }.items(),
        )
    ] + [
        GroupAction([
            Node(
                package='titan',
                node_executable='pharaon',
                output='screen',
            ),

            Node(
                package='strategix',
                node_executable='strategix',
                output='screen',
            ),

            Node(
                package='webots_ros2_core',
                node_executable='webots_launcher',
                arguments=['--mode=realtime', '--stdout', '--stderr', '--batch', '--no-sandbox', '--world=tools/simulation/worlds/cdr2020.wbt'],
                output='screen'
            ),
        ])
    ])
