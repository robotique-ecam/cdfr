#!/usr/bin/env python3


"""Titan supervisor launchfile. Navigation stack for all robots"""


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
            PythonLaunchDescriptionSource([get_package_share_directory(robot), '/launch/nav-core.py']),
            launch_arguments={
                'namespace': robot,
                'params_file': os.path.join(get_package_share_directory(robot), 'param', f'{robot}.yml')
            }.items(),
        ) for robot in robots
    ])
