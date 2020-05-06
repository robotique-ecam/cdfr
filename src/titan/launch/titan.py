#!/usr/bin/env python3


"""Titan supervisor launchfile."""


import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from robot import interfaces

launchers = ['core', 'interfaces']


def generate_launch_description():
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('titan'), f'/launch/{launcher}.py']),
        ) for launcher in launchers
    ])
