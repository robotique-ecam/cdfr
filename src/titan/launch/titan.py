#!/usr/bin/env python3


"""Titan supervisor launchfile."""


from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

launchers = ['core', 'interfaces']


def generate_launch_description():
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('titan'), f'/launch/{launcher}.py']),
        ) for launcher in launchers
    ])
