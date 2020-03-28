#!/usr/bin/env python3


"""Obelix launcher."""


import os
import launch
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    namespace = 'obelix'

    robot_launch_file_dir = os.path.join(get_package_share_directory('robot'), 'launch')

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_launch_file_dir, '/launch.py']),
            launch_arguments={
                'namespace': namespace,
            }.items(),
        )
    ])
