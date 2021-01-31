#!/usr/bin/env python3


"""Titan supervisor launchfile. Navigation stack for all robots"""


import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

robots = ["asterix"]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_package_share_directory(robot), "/launch/nav-core.py"]
                ),
                launch_arguments={
                    "namespace": robot,
                    "params_file": os.path.join(
                        get_package_share_directory(robot), "param", f"{robot}.yml"
                    ),
                }.items(),
            )
            for robot in robots
        ]
    )
