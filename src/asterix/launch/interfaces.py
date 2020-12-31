#!/usr/bin/env python3


"""Asterix interfaces launcher for simulation."""


import os

import launch
from robot import interfaces
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    namespace = "asterix"
    params = os.path.join(
        get_package_share_directory(namespace), "param", f"{namespace}.yml"
    )
    robot_launch_file_dir = os.path.dirname(interfaces.__file__)

    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [robot_launch_file_dir, "/interfaces.py"]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "params_file": params,
                }.items(),
            )
        ]
    )
