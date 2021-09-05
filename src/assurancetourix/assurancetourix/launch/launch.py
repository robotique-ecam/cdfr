#!/usr/bin/env python3


"""Assurancetourix launcher."""


import os
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from platform import machine


def generate_launch_description():

    params = os.path.join(
        get_package_share_directory("assurancetourix"), "param", "assurancetourix.yml"
    )
    launch_file_dir = os.path.dirname(__file__)
    simulation = True if machine() != "aarch64" else False

    launch_description = [
        DeclareLaunchArgument(
            "params",
            default_value=params,
            description="Full path to param file to load",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        Node(
            package="panoramix",
            executable="panoramix",
            output="screen",
            arguments=[],
        ),
        Node(
            package="strategix",
            executable="strategix",
            output="screen",
            arguments=[],
        ),
        Node(
            package="assurancetourix",
            executable="assurancetourix",
            output="screen",
            parameters=[params],
        ),
    ]

    if not simulation:
        launch_description.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_dir, "/launch_external.py"]),
            ),
        )

    return launch.LaunchDescription(launch_description)
