#!/usr/bin/env python3


"""External Assurancetourix modules launcher."""

import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("lh_enemies_tracker"),
        "param",
        "lh_enemies_tracker.yml",
    )
    return launch.LaunchDescription(
        [
            Node(package="pharaon", executable="pharaon", output="screen"),
            Node(
                package="lh_enemies_tracker",
                executable="lh_enemies_tracker",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                output="screen",
                arguments=["udp4", "-p", "8888", "-v6"],
            ),
        ],
    )
