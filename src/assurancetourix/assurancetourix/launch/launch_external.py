#!/usr/bin/env python3


"""External Assurancetourix modules launcher."""

import os
import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription(
        [
            Node(package="pharaon", executable="pharaon", output="screen"),
        ]
    )
