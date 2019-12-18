#!/usr/bin/env python3


"""Asterix launcher."""


import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('asterix'), 'asterix.urdf')
    asterix_params = os.path.join(get_package_share_directory('asterix'), 'param', 'asterix.yml')

    return launch.LaunchDescription([
        Node(package='drive', node_executable='drive', output='screen', parameters=[asterix_params]),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
    ])
