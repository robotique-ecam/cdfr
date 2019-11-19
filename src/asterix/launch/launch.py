#!/usr/bin/env python3


"""Asterix launcher."""


import launch
import launch_ros.actions


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('asterix','robot', 'asterix.urdf')

    return launch.LaunchDescription([
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',output='screen', arguments=[urdf]),
    ])
