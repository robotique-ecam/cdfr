#!/usr/bin/env python3


"""Asterix launcher."""


import launch
import launch_ros.actions


def generate_launch_description():
    main = launch_ros.actions.Node(package='asterix', node_executable='main', output='screen')
    return launch.LaunchDescription([main])
