#!/usr/bin/env python3


"""Obelix launcher."""


import launch
import launch_ros.actions


def generate_launch_description():
    main = launch_ros.actions.Node(package='obelix', node_executable='main', output='screen')
    return launch.LaunchDescription([main])
