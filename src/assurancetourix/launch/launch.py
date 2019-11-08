#!/usr/bin/env python3


"""Assurancetourix launcher."""


import launch
import launch_ros.actions


def generate_launch_description():
    leds = launch_ros.actions.Node(package='assurancetourix', node_executable='leds', output='screen')
    main = launch_ros.actions.Node(package='assurancetourix', node_executable='main', output='screen')
    return launch.LaunchDescription([main, leds])
