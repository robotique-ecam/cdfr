#!/usr/bin/env python3


"""Asterix launcher."""


from robot.launcher import generate_robot_launch_description


def generate_launch_description():
    """Launch the robot stack."""
    return generate_robot_launch_description('asterix')
