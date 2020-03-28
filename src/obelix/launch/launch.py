#!/usr/bin/env python3


"""Obelix launcher."""


from robot.launcher import generate_robot_launch_description


def generate_launch_description():
    """Launch the robot stack."""
    generate_robot_launch_description('obelix')
