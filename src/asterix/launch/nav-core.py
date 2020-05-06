#!/usr/bin/env python3


"""Asterix core launcher for simulation."""


from robot.launcher import generate_robot_launch_description


def generate_launch_description():
    """Launch the robot stack."""
    return generate_robot_launch_description('asterix', simulation=True)
