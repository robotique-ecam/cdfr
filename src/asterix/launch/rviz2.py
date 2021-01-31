#!/usr/bin/env python3


"""Launch namespaced Rviz2 for asterix."""


from titan.rviz2 import generate_rviz_launch_description


def generate_launch_description():
    """Launch asterix rviz2."""
    return generate_rviz_launch_description(namespace="asterix")
