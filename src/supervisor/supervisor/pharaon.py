#!/usr/bin/env python3


"""Simulation game manager <extern> controller."""

from os import environ

import rclpy
from webots_ros2_core.webots_node import WebotsNode

environ['WEBOTS_ROBOT_NAME'] = 'pharaon'


class Pharaon(WebotsNode):
    """Game manager node."""

    def __init__(self, args=None):
        super().__init__('pharaon', args)
        self.deploy_pharaon()

    def deploy_pharaon(self):
        self.robot.getMotor('primary_joint').setPosition(0)
        self.robot.getMotor('secondary_joint').setPosition(0)


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    pharaon = Pharaon(args=args)
    try:
        rclpy.spin(pharaon)
    except KeyboardInterrupt:
        pass
    pharaon.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
