#!/usr/bin/env python3


"""Simulation game manager <extern> controller."""

from os import environ

import rclpy
from std_srvs.srv import Trigger
from webots_ros2_core.webots_node import WebotsNode

# TODO : in launchfile
environ["WEBOTS_ROBOT_NAME"] = "pharaon"


class Pharaon(WebotsNode):
    """Game manager node."""

    def __init__(self, args=None):
        """Init pharaon sim node."""
        super().__init__("pharaon", args)
        self.create_service(Trigger, "/pharaon/deploy", self.deploy_pharaon)
        self.get_logger().info("Pharaon sim node ready")

    def deploy_pharaon(self, req, resp):
        """Deploy pharaon on service call."""
        self.get_logger().info("Started deployment")
        self.robot.getMotor("primary_joint").setPosition(0)
        self.robot.getMotor("secondary_joint").setPosition(0)
        self.get_logger().info("Finished deployment")
        resp.success = True
        return resp


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


if __name__ == "__main__":
    main()
