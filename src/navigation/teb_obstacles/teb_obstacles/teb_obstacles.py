#!/usr/bin/env python3


"""Teb_obstacles localisation node."""


import rclpy

from rclpy.node import Node

class Teb_obstacles(Node):

    def __init__(self):
        super().__init__("teb_dynamic_obstacles_node")
        self.allie = "obelix" if self.get_namespace().strip("/") == "asterix" else "asterix"
        self.get_logger().info('teb_dynamic_obstacles node is ready')


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    teb_obstacles = Teb_obstacles()
    try:
        rclpy.spin(teb_obstacles)
    except KeyboardInterrupt:
        pass
    teb_obstacles.destroy_node()
    rclpy.shutdown()
