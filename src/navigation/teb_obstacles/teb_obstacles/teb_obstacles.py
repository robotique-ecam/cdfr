#!/usr/bin/env python3


"""Teb_obstacles localisation node."""


import rclpy

from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

class Teb_obstacles(Node):

    def __init__(self):
        super().__init__("teb_dynamic_obstacles_node")
        self.allie = "obelix" if self.get_namespace().strip("/") == "asterix" else "asterix"
        self.allies_subscription_ = self.create_subscription(
            MarkerArray, '/allies_positions_markers', self.allies_subscription_callback, 10)
        self.ennemies_subscription_ = self.create_subscription(
            MarkerArray, '/ennemies_positions_markers', self.ennemies_subscription_callback, 10)
        self.allies_subscription_
        self.ennemies_subscription_
        self.get_logger().info('teb_dynamic_obstacles node is ready')

    def allies_subscription_callback(self, msg):
    def ennemies_subscription_callback(self, msg):

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
