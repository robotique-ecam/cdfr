#!/usr/bin/env python3


"""Sensors publishers."""


import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class VL53L1X(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.clock = rclpy.time.Time()
        self.publisher_ = self.create_publisher(Range, 'range_1', 10)

    def pushlish_range(self, range):
        """Publish a range message for the VL53L1X."""
        range_msg = Range()
        range_msg.header.stamp = self.clock.to_msg()
        range_msg.header.frame_id = "world"
        range_msg.radiation_type = 1
        range_msg.field_of_view = 0.47
        range_msg.min_range = 0.05
        range_msg.max_range = 4.0
        range_msg.range = float(range)
        self.publisher_.publish(range_msg)
        self.get_logger().info('Publishing: "%s"' % range)


class TinyHCSR04(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.clock = rclpy.time.Time()
        self.publisher_ = self.create_publisher(Range, 'range_2', 10)

    def pushlish_range(self, range):
        """Publish a range message for the VL53L1X."""
        range_msg = Range()
        range_msg.header.stamp = self.clock.to_msg()
        range_msg.header.frame_id = "world"
        range_msg.radiation_type = 0
        range_msg.field_of_view = 0.7
        range_msg.min_range = 0.01
        range_msg.max_range = 2.55
        range_msg.range = float(range)
        self.publisher_.publish(range_msg)
        self.get_logger().info('Publishing: "%s"' % range)
