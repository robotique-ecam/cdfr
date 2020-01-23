#!/usr/bin/env python3.5


"""Camera for assurancetourix ROS node."""


import rclpy
from time import sleep
from gpiozero import Button
from rclpy.node import Node
from std_msgs.msg import String
from assurancetourix.components.camera import Camera
from assurancetourix.components.tracker import Tracker
from assurancetourix.components.projector import Projector


class Main(Node):

    """Main assurancetourix class containing camera, object_detection and robots tracking"""

    def __init__(self):
        """Allocate objects inside the vision node."""
        super().__init__('main_node')
        self._status_leds_name = 'status_leds'
        self.button_right = Button(23, bounce_time=0.2)
        self.button_left = Button(24, bounce_time=0.2)
        self.button_right.when_pressed = self._set_side
        self.button_left.when_pressed = self._set_side
        self._set_side()
        self.status_leds = self.create_publisher(String, self._status_leds_name)
        while self.count_subscribers(self._status_leds_name) == 0:
            self.get_logger().debug("Waiting for status_leds to connect")
            sleep(0.1)
        msg = String()
        msg.data = "all-pulse"
        self.status_leds.publish(msg)
        self.camera = Camera()
        self.tracker = Tracker()
        msg.data = "all-off"
        self.status_leds.publish(msg)
        msg.data = "green-blink"
        self.status_leds.publish(msg)
        self.projector = Projector(self.side)

    def _set_side(self):
        """Handle components initialisation after side definition."""
        if self.button_left.is_pressed is True:
            self.side = 'left'
        else:
            self.side = 'right'


def main(args=None):
    rclpy.init(args=args)
    main = Main()
    rclpy.spin(main)
    main.destroy_node()
    rclpy.shutdown()
