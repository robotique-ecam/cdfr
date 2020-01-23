#!/usr/bin/env python3


"""LED Indicators listener for assurancetourix ROS node."""


import rclpy
from rclpy.node import Node
from gpiozero import PWMLED
from std_msgs.msg import String


LEDS = ["green", "orange", "red", "all"]
ACTIONS = ["blink", "pulse", "on", "off"]


class Indicators(Node):

    """ROS 2 Assurancetourix led indicators."""

    def __init__(self):
        """Init from superclass."""
        super().__init__('led_indicators_node')
        self.create_subscription(String, 'status_leds', self._receive_callback)
        self.leds = [PWMLED(14), PWMLED(15), PWMLED(18)]
        self.execute(["orange", "pulse"])

    def execute(self, commands):
        """Executes sanitized commands."""
        if commands[0] == "all":
            if commands[1] == "pulse":
                [led.pulse(background=True) for led in self.leds]
            elif commands[1] == "blink":
                [led.blink(background=True) for led in self.leds]
            elif commands[1] == "on":
                [led.on() for led in self.leds]
            elif commands[1] == "off":
                [led.off() for led in self.leds]
        else:
            led_index = LEDS.index(commands[0])
            if commands[1] == "pulse":
                self.leds[led_index].pulse(background=True)
            elif commands[1] == "blink":
                self.leds[led_index].blink(background=True)
            elif commands[1] == "on":
                self.leds[led_index].on()
            elif commands[1] == "off":
                self.leds[led_index].off()

    def _receive_callback(self, command):
        """Callback receiving commands from ROS listener and checking them."""
        commands = command.data.split("-")
        if len(commands) == 2:
            if commands[0] in LEDS:
                if commands[1] in ACTIONS:
                    self.get_logger().debug("status_leds received: " + command.data)
                    self.execute(commands)


def main(args=None):
    rclpy.init(args=args)
    leds = Indicators()
    rclpy.spin(leds)
    leds.destroy_node()
    rclpy.shutdown()
