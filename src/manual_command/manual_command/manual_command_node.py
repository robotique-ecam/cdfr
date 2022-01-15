#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from serial import Serial
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

"""
serial values between 1 and 254

forward velocity --> 1
backward velocity --> 254

turn right direction --> 254
turn left direction --> 1
"""


class ManualCommand(Node):
    def __init__(self):
        """Init manual_command node"""
        super().__init__("manual_command")

        self.max_vel_speed = 0.3
        self.max_dir_speed = 1.0
        serial_port = "/dev/ttyUSB0"

        self.deadzone = 15
        self.middle_value = 126.5
        self.msg = Twist()

        self.cmd_pb = self.create_publisher(Twist, "/asterix/cmd_vel", 10)

        self.get_enable_drivers_request = SetBool.Request()
        self.get_enable_drivers_request.data = True
        self.get_enable_drivers_client = self.create_client(
            SetBool, "/asterix/enable_drivers"
        )
        self.get_logger().info(
            "manual_command node is calling drive for motor arming..."
        )
        while not self.get_enable_drivers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.get_enable_drivers_client.call_async(self.get_enable_drivers_request)

        self.ser = Serial(port=serial_port, baudrate=115200)
        self.get_logger().info("manual_command node is ready")
        self.start_loop()

    def start_loop(self):
        self.synchronized = False
        self.sync()
        self.waiting_for_data = False
        self.create_timer(0.02, self.loop)

    def loop(self):
        if self.waiting_for_data:
            return
        else:
            self.waiting_for_data = True
            try:
                data = self.ser.read(size=4).hex()
                ts = self.get_clock().now().to_msg()
                if int(data[:2], 16) != 0:
                    self.synchronized = False
                    self.sync()
                else:
                    dir_coeff = self.extract_value(int(data[2:4], 16))
                    vel_coeff = self.extract_value(int(data[6:8], 16))
                    self.set_data(dir_coeff, vel_coeff)
                    self.publish_data()
                self.waiting_for_data = False
            except BaseException:
                self.set_data(0, 0)
                self.publish_data()
                self.get_enable_drivers_request.data = False
                self.get_logger().info(
                    "manual_command node is calling drive for motor disarming..."
                )
                self.get_enable_drivers_client.call_async(
                    self.get_enable_drivers_request
                )
                self.get_logger().warn("Exception during serial transmission !!!")

    def deadzone_check(self, value):
        if abs(value - self.middle_value) < self.deadzone:
            return True
        else:
            return False

    def extract_value(self, value):
        if self.deadzone_check(value):
            return 0
        else:
            return (self.middle_value - value) / (
                self.middle_value
            )  # velocity > 0 --> forward, direction > 0 --> left

    def set_data(self, dir_coeff, vel_coeff):
        self.msg.linear.x = -self.max_vel_speed * vel_coeff
        self.msg.angular.z = self.max_dir_speed * dir_coeff

    def sync(self):
        while not self.synchronized:
            read = self.ser.read().hex()
            if read == "00":
                self.ser.read(size=3)
                self.synchronized = True

    def publish_data(self):
        self.cmd_pb.publish(self.msg)


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    manual_command = ManualCommand()
    try:
        rclpy.spin(manual_command)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
