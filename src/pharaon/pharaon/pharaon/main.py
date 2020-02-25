#!/usr/bin python3


"""Service Node for Pharaon."""


import rclpy
import bluetooth
from rclpy.node import Node
from rclpy.action import ActionServer
from pharaon_msgs.action import Pharaon


bd_addr = "00:14:03:06:61:BA"


class PharaonActionServer(Node):

    def __init__(self):
        super().__init__('pharaon_action_server')
        self.bd_addr = bd_addr
        self.port = 1
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.sock.connect(self.bd_addr, self.port)
        self.sock.bind(("", self.port))
        self.sock.listen(1)
        self._action_server = ActionServer(self, Pharaon, 'pharaon', self.activate_callback)
        self.get_logger().info('Pharaon Action Server Initialized')

    def activate_callback(self, goal_handle):
        self.get_logger().info('Commanding Pharaon to deploy...')
        # Sending deploy over BT socket
        self.sock.send("deploy;")

        feedback_msg = Pharaon.Feedback()
        feedback_msg.deploying = True
        goal_handle.publish_feedback(feedback_msg)

        data = ""
        while 'deployed' not in data:
            self.sock.send("demandeStatus;")
            data = self.sock.recv(1024)

        goal_handle.succeed()

        result = Pharaon.Result()
        result.deployed = True
        return result

    def __del__(self):
        self.sock.close()


def main(args=None):
    rclpy.init(args=args)

    pharaon_service = PharaonActionServer()  # on instancie

    rclpy.spin(pharaon_service)
    rclpy.shutdown()
