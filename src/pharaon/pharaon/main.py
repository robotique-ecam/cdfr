#!/usr/bin python3


"""Service Node for Pharaon."""


import rclpy
from bluedot.btcomm import BluetoothClient
from rclpy.node import Node
from std_srvs.srv import Trigger


class PharaonService(Node):

    def __init__(self):
        super().__init__('pharaon_service')
        self.srv = self.create_service(Trigger, 'activate', self.activate_callback)
        self.bt = BluetoothClient('00:14:03:06:61:BA', self.data_received)
        self.get_logger().info('Pharaon has been started')

    def data_received(self, data):
        """Callback upon data received."""
        self.get_logger().info(data)

    def activate_callback(self, request, response):
        """Callback called upon trigger."""
        try:
            self.bt.send('deploy;')
            response.success = True
            response.message = 'Pharaon requested to deploy'
            self.get_logger().info(response.message)
        except BaseException as e:
            response.success = False
            response.message = f'Pharaon failed to deploy with {e}'
            self.get_logger().warn(response.message)


def main(args=None):
    rclpy.init(args=args)
    pharaon_service = PharaonService()
    try:
        rclpy.spin(pharaon_service)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
