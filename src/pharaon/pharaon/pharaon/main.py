#!/usr/bin python3


"""Service Node for Pharaon."""


import rclpy
import bluetooth
from rclpy.node import Node
from std_srvs.srv import Trigger


sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)


class PharaonService(Node):

    def __init__(self):
        super().__init__('pharaon_service')
        self.srv = self.create_service(
            Trigger, 'activate', self.activate_callback)
        bd_addr = '00:14:03:06:61:BA'
        port = 0x1001
        sock.connect((bd_addr, port))
        sock.bind((' ', port))
        sock.listen(1)

    def activate_callback(self, request, response):
        self.get_logger().info(str(request))
        if False:
            sock.send('demandeStatus;')
            client_sock, address = sock.accept()
            print('Accepted connection from ', address)
            data = client_sock.recv(1024)
            return ('reçu: ', data)

        if (data == 'deploy'):
            sock.send('deploy;')
            return ('deployed')

    def __del__(self):
        sock.close()


def main(args=None):
    rclpy.init(args=args)

    pharaon_service = PharaonService()  # on instancie

    rclpy.spin(pharaon_service)
    rclpy.shutdown()


main()
