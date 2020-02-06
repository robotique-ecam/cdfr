#!/usr/bin python3


"""Service Node for Pharaon."""


import rclpy
import serial
from rclpy.node import Node
import bluetooth


bd_addr = "00:14:03:06:61:BA"
port = 1

sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))


class PharaonService(Node):

    def __init__(self):
        super().__init__('pharaon_service')
        bd_addr = "00:14:03:06:61:BA"
        port = 1
        sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
        sock.connect(bd_addr, port)
        self.srv = self.create_service(activate, 'activate', self.activate_callback)

    def activate_callback(self, request, response):
        if (data == "demandeStatus"):

            sock.send("demandeStatus;")
            server_sock.bind(("",port));server_sock.listen(1)
            client_sock,address = server_sock.accept()
            print "Accepted connection from ",address
            data = client_sock.recv(1024)
            return ("reçu: ", data)

        if (data == "deploy"):
            sock.send("deploy;")
            return ("deployed")

def main(args=None):
    rclpy.init(args=args)

    pharaon_service = PharaonService() #on instancie

    rclpy.spin(pharaon_service)
    client_sock.close()
    server_sock.close()
    rclpy.shutdown()
