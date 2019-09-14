#!/usr/bin/env python3


import rclpy
import socket
from sensors import VL53L1X
from struct import unpack


localIP = "192.168.2.1"
localPort = 4096
bufferSize = 1024


# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))


def main(args=None):
    rclpy.init(args=args)

    sensor = VL53L1X()

    # Listen for incoming datagrams
    while(True):
        range, = unpack('<L', UDPServerSocket.recvfrom(bufferSize)[0])
        range /= 1000
        sensor.pushlish_range(range)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
