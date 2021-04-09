#!/usr/bin/env python3

""" Launch map sequence """

import rclpy

from lifecycle_msgs.srv import ChangeState


def launch_map(parent_node):
    client = parent_node.create_client(ChangeState, "map_server/change_state")
    request = ChangeState.Request()
    request.transition.id = 1
    future = client.call_async(request)
    rclpy.spin_until_future_complete(parent_node, future)
    request.transition.id = 3
    future = client.call_async(request)
    rclpy.spin_until_future_complete(parent_node, future)
