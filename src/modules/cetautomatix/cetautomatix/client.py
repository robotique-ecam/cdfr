##!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from strategix_msgs.action import StrategixAction


class StrategixActionClient(Node):
    def __init__(self):
        super().__init__('strategix_action_client')
        self._action_client = ActionClient(self, StrategixAction, '/strategix')

    def send_goal(self, sender, request, object):
        goal_msg = StrategixAction.Goal()
        goal_msg.sender = sender
        goal_msg.request = request
        goal_msg.object = object
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.list = future.result().result.todo
        self.get_logger().info('Result: {0}'.format(self.list))
