##!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from src.assurancetourix.strategix.action import StrategixAction


class StrategixActionClient(Node):
    def __init__(self):
        super().__init__('strategix_action_client')
        self._action_client = ActionClient(self, StrategixAction, 'strategix')

    def send_goal(self, robot, request, object):
        goal_msg = StrategixAction.Goal()
        goal_msg.robot = robot
        goal_msg.request = request
        goal_msg.object = object
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = StrategixActionClient()


if __name__ == '__main__':
    main()
