#!/usr/bin/env python3


"""Strategix action server and score counter."""

from os import environ

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from strategix.exceptions import MatchStartedException
from strategix.score import Score
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions


class StrategixActionServer(Node):
    def __init__(self):
        super().__init__('strategix_action_server')
        self.side = self.declare_parameter('side', 'blue')
        # TODO: remove on foxy release
        if environ['ROS_DISTRO'] == 'eloquent':
            self.set_parameters_callback(self._on_set_parameters)
        else:
            self.add_on_set_parameters_callback(self._on_set_parameters)
        self.score = Score()
        self.todo_srv = self.create_service(GetAvailableActions, '/strategix/available', self.available_callback)
        self.action_srv = self.create_service(ChangeActionStatus, '/strategix/action', self.action_callback)
        self.get_logger().info(f'Default side is {self.side.value}')
        self.get_logger().info('Strategix is ready')

    def excludeList(self):
        return self.score.excludeFromBlue if self.side.value == 'blue' else self.score.excludeFromYellow

    def _on_set_parameters(self, params):
        """Handle Parameter events especially for side."""
        result = SetParametersResult()
        try:
            for param in params:
                if param.name == 'side':
                    self.get_logger().info(f'Side changed {param.value}')
                    self.side = param
                else:
                    setattr(self, param.name, param)
            result.successful = True
        except MatchStartedException as e:
            result.reason = e
        return result

    def available_callback(self, request, response):
        self.get_logger().info('GET %s' % (request.sender))
        exclude = self.excludeList()
        response.available = [todo for todo in self.score.todoList if todo not in exclude]
        self.get_logger().info('AVAILABLE: %s' % (response.available))
        return response

    def action_callback(self, request, response):
        try:
            self.get_logger().info('%s %s %s' % (request.sender, request.request, request.action))
            if request.action == 'ARUCO42':
                response.success = True
            elif request.request == 'PREEMPT':
                response.success = self.score.preempt(request.action)
            elif request.request == 'DROP':
                response.success = self.score.release(request.action)
            elif request.request == 'CONFIRM':
                response.success = self.score.finish(request.action)
            else:
                raise BaseException
        except BaseException:
            self.get_logger().warn('Invalid call : %s %s %s' % (request.sender, request.request, request.action))
            response.success = False
        self.score.updateScore()
        return response


def main(args=None):
    rclpy.init(args=args)
    strategix = StrategixActionServer()
    try:
        rclpy.spin(strategix)
    except KeyboardInterrupt:
        pass
    strategix.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
