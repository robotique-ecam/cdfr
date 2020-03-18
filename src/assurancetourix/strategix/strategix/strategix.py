#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from strategix.score import Score
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions


class StrategixActionServer(Node):
    def __init__(self):
        super().__init__('strategix_action_server')
        self.score = Score()
        self.todo_srv = self.create_service(GetAvailableActions, '/strategix/available', self.available_callback)
        self.action_srv = self.create_service(ChangeActionStatus, '/strategix/action', self.action_callback)

    def available_callback(self, request, response):
        response.available = self.score.todoList
        return response

    def action_callback(self, request, response):
        if request.request == 'preempt':
            self.score.preempt(request.action)
        elif request.request == 'drop':
            self.score.release(request.action)
        elif request.request == 'complete':
            self.score.finish(request.action)
        return response


def main(args=None):
    rclpy.init(args=args)
    strategix = StrategixActionServer()
    rclpy.spin(strategix)
    strategix.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
