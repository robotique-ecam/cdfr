#!/usr/bin/env python3

from strategix.srv import StrategixSrv
import rclpy
from rclpy.node import Node
from score import Score


class Strategix(Node):
    def __init__(self):
        super().__init__('strategix')
        self.score = Score()
        self.srv = self.create_service(StrategixSrv, 'strategix', self.send_response)

    def send_response(self, request, response):
        response.todo = None
        print("Received: " + request.request, request.object + " from " + request.robot)
        if request.request == 'todo':
            response.todo = self.score.todoList
        elif request.request == 'preempt':
            self.score.preempt(request.object)
        elif request.request == 'release':
            self.score.release(request.object)
        elif request.request == 'finish':
            self.score.finish(request.object)
        return response


def main(args=None):
    rclpy.init(args=args)
    strategix = Strategix()
    rclpy.spin(strategix)
    strategix.destroy_node()
    rclpy.shutdown()
