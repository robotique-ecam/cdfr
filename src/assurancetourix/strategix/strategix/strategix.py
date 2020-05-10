#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from strategix.score import Score
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions

color = 'blue'


class StrategixActionServer(Node):
    def __init__(self):
        super().__init__('strategix_action_server')
        self.score = Score()
        self.todo_srv = self.create_service(GetAvailableActions, '/strategix/available', self.available_callback)
        self.action_srv = self.create_service(ChangeActionStatus, '/strategix/action', self.action_callback)
        self.get_logger().info('Strategix is ready')

    def excludeList(self):
        return self.score.excludeFromBlue if color == 'blue' else self.score.excludeFromYellow

    def available_callback(self, request, response):
        response.available = [todo for todo in self.score.todoList if todo not in self.excludeList()]
        self.get_logger().info('GET %s' % (request.sender))
        return response

    def action_callback(self, request, response):
        try:
            if request.request == 'PREEMPT':
                response.success = self.score.preempt(request.action)
            elif request.request == 'DROP':
                response.success = self.score.release(request.action)
            elif request.request == 'CONFIRM':
                response.success = self.score.finish(request.action)
            else:
                raise BaseException
            self.get_logger().info('%s %s %s' % (request.sender, request.request, request.action))
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
