#!/usr/bin/env python3


"""Strategix action server and score counter."""

from threading import Thread

import rclpy
from lcd_msgs.msg import Lcd
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from strategix.actions import actions
from strategix.exceptions import MatchStartedException
from strategix.score import Score
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions


class StrategixActionServer(Node):
    def __init__(self):
        super().__init__('strategix_action_server')
        self.side = self.declare_parameter('side', 'blue')
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.score = Score()
        self.todo_srv = self.create_service(GetAvailableActions, '/strategix/available', self.available_callback)
        self.action_srv = self.create_service(ChangeActionStatus, '/strategix/action', self.action_callback)
        self.lcd_driver = self.create_publisher(Lcd, '/obelix/lcd', 1)
        self.get_logger().info(f'Default side is {self.side.value}')
        self.get_logger().info('Strategix is ready')

    def _on_set_parameters(self, params):
        """Handle Parameter events especially for side."""
        result = SetParametersResult()
        try:
            for param in params:
                if param.name == 'side':
                    self.get_logger().warn(f'Side changed {param.value}')
                    self.side = param
                else:
                    setattr(self, param.name, param)
            result.successful = True
        except MatchStartedException as e:
            result.reason = e
        return result

    def available_callback(self, request, response):
        """Callback function when a robot needs the list of available actions"""
        self.get_logger().info(f'GET {request.sender}')
        if self.side.value == 'blue':
            response.available = [action for action in actions if not action.get("ONLY_YELLOW") and not action.get("STATUS")]
        else:
            response.available = [action for action in actions if not action.get("ONLY_BLUE") and not action.get("STATUS")]
        self.get_logger().info(f'AVAILABLE: {response.available}')
        return response

    def action_callback(self, request, response):
        """Callback function when a robot has to change its current action"""
        try:
            self.get_logger().info(f'{request.sender} {request.request} {request.action}')
            if request.request == 'PREEMPT':
                response.success = self.score.preempt(request.action, request.sender)
            elif request.request == 'DROP':
                response.success = self.score.release(request.action, request.sender)
            elif request.request == 'CONFIRM':
                response.success = self.score.finish(request.action, request.sender)
                # Detach update_score coroutine
                Thread(target=self.update_score).start()
            else:
                raise BaseException
        except BaseException:
            self.get_logger().warn(f'Invalid call : {request.sender} {request.request} {request.action}')
            response.success = False
        return response

    def update_score(self):
        """Update global score."""
        score = self.score.get_score()
        lcd_msg = Lcd()
        lcd_msg.line = 1
        lcd_msg.text = f'Score: {score}'
        self.lcd_driver.publish(lcd_msg)


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
