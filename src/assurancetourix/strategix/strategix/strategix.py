#!/usr/bin/env python3

from strategix_msgs.action import StrategixAction
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from score import Score


class StrategixActionServer(Node):
    def __init__(self):
        super().__init__('strategix_action_server')
        self.score = Score()
        self._action_server = ActionServer(self, StrategixAction, '/strategix', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        todo = None
        if goal_handle.request.request == 'todo':
            todo = self.score.todoList
        elif goal_handle.request.request == 'preempt':
            self.score.preempt(goal_handle.request.object)
        elif goal_handle.request.request == 'release':
            self.score.release(goal_handle.request.object)
        elif goal_handle.request.request == 'finish':
            self.score.finish(goal_handle.request.object)
        goal_handle.succeed()
        result = StrategixAction.Result()
        result.todo = todo
        return result


def main(args=None):
    rclpy.init(args=args)
    strategix = StrategixActionServer()
    rclpy.spin(strategix)
    strategix.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
