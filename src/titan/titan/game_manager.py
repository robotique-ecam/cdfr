#!/usr/bin/env python3


"""Simulation game manager <extern> controller."""

from os import environ, makedirs, path
from signal import SIGINT
from sys import exit
from time import sleep

import psutil

import rclpy
from std_srvs.srv import Trigger
from webots_ros2_core.webots_node import WebotsNode

environ['WEBOTS_ROBOT_NAME'] = 'game_manager'
animation_path = path.expanduser('~/animation/animation.html')


class GameManager(WebotsNode):
    """Game manager node."""

    def __init__(self, args=None):
        super().__init__('game_manager', args)
        self._trigger_start_asterix_request = Trigger.Request()
        self._trigger_start_asterix_client = self.create_client(Trigger, '/asterix/start')
        self.get_logger().info(f'Starting match')
        makedirs(path.dirname(animation_path), exist_ok=True)
        self.robot.animationStartRecording(animation_path)
        self._synchronous_call(self._trigger_start_asterix_client, self._trigger_start_asterix_request)
        self._startup_time = self.robot.getTime()
        self._timer = self.create_timer(2.0, self._timer_callback)

    def _timer_callback(self):
        if (self.robot.getTime() - self._startup_time) <= 100:
            self.get_logger().info(f'Match simulation completion : {round(self.robot.getTime() - self._startup_time, 1)}%')
        else:
            self.robot.animationStopRecording()
            self.get_logger().info(f'Match simulation completion : 100%')
            self.stop_ros()
            exit(0)

    def _synchronous_call(self, client, request):
        """Call service synchronously."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
        except BaseException:
            response = None
        return response

    def stop_ros(self):
        """Stop ros launch processes."""
        for p in psutil.process_iter(['pid', 'name', 'cmdline']):
            if 'ros2' in p.name() and 'launch' in p.cmdline():
                print(f'Sent SIGINT to ros2 launch {p.pid}')
                p.send_signal(SIGINT)


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    game_manager = GameManager(args=args)
    try:
        rclpy.spin(game_manager)
    except KeyboardInterrupt:
        pass
    game_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
