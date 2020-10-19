#!/usr/bin/env python3


"""Simulation game manager <extern> controller."""


from json import dumps
from os import environ, makedirs, path
from signal import SIGINT
from sys import exit
from zipfile import ZipFile

import psutil
from git import Repo

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
        makedirs(path.dirname(animation_path), exist_ok=True)
        while not self._trigger_start_asterix_client.wait_for_service(timeout_sec=60.0):
            self.get_logger().info('Waiting for asterix')
        self.get_logger().info(f'Starting match')
        self.robot.animationStartRecording(animation_path)
        self._synchronous_call(self._trigger_start_asterix_client, self._trigger_start_asterix_request)
        self._startup_time = self.robot.getTime()
        self._timer = self.create_timer(2.0, self._timer_callback)

    def fetch_index(self):
        """Fetch all objects indexes in webots."""
        ids = []
        for i in range(0, 2500):
            o = self.robot.getFromId(i)
            if o is not None:
                if o.getTypeName() in ['Table', 'Asterix', 'Obelix', 'RedSignal', 'GreenSIgnal']:
                    ids.append({'id': i, 'name': o.getField('name').getSFString(), 'type': o.getTypeName()})
        return ids

    def get_commit_filename(self):
        """Generate rbsd filename."""
        r = Repo(search_parent_directories=True)
        try:
            branch = r.active_branch.name.replace('/', '-').replace('.', '-')
        except TypeError:
            # Detached head state for tags
            branch = next((tag for tag in r.tags if tag.commit == r.head.commit), None)
        return f'ros-{branch}-{r.head.object.hexsha[:8]}.rbsd'

    def write_rbsd(self, filename):
        """Write rbsd file to disk."""
        ids = self.fetch_index()
        index = dumps({'id': ids})
        with open(path.join(path.dirname(animation_path), 'animation.json'), 'r') as m:
            moves = m.read()
        with ZipFile(filename, 'w') as rbsd:
            rbsd.writestr('index.json', index)
            rbsd.writestr('moves.json', moves)

    def _timer_callback(self):
        if (self.robot.getTime() - self._startup_time) <= 101:
            self.get_logger().info(f'Match simulation completion : {round(self.robot.getTime() - self._startup_time, 1)}%')
        else:
            self.robot.animationStopRecording()
            self.get_logger().info('Match simulation completion : 100%')
            filename = self.get_commit_filename()
            self.write_rbsd(path.join(path.dirname(animation_path), filename))
            self.get_logger().info(f'Wrote {filename}')
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
