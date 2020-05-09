#!/usr/bin/env python3


"""Simulation game manager <extern> controller."""

from os import environ

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from webots_ros2_core.webots_node import WebotsNode

environ['WEBOTS_ROBOT_NAME'] = 'game_manager'
data = {'time': [], 'speed_linear': [], 'speed_angular': [], 'rot': [], 'trans': []}


class GameManager(WebotsNode):
    """Game manager node."""

    def __init__(self, args=None):
        super().__init__('game_manager', args)
        self._tf_buffer = Buffer()
        self._startup_time = self.robot.getTime()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_pose_stamped = tf2_geometry_msgs.PoseStamped()
        self.create_subscription(Odometry, '/asterix/odom', self.compute_odom_error, 1)
        self.asterix = self.robot.getFromDef('ASTERIX')

    def get_relative_unix_timestamp(self, time):
        return time.sec + (time.nanosec * 1e-9) - self._startup_time

    def compute_odom_error(self, odom):
        """Compute odometry error."""
        # Get Position
        x, _, y = self.asterix.getPosition()
        y = 2 - y

        # Get orientation
        rot_matrix = self.asterix.getOrientation()
        _, _, theta = Rotation.from_matrix(np.array(rot_matrix).reshape(3, 3)).as_euler('xzy')

        try:
            tf = self._tf_buffer.lookup_transform('map', 'odom', odom.header.stamp, timeout=Duration(seconds=1.0))
            self._odom_pose_stamped.header = odom.header
            self._odom_pose_stamped.pose = odom.pose.pose
            tf_pose = tf2_geometry_msgs.do_transform_pose(self._odom_pose_stamped, tf)
        except LookupException:
            return

        ox = tf_pose.pose.position.x
        oy = tf_pose.pose.position.y

        # Get ROS orientation
        q = odom.pose.pose.orientation
        _, _, otheta = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        trans_error = np.abs(np.sqrt(np.sum(np.square([x, y]))) - np.sqrt(np.sum(np.square([ox, oy]))))
        rot_error = theta - otheta
        print('Odometry error', round(trans_error * 100, 3), 'cm -- Angular', rot_error, 'rads', flush=True)

        data['time'].append(self.get_relative_unix_timestamp(odom.header.stamp))
        data['speed_linear'].append(odom.twist.twist.linear.x)
        data['speed_angular'].append(odom.twist.twist.angular.z)
        data['rot'].append(rot_error)
        data['trans'].append(trans_error * 100)


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


main()

fig, (ax11, ax21) = plt.subplots(2, sharex=True)
color = 'tab:blue'
ax11.set_xlabel('sim time ($s$)')
ax11.plot(data['time'], data['trans'], color=color)
ax11.set_ylabel('odom trans error ($cm$)', color=color)
ax12 = ax11.twinx()
color = 'tab:orange'
ax12.plot(data['time'], data['speed_linear'], color=color)
ax12.set_ylabel('odom speed linear ($m.s^{-1}$)', color=color)

color = 'tab:blue'
ax21.plot(data['time'], data['rot'], color=color)
ax21.set_ylabel('odom rotational error ($rad$)', color=color)
ax22 = ax21.twinx()
color = 'tab:orange'
ax22.plot(data['time'], data['speed_angular'], color=color)
ax22.set_ylabel('odom speed angular ($rad.s^{-1}$)', color=color)

fig.tight_layout()
plt.show()
