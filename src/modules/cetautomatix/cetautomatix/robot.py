#!/usr/bin/env python3


import numpy as np

import py_trees
import rclpy
import tf2_geometry_msgs
from cetautomatix.magic_points import elements
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_srvs.srv import Trigger
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer


class Robot(Node):
    def __init__(self):
        super().__init__(node_name='robot')
        robot = self.get_namespace()
        self.position = (0.29, 1.33)
        self.length = 0.26
        self.width = 0.2
        self._triggered = False
        self._current_action = None
        self._get_available_client = self.create_client(GetAvailableActions, '/strategix/available')
        self._get_available_request = GetAvailableActions.Request()
        self._get_available_request.sender = robot
        self._change_action_status_client = self.create_client(ChangeActionStatus, '/strategix/action')
        self._change_action_status_request = ChangeActionStatus.Request()
        self._change_action_status_request.sender = robot
        self._trigger_start_robot_server = self.create_service(Trigger, 'start', self._start_robot_callback)
        self._get_trigger_deploy_pharaon_client = self.create_client(Trigger, '/pharaon/deploy')
        self._get_trigger_deploy_pharaon_request = Trigger.Request()
        self._odom_sub = self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.blackboard = py_trees.blackboard.Client(name='NavigateToPose')
        self.blackboard.register_key(key='goal', access=py_trees.common.Access.WRITE)
        self._tf_buffer = Buffer()
        self._odom_pose_stamped = tf2_geometry_msgs.PoseStamped()

    def _synchronous_call(self, client, request):
        """Call service synchronously."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
        except BaseException:
            response = None
        return response

    def fetch_available_actions(self):
        """Fetch available actions for BT."""
        response = self._synchronous_call(self._get_available_client, self._get_available_request)
        return response.available if response is not None else []

    def preempt_action(self, action):
        """Preempt an action for the BT."""
        self._change_action_status_request.action = str(action)
        self._change_action_status_request.request = 'PREEMPT'
        response = self._synchronous_call(self._change_action_status_client, self._change_action_status_request)
        if response is None:
            return False
        self._current_action = action if response.success else None
        if self._current_action is not None:
            self.get_goal_pose()
        return response.success

    def drop_current_action(self):
        """Drop an action for the BT."""
        if self._current_action is None:
            return False
        self._change_action_status_request.action = self._current_action
        self._change_action_status_request.request = 'DROP'
        response = self._synchronous_call(self._change_action_status_client, self._change_action_status_request)
        if response is None:
            return False
        self._current_action = None
        return response.success

    def confirm_current_action(self):
        """Confirm an action for the BT."""
        if self._current_action is None:
            return False
        self._change_action_status_request.action = self._current_action
        self._change_action_status_request.request = 'CONFIRM'
        response = self._synchronous_call(self._change_action_status_client, self._change_action_status_request)
        if response is None:
            return False
        self._current_action = None
        return response.success

    def start_actuator_action(self):
        self.get_logger().info('START ACTUATOR %s' % (self._current_action))
        if 'PHARE' in self._current_action:
            response = self._synchronous_call(self._get_trigger_deploy_pharaon_client, self._get_trigger_deploy_pharaon_request)
            return response.success
        else:
            return True

    def trigger_pavillons(self):
        # TODO
        self.get_logger().info('Triggered pavillons')

    def _start_robot_callback(self, req, resp):
        """Start robot."""
        self._triggered = True
        self.get_logger().info('Triggered robot starter')
        resp.success = True
        return resp

    def triggered(self):
        """Triggered var."""
        return self._triggered

    def _odom_callback(self, msg):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'odom', msg.header.stamp, timeout=Duration(seconds=1.0))
            self._odom_pose_stamped.header = msg.header
            self._odom_pose_stamped.pose = msg.pose.pose
            tf_pose = tf2_geometry_msgs.do_transform_pose(self._odom_pose_stamped, tf)
        except LookupException:
            return
        self.position = (tf_pose.pose.position.x, tf_pose.pose.position.y)

    def euler_to_quaternion(self, yaw, pitch=0, roll=0):
        """Conversion between euler angles and quaternions."""
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def compute_best_action(self, action_list):
        """Calculate best action to choose from its distance to the robot."""
        if not action_list:
            self._current_action = 'ARUCO42'
            return 'ARUCO42'
        distance_list = []
        # start = timeEndOfGame.time - 100.0
        for action in action_list:
            value = elements[action]
            distance = np.sqrt(
                (value[0] - self.position[0])**2 + (value[1] - self.position[1])**2)
            coeff_distance = 100 * (1 - distance / 3.6)
            distance_list.append(coeff_distance)
        best_action = action_list[distance_list.index(max(distance_list))]
        return best_action

    def get_goal_pose(self):
        """Get goal pose for action."""
        msg = NavigateToPose_Goal()
        if self._current_action is not None:
            value = elements[self._current_action]
            msg.pose.pose.position.x = float(value[0])
            msg.pose.pose.position.y = float(value[1])
            msg.pose.pose.position.z = 0.0
            try:
                if value[2] is not None:
                    q = self.euler_to_quaternion(value[2])
                    if value[2] == 0:
                        msg.pose.pose.position.x -= self.width / 2
                    elif value[2] == 180:
                        msg.pose.pose.position.x += self.width / 2
                    elif value[2] == 90:
                        msg.pose.pose.position.y -= self.width / 2
                    elif value[2] == -90:
                        msg.pose.pose.position.y += self.width / 2
                else:
                    q = self.euler_to_quaternion(0)
            except IndexError:
                # TODO: robot angle
                q = self.euler_to_quaternion(0)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
        self.blackboard.goal = msg
