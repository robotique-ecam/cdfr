#!/usr/bin/env python3


import numpy as np

import py_trees
import rclpy
from cetautomatix.magic_points import elements
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions


class Robot(Node):
    def __init__(self):
        super().__init__(node_name='robot')
        robot = self.get_namespace()
        self._triggered = False
        self._current_action = None
        self._get_available_client = self.create_client(GetAvailableActions, '/strategix/available')
        self._change_action_status_client = self.create_client(ChangeActionStatus, '/strategix/action')
        self._trigger_start_robot_server = self.create_service(Trigger, 'start', self._start_robot_callback)
        self._get_trigger_deploy_pharaon_client = self.create_client(Trigger, '/pharaon/deploy/')
        self._get_available_request = GetAvailableActions.Request()
        self._change_action_status_request = ChangeActionStatus.Request()
        self._get_trigger_deploy_pharaon_request = Trigger.Request()
        self._get_available_request.sender = robot
        self._change_action_status_request.sender = robot
        self._odom_sub = self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        self.blackboard = py_trees.blackboard.Client(name='NavigateToPose')
        self.blackboard.register_key(key='goal', access=py_trees.common.Access.WRITE)

    def _synchronous_call(self, client, request):
        """Synchronous service call util function."""
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
        self.get_goal_pose()
        self._change_action_status_request.action = action
        self._change_action_status_request.request = 'PREEMPT'
        response = self._synchronous_call(self._change_action_status_client, self._change_action_status_request)
        if response is None:
            return False
        self._current_action = action if response.success else None
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
        if 'PHARE' in self._current_action:
            response = self._synchronous_call(self._get_trigger_deploy_pharaon_client, self._get_trigger_deploy_pharaon_request)
            return response.success
        else:
            return True

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
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def euler_to_quaternion(self, yaw, pitch=0, roll=0):
        """Conversion between euler angles and quaternions."""
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def compute_best_action(self, list):
        action_coeff_list = []
        # start = timeEndOfGame.time - 100.0
        for action in list:
            for key, value in elements.items():
                if key == action:
                    distance = np.sqrt(
                        (value[0] - self.position[0])**2 + (value[1] - self.position[1])**2)
                    coeff_distance = distance * 100 / 3.6
                    action_coeff_list.append((key, coeff_distance))
                    break
        max = 0
        best_action = None
        for action in action_coeff_list:
            if action[1] > max:
                max = action[1]
                best_action = action[0]
        self._current_action = best_action
        return best_action

    def get_goal_pose(self):
        """Get goal pose for action."""
        msg = NavigateToPose_Goal()
        if self._current_action is not None:
            value = elements[self._current_action]
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.position.x = float(value[0])
            msg.pose.pose.position.y = float(value[1])
            try:
                q = self.euler_to_quaternion(value[2] if value[2] is not None else 0)
            except IndexError:
                # TODO: robot angle
                q = self.euler_to_quaternion(0)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
        self.blackboard.goal = msg
