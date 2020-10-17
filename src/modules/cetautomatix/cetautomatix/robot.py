#!/usr/bin/env python3


from time import sleep
from importlib import import_module

import numpy as np

import py_trees
import rclpy
import tf2_geometry_msgs
from cetautomatix.selftest import Selftest
from cetautomatix.magic_points import elements
from cetautomatix.strategy_modes import get_time_coeff
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer


class Robot(Node):
    def __init__(self):
        super().__init__(node_name='robot')
        self._triggered = False
        self._position = None
        self._start_time = None
        self._current_action = None
        robot = self.get_namespace().strip('/')
        # parameters interfaces
        self.declare_parameter('length')
        self.declare_parameter('width')
        self.declare_parameter('strategy_mode')
        self.length_param = self.get_parameter('length')
        self.width_param = self.get_parameter('width')
        self.strategy_mode_param = self.get_parameter('strategy_mode')
        # Bind actuators
        self.actuators = import_module(f'actuators.{robot}').actuators
        # Do selftest
        self.selftest = Selftest(self)
        # strategix client interfaces
        self._get_available_request = GetAvailableActions.Request()
        self._get_available_request.sender = robot
        self._get_available_client = self.create_client(GetAvailableActions, '/strategix/available')
        self._change_action_status_request = ChangeActionStatus.Request()
        self._change_action_status_request.sender = robot
        self._change_action_status_client = self.create_client(ChangeActionStatus, '/strategix/action')
        # Phararon delploy client interfaces
        self._get_trigger_deploy_pharaon_request = Trigger.Request()
        self._get_trigger_deploy_pharaon_client = self.create_client(Trigger, '/pharaon/deploy')
        # Odometry subscriber
        self._tf_buffer = Buffer()
        self._odom_pose_stamped = tf2_geometry_msgs.PoseStamped()
        self._odom_sub = self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        # Hacky way to init position
        self._odom_callback(self._odom_pose_stamped)
        # Py-Trees blackboard to send NavigateToPose actions
        self.blackboard = py_trees.blackboard.Client(name='NavigateToPose')
        self.blackboard.register_key(key='goal', access=py_trees.common.Access.WRITE)
        # Wait for strategix as this can block the behavior Tree
        while not self._get_available_client.wait_for_service(timeout_sec=5):
            self.get_logger().warn('Failed to contact strategix services ! Has it been started ?')
        # Robot trigger service
        self._trigger_start_robot_server = self.create_service(Trigger, 'start', self._start_robot_callback)
        # Reached initialised state
        self.get_logger().info('Cetautomatix ROS node has been started')

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
        if action is None:
            return False
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
        self.get_logger().info('Triggered pavillons')
        self.actuators.raiseTheFlag()
        self._change_action_status_request.action = "PAVILLON"
        self._change_action_status_request.request = 'CONFIRM'
        response = self._synchronous_call(self._change_action_status_client, self._change_action_status_request)
        if response is None:
            return False
        return response.success

    def _start_robot_callback(self, req, resp):
        """Start robot."""
        self._triggered = True
        self.get_logger().info('Triggered robot starter')
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        resp.success = True
        return resp

    def triggered(self):
        """Triggered var."""
        return self._triggered

    def _odom_callback(self, msg):
        try:
            # Get latest transform with 0
            tf = self._tf_buffer.lookup_transform('map', 'odom', Time(0))
            self._odom_pose_stamped.header = msg.header
            self._odom_pose_stamped.pose = msg.pose.pose
            tf_pose = tf2_geometry_msgs.do_transform_pose(self._odom_pose_stamped, tf)
        except LookupException:
            return
        self._position = (tf_pose.pose.position.x, tf_pose.pose.position.y)

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
            return None
        coefficient_list = []
        for action in action_list:
            coefficient = 0
            element = elements[action]
            distance = np.sqrt(
                (element["X"] - self._position[0])**2 + (element["Y"] - self._position[1])**2)
            coefficient += 100 * (1 - distance / 3.6)
            coefficient += get_time_coeff(self.get_clock().now().nanoseconds * 1e-9 - self._start_time, action, self.strategy_mode_param.value)
            coefficient_list.append(coefficient)
        best_action = action_list[coefficient_list.index(max(coefficient_list))]
        return best_action

    def get_goal_pose(self):
        """Get goal pose for action."""
        msg = NavigateToPose_Goal()
        msg.pose.header.frame_id = 'map'
        if self._current_action is not None:
            element = elements[self._current_action]
            msg.pose.pose.position.x = float(element["X"])
            msg.pose.pose.position.y = float(element["Y"])
            msg.pose.pose.position.z = 0.0
            theta = element.get("Rot")
            if theta is not None:
                theta %= 360
                q = self.euler_to_quaternion(theta)
                # TODO : Changement de repère plutot
                if theta == 0:
                    msg.pose.pose.position.x -= self.length_param.value / 2
                elif theta == 180:
                    msg.pose.pose.position.x += self.length_param.value / 2
                elif theta == 90:
                    msg.pose.pose.position.y -= self.width_param.value / 2
                elif theta == 270:
                    msg.pose.pose.position.y += self.width_param.value / 2
            else:
                self.get_logger().warn(f'NavigateToPose for action {self._current_action} invoqued without angle')
                q = self.euler_to_quaternion(0)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
        self.blackboard.goal = msg
