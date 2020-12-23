#!/usr/bin/env python3


import math
import time
from importlib import import_module
from signal import SIGINT
from subprocess import call
from threading import Thread
from platform import machine

import numpy as np
import psutil

import py_trees
import rclpy
from actuators.actuators import NC, NO
from actuators_srvs.srv import Slider
from cetautomatix import tf2_geometry_msgs
from cetautomatix.magic_points import elements
from cetautomatix.selftest import Selftest
from cetautomatix.strategy_modes import get_time_coeff
from lcd_msgs.msg import Lcd
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Duration, Time
from std_srvs.srv import SetBool, Trigger
from strategix.actions import actions
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions
from tf2_ros import LookupException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Robot(Node):
    def __init__(self):
        super().__init__(node_name='cetautomatix')
        # Detect simulation mode
        self.simulation = True if machine() != 'aarch64' else False
        self.i = 0
        self.stupid_actions = ['STUPID_1', 'STUPID_2', 'STUPID_3']
        self._triggered = False
        self._position = None
        self._orientation = None
        self._start_time = None
        self._current_action = None
        self.robot = self.get_namespace().strip('/')
        # parameters interfaces
        self.side = self.declare_parameter('side', 'blue')
        self.declare_parameter('length')
        self.declare_parameter('width')
        self.declare_parameter('strategy_mode')
        self.length_param = self.get_parameter('length')
        self.width_param = self.get_parameter('width')
        self.strategy_mode_param = self.get_parameter('strategy_mode')
        # Bind actuators
        self.actuators = import_module(f'actuators.{self.robot}').actuators
        # Do selftest
        self.selftest = Selftest(self)
        # Prechill engines
        self.actuators.setFansEnabled(True)
        # Stop ros service
        self._stop_ros_service = self.create_service(Trigger, 'stop', self._stop_robot_callback)
        # strategix client interfaces
        self._get_available_request = GetAvailableActions.Request()
        self._get_available_request.sender = self.robot
        self._get_available_client = self.create_client(GetAvailableActions, '/strategix/available')
        self._change_action_status_request = ChangeActionStatus.Request()
        self._change_action_status_request.sender = self.robot
        self._change_action_status_client = self.create_client(ChangeActionStatus, '/strategix/action')
        # Phararon delploy client interfaces
        self._get_trigger_deploy_pharaon_request = Trigger.Request()
        self._get_trigger_deploy_pharaon_client = self.create_client(Trigger, '/pharaon/deploy')
        # Slider driver
        self._set_slider_position_request = Slider.Request()
        self._set_slider_position_client = self.create_client(Slider, 'slider_position')
        # Odometry subscriber
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_pose_stamped = tf2_geometry_msgs.PoseStamped()
        self._odom_callback(self._odom_pose_stamped)
        self._odom_sub = self.create_subscription(Odometry, 'odom', self._odom_callback, 1)
        # Py-Trees blackboard to send NavigateToPose actions
        self.blackboard = py_trees.blackboard.Client(name='NavigateToPose')
        self.blackboard.register_key(key='goal', access=py_trees.common.Access.WRITE)
        # LCD driver direct access
        self._lcd_driver_pub = self.create_publisher(Lcd, '/obelix/lcd', 1)
        # Wait for strategix as this can block the behavior Tree
        while not self._get_available_client.wait_for_service(timeout_sec=5):
            self.get_logger().warn('Failed to contact strategix services ! Has it been started ?')
        # Enable stepper drivers
        if not self.simulation:
            self._get_enable_drivers_request = SetBool.Request()
            self._get_enable_drivers_request.data = True
            self._get_enable_drivers_client = self.create_client(SetBool, 'enable_drivers')
            self._synchronous_call(self._get_enable_drivers_client, self._get_enable_drivers_request)
        # Robot trigger service
        self._trigger_start_robot_server = self.create_service(Trigger, 'start', self._start_robot_callback)
        if self.robot == 'obelix':
            self._trigger_start_asterix_request = Trigger.Request()
            self._trigger_start_asterix_client = self.create_client(Trigger, '/asterix/start')

        # Reached initialized state
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

    def set_slider_position(self, position: int):
        """Set slider position with position in range 0 to 255."""
        self._set_slider_position_request.position = position
        return self._synchronous_call(self._set_slider_position_client, self._set_slider_position_request)

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
        self.get_logger().info(f'START ACTUATOR {self._current_action}')
        if 'PHARE' in self._current_action:
            response = self._synchronous_call(self._get_trigger_deploy_pharaon_client, self._get_trigger_deploy_pharaon_request)
            return response.success
        elif 'GOB' in self._current_action:
            self.actuators.PUMPS[self.current_pump]['STATUS'] = self._current_action
            servo = self.actuators.DYNAMIXELS[self.current_pump]
            self.actuators.arbotix.setPosition(self.current_pump, servo.get('up'))
            return True
        elif 'ECUEIL' in self._current_action:
            pump_list = []
            i = 0
            for pump_id, pump_dict in self.actuators.PUMPS.items():
                if pump_dict.get("type") == NO:
                    pump_list.append(pump_id)
                    self.actuators.PUMPS[pump_id]['STATUS'] = actions[self._current_action].get('GOBS')[i]
                    i += 1
            self.actuators.setPumpsEnabled(True, pump_list)
            # TODO: Fermer Herse
            self.set_slider_position(255)
            return True
        elif 'CHENAL' in self._current_action:
            color = 'GREEN' if 'VERT' in self._current_action else 'RED'
            pump_list = []
            for pump_id, pump_dict in self.actuators.PUMPS.items():
                if pump_dict.get("type") == self.drop and pump_dict.get("STATUS") == color:
                    pump_list.append(pump_id)
            if self.drop == NC:
                for pump in pump_list:
                    servo = self.actuators.DYNAMIXELS[pump]
                    self.actuators.arbotix.setPosition(pump, servo.get('down'))
            else:
                self.set_slider_position(100)
                # TODO: Ouvrir herse
            self.actuators.setPumpsEnabled(False, pump_list)
            for pump in pump_list:
                del self.actuators.PUMPS[pump]["STATUS"]
            return True
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
        if self.robot == 'obelix':
            # TODO : Fix sync call
            # Thread(target=self._synchronous_call, args=[self._trigger_start_asterix_client, self._trigger_start_asterix_request]).start()
            self.get_logger().info('Obelix triggered Asterix')
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        lcd_msg = Lcd()
        lcd_msg.line = 0
        lcd_msg.text = f'{self.robot.capitalize()} running'.ljust(16)
        self._lcd_driver_pub.publish(lcd_msg)
        lcd_msg.line = 1
        lcd_msg.text = 'Score: 0'.ljust(16)
        self._lcd_driver_pub.publish(lcd_msg)
        if hasattr(resp, 'success'):
            resp.success = True
        return resp

    def _stop_robot_callback(self, req, resp):
        """Stop robot / ROS."""
        self.stop_ros(shutdown=False)
        resp.sucess = True
        return resp

    def triggered(self):
        """Triggered var."""
        return self._triggered

    def _odom_callback(self, msg):
        try:
            # Get latest transform
            tf = self._tf_buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=1.0))
            self._odom_pose_stamped.header = msg.header
            self._odom_pose_stamped.pose = msg.pose.pose
            tf_pose = tf2_geometry_msgs.do_transform_pose(self._odom_pose_stamped, tf)
        except LookupException:
            self.get_logger().warn('Failed to lookup_transform from map to odom')
            return
        except ConnectivityException:
            self.get_logger().warn("ConnectivityException, 'map' and 'base_link' are not part of the same tree")
            return
        self._position = (tf_pose.pose.position.x, tf_pose.pose.position.y)
        self._orientation = self.quaternion_to_euler(tf_pose.pose.orientation.x, tf_pose.pose.orientation.y, tf_pose.pose.orientation.z, tf_pose.pose.orientation.w)

    def euler_to_quaternion(self, yaw, pitch=0, roll=0):
        """Conversion between euler angles and quaternions."""
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return (qx, qy, qz, qw)

    def quaternion_to_euler(self, x, y, z, w):
        """Conversion between quaternions and euler angles."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)
        return (X, Y, Z)

    def get_position(self, element, initial_orientation, diff_orientation, offset):
        element = elements[element]
        element_x, element_y = element.get("X"), element.get("Y")
        offset_x, offset_y = offset[0], offset[1]

        alpha = initial_orientation[0] - np.pi / 2
        v = (offset_x * np.cos(alpha) - offset_y * np.sin(alpha),
             offset_x * np.sin(alpha) + offset_y * np.cos(alpha))
        v = (v[0] * np.cos(diff_orientation) - v[1] * np.sin(diff_orientation),
             v[0] * np.sin(diff_orientation) + v[1] * np.cos(diff_orientation))
        return (element_x - v[0], element_y - v[1])

    def get_orientation(self, element, robot_pos, initial_orientation):
        if robot_pos is None:
            return (0, 0)
        element = elements[element]
        element_x, element_y = element.get('X'), element.get('Y')
        robot_pos_x, robot_pos_y = robot_pos[0], robot_pos[1]

        u = (np.cos(initial_orientation[0]), np.sin(initial_orientation[0]))
        u = u / (np.sqrt(u[0]**2 + u[1]**2))
        if element.get('Rot') is None:
            v = (element_x - robot_pos_x, element_y - robot_pos_y)
            v = v / (np.sqrt(v[0]**2 + v[1]**2))
            theta = np.arccos(
                np.dot(u, v) / (np.sqrt(u[0]**2 + u[1]**2) * np.sqrt(v[0]**2 + v[1]**2)))
            if robot_pos_y <= element_y:
                theta = theta
            else:
                theta = -theta
        else:
            angle = element.get("Rot")
            theta = np.deg2rad(angle) - initial_orientation[0]
        phi = initial_orientation[0] + theta
        return (theta, phi)

    def compute_best_action(self, action_list):
        """Calculate best action to choose from its distance to the robot."""
        return self.stupid_actions[self.i % len(self.stupid_actions)]
        check = self.check_to_empty()
        self.get_logger().info(f'{check}')
        if check is not None:
            action_list = check
        if not action_list:
            return None
        coefficient_list = []
        for action in action_list:
            coefficient = 0
            element = elements[action]
            distance = np.sqrt(
                (element['X'] - self._position[0])**2 + (element['Y'] - self._position[1])**2)
            coefficient += 100 * (1 - distance / 3.6)
            coefficient += get_time_coeff(self.get_clock().now().nanoseconds * 1e-9 - self._start_time, action, self.strategy_mode_param.value)
            coefficient_list.append(coefficient)
        best_action = action_list[coefficient_list.index(max(coefficient_list))]
        return best_action

    def check_to_empty(self):
        empty_behind, empty_front = True, True
        for pump_id, pump_dict in self.actuators.PUMPS.items():
            if pump_dict.get('STATUS') is None:
                if pump_dict.get('type') == NO:
                    empty_behind = False
                else:
                    empty_front = False
        self.get_logger().info(f'behind: {empty_behind}, front: {empty_front}')
        if empty_behind or empty_front:
            if self.side.value == 'blue':
                return ["CHENAL_BLEU_VERT_1", "CHENAL_BLEU_ROUGE_1"]
            else:
                return ["CHENAL_JAUNE_VERT_1", "CHENAL_JAUNE_ROUGE_1"]
        return None

    def get_goal_pose(self):
        """Get goal pose for action."""
        msg = NavigateToPose_Goal()
        msg.pose.header.frame_id = 'map'
        # if self._current_action is not None:
        #     offset = (0, 0)
        #     if 'GOB' in self._current_action:
        #         for pump_id, pump_dict in self.actuators.PUMPS.items():
        #             if pump_dict.get('type') == NC and not pump_dict.get('STATUS'):
        #                 self.current_pump = pump_id
        #                 offset = pump_dict.get("pos")
        #                 self.actuators.setPumpsEnabled(True, [pump_id])
        #                 break
        #     elif 'ECUEIL' in self._current_action:
        #         for pump_id, pump_dict in self.actuators.PUMPS.items():
        #             if pump_dict.get('type') == NO and pump_dict.get('STATUS') is not None:
        #                 offset = pump_dict.get("pos")
        #                 break
        #         self.set_slider_position(0)
        #     elif 'CHENAL' in self._current_action:
        #         arriere = False
        #         for pump_id, pump_dict in self.actuators.PUMPS.items():
        #             if pump_dict.get('type') == NO and pump_dict.get('STATUS') is not None:
        #                 arriere = True
        #         if arriere is True:
        #             offset = (0, -0.1)
        #             elements[self._current_action]["Rot"] = -90
        #             self.drop = NO
        #         else:
        #             offset = (0, 0.1)
        #             elements[self._current_action]["Rot"] = 90
        #             self.drop = NC
        offset = (0, 0)
        (theta, phi) = self.get_orientation(self._current_action, self._position, self._orientation)
        position = self.get_position(self._current_action, self._orientation, theta, offset)
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = 0.0
        q = self.euler_to_quaternion(phi)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.blackboard.goal = msg
        self.i += 1
        time.sleep(1)

    def stop_ros(self, shutdown=True):
        """Stop ros launch processes."""
        # Publish "Robot is done"
        lcd_msg = Lcd()
        lcd_msg.line = 0
        lcd_msg.text = f'{self.robot.capitalize()} is done!'.ljust(16)
        self._lcd_driver_pub.publish(lcd_msg)
        if not self.simulation:
            # Disable stepper drivers
            self._get_enable_drivers_request.data = False
            self._synchronous_call(self._get_enable_drivers_client, self._get_enable_drivers_request)
            # Stop fans and relax servos
            self.actuators.disableDynamixels()
            self.actuators.setFansEnabled(False)
            # Shutdown ROS
            for p in psutil.process_iter(['pid', 'name', 'cmdline']):
                if 'ros2' in p.name() and 'launch' in p.cmdline():
                    self.get_logger().warn(f'Sent SIGINT to ros2 launch {p.pid}')
                    p.send_signal(SIGINT)
            # shutdown linux
            if shutdown:
                call(['shutdown', '-h', 'now'])

    def destroy_node(self):
        """Handle SIGINT/global destructor."""
        self.actuators.disableDynamixels()
        self.actuators.setFansEnabled(False)
        super().destroy_node()
