#!/usr/bin/env python3

import rclpy
import py_trees
import psutil
import math
import numpy as np
from signal import SIGINT
from subprocess import call
from importlib import import_module
from platform import machine
from rclpy.node import Node
from lcd_msgs.msg import Lcd
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal
from strategix_msgs.srv import GetAvailableActions, ChangeActionStatus
from strategix.actions import actions
from strategix.strategy_modes import get_time_coeff
from cetautomatix.selftest import Selftest


class Robot(Node):
    def __init__(self):
        super().__init__(node_name="cetautomatix")
        # Detect simulation mode
        self.simulation = True if machine() != "aarch64" else False
        self.name = self.get_namespace().strip("/")
        # Declare parameters
        self.triggered = False
        self.declare_parameter("length")
        self.declare_parameter("width")
        self.declare_parameter("strategy_mode")
        self.length = self.get_parameter("length")
        self.width = self.get_parameter("width")
        self.strategy_mode = self.get_parameter("strategy_mode")
        # Bind actuators
        self.actuators = import_module(f"actuators.{self.name}").actuators
        # Do selftest
        self.selftest = Selftest(self)
        # Prechill engines
        self.actuators.setFansEnabled(True)
        # Create empty action queue
        self.action_list = []
        self.current_action = None
        # Stop ROS service
        self.stop_ros_service = self.create_service(
            Trigger, "stop", self.stop_robot_callback
        )
        # Strategix client to get available actions
        self.get_available_actions_request = GetAvailableActions.Request()
        self.get_available_actions_request.sender = self.name
        self.get_available_actions_client = self.create_client(
            GetAvailableActions, "/strategix/available"
        )
        # Strategix client to change the status of an action
        self.change_action_status_request = ChangeActionStatus.Request()
        self.change_action_status_request.sender = self.name
        self.change_action_status_client = self.create_client(
            ChangeActionStatus, "/strategix/action"
        )
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            PoseStamped, "odom_map_relative", self.odom_callback, 1
        )
        # Py-Trees blackboard to send NavigateToPose actions
        self.blackboard = py_trees.blackboard.Client(name="NavigateToPose")
        self.blackboard.register_key(key="goal", access=py_trees.common.Access.WRITE)
        # LCD driver direct access
        self.lcd_driver_pub = self.create_publisher(Lcd, "/obelix/lcd", 1)
        # Wait for Strategix as this can block the whole Behaviour Tree
        while not self.get_available_actions_client.wait_for_service(timeout_sec=5):
            self.get_logger().warn(
                "Failed to contact strategix services ! Has it been started ?"
            )
        # Enable stepper drivers
        if not self.simulation:
            self.get_enable_drivers_request = SetBool.Request()
            self.get_enable_drivers_request.data = True
            self.get_enable_drivers_client = self.create_client(
                SetBool, "enable_drivers"
            )
            self.synchronous_call(
                self.get_enable_drivers_client, self.get_enable_drivers_request
            )
        # Pharaon trigger client
        self.trigger_deploy_pharaon_request = Trigger.Request()
        self.trigger_deploy_pharaon_client = self.create_client(
            Trigger, "/pharaon/deploy"
        )
        # Robot start trigger service
        self.trigger_start_robot_server = self.create_service(
            Trigger, "start", self.start_robot_callback
        )
        if self.name == "obelix":
            self.trigger_start_asterix_request = Trigger.Request()
            self.trigger_start_asterix_client = self.create_client(
                Trigger, "/asterix/start"
            )
        # Reached initialized state
        self.get_logger().info("Cetautomatix ROS node has been started")

    """~~~~~~~~~~~~~~ BASE FUNCTIONS ~~~~~~~~~~~~~~"""

    def synchronous_call(self, client, request):
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
        self.actuators.setSliderPosition(position)

    def fetch_available_actions(self):
        """Fetch available actions from Strategix."""
        response = self.synchronous_call(
            self.get_available_actions_client, self.get_available_actions_request
        )
        return response.available if response is not None else []

    def preempt_action(self, action):
        """Preempt an action for the BT."""
        if action is None:
            return False
        self.change_action_status_request.action = str(action)
        self.change_action_status_request.request = "PREEMPT"
        response = self.synchronous_call(
            self.change_action_status_client, self.change_action_status_request
        )
        if response is None:
            return False
        self.current_action = action if response.success else None
        if self.current_action is not None:
            self.set_goal_pose()
        actions.get(self.current_action).preempt_action(self, actions)
        return response.success

    def drop_current_action(self):
        """Drop an action for the BT."""
        if self.current_action is None:
            return False
        self.change_action_status_request.action = self.current_action
        self.change_action_status_request.request = "DROP"
        response = self.synchronous_call(
            self.change_action_status_client, self.change_action_status_request
        )
        if response is None:
            return False
        actions.get(self.current_action).release_action(self)
        self.current_action = None
        return response.success

    def confirm_current_action(self):
        """Confirm an action for the BT."""
        if self.current_action is None:
            return False
        self.change_action_status_request.action = self.current_action
        self.change_action_status_request.request = "CONFIRM"
        response = self.synchronous_call(
            self.change_action_status_client, self.change_action_status_request
        )
        if response is None:
            return False
        actions.get(self.current_action).finish_action(self)
        self.current_action = None
        return response.success

    def start_actuator_action(self):
        """Start the actuator action after the robot has reached its destination."""
        self.get_logger().info(f"START ACTUATOR {self.current_action}")
        return actions.get(self.current_action).start_actuator(self)

    def start_robot_callback(self, req, resp):
        """Start robot."""
        self.triggered = True
        self.get_logger().info("Triggered robot starter")
        if self.name == "obelix":
            # TODO : Fix sync call
            # Thread(target=self.synchronous_call, args=[self.trigger_start_asterix_client, self.trigger_start_asterix_request]).start()
            self.get_logger().info("Obelix triggered Asterix")
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        lcd_msg = Lcd()
        lcd_msg.line = 0
        lcd_msg.text = f"{self.name.capitalize()} running".ljust(16)
        self.lcd_driver_pub.publish(lcd_msg)
        lcd_msg.line = 1
        lcd_msg.text = "Score: 0".ljust(16)
        self.lcd_driver_pub.publish(lcd_msg)
        if hasattr(resp, "success"):
            resp.success = True
        return resp

    def odom_callback(self, msg):
        """Odom callback"""
        q = msg.pose.orientation
        self.position = (msg.pose.position.x, msg.pose.position.y)
        self.orientation = self.quaternion_to_euler(q.x, q.y, q.z, q.w)[2]

    def stop_robot_callback(self, req, resp):
        """Stop robot / ROS."""
        self.stop_ros(shutdown=False)
        resp.sucess = True
        return resp

    def stop_ros(self, shutdown=True):
        """Stop ros launch processes."""
        # Publish "Robot is done"
        lcd_msg = Lcd()
        lcd_msg.line = 0
        lcd_msg.text = f"{self.name.capitalize()} is done!".ljust(16)
        self.lcd_driver_pub.publish(lcd_msg)
        if not self.simulation:
            # Disable stepper drivers
            self.get_enable_drivers_request.data = False
            self.synchronous_call(
                self.get_enable_drivers_client, self.get_enable_drivers_request
            )
            # Stop fans and relax servos
            self.actuators.disableDynamixels()
            self.actuators.setFansEnabled(False)
            # Shutdown ROS
            for p in psutil.process_iter(["pid", "name", "cmdline"]):
                if "ros2" in p.name() and "launch" in p.cmdline():
                    self.get_logger().warn(f"Sent SIGINT to ros2 launch {p.pid}")
                    p.send_signal(SIGINT)
            # Shutdown Linux
            if shutdown:
                call(["shutdown", "-h", "now"])

    def euler_to_quaternion(self, yaw, pitch=0, roll=0):
        """Conversion between euler angles and quaternions."""
        yaw, pitch, roll = np.deg2rad(yaw), np.deg2rad(pitch), np.deg2rad(roll)
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
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

    def destroy_node(self):
        """Handle SIGINT/global destructor."""
        self.actuators.disableDynamixels()
        self.actuators.setFansEnabled(False)
        super().destroy_node()

    def set_goal_pose(self):
        """Set goal pose of action in blackboard."""
        msg = NavigateToPose_Goal()
        msg.pose.header.frame_id = "map"
        orientation = actions.get(self.current_action).get_initial_orientation(self)
        position = actions.get(self.current_action).get_initial_position(self)
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = 0.0
        q = self.euler_to_quaternion(orientation)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.blackboard.goal = msg

    """~~~~~~~~~~~~~~ CUSTOM FUNCTIONS ~~~~~~~~~~~~~~"""

    def trigger_pavillons(self):
        """Function to trigger the Pavillon after 95 seconds."""
        self.get_logger().info("Triggered pavillons")
        self.actuators.raiseTheFlag()
        self.change_action_status_request.action = "PAVILLON"
        self.change_action_status_request.request = "CONFIRM"
        response = self.synchronous_call(
            self.change_action_status_client, self.change_action_status_request
        )
        if response is None:
            return False
        return response.success

    def compute_best_action(self, action_list):
        """Calculate best action to choose from its distance to the robot and the time passed."""
        if not action_list:
            return None
        coefficient_list = []
        for action_id in action_list:
            action = actions.get(action_id)
            coefficient = 0
            distance = np.sqrt(
                (action.position[0] - self.position[0]) ** 2
                + (action.position[1] - self.position[1]) ** 2
            )
            coefficient += 100 * (1 - distance / 3.6)
            coefficient += get_time_coeff(
                self.get_clock().now().nanoseconds * 1e-9 - self.start_time,
                action,
                self.strategy_mode.value,
            )
            coefficient_list.append(coefficient)
        # Return best action based on the one with a maximum coefficient
        best_action = action_list[coefficient_list.index(max(coefficient_list))]
        return best_action
