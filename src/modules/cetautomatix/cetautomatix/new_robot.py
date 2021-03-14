#!/usr/bin/env python3


from importlib import import_module
from platform import machine
from rclpy.node import Node
from lcd_msgs.msg import Lcd
from strategix_msgs.srv import GetAvailableActions, ChangeActionStatus
from strategix.actions import actions
import Queue
import rclpy
import py_trees


class Robot(Node):
    def __init__(self):
        super().__init__(node_name="cetautomatix")
        # Detect simulation mode
        self.simulation = True if machine() != "aarch64" else False
        self.name = self.get_namespace().strip("/")
        # Bind actuators
        self.actuators = import_module(f"actuators.{self.name}").actuators
        # Create empty action queue
        self.action_queue = Queue.Queue()
        self.current_action = None
        # Strategix client to get available actions
        self.get_available_actions_request = GetAvailableActions.Request()
        self.get_available_actions_request.sender = self.name
        self.get_available_actions_client = self.create_client(GetAvailableActions, "/strategix/available")
        # Strategix client to change the status of an action
        self.change_action_status_request = ChangeActionStatus.Request()
        self.change_action_status_request.sender = self.name
        self.change_action_status_client = self.create_client(ChangeActionStatus, "/strategix/action")
        # Py-Trees blackboard to send NavigateToPose actions
        self.blackboard = py_trees.blackboard.Client(name="NavigateToPose")
        self.blackboard.register_key(key="goal", access=py_trees.common.Access.WRITE)
        # LCD driver direct access
        self.lcd_driver_pub = self.create_publisher(Lcd, "/obelix/lcd", 1)
        # Wait for Strategix as this can block the whole Behaviour Tree
        while not self.get_available_client.wait_for_service(timeout_sec=5):
            self.get_logger().warn("Failed to contact strategix services ! Has it been started ?")

    def synchronous_call(self, client, request):
        """Call service synchronously."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
        except BaseException:
            response = None
        return response

    def fetch_available_actions(self):
        """Fetch available actions from Strategix."""
        response = self.synchronous_call(self.get_available_actions_client, self.get_available_actions_request)
        return response.available if response is not None else []

    def preempt_action(self, action):
        """Preempt an action for the BT."""
        if action is None:
            return False
        self.change_action_status_request.action = str(action)
        self.change_action_status_request.request = "PREEMPT"
        response = self.synchronous_call(self.change_action_status_client, self.change_action_status_request)
        if response is None:
            return False
        self.current_action = action if response.success else None
        if self.current_action is not None:
            self.get_goal_pose()
        return response.success

    def drop_current_action(self):
        """Drop an action for the BT."""
        if self.current_action is None:
            return False
        self.change_action_status_request.action = self.current_action
        self.change_action_status_request.request = "DROP"
        response = self.synchronous_call(self.change_action_status_client, self.change_action_status_request)
        if response is None:
            return False
        self.current_action = None
        return response.success

    def confirm_current_action(self):
        """Confirm an action for the BT."""
        if self.current_action is None:
            return False
        self.change_action_status_request.action = self.current_action
        self.change_action_status_request.request = "CONFIRM"
        response = self.synchronous_call(self.change_action_status_client, self.change_action_status_request)
        if response is None:
            return False
        self.current_action = None
        return response.success

    def start_actuator_action(self):
        self.get_logger().info(f"START ACTUATOR {self.current_action}")
        return actions.get(self.current_action).start_actuator(self)

    def trigger_pavillons(self):
        self.get_logger().info("Triggered pavillons")
        self.actuators.raiseTheFlag()
        self.change_action_status_request.action = "PAVILLON"
        self.change_action_status_request.request = "CONFIRM"
        response = self.synchronous_call(self.change_action_status_client, self.change_action_status_request)
        if response is None:
            return False
        return response.success
