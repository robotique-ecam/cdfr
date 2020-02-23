#!/usr/bin/env python3


"""High level interface for ROS 2 Actions Server to be used in Behavior Trees."""


import sys
import rclpy
import geometry_msgs.msg as geometry_msgs
import py_trees_ros.mock.actions
import py_trees_ros_interfaces.action as py_trees_actions


class MoveBase(py_trees_ros.mock.actions.GenericServer):

    """Move Base."""

    def __init__(self, duration=None):
        super().__init__(
            node_name="move_base_controller",
            action_name="move_base",
            action_type=py_trees_actions.MoveBase,
            generate_feedback_message=self.generate_feedback_message,
            duration=duration
        )
        self.pose = geometry_msgs.PoseStamped()
        self.pose.pose.position = geometry_msgs.Point(x=0.0, y=0.0, z=0.0)
