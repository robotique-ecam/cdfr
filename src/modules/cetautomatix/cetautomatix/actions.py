#!/usr/bin/env python3

import py_trees
from magic_points import elements
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import py_trees_ros.mock.actions
import py_trees_ros_interfaces.action as py_trees_actions


class Robot(Node):
    def __init__(
        self,
        goal_pose: [elements["ARUCO42"], elements["GOBROUGE"]["GOB7"]],
        tolerance_position: [0.5],
        timeout: [60],
    ):
        super().__init__(node_name='drone_tester')

        self.goal_pose_array = goal_pose
        self.timeout_array = timeout
        self.tolerance_position_array = tolerance_position
        self.number_of_goals = len(self.goal_pose_array)
        self.index_of_goals = 0
        self.goal_pose = self.goal_pose_array[self.index_of_goals]

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def getGoalPose(self, index_of_goal):
        msg = PoseStamped()

        self.goal_pose = self.goal_pose_array[index_of_goal]

        msg.pose.position.x = float(self.goal_pose[0])
        msg.pose.position.y = float(self.goal_pose[1])
        msg.pose.position.z = 0

        q = self.euler_to_quaternion(float(self.goal_pose[2]), 0, 0)

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        return msg


rrr = Robot()

robot = py_trees.composites.Parallel(
        name="Asterix",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
"""end_of_match = py_trees.decorators.EternalGuard(
        name="End of match?",
        condition=timer,
        child=stop_robots
    )"""
move_1 = py_trees_ros.actions.ActionClient(
        name="Move 1",
        action_type=py_trees_actions.MoveBase,
        action_name="move_1",
        action_goal=rrr.getGoalPose(0)
    )
move_2 = py_trees_ros.actions.ActionClient(
        name="Move 2",
        action_type=py_trees_actions.MoveBase,
        action_name="move_2",
        action_goal=rrr.getGoalPose(1)
    )
robot.add_children([move_1, move_2])
