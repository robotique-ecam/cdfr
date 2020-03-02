#!/usr/bin/env python3

import sys
import rclpy
import time
import numpy as np
from magic_points import elements
from rclpy.node import Node

import py_trees
import py_trees_ros
import py_trees.console as console
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal
from nav2_msgs.action import NavigateToPose


class Robot(Node):
    def __init__(
        self,
        goal_pose=[elements["ARUCO42"], elements["GOBROUGE"]["GOB7"]],
        tolerance_position=[0.5],
        timeout=[60],
    ):
        super().__init__(node_name='robot')

        self.goal_pose_array = goal_pose
        self.timeout_array = timeout
        self.tolerance_position_array = tolerance_position
        self.number_of_goals = len(self.goal_pose_array)
        self.index_of_goals = 0
        self.goal_pose = self.goal_pose_array[self.index_of_goals]

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - \
            np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + \
            np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - \
            np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + \
            np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]

    def getGoalPose(self, index_of_goal):
        msg = NavigateToPose_Goal()
        self.goal_pose = self.goal_pose_array[index_of_goal]
        msg.pose.pose.position.z = 0.0
        if len(self.goal_pose) != 4:
            msg.pose.pose.position.x = float(self.goal_pose[0])
            msg.pose.pose.position.y = float(self.goal_pose[1])
        else:
            pass

        if len(self.goal_pose) == 3:
            q = self.euler_to_quaternion(float(self.goal_pose[2]), 0, 0)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
        else:
            pass

        return msg


rclpy.init(args=None)
robot = Robot()
timePavillon = time.time() + 5.0
timeEndOfGame = time.time() + 30.0


def create_root() -> py_trees.behaviour.Behaviour:
    def conditionEndOfGame():
        return True if time.time() > timeEndOfGame else False

    def conditionPavillon():
        return True if time.time() > timePavillon else False
    idle = py_trees.behaviours.Success("Idle")
    idle2 = py_trees.behaviours.Success("Idle2")
    move = py_trees_ros.actions.ActionClient(
        name="Move",
        action_type=NavigateToPose,
        action_name="NavigateToPose",
        action_goal=robot.getGoalPose(0)
    )
    oneShotPavillon = py_trees.decorators.OneShot(
        name="OneShot",
        child=idle2
    )
    guardPavillon = py_trees.decorators.EternalGuard(
            name="Hisser pavillons?",
            condition=conditionPavillon,
            child=oneShotPavillon
        )
    guardEndOfGame = py_trees.decorators.EternalGuard(
            name="End of game?",
            condition=conditionEndOfGame,
            child=idle
        )
    failureIsRunning = py_trees.decorators.FailureIsRunning(
        name="Failure Is Running",
        child=guardPavillon
    )
    actions = py_trees.composites.Parallel(
        name="Actions",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[failureIsRunning, move]
    )
    root = py_trees.composites.Selector(
        name="Asterix",
        children=[guardEndOfGame, actions]
    )

    return root


def main():
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    tree.node.destroy_node()
    rclpy.shutdown()


main()
