#!/usr/bin/env python3

import sys
import rclpy
import time
import numpy as np
import RPi.GPIO as GPIO
import py_trees
import py_trees_ros
import py_trees.console as console
from magic_points import elements
from rclpy.node import Node
from strategix_msgs.action import StrategixAction
from subscriber import OdomSubscriber
from client import StrategixActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal


class Robot(Node):
    def __init__(self, odom_subscriber):
        super().__init__(node_name='robot')
        self.objective = None
        self.odom_subscriber = odom_subscriber

    def best_action(self, list):
        action_coeff_list = []
        position = self.odom_subscriber.position
        # start = timeEndOfGame.time - 100.0
        for action in list:
            for key, value in elements.items():
                if key == action:
                    distance = np.sqrt(
                        (value[0] - position[0])**2 + (value[1] - position[1])**2)
                    coeff_distance = distance * 100 / 3.6
                    action_coeff_list.append((key, coeff_distance))
                    break
        max = 0
        best_action = None
        for action in action_coeff_list:
            if action[1] > max:
                max = action[1]
                best_action = action[0]
        self.objective = best_action

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

    def getGoalPose(self):
        msg = NavigateToPose_Goal()
        for key, value in elements.items():
            if key == self.objective:
                msg.pose.pose.position.z = 0.0
                msg.pose.pose.position.x = float(value[0])
                msg.pose.pose.position.y = float(value[1])
                q = self.euler_to_quaternion(float(value[2]), 0, 0)
                msg.pose.pose.orientation.x = q[0]
                msg.pose.pose.orientation.y = q[1]
                msg.pose.pose.orientation.z = q[2]
                msg.pose.pose.orientation.w = q[3]
                break
        return msg


class Time(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, duration: float):
        super().__init__(name=name)
        self.duration = duration

    def update(self):
        self.time = time.time() + self.duration
        return py_trees.common.Status.SUCCESS


class SendToStrategix(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, request: str, sender=' ', object=' '):
        super().__init__(name=name)
        self.sender = sender
        self.request = request
        self.object = object

    def update(self):
        strategix_action_client.send_goal(
            self.sender, self.request, self.object)
        return py_trees.common.Status.SUCCESS


class NewObjective(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, robot):
        super().__init__(name=name)
        self.robot = robot

    def update(self):
        self.robot.best_action(strategix_action_client.list)
        return py_trees.common.Status.SUCCESS


rclpy.init(args=None)
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
odom_subscriber = OdomSubscriber()
strategix_action_client = StrategixActionClient()
robot = Robot(odom_subscriber)


def create_tree() -> py_trees.behaviour.Behaviour:
    global timeEndOfGame, timePavillon
    """Give objective & go to this objective"""
    move = py_trees_ros.actions.ActionClient(
        name="Move",
        action_type=NavigateToPose,
        action_name="NavigateToPose",
        action_goal=robot.getGoalPose()
    )
    moveSiF = py_trees.decorators.SuccessIsFailure(
        name="Success Is Failure",
        child=move
    )
    askList = SendToStrategix(
        name='Ask for List',
        request='todo'
    )
    waitForData = py_trees_ros.subscribers.WaitForData(
        topic_name='strategix',
        topic_type=StrategixAction,
        name="Wait for data",
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    create_objective = NewObjective(name='Create new objective', robot=robot)
    new_objective = py_trees.composites.Sequence(
        name='New Objective',
        children=[askList, waitForData, create_objective, move])
    objective = py_trees.composites.Selector(
        name='Objective',
        children=[moveSiF, new_objective]
    )

    """Oneshot Pavillon"""
    def conditionPavillon():
        return True if time.time() > timePavillon.time else False
    idle2 = py_trees.behaviours.Success("Idle2")
    oneShotPavillon = py_trees.decorators.OneShot(
        name="OneShot Pavillon",
        child=idle2
    )
    guardPavillon = py_trees.decorators.EternalGuard(
        name="Hisser pavillons?",
        condition=conditionPavillon,
        child=oneShotPavillon
    )
    pavillonFiR = py_trees.decorators.FailureIsRunning(
        name="Failure Is Running",
        child=guardPavillon
    )

    """Actions"""
    actions = py_trees.composites.Parallel(
        name="Actions",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[pavillonFiR, objective]
    )

    """Guard End of Game"""
    def conditionEndOfGame():
        return True if time.time() > timeEndOfGame.time else False
    idle = py_trees.behaviours.Success("Idle")
    guardEndOfGame = py_trees.decorators.EternalGuard(
        name="End of game?",
        condition=conditionEndOfGame,
        child=idle
    )

    """Setup timers needed for the tree only when Goupille is activated"""
    timeEndOfGame = Time(name="End Of Game Time", duration=100.0)
    timePavillon = Time(name="Pavillon Time", duration=95.0)
    timeSetup = py_trees.composites.Parallel(
        name="Time",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[timeEndOfGame, timePavillon]
    )
    oneShotGoupille = py_trees.decorators.OneShot(
        name="OneShot Goupille",
        child=timeSetup
    )
    goupilleSiF = py_trees.decorators.SuccessIsFailure(
        name="Success Is Failure",
        child=oneShotGoupille
    )

    """Asterix Root"""
    asterix = py_trees.composites.Selector(
        name="Asterix",
        children=[goupilleSiF, guardEndOfGame, actions]
    )

    """Goupille Guard"""
    def conditionGoupille():
        return False if GPIO.input(27) else True
    guardGoupille = py_trees.decorators.EternalGuard(
        name="Goupille?",
        condition=conditionGoupille,
        child=asterix
    )

    """Root of the Tree"""
    root = py_trees.composites.Sequence(
        name='Root',
        children=[guardGoupille]
    )

    return root


def main():
    root = create_tree()
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
