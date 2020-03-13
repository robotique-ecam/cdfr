#!/usr/bin/env python3


import time
import rclpy
import py_trees
import py_trees_ros
import RPi.GPIO as GPIO
from cetautomatix.robot import Robot
from nav2_msgs.action import NavigateToPose
from strategix_msgs.action import StrategixAction
from cetautomatix.custom_behaviours import Time, NewObjective


rclpy.init(args=None)
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
robot = Robot()


def create_tree() -> py_trees.behaviour.Behaviour:
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
    goal_msg = StrategixAction.Goal()
    goal_msg.sender = ' '
    goal_msg.request = 'todo'
    goal_msg.object = ' '
    askList = py_trees_ros.action_clients.FromConstant(
        action_type=StrategixAction,
        action_name='/strategix',
        action_goal=goal_msg,
        name='Ask for List',
    )
    print(dir(askList))
    create_objective = NewObjective(
        name='Create new objective',
        robot=robot,
        list=askList.result_message
    )
    new_objective = py_trees.composites.Sequence(
        name='New Objective',
        children=[askList, create_objective, move])
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
