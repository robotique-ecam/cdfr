#!/usr/bin/env python3


import time
import rclpy
import py_trees
import py_trees_ros
try:
    import RPi.GPIO as GPIO
except ImportError:
    from cetautomatix.utils.simulation import GPIOSim
    print("Not running on RPI. Fallback to simulation !!!", flush=True)
    GPIO = GPIOSim()
from cetautomatix.robot import Robot
from nav2_msgs.action import NavigateToPose
from strategix_msgs.action import StrategixAction
from cetautomatix.custom_behaviours import Time, NewAction, ConfirmAction


rclpy.init(args=None)
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
robot = Robot()


def create_tree() -> py_trees.behaviour.Behaviour:
    """Give objective & go to this objective
    move = py_trees_ros.actions.ActionClient(
        name="NavigateToPose",
        action_type=NavigateToPose,
        action_name="NavigateToPose",
        action_goal=robot.getGoalPose()
    )

    def conditionMove():
        return True if robot.objective is None else False

    guardMove = py_trees.decorators.EternalGuard(
        name="Need objective ?",
        condition=conditionMove,
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

    def strategix_result_callback(arg):
        todo = arg.result().result.todo
        setattr(askList, 'result_message', todo)
        print("Strategix said", todo, flush=True)

    setattr(askList, 'result_message', [])
    askList.get_result_callback = strategix_result_callback

    create_objective = NewObjective(
        name='Create new objective',
        robot=robot,
        list=askList.result_message
    )

    new_objective = py_trees.composites.Sequence(
        name='New Objective',
        children=[askList, create_objective]
    )"""
    # Execute
    navigate = py_trees_ros.actions.ActionClient(
        name="NavigateToPose",
        action_type=NavigateToPose,
        action_name="NavigateToPose",
        action_goal=robot.getGoalPose()
    )

    actuator = py_trees.behaviours.Success(name="Actuators")

    execute = py_trees.composites.Sequence(
        name='Execute',
        children=[navigate, actuator]
    )

    # Actions
    confirm_action = ConfirmAction(
        name='Confirm Action',
        robot=robot
    )

    new_action = NewAction(
        name='New Action',
        robot=robot
    )

    actions = py_trees.composites.Sequence(
        name='Actions',
        children=[new_action, execute, comfirm_action]
    )

    # Oneshot Pavillon
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

    # All Actions
    all_actions = py_trees.composites.Parallel(
        name="All Actions",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[pavillonFiR, actions]
    )

    # Guard End of Game
    def conditionEndOfGame():
        return True if time.time() > timeEndOfGame.time else False
    idle = py_trees.behaviours.Success("Idle")
    guardEndOfGame = py_trees.decorators.EternalGuard(
        name="End of game?",
        condition=conditionEndOfGame,
        child=idle
    )

    # Setup timers needed for the tree only when Goupille is activated
    timeEndOfGame = Time(name="End Of Game Timer", duration=100.0)

    timePavillon = Time(name="Pavillon Timer", duration=95.0)

    timeSetup = py_trees.composites.Parallel(
        name="Setup Timers",
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

    # Asterix Root
    asterix = py_trees.composites.Selector(
        name="Asterix",
        children=[goupilleSiF, guardEndOfGame, actions]
    )

    # Goupille Guard
    def conditionGoupille():
        return False if GPIO.input(27) else True

    guardGoupille = py_trees.decorators.EternalGuard(
        name="Goupille?",
        condition=conditionGoupille,
        child=asterix
    )

    # Root of the Tree
    root = py_trees.composites.Sequence(
        name='Root',
        children=[guardGoupille]
    )

    return root
