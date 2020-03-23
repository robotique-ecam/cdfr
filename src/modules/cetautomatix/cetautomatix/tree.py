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
from cetautomatix.custom_behaviours import Time, NewAction, ConfirmAction, ReleaseAction


rclpy.init(args=None)
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
robot = Robot()


def create_tree() -> py_trees.behaviour.Behaviour:

    # Execute
    navigate = py_trees_ros.action_clients.FromBlackboard(
        action_type=NavigateToPose,
        action_name="NavigateToPose",
        name="NavigateToPose",
        key='goal',
        generate_feedback_message=robot.get_goal_pose()
    )

    actuator = py_trees.behaviours.Success(name="Actuators")

    execute = py_trees.composites.Sequence(
        name='ExecuteAction',
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

    release_action = ReleaseAction(
        name="ReleaseAction",
        robot=robot
    )

    failure_handler = py_trees.composites.Selector(
        name="FailureHandler",
        children=[execute, release_action]
    )

    actions = py_trees.composites.Sequence(
        name='Actions',
        children=[new_action, failure_handler, confirm_action]
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
