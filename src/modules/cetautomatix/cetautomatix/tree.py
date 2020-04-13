#!/usr/bin/env python3

import py_trees
import py_trees_ros
import rclpy
from cetautomatix.custom_behaviours import (ConfirmAction, EndOfGameAction,
                                            NewAction, PavillonAction,
                                            ReleaseAction, SetupTimersAction)
from cetautomatix.robot import Robot
from nav2_msgs.action import NavigateToPose

try:
    import RPi.GPIO as GPIO
except ImportError:
    from cetautomatix.utils.simulation import GPIOSim
    print('Not running on RPI. Fallback to simulation !!!', flush=True)
    GPIO = GPIOSim()


rclpy.init(args=None)
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
robot = Robot()


def create_tree() -> py_trees.behaviour.Behaviour:
    # Execute
    navigate = py_trees_ros.action_clients.FromBlackboard(
        action_type=NavigateToPose,
        action_name='navigate_to_pose',
        name='NavigateToPose',
        key='goal',
        generate_feedback_message=robot.get_goal_pose()
    )

    actuator = py_trees.behaviours.Success(name='Actuators')

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
        name='ReleaseAction',
        robot=robot
    )

    failure_handler = py_trees.composites.Selector(
        name='FailureHandler',
        children=[execute, release_action]
    )

    actions = py_trees.composites.Sequence(
        name='Actions',
        children=[new_action, failure_handler, confirm_action]
    )

    pavillon = PavillonAction(name='Pavillon Action')

    # Asterix Root
    all_actions = py_trees.composites.Parallel(
        name='All Actions',
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[pavillon, actions]
    )

    end_of_game = EndOfGameAction(name='End Of Game?')

    setup_timers = SetupTimersAction(
        name='Setup Timers',
        actions={pavillon: 95.0, end_of_game: 100.0}
    )
    asterix = py_trees.composites.Selector(
        name='Asterix',
        children=[setup_timers, end_of_game, all_actions]
    )

    # Goupille Guard
    def conditionGoupille():
        return False if GPIO.input(27) else True

    guardGoupille = py_trees.decorators.EternalGuard(
        name='Goupille?',
        condition=conditionGoupille,
        child=asterix
    )

    # Root of the Tree
    root = py_trees.composites.Sequence(
        name='Root',
        children=[guardGoupille]
    )

    return root
