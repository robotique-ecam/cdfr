#!/usr/bin/env python3


"""Behavior tree definition."""


import py_trees
import py_trees_ros
from cetautomatix.custom_behaviours import (ConfirmAction, EndOfGameAction,
                                            NewAction, PavillonAction,
                                            ReleaseAction, SetupTimersAction,
                                            ActuatorAction, GoupilleWatchdog)
from nav2_msgs.action import NavigateToPose


def create_tree(robot) -> py_trees.behaviour.Behaviour:
    """Create py_tree nodes."""
    navigate = py_trees_ros.action_clients.FromBlackboard(
        action_type=NavigateToPose,
        action_name='navigate_to_pose',
        name='NavigateToPose',
        key='goal'
    )

    actuator = ActuatorAction(
        name='ActuatorsAction',
        robot=robot
    )

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

    actions_loop = py_trees.decorators.SuccessIsRunning(
        actions,
        name='ActionsLoop'
    )

    pavillon = PavillonAction(
        name='Pavillon Action',
        robot=robot
    )

    # Asterix Root
    all_actions = py_trees.composites.Parallel(
        name='All Actions',
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[pavillon, actions_loop]
    )

    end_of_game = EndOfGameAction(
        name='End Of Game?',
        robot=robot
    )

    setup_timers = SetupTimersAction(
        name='Setup Timers',
        robot=robot,
        actions={pavillon: 95.0, end_of_game: 100.0}
    )
    asterix = py_trees.composites.Selector(
        name='Asterix',
        children=[setup_timers, end_of_game, all_actions]
    )

    guardGoupille = GoupilleWatchdog(
        name='Goupille?',
        robot=robot
    )

    # Root of the Tree
    root = py_trees.composites.Sequence(
        name='Root',
        children=[guardGoupille, asterix]
    )

    return root
