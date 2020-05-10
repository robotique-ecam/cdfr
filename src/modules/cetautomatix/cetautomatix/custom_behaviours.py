#!/usr/bin/env python3


import time

import py_trees


class NewAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot):
        super().__init__(name=name)
        self.robot = robot

    def update(self):
        if self.robot._current_action is None:
            available = self.robot.fetch_available_actions()
            best_action = self.robot.compute_best_action(available)
            return py_trees.common.Status.SUCCESS if self.robot.preempt_action(best_action) else py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS


class ConfirmAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot):
        super().__init__(name=name)
        self.robot = robot

    def update(self):
        return py_trees.common.Status.SUCCESS if self.robot.confirm_current_action() else py_trees.common.Status.FAILURE


class ReleaseAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot):
        super().__init__(name=name)
        self.robot = robot

    def update(self):
        return py_trees.common.Status.SUCCESS if self.robot.drop_current_action() else py_trees.common.Status.FAILURE


class ActuatorAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot):
        super().__init__(name=name)
        self.robot = robot

    def update(self):
        return py_trees.common.Status.SUCCESS if self.robot.start_actuator_action() else py_trees.common.Status.FAILURE


class SetupTimersAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, actions):
        super().__init__(name=name)
        self.actions = actions
        self.oneshot = 0

    def update(self):
        if self.oneshot < 1:
            self.oneshot += 1
            # Setup Timers
            for action, duration in self.actions.items():
                action.time = time.time() + duration
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class PavillonAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.time = 10000
        self.oneshot = 0

    def update(self):
        if time.time() > self.time:
            if self.oneshot < 1:
                self.oneshot += 1
                # Code to activate PavillonAction
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class EndOfGameAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name=name)
        self.time = 10000
        self.oneshot = 0

    def update(self):
        if time.time() > self.time:
            if self.oneshot < 1:
                self.oneshot += 1
                # Code to end every action
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
