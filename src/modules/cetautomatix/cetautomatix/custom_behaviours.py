#!/usr/bin/env python3


import py_trees

try:
    import RPi.GPIO as GPIO
except ImportError:
    print('Not running on RPI. Fallback to ros service call !!!', flush=True)
    GPIO = None


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
    def __init__(self, name, robot, actions):
        super().__init__(name=name)
        self.robot = robot
        self.actions = actions
        self.visited = False

    def update(self):
        if not self.visited:
            self.visited = True
            # Setup Timers
            self.robot._start_robot_callback(None, None)
            for action, duration in self.actions.items():
                action.time = self.robot.get_clock().now().nanoseconds * 1e-9 + duration
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class PavillonAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot):
        super().__init__(name=name)
        self.robot = robot
        self.time = 95000
        self.visited = False

    def update(self):
        if self.robot.get_clock().now().nanoseconds * 1e-9 > self.time:
            if not self.visited:
                self.visited = True
                self.robot.trigger_pavillons()
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class EndOfGameAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot):
        super().__init__(name=name)
        self.robot = robot
        self.time = 1e5
        self.visited = False

    def update(self):
        if self.robot.get_clock().now().nanoseconds * 1e-9 > self.time:
            if not self.visited:
                self.visited = True
                # Code to end every action
                self.robot.stop_ros()
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class GoupilleWatchdog(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot):
        super().__init__(name=name)
        self.robot = robot

    def setup(self, node):
        """Setup GPIOs if existing."""
        if GPIO is None:
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def update(self):
        """Guard condition for goupille."""
        if GPIO is None or self.robot.robot == 'asterix':
            if self.robot.triggered():
                return py_trees.common.Status.SUCCESS
        elif not GPIO.input(17) or self.robot.triggered():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
