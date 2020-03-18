#!/usr/bin/env python3


import time
import py_trees


class Time(py_trees.behaviour.Behaviour):
    def __init__(self, name, duration):
        super().__init__(name=name)
        self.duration = duration

    def update(self):
        self.time = time.time() + self.duration
        return py_trees.common.Status.SUCCESS


class NewObjective(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot, list):
        super().__init__(name=name)
        self.robot = robot
        self.list = list

    def update(self):
        self.robot.compute_best_action(self.list)
        print("Best action is", self.robot.objective, flush=True)
        return py_trees.common.Status.SUCCESS
