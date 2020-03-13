#!/usr/bin/env python3


class Action:
    def __init__(self, name):
        self.name = name
        self.executer = None

    def __repr__(self):
        return self.name


class BonPort(Action):
    def __init__(self, name):
        super().__init__(name)
        self.pos = 'Out'


class Gobelet(Action):
    def __init__(self, name, color):
        super().__init__(name)
        self.color = color
        self.inChenal = False


class MancheAir(Action):
    def __init__(self, name):
        super().__init__(name)


class Pavillon(Action):
    def __init__(self, name):
        super().__init__(name)
        self.raised = False


class Phare(Action):
    def __init__(self, name):
        super().__init__(name)
        self.state = 0
