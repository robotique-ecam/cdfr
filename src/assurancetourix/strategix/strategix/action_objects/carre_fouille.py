from .action import Action
from .utils import navigate_to_point
import numpy as np

POSITIONS = [0.6675, 0.8525, 1.0375, 1.2225, 1.4075, 1.5925, 1.7775, 1.9625, 2.1475, 2.3325]


class CarreFouille(Action):
    def __init__(self, **kwargs):
        super().__init__(position=(1.5, 0), **kwargs)
        self.offset = 0.3
        self.rotation = 0

    def get_initial_position(self, robot):
        if self.side == "blue":
            self.positions = POSITIONS
        else:
            self.positions = reversed(POSITIONS)
        return (self.positions[0], self.offset)

    def start_actuator(self, robot):
        color = robot.actuators.getResistanceColor()

        if color == self.side:
            # Push carré
            navigate_to_point((self.positions[1], self.offset), self.rotation, 2.0)
            # Push carré
        else:
            navigate_to_point((self.positions[1], self.offset), self.rotation, 2.0)
            # Push carré
            navigate_to_point((self.positions[2], self.offset), self.rotation, 2.0)
            # Push carré

        navigate_to_point((self.positions[3], self.offset), self.rotation, 2.0)
        color = robot.actuators.getResistanceColor()

        if color == self.side:
            # Push carré
            navigate_to_point((self.positions[6], self.offset), self.rotation, 2.0)
            # Push carré
        else:
            navigate_to_point((self.positions[4], self.offset), self.rotation, 2.0)
            # Push carré
            navigate_to_point((self.positions[5], self.offset), self.rotation, 2.0)
            # Push carré
