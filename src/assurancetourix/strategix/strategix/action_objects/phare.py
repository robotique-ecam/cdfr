from .action import Action
from .utils import navigate_to_point
import time
import numpy as np


class Phare(Action):
    def __init__(self, position, **kwargs):
        super().__init__(position, **kwargs)
        # (x diff between init and final pos, y diff from wall)
        self.rotation = np.pi
        self.displacement = (0.4, 0.4)

    def get_initial_position(self, robot):
        # return (self.position[0], self.position[1] - robot.length.value / 2 - 0.1)
        return (self.position[0] + self.displacement[0], self.position[1] - self.displacement[1])

    def start_actuator(self, robot):
        navigate_to_point((self.position[0], self.position[1] - 0.28), self.rotation, 2.0)
        robot.get_logger().info("Servo on")
        time.sleep(2)
        robot.get_logger().info("Servo off")
        navigate_to_point((self.position[0] + self.displacement[0], self.position[1] - self.displacement[1]), self.rotation, 2.0)
        return True
