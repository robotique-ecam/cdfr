from .action import Action
from .utils import navigate_to_point
import numpy as np


class Chenal(Action):
    def __init__(self, position, **kwargs):
        super().__init__(position, **kwargs)
        self.rotation = np.pi / 2
        self.iter = 0

    def get_initial_position(self, robot):
        self.offset_x = -0.3 if robot.get_side() == "yellow" else 0.3
        return (
            self.position[0] + self.offset_x,
            self.position[1] + 0.2 + self.iter * 0.1,
        )

    def start_actuator(self, robot):
        navigate_to_point(
            (self.position[0] + self.offset_x, self.position[1] + self.iter * 0.1),
            self.rotation,
            2.0,
        )
        for pump_id in robot.actuators.PUMPS.keys():
            robot.actuators.asterix_drop(pump_id)
        # robot.actuators.asterix_drop_all() Not working right now IO Error
        navigate_to_point(
            (
                self.position[0] + self.offset_x,
                self.position[1] + 0.2 + self.iter * 0.1,
            ),
            self.rotation,
            2.0,
        )
        self.iter += 1
        return True

    def finish_action(self, robot):
        for pump_id in robot.actuators.PUMPS.keys():
            robot.actuators.PUMPS.get(pump_id).pop("status")
