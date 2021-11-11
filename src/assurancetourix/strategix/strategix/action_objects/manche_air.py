from .action import Action
from .utils import navigate_to_point


class MancheAir(Action):
    def __init__(self, position, **kwargs):
        super().__init__(position, **kwargs)
        self.rotation = 0

    def get_initial_position(self, robot):
        self.offset_x = -0.5 if robot.get_side() == "yellow" else 0.5
        return (
            self.position[0] + self.offset_x,
            0.25,
        )

    def start_actuator(self, robot):
        navigate_to_point(
            self.position,
            self.rotation,
            2.0,
        )
        robot.actuators.setServoPosition("arm", 50)
        new_x = -0.72 if robot.get_side() == "yellow" else 0.72
        navigate_to_point(
            (self.position[0] + new_x, self.position[1]),
            self.rotation,
            2.0,
        )
        robot.actuators.setServoPosition("arm", 82)
        return True
