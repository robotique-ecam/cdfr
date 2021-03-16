from .action import Action


class MancheAir(Action):
    def __init__(self, position, **kwargs):
        super().__init__(position, **kwargs)
        self.step = 0

    def get_initial_position(self, robot):
        if self.step == 0:
            offset = -0.1 if self.tags["ONLY_SIDE"] == "blue" else 0.1
        else:
            offset = 0.1 if self.tags["ONLY_SIDE"] == "blue" else -0.1
        position = (self.position[0] + offset, robot.width_param / 2 + 0.05)
        return position

    def start_actuator(self, robot):
        if self.step == 0:
            # Open robot arm - robot.openArm()
            # Force new objective to be next position
            pass
        else:
            # Close robot arm - robot.closeArm()
            pass
        self.step += 1
