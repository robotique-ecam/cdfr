from .action import Action


class Phare(Action):
    def __init__(self, position, **kwargs):
        super().__init__(position, **kwargs)
        self.rotation = 90

    def get_initial_position(self, robot):
        return (self.position[0], self.position[1] - robot.length / 2 - 0.1)

    def start_actuator(self, robot):
        response = robot.synchronous_call(
            robot.trigger_deploy_pharaon_client,
            robot.trigger_deploy_pharaon_request,
        )
        return response.success
