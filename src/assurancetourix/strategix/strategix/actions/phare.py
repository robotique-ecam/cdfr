from .action import Action
from std_srvs.srv import Trigger


class Phare(Action):
    def __init__(self, position, **kwargs):
        super().__init__(position, **kwargs)
        self.orientation = 90
        self.get_trigger_deploy_pharaon_request = Trigger.Request()
        self.get_trigger_deploy_pharaon_client = self.create_client(
            Trigger, "/pharaon/deploy"
        )

    def get_initial_position(self, robot):
        return (self.position[0], self.position[1] - robot.width / 2)

    def start_actuator(self, robot):
        response = robot.synchronous_call(
            self.get_trigger_deploy_pharaon_client,
            self.get_trigger_deploy_pharaon_request,
        )
        return response.success
