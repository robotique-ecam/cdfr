"""Base Action class for all types of action"""


class Action:
    def __init__(self, position=(0, 0), **kwargs):
        self.position = position
        self.rotation = kwargs.get("rotation", 0)
        self.tags = kwargs.get("tags", {})

    def get_initial_position(self, robot=None):
        """Function called when the robot needs to get the position of its objective"""
        return self.position

    def get_initial_orientation(self, robot=None):
        """Function called when the robot needs to get the orientation of its objective"""
        return self.rotation

    def preempt_action(self, robot=None, action_list=None):
        """Function called when the action is preempted"""
        pass

    def finish_action(self, robot=None):
        """Function called when the action is finished"""
        pass

    def release_action(self, robot=None):
        """Function called when the action is released"""
        pass

    def start_actuator(self, robot=None):
        """Function called when the robot reaches its objective and starts the actuator"""
        pass
