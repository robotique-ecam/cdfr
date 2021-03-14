class Action:
    def __init__(self, position, **kwargs):
        self.position = position
        self.rotation = kwargs.get('rotation', 0)
        self.tags = kwargs.get('tags', {})

    def get_initial_position(self, robot):
        return self.position

    def get_initial_orientation(self, robot):
        return self.rotation

    def start_actuator(self, robot):
        pass
