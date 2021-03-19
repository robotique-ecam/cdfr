from .action import Action


class Gobelet(Action):
    def __init__(self, position, color, **kwargs):
        super().__init__(position, **kwargs)
        self.color = color
