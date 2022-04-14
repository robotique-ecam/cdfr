from .action import Action
import numpy as np

class Distributeur(Action):
    def __init__(self, position=(0, 0), **kwargs):
        super().__init__(position, **kwargs)

    def get_initial_position(self, robot=None):
        offset = 0.2
        rotation = np.deg2rad(self.rotation)
        
        x, y = offset * np.cos(rotation), offset * np.sin(rotation)
        x2, y2 = x + self.position[0], y + self.position[1]
        
        return (x2, y2)

    def preempt_action(self, robot=None, action_list=None):
        return True
