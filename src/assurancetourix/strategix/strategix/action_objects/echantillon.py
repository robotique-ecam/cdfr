from .action import Action
import numpy as np

class Echantillon(Action):
    def __init__(self, position=(0, 0), color: str = None, **kwargs):
        super().__init__(position, **kwargs)
        self.color = color

    def get_initial_position(self, robot=None):
        if robot.name == "asterix":  # Border Echantillon
            offset = 0.01
            rotation = np.deg2rad(self.rotation)
            
            x, y = offset * np.cos(rotation), offset * np.sin(rotation)
            x2, y2 = self.position[0] - x, self.position[1] - y
            
            return (x2, y2)
        
        else:  # Field Echantillon
            # Calculate position as with last year gobelet
            return (0, 0)

    def get_initial_orientation(self, robot=None):
        if robot.name == "asterix":  # Border Echantillon
            return self.rotation
        
        else:  # Field Echantillon
            # Calculate orientation as with last year gobelet
            return 0

    def preempt_action(self, robot=None, action_list=None):
        if robot.name == "asterix":  # Border Echantillon
            # Should only move a single servo/dynamixel and activate pump
            return True
        
        else:  # Field Echantillon
            robot.actuators.obelix_grab_hex(index=0)
