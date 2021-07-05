from .action import Action
import numpy as np
from PyKDL import Vector, Rotation, Frame


class Gobelet(Action):
    def __init__(self, position, color, **kwargs):
        super().__init__(position, **kwargs)
        self.color = color
        self.pump_id = None

    def get_initial_orientation(self, robot):
        theta = np.arctan2(
            (self.position[1] - robot.position[1]),
            (self.position[0] - robot.position[0])
        )
        return theta

    def get_initial_position(self, robot):
        if robot.simulation:
            robot_to_gob = (0.04, 0.08)
        else:
            robot_to_gob = robot.actuators.PUMPS.get(self.pump_id).get("pos")
        vector_robot_to_gob = Vector(robot_to_gob[0], robot_to_gob[1], 0)
        # Find the angle between the robot's and the gobelet's position (theta)
        theta = self.get_initial_orientation(robot)
        # Frame between gobelet and center of robot
        frame_robot_to_gob = Frame(Rotation.RotZ(theta), vector_robot_to_gob)
        # Frame between gobelet and map
        frame_map_to_gob = Frame(
            Rotation.Identity(), Vector(self.position[0], self.position[1], 0)
        )
        # Frame between robot and map
        frame_map_to_robot = frame_map_to_gob * frame_robot_to_gob.Inverse()
        return (frame_map_to_robot.p.x(), frame_map_to_robot.p.y())

    def preempt_action(self, robot, action_list):
        if robot.simulation:
            return
        # Find the first available pump
        for pump_id, pump_dict in robot.actuators.PUMPS.items():
            if pump_dict.get("status") is None:
                self.pump_id = pump_id
                # Find the id of this Gobelet
                for action_id, action_dict in action_list.items():
                    if action_dict == self:
                        pump_dict["status"] = action_id
                        robot.get_logger().info(
                            f"Pump {pump_id} preempted {action_id}."
                        )
                        robot.actuators.setPumpsEnabled(True, [self.pump_id])
                        return

    def release_action(self, robot):
        if robot.simulation:
            return
        robot.actuators.PUMPS.get(self.pump_id).pop("status")
        self.pump_id = None

    def finish_action(self, robot):
        if robot.simulation:
            return
        robot.actuators.PUMPS.get(self.pump_id).pop("status")
        self.pump_id = None

    def start_actuator(self, robot):
        robot.actuators.grabCups([self.pump_id])
