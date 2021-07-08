from .action import Action
import numpy as np
from PyKDL import Vector, Rotation
from tf2_kdl import transform_to_kdl, do_transform_frame
from geometry_msgs.msg import TransformStamped, PoseStamped


class Gobelet(Action):
    def __init__(self, position, color, **kwargs):
        super().__init__(position, **kwargs)
        self.color = color
        self.pump_id = None

    def get_initial_orientation(self, robot):
        vec_pose_to_goal = Vector(
            self.position[0] - robot.get_position()[0],
            self.position[1] - robot.get_position()[1],
            0,
        )
        vec_pose_to_goal.Normalize()

        return np.pi + (
            np.arccos(vec_pose_to_goal[0])
            if vec_pose_to_goal[1] > 0
            else -np.arccos(vec_pose_to_goal[0])
        )

    def get_initial_position(self, robot):
        if robot.simulation:
            robot_to_gob = (0.04, 0.08)
        else:
            robot_to_gob = robot.actuators.PUMPS.get(self.pump_id).get("pos")

        gob_to_robot = TransformStamped()
        gob_to_robot.transform.translation.x = -robot_to_gob[0]
        gob_to_robot.transform.translation.y = -robot_to_gob[1]

        goal_pose = TransformStamped()
        goal_pose.transform.translation.x = self.position[0]
        goal_pose.transform.translation.y = self.position[1]

        robot_pose = PoseStamped()
        robot_pose.pose.position.x = robot.get_position()[0]
        robot_pose.pose.position.y = robot.get_position()[1]

        angle = self.get_initial_orientation(robot)

        rot = Rotation.RotZ(angle)

        q = rot.GetQuaternion()

        goal_pose.transform.rotation.x = q[0]
        goal_pose.transform.rotation.y = q[1]
        goal_pose.transform.rotation.z = q[2]
        goal_pose.transform.rotation.w = q[3]

        gob_to_robot_kdl = transform_to_kdl(gob_to_robot)

        robot_goal_pose_kdl = do_transform_frame(gob_to_robot_kdl, goal_pose)

        return (robot_goal_pose_kdl.p.x(), robot_goal_pose_kdl.p.y())

    def preempt_action(self, robot, action_list):
        if robot.simulation:
            return
        # Find the first available pump
        for pump_id, pump_dict in robot.actuators.PUMPS.items():
            if pump_dict.get("status") is None:
                self.pump_id = pump_id
                # Oneliner to find the id (key) of this Gobelet (value)
                action_id = list(action_list.keys())[
                    list(action_list.values()).index(self)
                ]
                robot.actuators.PUMPS.get(pump_id)["status"] = action_id
                robot.get_logger().info(f"Pump {pump_id} preempted {action_id}.")
                # robot.actuators.setPumpsEnabled(True, [self.pump_id])
                # robot.actuators.setPumpsEnabled(F, [self.pump_id])
                return

    def release_action(self, robot):
        if robot.simulation:
            return
        robot.actuators.PUMPS.get(self.pump_id).pop("status")
        self.pump_id = None

    def finish_action(self, robot):
        if robot.simulation:
            return

    def start_actuator(self, robot):
        robot.actuators.asterix_grab(self.pump_id)
        return True
