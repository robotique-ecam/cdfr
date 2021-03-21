#!/usr/bin/env python3


"""Vlx readjustement for localisation"""

import numpy as np

from localisation.sensors_sim import Sensors
from localisation.utils import in_rectangle, quaternion_to_euler

bottom_blue = [0.0, 0.0, 0.9, 1.0]
top_blue = [0.0, 1.0, 1.5, 2.0]

bottom_yellow = [2.1, 0.0, 3.0, 1.0]
top_yellow = [1.5, 1.0, 3.0, 2.0]

vlx_lat_x, vlx_lat_y = 80.0, 130.5
vlx_face_x, vlx_face_y = 100.5, 110.0

class VlxReadjustement:
    def __init__(self, parent_node):
        self.parent = parent_node
        self.sensors = Sensors(parent_node, [30, 31, 32, 33, 34, 35])
        self.parent.create_timer(1, self.testVlx)
        self.vlx_readjustement = False

    def testVlx(self):

        values = self.sensors.get_distances()
        """
        self.parent.get_logger().info(
            f"{values[30]}, {values[31]}, {values[32]}, {values[33]}, {values[34]}, {values[35]}, {self.parent.robot_pose.pose.position.x}"
        )
        """
        if in_rectangle(bottom_blue, self.parent.robot_pose):
            self.parent.get_logger().info("bottom_blue")
        elif in_rectangle(top_blue, self.parent.robot_pose):
            self.parent.get_logger().info("top_blue")
            pose_considered = self.parent.robot_pose.pose
            angle = quaternion_to_euler(pose_considered.orientation)[2]
            if angle > np.pi/4 and angle < 3*np.pi/4:
                d1, d2, d3 = values[31], values[32], values[33]
                d1_p, d2_p, d3_p = vlx_0x31_pos, vlx_0x32_pos, vlx_0x33_pos
            elif angle < -np.pi/4 and angle > -3*np.pi/4:
                d1, d2, d3 = values[34], values[35], values[30]
                d1_p, d2_p, d3_p = vlx_0x34_pos, vlx_0x35_pos, vlx_0x30_pos
            elif angle > -np.pi/4 and angle < np.pi/4:
                d1, d2, d3 = values[34], values[35], values[33]
                d1_p, d2_p, d3_p = vlx_0x34_pos, vlx_0x35_pos, vlx_0x33_pos
            else:
                d1, d2, d3 = values[31], values[32], values[30]
                d1_p, d2_p, d3_p = vlx_0x31_pos, vlx_0x32_pos, vlx_0x30_pos

            x, y, theta = self.get_pose_from_vlx(d1, d2, d3, d1_p, d2_p, d3_p)
            self.parent.get_logger().info(
                f"x:{x/1000}, y:{y/1000}, theta:{np.degrees(theta)}"
            )
        elif in_rectangle(bottom_yellow, self.parent.robot_pose):
            self.parent.get_logger().info("bottom_yellow")
        elif in_rectangle(top_yellow, self.parent.robot_pose):
            self.parent.get_logger().info("top_yellow")
        else:
            self.parent.get_logger().info("None")

    def get_pose_from_vlx(self, d1, d2, d3, d1_pos, d2_pos, d3_pos):
        theta = np.arctan((d2 - d1) / (2 * np.abs(d1_pos[1])))
        if d3_pos == vlx_0x30_pos:
            x = (d3 + np.abs(d3_pos[1])) * np.cos(theta) - d3_pos[0] * np.sin(theta)
        else:
            x = (d3 + np.abs(d3_pos[1])) * np.cos(theta) + d3_pos[0] * np.sin(theta)
        y = ((d1 + d2) / 2 + np.abs(d1_pos[0])) * np.cos(theta)
        return (x, y, theta)

    def get_vlx_from_pose(self, robot_pose, d1_pos, d2_pos, d3_pos):
        theta = np.pi - quaternion_to_euler(robot_pose.pose.orientation)[2]
        self.parent.get_logger().info(
            f"y:{2000 - robot_pose.pose.position.y*1000}, theta:{theta}, tan:{np.tan(theta)}"
        )
        d1 = ( 2000 - robot_pose.pose.position.y*1000 ) / np.cos(theta) - d1_pos[1] * np.tan(theta) - d1_pos[0]
        d2 = ( 2000 - robot_pose.pose.position.y*1000 ) / np.cos(theta) - d2_pos[1] * np.tan(theta) - d2_pos[0]
        d3 = robot_pose.pose.position.x*1000 / np.cos(theta) + d3_pos[0] * np.tan(theta) - d3_pos[1]
        return (d1, d2, d3)
