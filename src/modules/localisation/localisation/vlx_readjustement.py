#!/usr/bin/env python3


"""Vlx readjustement for localisation"""

import numpy as np
import copy

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
            pose_considered = copy.deepcopy(self.parent.robot_pose.pose)
            x_wall, y_wall = (
                pose_considered.position.x * 1000,
                2000 - pose_considered.position.y * 1000,
            )
            angle = quaternion_to_euler(pose_considered.orientation)[2]
            if angle > np.pi / 4 and angle < 3 * np.pi / 4:
                d1, d2, d3 = values[31], values[32], values[33]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = np.pi / 2 - angle
                x_est = lambda x, y: x / 1000
                y_est = lambda x, y: (2000 - y) / 1000
                theta_est = lambda t: t + np.pi / 2
            elif angle < -np.pi / 4 and angle > -3 * np.pi / 4:
                d1, d2, d3 = values[34], values[35], values[30]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = -np.pi / 2 - angle
                x_est = lambda x, y: x / 1000
                y_est = lambda x, y: (2000 - y) / 1000
                theta_est = lambda t: t - np.pi / 2
            elif angle > -np.pi / 4 and angle < np.pi / 4:
                d1, d2, d3 = values[34], values[35], values[33]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = 0 - angle
                x_est = lambda x, y: y / 1000
                y_est = lambda x, y: (2000 - x) / 1000
                theta_est = lambda t: t
            else:
                d1, d2, d3 = values[31], values[32], values[30]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = np.pi - angle
                x_est = lambda x, y: y / 1000
                y_est = lambda x, y: (2000 - x) / 1000
                theta_est = lambda t: (t - np.pi) if t > 0 else (t + np.pi)

            x, y, theta = self.get_pose_from_vlx(
                d1, d2, d3, True if d3 == values[30] else False
            )
            self.parent.get_logger().info(
                f"algo x:{x/1000}, y:{y/1000}, theta:{np.degrees(theta)}"
            )
            self.parent.get_logger().info(
                f"computed x:{x_est(x, y)}, y:{y_est(x, y)}, theta:{theta_est(theta)}"
            )
            d1_est, d2_est, d3_est = self.get_vlx_from_pose(
                robot_pose_wall_relative,
                rectif_angle,
                True if d3 == values[30] else False,
            )
            self.parent.get_logger().info(
                f"error computed vlx - true vlx d1:{d1_est - d1}, d2:{d2_est- d2}, d3:{d3_est -d3}"
            )
            self.parent.get_logger().info(
                f"error computed pose - true pose x:{round(x_est(x, y)-pose_considered.position.x, 4)}, \
                y:{round(y_est(x, y)-pose_considered.position.y, 4)}, \
                theta:{round(theta_est(theta)-angle, 4)}"
            )

        elif in_rectangle(bottom_yellow, self.parent.robot_pose):
            self.parent.get_logger().info("bottom_yellow")
        elif in_rectangle(top_yellow, self.parent.robot_pose):
            self.parent.get_logger().info("top_yellow")
        else:
            self.parent.get_logger().info("None")

    def get_pose_from_vlx(self, d1, d2, d3, vlx_0x30):
        theta = np.arctan((d2 - d1) / (2 * vlx_face_y))
        if vlx_0x30:
            x = (d3 + vlx_lat_y) * np.cos(theta) - vlx_lat_x * np.sin(theta)
        else:
            x = (d3 + vlx_lat_y) * np.cos(theta) + vlx_lat_x * np.sin(theta)
        y = ((d1 + d2) / 2 + vlx_face_x) * np.cos(theta)
        return (x, y, theta)

    def get_vlx_from_pose(self, robot_pose_wall_relative, theta, vlx_0x30):
        d1 = (
            (robot_pose_wall_relative[1]) / np.cos(theta)
            + vlx_face_y * np.tan(theta)
            - vlx_face_x
        )
        d2 = (
            (robot_pose_wall_relative[1]) / np.cos(theta)
            - vlx_face_y * np.tan(theta)
            - vlx_face_x
        )
        if vlx_0x30:
            d3 = (
                robot_pose_wall_relative[0] / np.cos(theta)
                - vlx_lat_x * np.tan(theta)
                - vlx_lat_y
            )
        else:
            d3 = (
                robot_pose_wall_relative[0] / np.cos(theta)
                + vlx_lat_x * np.tan(theta)
                - vlx_lat_y
            )
        return (d1, d2, d3)
