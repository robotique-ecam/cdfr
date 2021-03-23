#!/usr/bin/env python3


"""Vlx readjustement for localisation"""

import numpy as np
import copy

from localisation.sensors_sim import Sensors
from localisation.utils import in_rectangle, quaternion_to_euler, is_simulation

bottom_blue_area = [0.0, 0.0, 0.9, 1.0]
top_blue_area = [0.0, 1.0, 1.5, 2.0]

bottom_yellow_area = [2.1, 0.0, 3.0, 1.0]
top_yellow_area = [1.5, 1.0, 3.0, 2.0]


class VlxReadjustement:
    def __init__(self, parent_node):
        self.parent = parent_node
        self.sensors = Sensors(parent_node, [30, 31, 32, 33, 34, 35])
        if is_simulation():
            self.sim_offset = 0.0215
        else:
            self.sim_offset = 0.0
        # self.parent.create_timer(1, self.testVlx)
        self.parent.declare_parameter("vlx_lat_x", 0.0)
        self.parent.declare_parameter("vlx_lat_y", 0.0)
        self.parent.declare_parameter("vlx_face_x", 0.0)
        self.parent.declare_parameter("vlx_face_y", 0.0)
        self.vlx_lat_x = self.parent.get_parameter("vlx_lat_x")._value
        self.vlx_lat_y = self.parent.get_parameter("vlx_lat_y")._value
        self.vlx_face_x = self.parent.get_parameter("vlx_face_x")._value
        self.vlx_face_y = self.parent.get_parameter("vlx_face_y")._value
        self.vlx_readjustement = False

    def testVlx(self):

        values = self.sensors.get_distances()
        pose_considered = copy.deepcopy(self.parent.robot_pose.pose)
        angle = quaternion_to_euler(pose_considered.orientation)[2]

        if in_rectangle(bottom_blue_area, self.parent.robot_pose):
            self.parent.get_logger().info("bottom_blue_area")
            x_wall, y_wall = (
                pose_considered.position.x * 1000,
                (pose_considered.position.y - self.sim_offset) * 1000,
            )
            if angle > np.pi / 4 and angle < 3 * np.pi / 4:
                d1, d2, d3 = values[34], values[35], values[33]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = np.pi / 2 - angle
                x_est = lambda x, y: x / 1000
                y_est = lambda x, y: y / 1000 - self.sim_offset
                theta_est = lambda t: t + np.pi / 2
                case = 1
            elif angle < -np.pi / 4 and angle > -3 * np.pi / 4:
                d1, d2, d3 = values[31], values[32], values[30]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = -np.pi / 2 - angle
                x_est = lambda x, y: x / 1000
                y_est = lambda x, y: y / 1000 - self.sim_offset
                theta_est = lambda t: t - np.pi / 2
                case = 2
            elif angle > -np.pi / 4 and angle < np.pi / 4:
                d1, d2, d3 = values[34], values[35], values[30]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = 0 - angle
                x_est = lambda x, y: y / 1000
                y_est = lambda x, y: x / 1000 - self.sim_offset
                theta_est = lambda t: t
                case = 3
            else:
                d1, d2, d3 = values[31], values[32], values[33]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = np.pi - angle
                x_est = lambda x, y: y / 1000
                y_est = lambda x, y: x / 1000 - self.sim_offset
                theta_est = lambda t: (t - np.pi) if t > 0 else (t + np.pi)
                case = 4


            x, y, theta = self.get_pose_from_vlx(
                d1, d2, d3, True if d3 == values[30] else False
            )
            new_x, new_y, new_theta = x_est(x, y), y_est(x, y), theta_est(theta)
            d1_est, d2_est, d3_est = self.get_vlx_from_pose(
                robot_pose_wall_relative,
                rectif_angle,
                True if d3 == values[30] else False,
            )
            d1_proj_est, d2_proj_est, d3_proj_est = self.est_proj_wall(
                d1_est,
                d2_est,
                d3_est,
                rectif_angle,
                robot_pose_wall_relative,
                case,
                True,
                True if case in [1, 2] else False,
            )

            self.parent.get_logger().info(f"test d1:{d1_proj_est}")
            self.parent.get_logger().info(f"test d2:{d2_proj_est}")
            self.parent.get_logger().info(f"test d3:{d3_proj_est}")

        elif in_rectangle(top_blue_area, self.parent.robot_pose):
            self.parent.get_logger().info("top_blue_area")
            x_wall, y_wall = (
                pose_considered.position.x * 1000,
                2000 - pose_considered.position.y * 1000,
            )
            if angle > np.pi / 4 and angle < 3 * np.pi / 4:
                d1, d2, d3 = values[31], values[32], values[33]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = np.pi / 2 - angle
                x_est = lambda x, y: x / 1000
                y_est = lambda x, y: (2000 - y) / 1000
                theta_est = lambda t: t + np.pi / 2
                case = 1
            elif angle < -np.pi / 4 and angle > -3 * np.pi / 4:
                d1, d2, d3 = values[34], values[35], values[30]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = -np.pi / 2 - angle
                x_est = lambda x, y: x / 1000
                y_est = lambda x, y: (2000 - y) / 1000
                theta_est = lambda t: t - np.pi / 2
                case = 2
            elif angle > -np.pi / 4 and angle < np.pi / 4:
                d1, d2, d3 = values[34], values[35], values[33]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = 0 - angle
                x_est = lambda x, y: y / 1000
                y_est = lambda x, y: (2000 - x) / 1000
                theta_est = lambda t: t
                case = 3
            else:
                d1, d2, d3 = values[31], values[32], values[30]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = np.pi - angle
                x_est = lambda x, y: y / 1000
                y_est = lambda x, y: (2000 - x) / 1000
                theta_est = lambda t: (t - np.pi) if t > 0 else (t + np.pi)
                case = 4

            x, y, theta = self.get_pose_from_vlx(
                d1, d2, d3, True if d3 == values[30] else False
            )
            new_x, new_y, new_theta = x_est(x, y), y_est(x, y), theta_est(theta)
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
                f"error computed pose - true pose x:{round(new_x-pose_considered.position.x, 4)}, \
                y:{round(new_y-pose_considered.position.y, 4)}, \
                theta:{round(new_theta-angle, 4)}"
            )
            d1_proj_est, d2_proj_est, d3_proj_est = self.est_proj_wall(
                d1_est,
                d2_est,
                d3_est,
                rectif_angle,
                robot_pose_wall_relative,
                case,
            )
            self.parent.get_logger().info(f"test d1:{d1_proj_est}")
            self.parent.get_logger().info(f"test d2:{d2_proj_est}")
            self.parent.get_logger().info(f"test d3:{d3_proj_est}")

        elif in_rectangle(bottom_yellow_area, self.parent.robot_pose):
            self.parent.get_logger().info("bottom_yellow_area")
            x_wall, y_wall = (
                3000 - (pose_considered.position.x - self.sim_offset) * 1000,
                (pose_considered.position.y - self.sim_offset) * 1000,
            )
            if angle > np.pi / 4 and angle < 3 * np.pi / 4:
                d1, d2, d3 = values[34], values[35], values[30]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = np.pi / 2 - angle
                x_est = lambda x, y: (3000 - x) / 1000 + self.sim_offset
                y_est = lambda x, y: y / 1000 - self.sim_offset
                theta_est = lambda t: t + np.pi / 2
                case = 1
            elif angle < -np.pi / 4 and angle > -3 * np.pi / 4:
                d1, d2, d3 = values[31], values[32], values[33]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = -np.pi / 2 - angle
                x_est = lambda x, y: (3000 - x) / 1000 + self.sim_offset
                y_est = lambda x, y: y / 1000 - self.sim_offset
                theta_est = lambda t: t - np.pi / 2
                case = 2
            elif angle > -np.pi / 4 and angle < np.pi / 4:
                d1, d2, d3 = values[31], values[32], values[30]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = 0 - angle
                x_est = lambda x, y: (3000 - y) / 1000 + self.sim_offset
                y_est = lambda x, y: x / 1000 - self.sim_offset
                theta_est = lambda t: t
                case = 3
            else:
                d1, d2, d3 = values[34], values[35], values[33]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = np.pi - angle
                x_est = lambda x, y: (3000 - y) / 1000 + self.sim_offset
                y_est = lambda x, y: x / 1000 - self.sim_offset
                theta_est = lambda t: (t - np.pi) if t > 0 else (t + np.pi)
                case = 4

            x, y, theta = self.get_pose_from_vlx(
                d1, d2, d3, True if d3 == values[30] else False
            )
            new_x, new_y, new_theta = x_est(x, y), y_est(x, y), theta_est(theta)
            """
            self.parent.get_logger().info(
                f"algo x:{x/1000}, y:{y/1000}, theta:{np.degrees(theta)}"
            )
            self.parent.get_logger().info(
                f"computed x:{x_est(x, y)}, y:{y_est(x, y)}, theta:{theta_est(theta)}"
            )
            """
            d1_est, d2_est, d3_est = self.get_vlx_from_pose(
                robot_pose_wall_relative,
                rectif_angle,
                True if d3 == values[30] else False,
            )
            self.parent.get_logger().info(
                f"error computed vlx - true vlx d1:{d1_est - d1}, d2:{d2_est- d2}, d3:{d3_est -d3}"
            )
            self.parent.get_logger().info(
                f"error computed pose - true pose x:{round(new_x-pose_considered.position.x, 4)}, \
                y:{round(new_y-pose_considered.position.y, 4)}, \
                theta:{round(new_theta-angle, 4)}"
            )
            d1_proj_est, d2_proj_est, d3_proj_est = self.est_proj_wall(
                d1_est,
                d2_est,
                d3_est,
                rectif_angle,
                robot_pose_wall_relative,
                case,
                False,
                True,
            )

            self.parent.get_logger().info(f"test d1:{d1_proj_est}")
            self.parent.get_logger().info(f"test d2:{d2_proj_est}")
            self.parent.get_logger().info(f"test d3:{d3_proj_est}")


        elif in_rectangle(top_yellow_area, self.parent.robot_pose):
            self.parent.get_logger().info("top_yellow_area")
            x_wall, y_wall = (
                3000 - (pose_considered.position.x - self.sim_offset) * 1000,
                2000 - pose_considered.position.y * 1000,
            )
            if angle > np.pi / 4 and angle < 3 * np.pi / 4:
                d1, d2, d3 = values[31], values[32], values[30]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = np.pi / 2 - angle
                x_est = lambda x, y: (3000 - x) / 1000 + self.sim_offset
                y_est = lambda x, y: (2000 - y) / 1000
                theta_est = lambda t: t + np.pi / 2
                case = 1
            elif angle < -np.pi / 4 and angle > -3 * np.pi / 4:
                d1, d2, d3 = values[34], values[35], values[33]
                robot_pose_wall_relative = [x_wall, y_wall]
                rectif_angle = -np.pi / 2 - angle
                x_est = lambda x, y: (3000 - x) / 1000 + self.sim_offset
                y_est = lambda x, y: (2000 - y) / 1000
                theta_est = lambda t: t - np.pi / 2
                case = 2
            elif angle > -np.pi / 4 and angle < np.pi / 4:
                d1, d2, d3 = values[31], values[32], values[33]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = 0 - angle
                x_est = lambda x, y: (3000 - y) / 1000 + self.sim_offset
                y_est = lambda x, y: (2000 - x) / 1000
                theta_est = lambda t: t
                case = 3
            else:
                d1, d2, d3 = values[34], values[35], values[30]
                robot_pose_wall_relative = [y_wall, x_wall]
                rectif_angle = np.pi - angle
                x_est = lambda x, y: (3000 - y) / 1000 + self.sim_offset
                y_est = lambda x, y: (2000 - x) / 1000
                theta_est = lambda t: (t - np.pi) if t > 0 else (t + np.pi)
                case = 4

            x, y, theta = self.get_pose_from_vlx(
                d1, d2, d3, True if d3 == values[30] else False
            )
            new_x, new_y, new_theta = x_est(x, y), y_est(x, y), theta_est(theta)
            """
            self.parent.get_logger().info(
                f"algo x:{x/1000}, y:{y/1000}, theta:{np.degrees(theta)}"
            )
            self.parent.get_logger().info(
                f"computed x:{x_est(x, y)}, y:{y_est(x, y)}, theta:{theta_est(theta)}"
            )
            """
            d1_est, d2_est, d3_est = self.get_vlx_from_pose(
                robot_pose_wall_relative,
                rectif_angle,
                True if d3 == values[30] else False,
            )
            self.parent.get_logger().info(
                f"error computed vlx - true vlx d1:{d1_est - d1}, d2:{d2_est- d2}, d3:{d3_est -d3}"
            )
            self.parent.get_logger().info(
                f"error computed pose - true pose x:{round(new_x-pose_considered.position.x, 4)}, \
                y:{round(new_y-pose_considered.position.y, 4)}, \
                theta:{round(new_theta-angle, 4)}"
            )
            d1_proj_est, d2_proj_est, d3_proj_est = self.est_proj_wall(
                d1_est,
                d2_est,
                d3_est,
                rectif_angle,
                robot_pose_wall_relative,
                case,
                True,
                True if case in [3, 4] else False,
            )

            self.parent.get_logger().info(f"test d1:{d1_proj_est}")
            self.parent.get_logger().info(f"test d2:{d2_proj_est}")
            self.parent.get_logger().info(f"test d3:{d3_proj_est}")

        else:
            self.parent.get_logger().info("None")

    def get_pose_from_vlx(self, d1, d2, d3, vlx_0x30):
        theta = np.arctan((d2 - d1) / (2 * self.vlx_face_y))
        if vlx_0x30:
            x = (d3 + self.vlx_lat_y) * np.cos(theta) - self.vlx_lat_x * np.sin(theta)
        else:
            x = (d3 + self.vlx_lat_y) * np.cos(theta) + self.vlx_lat_x * np.sin(theta)
        y = ((d1 + d2) / 2 + self.vlx_face_x) * np.cos(theta)
        return (x, y, theta)

    def get_vlx_from_pose(self, robot_pose_wall_relative, theta, vlx_0x30):
        d1 = (
            (robot_pose_wall_relative[1]) / np.cos(theta)
            + self.vlx_face_y * np.tan(theta)
            - self.vlx_face_x
        )
        d2 = (
            (robot_pose_wall_relative[1]) / np.cos(theta)
            - self.vlx_face_y * np.tan(theta)
            - self.vlx_face_x
        )
        if vlx_0x30:
            d3 = (
                robot_pose_wall_relative[0] / np.cos(theta)
                - self.vlx_lat_x * np.tan(theta)
                - self.vlx_lat_y
            )
        else:
            d3 = (
                robot_pose_wall_relative[0] / np.cos(theta)
                + self.vlx_lat_x * np.tan(theta)
                - self.vlx_lat_y
            )
        return (d1, d2, d3)

    def est_proj_wall(
        self,
        d1_est,
        d2_est,
        d3_est,
        rectif_angle,
        robot_pose_wall_relative,
        case,
        inv_angle=False,
        inv_lat=False,
    ):
        if case in [1, 2]:
            considered_angle = rectif_angle
            considered_vlx_face_y = self.vlx_face_y
        else:
            considered_angle = -rectif_angle
            considered_vlx_face_y = -self.vlx_face_y

        if case in [2, 3]:
            considered_vlx_lat_x = -self.vlx_lat_x
        else:
            considered_vlx_lat_x = self.vlx_lat_x
        if inv_angle:
            considered_angle = -considered_angle
        if inv_lat:
            considered_vlx_lat_x = -considered_vlx_lat_x

        d1_est_proj_wall = (
            robot_pose_wall_relative[0]
            + (d1_est + self.vlx_face_x) * np.sin(considered_angle)
            + considered_vlx_face_y * np.cos(considered_angle)
        )
        d2_est_proj_wall = (
            robot_pose_wall_relative[0]
            + (d2_est + self.vlx_face_x) * np.sin(considered_angle)
            - considered_vlx_face_y * np.cos(considered_angle)
        )
        d3_est_proj_wall = (
            robot_pose_wall_relative[1]
            - (d3_est + self.vlx_lat_y) * np.sin(considered_angle)
            - considered_vlx_lat_x * np.cos(considered_angle)
        )
        return (d1_est_proj_wall, d2_est_proj_wall, d3_est_proj_wall)
