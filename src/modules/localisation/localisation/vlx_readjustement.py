#!/usr/bin/env python3


"""Vlx readjustement for localisation"""

import numpy as np
import copy
import time
import threading

from localisation.sensors_sim import Sensors
from localisation.utils import *
from geometry_msgs.msg import Pose
from builtin_interfaces.msg._time import Time

from datetime import datetime

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
        self.parent.declare_parameter("vlx_lat_x", 0.0)
        self.parent.declare_parameter("vlx_lat_y", 0.0)
        self.parent.declare_parameter("vlx_face_x", 0.0)
        self.parent.declare_parameter("vlx_face_y", 0.0)
        self.vlx_lat_x = self.parent.get_parameter("vlx_lat_x")._value
        self.vlx_lat_y = self.parent.get_parameter("vlx_lat_y")._value
        self.vlx_face_x = self.parent.get_parameter("vlx_face_x")._value
        self.vlx_face_y = self.parent.get_parameter("vlx_face_y")._value
        self.values_stamped_array = [
            VlxStamped(
                self.sensors.get_distances(),
                self.get_clock().now()
                if not is_simulation()
                else self.sensors.get_time_stamp(),
            )
        ]
        self.thread_continuous_sampling = None
        self.continuous_sampling = 0
        self.near_walls_last_check_stamp = None

    def near_wall_routine(self, pose_stamped):
        actual_stamp = (
            self.get_clock().now()
            if not is_simulation()
            else self.sensors.get_time_stamp()
        )
        if (
            abs(
                float(
                    pose_stamped.header.stamp.sec
                    + pose_stamped.header.stamp.nanosec * 1e-9
                )
                - float(self.near_walls_last_check_stamp.sec + self.near_walls_last_check_stamp.nanosec * 1e-9)
            )
            < 0.5
        ):
            self.parent.get_logger().warn("near_wall_routine")
            self.try_to_readjust_with_vlx(
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.orientation,
                pose_stamped.header.stamp,
            )
            self.near_walls_last_check_stamp = (
                self.get_clock().now()
                if not is_simulation()
                else self.sensors.get_time_stamp()
            )

    def start_near_wall_routine(self, pose_stamped):
        if self.near_walls_last_check_stamp == None:
            self.near_walls_last_check_stamp = Time(sec=0, nanosec=0)
            if self.thread_continuous_sampling != None:
                self.stop_continuous_sampling_thread()
            self.start_continuous_sampling_thread(0.0, 2)
            self.near_wall_routine(pose_stamped)
        else:
            self.parent.get_logger().info("near wall routine already running")

    def stop_near_wall_routine(self):
        if self.near_walls_last_check_stamp != None:
            self.near_walls_last_check_stamp = None
            self.stop_continuous_sampling_thread()
        else:
            self.parent.get_logger().info("near wall routine already stopped")

    def start_continuous_sampling_thread(self, sleep_time, continuous_samp):
        if self.thread_continuous_sampling == None:
            self.thread_continuous_sampling = threading.Thread(
                target=self.continuous_vlx_sampling,
                args=(
                    sleep_time,
                    continuous_samp,
                ),
            )
            self.thread_continuous_sampling.start()
        else:
            self.parent.get_logger().info("vlx_thread thread is already running !")

    def stop_continuous_sampling_thread(self):
        if self.thread_continuous_sampling != None:
            self.continuous_sampling = 0
            self.thread_continuous_sampling.join()
            self.thread_continuous_sampling = None
        else:
            self.parent.get_logger().info("vlx_thread thread isn't running !")

    def continuous_vlx_sampling(self, sleep_time_before_sampling, continuous_samp):
        self.continuous_sampling = continuous_samp
        self.parent.get_logger().warn(
            f"self.continuous_sampling:{self.continuous_sampling}"
        )
        time.sleep(sleep_time_before_sampling)
        while self.continuous_sampling != 0:
            actual_stamp = (
                self.get_clock().now()
                if not is_simulation()
                else self.sensors.get_time_stamp()
            )
            vlx_values = self.sensors.get_distances()
            self.values_stamped_array.insert(0, VlxStamped(vlx_values, actual_stamp))
            if len(self.values_stamped_array) > 10:
                self.values_stamped_array.pop()
            time.sleep(0.02)

    def try_to_readjust_with_vlx(self, x, y, q, stamp):
        send_tf = True
        for i in range(len(self.values_stamped_array)):
            if (
                abs(
                    float(
                        self.values_stamped_array[i].stamp.sec
                        + self.values_stamped_array[i].stamp.nanosec * 1e-9
                    )
                    - float(stamp.sec + stamp.nanosec * 1e-9)
                )
                < 0.06
            ):
                new_stamp = self.values_stamped_array[i].stamp
                self.stop_continuous_sampling_thread()

                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.orientation = q
                news = self.compute_data(pose, self.values_stamped_array[i].values)
                if news != None:
                    send_tf = False
                    self.parent.get_logger().warn("publish affined position")
                    self.parent.create_and_send_tf(news[0], news[1], news[2], new_stamp)
                break
        if send_tf:
            self.parent.create_and_send_tf(x, y, q, stamp)
        if self.continuous_sampling == 1:
            self.stop_continuous_sampling_thread()
        if self.near_walls_last_check_stamp != None:
            self.start_continuous_sampling_thread(0.0, 2)
        later = datetime.now()
        delay = (later - now).total_seconds()
        self.parent.get_logger().info(f"delais: {delay}")

    def compute_data(self, pose_considered, vlx_values):

        data = self.fetch_data(pose_considered, vlx_values)

        if data == None:
            self.parent.get_logger().info("data_None")
            return None
        else:
            self.parent.get_logger().info("we can (hopefully) compute !")
            x, y, theta = self.get_pose_from_vlx(
                data["d"][0], data["d"][1], data["d"][2], data["vlx_0x30"]
            )
            new_x, new_y, new_theta = (
                data["pose_est"][0](x, y),
                data["pose_est"][1](x, y),
                data["pose_est"][2](theta),
            )
            d1_est, d2_est, d3_est = self.get_vlx_from_pose(
                [data["wall_relative"][0], data["wall_relative"][1]],
                data["wall_relative"][2],
                data["vlx_0x30"],
            )
            d1_proj_est, d2_proj_est, d3_proj_est = self.est_proj_wall(
                d1_est,
                d2_est,
                d3_est,
                data["wall_relative"][2],
                [data["wall_relative"][0], data["wall_relative"][1]],
                data["case"],
                data["inv_angle"],
                data["inv_lat"],
            )

            d = data["d"]
            self.parent.get_logger().info(
                f"error computed vlx - true vlx d1:{d1_est - d[0]}, d2:{d2_est- d[1]}, d3:{d3_est -d[2]}"
            )
            self.parent.get_logger().info(
                f"error computed pose - true pose x:{round(new_x-pose_considered.position.x, 4)}, \
                y:{round(new_y-pose_considered.position.y, 4)}, \
                theta:{round(new_theta-quaternion_to_euler(pose_considered.orientation)[2], 4)}"
            )
            self.parent.get_logger().info(f"test d1:{d1_proj_est}")
            self.parent.get_logger().info(f"test d2:{d2_proj_est}")
            self.parent.get_logger().info(f"test d3:{d3_proj_est}")

            if (
                d1_est > 0
                and d2_est > 0
                and d3_est > 0
                and abs(new_x - pose_considered.position.x) < 0.05
                and abs(new_y - pose_considered.position.y) < 0.05
                and abs(new_theta - quaternion_to_euler(pose_considered.orientation)[2])
                < 0.1
            ):
                self.parent.get_logger().warn(f"able to correct")
                return new_x, new_y, euler_to_quaternion(new_theta)
            else:
                return None

    def fetch_data(self, pose_considered, values):

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

            inv_angle_condition = True
            inv_lat_condition = True if case in [1, 2] else False

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

            inv_angle_condition = False
            inv_lat_condition = False

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

            inv_angle_condition = False
            inv_lat_condition = True

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

            inv_angle_condition = True
            inv_lat_condition = True if case in [3, 4] else False

        else:
            self.parent.get_logger().info("None")
            return None

        return {
            "d": [d1, d2, d3],
            "wall_relative": [
                robot_pose_wall_relative[0],
                robot_pose_wall_relative[1],
                rectif_angle,
            ],
            "pose_est": [x_est, y_est, theta_est],
            "case": case,
            "vlx_0x30": True if d3 == values[30] else False,
            "inv_angle": inv_angle_condition,
            "inv_lat": inv_lat_condition,
        }

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


class VlxStamped:
    def __init__(self, values, stamp):
        self.values = copy.deepcopy(values)
        self.stamp = stamp
