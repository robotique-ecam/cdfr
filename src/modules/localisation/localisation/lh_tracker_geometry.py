#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import random
import copy
import localisation.constants as csts

from tf2_kdl import PyKDL, do_transform_frame, transform_to_kdl, do_transform_vector
from rclpy.node import Node
from lh_tracker_msgs.msg import LHtracker
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from localisation.lh_geometry import LH2Geometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from datetime import datetime


class LH_tracker_geometry:
    def __init__(self, parent_node):
        """Init LH_tracker_geometry node"""
        self.parent_node = parent_node
        self.line_and_sensors_position_viz = False
        self.sensor_height = 0.415
        self.possible_pairs = [[0, 1], [2, 3], [4, 5], [6, 7]]

        self.geometry = LH2Geometry()
        self.init_est_marker()
        self.init_line_list()
        self.init_sensors_markers()
        self.calibration_done = False
        self.final_tf = TransformStamped()
        self.tracker_pose = Point()
        self.lh_est_pose_pb = self.parent_node.create_publisher(
            Marker, "lh_est_pose", 10
        )
        self.subscription = self.parent_node.create_subscription(
            LHtracker, "lh_msgs", self.lh_sub_callback, 10
        )
        if self.line_and_sensors_position_viz:
            self.lh_lines_pb = self.parent_node.create_publisher(Marker, "lh_lines", 10)
            self.sensors_est_position_pb = self.parent_node.create_publisher(
                Marker, "sensors_est_position", 10
            )

    def init_est_marker(self):
        self.marker_estimated_pose = Marker()
        self.marker_estimated_pose.id = 1
        self.marker_estimated_pose.header.frame_id = "map"
        self.marker_estimated_pose.type = 0  # arrow
        self.marker_estimated_pose.scale.x = 0.3
        self.marker_estimated_pose.scale.y = 0.1
        self.marker_estimated_pose.scale.z = 0.1
        self.marker_estimated_pose.color.a = 1.0
        self.marker_estimated_pose.lifetime.sec = 5
        self.marker_estimated_pose.color.r = 1.0

    def init_line_list(self):
        self.line_list = Marker()
        self.line_list.header.frame_id = "map"
        self.line_list.type = 5  # line_list
        self.line_list.color.r = 1.0
        self.line_list.color.g = 1.0
        self.line_list.color.b = 1.0
        self.line_list.color.a = 1.0
        self.line_list.scale.x = 0.005
        self.line_list.lifetime.sec = 10
        for _ in range(4):
            self.line_list.colors.append(ColorRGBA())
            self.line_list.colors[-1].a = 1.0
        self.line_list.colors[0].r = 1.0
        self.line_list.colors[1].g = 1.0
        self.line_list.colors[2].b = 1.0
        for _ in range(8):
            self.line_list.points.append(Point())

    def init_sensors_markers(self):
        self.sensors_est_position = Marker()
        self.sensors_est_position.header.frame_id = "map"
        self.sensors_est_position.type = 7  # line_list
        self.sensors_est_position.color.a = 1.0
        self.sensors_est_position.scale.x = 0.015
        self.sensors_est_position.scale.y = 0.015
        self.sensors_est_position.scale.z = 0.015
        self.sensors_est_position.lifetime.sec = 10
        for _ in range(4):
            self.sensors_est_position.colors.append(ColorRGBA())
            self.sensors_est_position.colors[-1].r = 1.0
            self.sensors_est_position.colors[-1].a = 1.0
            self.sensors_est_position.points.append(Point())
            self.sensors_est_position.points[-1].z = self.sensor_height

    def lh_sub_callback(self, msg):
        if not self.calibration_done:
            return
        else:
            self.nominal_localisation(msg)

    def nominal_localisation(self, msg):
        sensors_id_list = [msg.id_first_sensor, msg.id_second_sensor]

        if msg.sensors_nb_recovered == 4:
            sensors_id_list.append(msg.id_third_sensor)
            sensors_id_list.append(msg.id_fourth_sensor)

        azimuth1, elevation1 = self.geometry.get_azimuth_elevation_from_iteration(
            msg.first_sensor_first_iteration, msg.first_sensor_second_iteration
        )
        x1, y1, z1 = self.project_vector_to_map(
            self.get_x_y_z_from_azimuth_elevation(azimuth1, elevation1)
        )
        self.line_list.points[1].x = x1
        self.line_list.points[1].y = y1
        self.line_list.points[1].z = z1

        azimuth2, elevation2 = self.geometry.get_azimuth_elevation_from_iteration(
            msg.second_sensor_first_iteration, msg.second_sensor_second_iteration
        )
        x2, y2, z2 = self.project_vector_to_map(
            self.get_x_y_z_from_azimuth_elevation(azimuth2, elevation2)
        )
        self.line_list.points[3].x = x2
        self.line_list.points[3].y = y2
        self.line_list.points[3].z = z2

        if msg.sensors_nb_recovered > 2:
            azimuth3, elevation3 = self.geometry.get_azimuth_elevation_from_iteration(
                msg.third_sensor_first_iteration, msg.third_sensor_second_iteration
            )
            x3, y3, z3 = self.project_vector_to_map(
                self.get_x_y_z_from_azimuth_elevation(azimuth3, elevation3)
            )
        else:
            x3 = self.final_tf.transform.translation.x
            y3 = self.final_tf.transform.translation.y
            z3 = 0.0
        self.line_list.points[5].x = x3
        self.line_list.points[5].y = y3
        self.line_list.points[5].z = z3

        if msg.sensors_nb_recovered > 3:
            azimuth4, elevation4 = self.geometry.get_azimuth_elevation_from_iteration(
                msg.fourth_sensor_first_iteration, msg.fourth_sensor_second_iteration
            )
            x4, y4, z4 = self.project_vector_to_map(
                self.get_x_y_z_from_azimuth_elevation(azimuth4, elevation4)
            )
        else:
            x4 = self.final_tf.transform.translation.x
            y4 = self.final_tf.transform.translation.y
            z4 = 0.0
        self.line_list.points[7].x = x4
        self.line_list.points[7].y = y4
        self.line_list.points[7].z = z4

        self.line_list.header.stamp = self.parent_node.get_clock().now().to_msg()
        if self.line_and_sensors_position_viz:
            self.lh_lines_pb.publish(self.line_list)

        (
            self.sensors_est_position.points[0].x,
            self.sensors_est_position.points[0].y,
        ) = self.x_y_at_h(self.line_list.points[1])
        (
            self.sensors_est_position.points[1].x,
            self.sensors_est_position.points[1].y,
        ) = self.x_y_at_h(self.line_list.points[3])
        (
            self.sensors_est_position.points[2].x,
            self.sensors_est_position.points[2].y,
        ) = self.x_y_at_h(self.line_list.points[5])
        (
            self.sensors_est_position.points[3].x,
            self.sensors_est_position.points[3].y,
        ) = self.x_y_at_h(self.line_list.points[7])
        self.sensors_est_position.header.stamp = (
            self.parent_node.get_clock().now().to_msg()
        )
        if self.line_and_sensors_position_viz:
            self.sensors_est_position_pb.publish(self.sensors_est_position)

        recovered_pairs = self.identify_pairs(sensors_id_list)
        avg_nb = 0
        self.tracker_pose = Point()
        cos_sin_angles = [0.0, 0.0]

        if self.possible_pairs[0] in recovered_pairs:
            x, y, theta = self.localisation_zero_first(sensors_id_list)
            self.tracker_pose.x += x
            self.tracker_pose.y += y
            cos_sin_angles[0] += np.cos(theta)
            cos_sin_angles[1] += np.sin(theta)
            avg_nb += 1

        if self.possible_pairs[1] in recovered_pairs:
            x, y, theta = self.localisation_second_third(sensors_id_list)
            self.tracker_pose.x += x
            self.tracker_pose.y += y
            cos_sin_angles[0] += np.cos(theta)
            cos_sin_angles[1] += np.sin(theta)
            avg_nb += 1

        if self.possible_pairs[2] in recovered_pairs:
            x, y, theta = self.localisation_fourth_fifth(sensors_id_list)
            self.tracker_pose.x += x
            self.tracker_pose.y += y
            cos_sin_angles[0] += np.cos(theta)
            cos_sin_angles[1] += np.sin(theta)
            avg_nb += 1

        if self.possible_pairs[3] in recovered_pairs:
            x, y, theta = self.localisation_sixth_seventh(sensors_id_list)
            self.tracker_pose.x += x
            self.tracker_pose.y += y
            cos_sin_angles[0] += np.cos(theta)
            cos_sin_angles[1] += np.sin(theta)
            avg_nb += 1

        if avg_nb != 0:
            self.tracker_pose.x = self.tracker_pose.x / avg_nb
            self.tracker_pose.y = self.tracker_pose.y / avg_nb
            self.tracker_pose.z = np.arctan2(cos_sin_angles[1], cos_sin_angles[0])

            self.marker_estimated_pose.pose.position.x = self.tracker_pose.x
            self.marker_estimated_pose.pose.position.y = self.tracker_pose.y
            self.marker_estimated_pose.pose.position.z = 0.0
            rot = PyKDL.Rotation().RotZ(self.tracker_pose.z)
            q = rot.GetQuaternion()
            self.marker_estimated_pose.pose.orientation.x = q[0]
            self.marker_estimated_pose.pose.orientation.y = q[1]
            self.marker_estimated_pose.pose.orientation.z = q[2]
            self.marker_estimated_pose.pose.orientation.w = q[3]
            self.marker_estimated_pose.header.stamp = (
                self.parent_node.get_clock().now().to_msg()
            )
            self.parent_node.create_and_send_tf(
                self.marker_estimated_pose.pose.position.x,
                self.marker_estimated_pose.pose.position.y,
                self.marker_estimated_pose.pose.orientation,
                self.marker_estimated_pose.header.stamp,
            )
            self.lh_est_pose_pb.publish(self.marker_estimated_pose)

    def localisation_zero_first(self, sensors_id_list):
        index_zero = sensors_id_list.index(0)
        index_first = sensors_id_list.index(1)
        point_zero = self.sensors_est_position.points[index_zero]
        point_first = self.sensors_est_position.points[index_first]

        vector_zero_to_first = PyKDL.Vector(
            x=(point_first.x - point_zero.x), y=(point_first.y - point_zero.y), z=0.0
        )
        vector_zero_to_first.Normalize()
        theta = (
            np.arccos(vector_zero_to_first.x())
            if vector_zero_to_first.y() > 0
            else -np.arccos(vector_zero_to_first.x())
        )

        x = (
            csts.dist_from_center * np.cos(theta - np.pi / 2)
            + (point_zero.x + point_first.x) / 2
        )
        y = (
            csts.dist_from_center * np.sin(theta - np.pi / 2)
            + (point_zero.y + point_first.y) / 2
        )

        return (x, y, theta)

    def localisation_second_third(self, sensors_id_list):
        index_second = sensors_id_list.index(2)
        index_third = sensors_id_list.index(3)
        point_second = self.sensors_est_position.points[index_second]
        point_third = self.sensors_est_position.points[index_third]

        vector_second_to_third = PyKDL.Vector(
            x=(point_third.x - point_second.x),
            y=(point_third.y - point_second.y),
            z=0.0,
        )
        vector_second_to_third.Normalize()
        theta = (
            np.arccos(vector_second_to_third.x())
            if vector_second_to_third.y() > 0
            else -np.arccos(vector_second_to_third.x())
        ) + np.pi / 2

        x = (
            csts.dist_from_center * np.cos(theta - np.pi)
            + (point_second.x + point_third.x) / 2
        )
        y = (
            csts.dist_from_center * np.sin(theta - np.pi)
            + (point_second.y + point_third.y) / 2
        )

        return (x, y, theta)

    def localisation_fourth_fifth(self, sensors_id_list):
        index_fourth = sensors_id_list.index(4)
        index_fifth = sensors_id_list.index(5)
        point_fourth = self.sensors_est_position.points[index_fourth]
        point_fifth = self.sensors_est_position.points[index_fifth]

        vector_fourth_to_fifth = PyKDL.Vector(
            x=(point_fifth.x - point_fourth.x),
            y=(point_fifth.y - point_fourth.y),
            z=0.0,
        )
        vector_fourth_to_fifth.Normalize()
        theta = (
            np.arccos(vector_fourth_to_fifth.x())
            if vector_fourth_to_fifth.y() > 0
            else -np.arccos(vector_fourth_to_fifth.x())
        ) + np.pi

        x = (
            csts.dist_from_center * np.cos(theta + np.pi / 2)
            + (point_fourth.x + point_fifth.x) / 2
        )
        y = (
            csts.dist_from_center * np.sin(theta + np.pi / 2)
            + (point_fourth.y + point_fifth.y) / 2
        )

        return (x, y, theta)

    def localisation_sixth_seventh(self, sensors_id_list):
        index_sixth = sensors_id_list.index(6)
        index_seventh = sensors_id_list.index(7)
        point_sixth = self.sensors_est_position.points[index_sixth]
        point_seventh = self.sensors_est_position.points[index_seventh]

        vector_sixth_to_seven = PyKDL.Vector(
            x=(point_seventh.x - point_sixth.x),
            y=(point_seventh.y - point_sixth.y),
            z=0.0,
        )
        vector_sixth_to_seven.Normalize()
        theta = (
            np.arccos(vector_sixth_to_seven.x())
            if vector_sixth_to_seven.y() > 0
            else -np.arccos(vector_sixth_to_seven.x())
        ) - np.pi / 2

        x = (
            csts.dist_from_center * np.cos(theta)
            + (point_sixth.x + point_seventh.x) / 2
        )
        y = (
            csts.dist_from_center * np.sin(theta)
            + (point_sixth.y + point_seventh.y) / 2
        )

        return (x, y, theta)

    def identify_pairs(self, sensors_id_list):
        recovered_pairs = []
        for pair in self.possible_pairs:
            recovered_ids = 0
            for id in pair:
                for id_recovered in sensors_id_list:
                    if id == id_recovered:
                        recovered_ids += 1
            if recovered_ids == 2:
                recovered_pairs.append(pair)
        return recovered_pairs

    def x_y_at_h(self, point):
        x = lambda x1, t: self.final_tf.transform.translation.x + t * (
            x1 - self.final_tf.transform.translation.x
        )
        y = lambda y1, t: self.final_tf.transform.translation.y + t * (
            y1 - self.final_tf.transform.translation.y
        )
        t = (self.sensor_height - self.final_tf.transform.translation.z) / (
            point.z - self.final_tf.transform.translation.z
        )
        return (x(point.x, t), y(point.y, t))

    def get_x_y_z_from_azimuth_elevation(self, azimuth, elevation):
        radius = 2.5
        latitude = np.pi / 2 - elevation
        x = radius * np.cos(azimuth) * np.sin(latitude)
        y = radius * np.sin(azimuth) * np.sin(latitude)
        z = radius * np.cos(latitude)
        return (x, y, z)

    def project_vector_to_map(self, triplet):
        kdl_vector = PyKDL.Vector(x=triplet[0], y=triplet[1], z=triplet[2])
        projected_vector = do_transform_vector(kdl_vector, self.final_tf)
        return (projected_vector.x(), projected_vector.y(), projected_vector.z())

    def update_lines_origin(self):
        for i in range(4):
            self.line_list.points[i * 2].x = self.final_tf.transform.translation.x
            self.line_list.points[i * 2].y = self.final_tf.transform.translation.y
            self.line_list.points[i * 2].z = self.final_tf.transform.translation.z


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    lh_tracker_geometry = LH_tracker_geometry()
    try:
        rclpy.spin(lh_tracker_geometry)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
