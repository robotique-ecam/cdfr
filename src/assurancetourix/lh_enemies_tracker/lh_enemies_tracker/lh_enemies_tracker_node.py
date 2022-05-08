#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import random
import copy
import lh_enemies_tracker.constants as csts

from tf2_kdl import PyKDL, do_transform_frame, transform_to_kdl, do_transform_vector
from rclpy.node import Node
from lh_tracker_msgs.msg import LHtracker
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from lh_enemies_tracker.lh_geometry import LH2Geometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from transformix_msgs.srv import InitialStaticTFsrv

from datetime import datetime


class LH_enemies_tracker(Node):
    def __init__(self):
        """Init LH_tracker_geometry node"""
        super().__init__("lh_enemies_tracker")
        self.possible_pairs = [[0, 1], [2, 3], [4, 5], [6, 7]]
        self.geometry = LH2Geometry()

        self.get_yaml_params()
        self.init_est_marker()
        self.init_est_marker()
        self.init_line_list()
        self.init_sensors_markers()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.avg_nb = 200
        self.global_index = 0
        self.calibration_done = False

        self.saved_iteration_list = LHtracker()

        self.final_tf = TransformStamped()

        self.lh_est_pose_pb = self.create_publisher(
            MarkerArray, "ennemies_positions_markers", 10
        )
        self.subscription = self.create_subscription(
            LHtracker, "/first_enemy/lh_msgs", self.lh_sub_callback_first_enemy, 10
        )
        self.subscription1 = self.create_subscription(
            LHtracker, "/second_enemy/lh_msgs", self.lh_sub_callback_second_enemy, 10
        )

        self.lh_tf_to_asterix_client = self.create_client(
            InitialStaticTFsrv, "/asterix/tf_map_lh"
        )
        self.lh_tf_to_asterix_request = InitialStaticTFsrv.Request()
        self.spinning_lh_tf_to_asterix_request = False

        self.lh_tf_to_obelix_client = self.create_client(
            InitialStaticTFsrv, "/obelix/tf_map_lh"
        )
        self.lh_tf_to_obelix_request = InitialStaticTFsrv.Request()
        self.spinning_lh_tf_to_obelix_request = False

        if self.lh_line_and_sensors_position_viz:
            self.sensors_est_position_pb = self.create_publisher(
                MarkerArray, "enemies_sensors_est_position", 10
            )
            self.lh_lines_pb = self.create_publisher(
                MarkerArray, "enemies_lh_lines", 10
            )

        self.get_logger().info("lh_tracker_geometry node is ready")

    def get_yaml_params(self):
        self.declare_parameter("calibration_position_x", 1.5)
        self.declare_parameter("calibration_position_y", 0.1)
        self.declare_parameter("calibration_position_z", 0.415)
        self.declare_parameter("calibration_theta_degrees", 45)
        self.declare_parameter("enemies_sensor_height", 0.415)
        self.declare_parameter("lh_line_and_sensors_position_viz", True)

        calib_x = self.get_parameter("calibration_position_x")._value
        calib_y = self.get_parameter("calibration_position_y")._value
        calib_z = self.get_parameter("calibration_position_z")._value
        calib_theta = self.get_parameter("calibration_theta_degrees")._value
        self.sensor_height = self.get_parameter("enemies_sensor_height")._value
        self.lh_line_and_sensors_position_viz = self.get_parameter(
            "lh_line_and_sensors_position_viz"
        )._value

        self.initial_tf = self.kdl_to_transform(
            PyKDL.Frame(
                R=PyKDL.Rotation().EulerZYX(np.deg2rad(calib_theta), 0.0, 0.0),
                V=PyKDL.Vector(x=calib_x, y=calib_y, z=calib_z),
            )
        )

    def init_est_marker(self):
        self.marker_estimated_pose = MarkerArray()
        for i in range(2):
            self.marker_estimated_pose.markers.append(Marker())
            self.marker_estimated_pose.markers[i].id = i
            self.marker_estimated_pose.markers[i].header.frame_id = "map"
            self.marker_estimated_pose.markers[i].type = 0  # arrow
            self.marker_estimated_pose.markers[i].scale.x = 0.3
            self.marker_estimated_pose.markers[i].scale.y = 0.1
            self.marker_estimated_pose.markers[i].scale.z = 0.1
            self.marker_estimated_pose.markers[i].color.a = 1.0
            self.marker_estimated_pose.markers[i].lifetime.sec = 5
            self.marker_estimated_pose.markers[i].color.r = 1.0
            self.marker_estimated_pose.markers[i].pose.position.x = -1.0

    def init_line_list(self):
        self.line_list = MarkerArray()
        for i in range(2):
            self.line_list.markers.append(Marker())
            self.line_list.markers[i].id = i
            self.line_list.markers[i].header.frame_id = "map"
            self.line_list.markers[i].type = 5  # line_list
            self.line_list.markers[i].color.r = 1.0
            self.line_list.markers[i].color.g = 1.0
            self.line_list.markers[i].color.b = 1.0
            self.line_list.markers[i].color.a = 1.0
            self.line_list.markers[i].scale.x = 0.005
            self.line_list.markers[i].lifetime.sec = 10
            for _ in range(4):
                self.line_list.markers[i].colors.append(ColorRGBA())
                self.line_list.markers[i].colors[-1].a = 1.0
            self.line_list.markers[i].colors[0].r = 1.0
            self.line_list.markers[i].colors[1].g = 1.0
            self.line_list.markers[i].colors[2].b = 1.0
            for _ in range(8):
                self.line_list.markers[i].points.append(Point())

    def init_sensors_markers(self):
        self.sensors_est_position = MarkerArray()
        for i in range(2):
            self.sensors_est_position.markers.append(Marker())
            self.sensors_est_position.markers[i].id = i
            self.sensors_est_position.markers[i].header.frame_id = "map"
            self.sensors_est_position.markers[i].type = 7  # line_list
            self.sensors_est_position.markers[i].color.a = 1.0
            self.sensors_est_position.markers[i].scale.x = 0.015
            self.sensors_est_position.markers[i].scale.y = 0.015
            self.sensors_est_position.markers[i].scale.z = 0.015
            self.sensors_est_position.markers[i].lifetime.sec = 10
            for _ in range(4):
                self.sensors_est_position.markers[i].colors.append(ColorRGBA())
                self.sensors_est_position.markers[i].colors[-1].r = 1.0
                self.sensors_est_position.markers[i].colors[-1].a = 1.0
                self.sensors_est_position.markers[i].points.append(Point())
                self.sensors_est_position.markers[i].points[-1].z = self.sensor_height

    def lh_sub_callback_first_enemy(self, msg):
        self.lh_sub_callback(msg, 0)

    def lh_sub_callback_second_enemy(self, msg):
        self.lh_sub_callback(msg, 1)

    def lh_sub_callback(self, msg, enemy_id):
        if not self.calibration_done and msg.sensors_nb_recovered == 8:
            self.calibration_process(msg)
        elif self.calibration_done and msg.sensors_nb_recovered != 8:
            self.nominal_localisation(msg, enemy_id)

    def nominal_localisation(self, msg, enemy_id):
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
        self.line_list.markers[enemy_id].points[1].x = x1
        self.line_list.markers[enemy_id].points[1].y = y1
        self.line_list.markers[enemy_id].points[1].z = z1

        azimuth2, elevation2 = self.geometry.get_azimuth_elevation_from_iteration(
            msg.second_sensor_first_iteration, msg.second_sensor_second_iteration
        )
        x2, y2, z2 = self.project_vector_to_map(
            self.get_x_y_z_from_azimuth_elevation(azimuth2, elevation2)
        )
        self.line_list.markers[enemy_id].points[3].x = x2
        self.line_list.markers[enemy_id].points[3].y = y2
        self.line_list.markers[enemy_id].points[3].z = z2

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
        self.line_list.markers[enemy_id].points[5].x = x3
        self.line_list.markers[enemy_id].points[5].y = y3
        self.line_list.markers[enemy_id].points[5].z = z3

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
        self.line_list.markers[enemy_id].points[7].x = x4
        self.line_list.markers[enemy_id].points[7].y = y4
        self.line_list.markers[enemy_id].points[7].z = z4

        self.line_list.markers[enemy_id].header.stamp = self.get_clock().now().to_msg()
        if self.lh_line_and_sensors_position_viz:
            self.lh_lines_pb.publish(self.line_list)

        (
            self.sensors_est_position.markers[enemy_id].points[0].x,
            self.sensors_est_position.markers[enemy_id].points[0].y,
        ) = self.x_y_at_h(self.line_list.markers[enemy_id].points[1])
        (
            self.sensors_est_position.markers[enemy_id].points[1].x,
            self.sensors_est_position.markers[enemy_id].points[1].y,
        ) = self.x_y_at_h(self.line_list.markers[enemy_id].points[3])
        (
            self.sensors_est_position.markers[enemy_id].points[2].x,
            self.sensors_est_position.markers[enemy_id].points[2].y,
        ) = self.x_y_at_h(self.line_list.markers[enemy_id].points[5])
        (
            self.sensors_est_position.markers[enemy_id].points[3].x,
            self.sensors_est_position.markers[enemy_id].points[3].y,
        ) = self.x_y_at_h(self.line_list.markers[enemy_id].points[7])
        self.sensors_est_position.markers[enemy_id].header.stamp = (
            self.get_clock().now().to_msg()
        )
        if self.lh_line_and_sensors_position_viz:
            self.sensors_est_position_pb.publish(self.sensors_est_position)

        recovered_pairs = self.identify_pairs(sensors_id_list)
        avg_nb = 0
        self.tracker_pose = Point()
        cos_sin_angles = [0.0, 0.0]

        if self.possible_pairs[0] in recovered_pairs:
            x, y, theta = self.localisation_zero_first(sensors_id_list, enemy_id)
            self.tracker_pose.x += x
            self.tracker_pose.y += y
            cos_sin_angles[0] += np.cos(theta)
            cos_sin_angles[1] += np.sin(theta)
            avg_nb += 1

        if self.possible_pairs[1] in recovered_pairs:
            x, y, theta = self.localisation_second_third(sensors_id_list, enemy_id)
            self.tracker_pose.x += x
            self.tracker_pose.y += y
            cos_sin_angles[0] += np.cos(theta)
            cos_sin_angles[1] += np.sin(theta)
            avg_nb += 1

        if self.possible_pairs[2] in recovered_pairs:
            x, y, theta = self.localisation_fourth_fifth(sensors_id_list, enemy_id)
            self.tracker_pose.x += x
            self.tracker_pose.y += y
            cos_sin_angles[0] += np.cos(theta)
            cos_sin_angles[1] += np.sin(theta)
            avg_nb += 1

        if self.possible_pairs[3] in recovered_pairs:
            x, y, theta = self.localisation_sixth_seventh(sensors_id_list, enemy_id)
            self.tracker_pose.x += x
            self.tracker_pose.y += y
            cos_sin_angles[0] += np.cos(theta)
            cos_sin_angles[1] += np.sin(theta)
            avg_nb += 1

        if avg_nb != 0:
            self.tracker_pose.x = self.tracker_pose.x / avg_nb
            self.tracker_pose.y = self.tracker_pose.y / avg_nb
            self.tracker_pose.z = np.arctan2(cos_sin_angles[1], cos_sin_angles[0])

            self.marker_estimated_pose.markers[
                enemy_id
            ].pose.position.x = self.tracker_pose.x
            self.marker_estimated_pose.markers[
                enemy_id
            ].pose.position.y = self.tracker_pose.y
            self.marker_estimated_pose.markers[enemy_id].pose.position.z = 0.0
            rot = PyKDL.Rotation().RotZ(self.tracker_pose.z)
            q = rot.GetQuaternion()
            self.marker_estimated_pose.markers[enemy_id].pose.orientation.x = q[0]
            self.marker_estimated_pose.markers[enemy_id].pose.orientation.y = q[1]
            self.marker_estimated_pose.markers[enemy_id].pose.orientation.z = q[2]
            self.marker_estimated_pose.markers[enemy_id].pose.orientation.w = q[3]
            self.marker_estimated_pose.markers[enemy_id].header.stamp = (
                self.get_clock().now().to_msg()
            )
            self.lh_est_pose_pb.publish(self.marker_estimated_pose)

    def localisation_zero_first(self, sensors_id_list, enemy_id):
        index_zero = sensors_id_list.index(0)
        index_first = sensors_id_list.index(1)
        point_zero = self.sensors_est_position.markers[enemy_id].points[index_zero]
        point_first = self.sensors_est_position.markers[enemy_id].points[index_first]

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

    def localisation_second_third(self, sensors_id_list, enemy_id):
        index_second = sensors_id_list.index(2)
        index_third = sensors_id_list.index(3)
        point_second = self.sensors_est_position.markers[enemy_id].points[index_second]
        point_third = self.sensors_est_position.markers[enemy_id].points[index_third]

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

    def localisation_fourth_fifth(self, sensors_id_list, enemy_id):
        index_fourth = sensors_id_list.index(4)
        index_fifth = sensors_id_list.index(5)
        point_fourth = self.sensors_est_position.markers[enemy_id].points[index_fourth]
        point_fifth = self.sensors_est_position.markers[enemy_id].points[index_fifth]

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

    def localisation_sixth_seventh(self, sensors_id_list, enemy_id):
        index_sixth = sensors_id_list.index(6)
        index_seventh = sensors_id_list.index(7)
        point_sixth = self.sensors_est_position.markers[enemy_id].points[index_sixth]
        point_seventh = self.sensors_est_position.markers[enemy_id].points[
            index_seventh
        ]

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

    def calibration_process(self, msg):
        if msg.sensors_nb_recovered > 3 and self.global_index < self.avg_nb:
            self.global_index += 1
            self.saved_iteration_list.first_sensor_first_iteration += (
                msg.first_sensor_first_iteration
            )
            self.saved_iteration_list.first_sensor_second_iteration += (
                msg.first_sensor_second_iteration
            )

            self.saved_iteration_list.second_sensor_first_iteration += (
                msg.second_sensor_first_iteration
            )
            self.saved_iteration_list.second_sensor_second_iteration += (
                msg.second_sensor_second_iteration
            )

            self.saved_iteration_list.third_sensor_first_iteration += (
                msg.third_sensor_first_iteration
            )
            self.saved_iteration_list.third_sensor_second_iteration += (
                msg.third_sensor_second_iteration
            )

            self.saved_iteration_list.fourth_sensor_first_iteration += (
                msg.fourth_sensor_first_iteration
            )
            self.saved_iteration_list.fourth_sensor_second_iteration += (
                msg.fourth_sensor_second_iteration
            )
        else:
            self.saved_iteration_list.first_sensor_first_iteration = int(
                self.saved_iteration_list.first_sensor_first_iteration / self.avg_nb
            )
            self.saved_iteration_list.first_sensor_second_iteration = int(
                self.saved_iteration_list.first_sensor_second_iteration / self.avg_nb
            )

            self.saved_iteration_list.second_sensor_first_iteration = int(
                self.saved_iteration_list.second_sensor_first_iteration / self.avg_nb
            )
            self.saved_iteration_list.second_sensor_second_iteration = int(
                self.saved_iteration_list.second_sensor_second_iteration / self.avg_nb
            )

            self.saved_iteration_list.third_sensor_first_iteration = int(
                self.saved_iteration_list.third_sensor_first_iteration / self.avg_nb
            )
            self.saved_iteration_list.third_sensor_second_iteration = int(
                self.saved_iteration_list.third_sensor_second_iteration / self.avg_nb
            )

            self.saved_iteration_list.fourth_sensor_first_iteration = int(
                self.saved_iteration_list.fourth_sensor_first_iteration / self.avg_nb
            )
            self.saved_iteration_list.fourth_sensor_second_iteration = int(
                self.saved_iteration_list.fourth_sensor_second_iteration / self.avg_nb
            )

            angles_list = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]
            azimuth1, elevation1 = self.geometry.get_azimuth_elevation_from_iteration(
                self.saved_iteration_list.first_sensor_first_iteration,
                self.saved_iteration_list.first_sensor_second_iteration,
            )
            angles_list[msg.id_first_sensor] = (-np.tan(azimuth1), -np.tan(elevation1))

            azimuth2, elevation2 = self.geometry.get_azimuth_elevation_from_iteration(
                self.saved_iteration_list.second_sensor_first_iteration,
                self.saved_iteration_list.second_sensor_second_iteration,
            )
            angles_list[msg.id_second_sensor] = (-np.tan(azimuth2), -np.tan(elevation2))

            azimuth3, elevation3 = self.geometry.get_azimuth_elevation_from_iteration(
                self.saved_iteration_list.third_sensor_first_iteration,
                self.saved_iteration_list.third_sensor_second_iteration,
            )
            angles_list[msg.id_third_sensor] = (-np.tan(azimuth3), -np.tan(elevation3))

            azimuth4, elevation4 = self.geometry.get_azimuth_elevation_from_iteration(
                self.saved_iteration_list.fourth_sensor_first_iteration,
                self.saved_iteration_list.fourth_sensor_second_iteration,
            )
            angles_list[msg.id_fourth_sensor] = (-np.tan(azimuth4), -np.tan(elevation4))

            self.triangulate(angles_list)

            if self.final_tf.transform.translation.x != 0:
                self.calibration_done = True
                self.get_logger().info(
                    "Calibration done, sending tf to asterix and obelix"
                )

                self.lh_tf_to_asterix_request.map_odom_static_tf = self.final_tf
                self.future_tf_service_asterix = (
                    self.lh_tf_to_asterix_client.call_async(
                        self.lh_tf_to_asterix_request
                    )
                )
                self.spinning_lh_tf_to_asterix_request = True
                self.timer_check_tf_service_asterix = self.create_timer(
                    2.0, self.check_tf_service_asterix
                )

                self.lh_tf_to_obelix_request.map_odom_static_tf = self.final_tf
                self.future_tf_service_obelix = self.lh_tf_to_obelix_client.call_async(
                    self.lh_tf_to_obelix_request
                )
                self.spinning_lh_tf_to_obelix_request = True
                self.timer_check_tf_service_obelix = self.create_timer(
                    2.0, self.check_tf_service_obelix
                )

                for i in range(len(self.line_list.markers)):
                    for j in range(len(self.line_list.markers[i].points)):
                        self.line_list.markers[i].points[
                            j
                        ].x = self.final_tf.transform.translation.x
                        self.line_list.markers[i].points[
                            j
                        ].y = self.final_tf.transform.translation.y
                        self.line_list.markers[i].points[
                            j
                        ].z = self.final_tf.transform.translation.z

    def check_tf_service_asterix(self):
        if self.future_tf_service_asterix.done():
            try:
                response = self.future_tf_service_asterix.result()
            except Exception as e:
                self.get_logger().info("Asterix lh_tf service call failed %r" % (e,))
                self.get_logger().warn(
                    "Asterix lh_tf sending to drive failed, retrying..."
                )
                self.future_tf_service_asterix = (
                    self.lh_tf_to_asterix_client.call_async(
                        self.lh_tf_to_asterix_request
                    )
                )

            else:
                if response.acquittal.data:
                    self.get_logger().info("Asterix lh_tf successfuly sent to drive")
                    self.destroy_timer(self.timer_check_tf_service_asterix)
                    self.spinning_lh_tf_to_asterix_request = False
                else:
                    self.get_logger().warn(
                        "Asterix lh_tf sending to drive failed, retrying..."
                    )
                    self.future_tf_service_asterix = (
                        self.lh_tf_to_asterix_client.call_async(
                            self.lh_tf_to_asterix_request
                        )
                    )
            return
        self.get_logger().info("Asterix lh_tf service cancellation + recall")
        self.future_tf_service_asterix.cancel()
        self.future_tf_service_asterix = self.lh_tf_to_asterix_client.call_async(
            self.lh_tf_to_asterix_request
        )

    def check_tf_service_obelix(self):
        if self.future_tf_service_obelix.done():
            try:
                response = self.future_tf_service_obelix.result()
            except Exception as e:
                self.get_logger().info("Obelix lh_tf service call failed %r" % (e,))
                self.get_logger().warn(
                    "Obelix lh_tf sending to drive failed, retrying..."
                )
                self.future_tf_service_obelix = self.lh_tf_to_obelix_client.call_async(
                    self.lh_tf_to_obelix_request
                )

            else:
                if response.acquittal.data:
                    self.get_logger().info("Obelix lh_tf successfuly sent to drive")
                    self.destroy_timer(self.timer_check_tf_service_obelix)
                    self.spinning_lh_tf_to_obelix_request = False
                else:
                    self.get_logger().warn(
                        "Obelix lh_tf sending to drive failed, retrying..."
                    )
                    self.future_tf_service_obelix = (
                        self.lh_tf_to_obelix_client.call_async(
                            self.lh_tf_to_obelix_request
                        )
                    )
            return
        self.get_logger().info("Obelix lh_tf service cancellation + recall")
        self.future_tf_service_obelix.cancel()
        self.future_tf_service_obelix = self.lh_tf_to_obelix_client.call_async(
            self.lh_tf_to_obelix_request
        )

    def triangulate(self, angles_list):
        points_3D = np.array(
            [
                csts.sensor_id_to_position[0],
                csts.sensor_id_to_position[1],
                csts.sensor_id_to_position[2],
                csts.sensor_id_to_position[3],
            ]
        )
        points_2D = np.array(
            angles_list,
            dtype="double",
        )
        camera_matrix = np.array(
            [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
            ],
            dtype="double",
        )
        dist_coeffs = np.zeros((4, 1))

        ref_frame = PyKDL.Frame(
            R=PyKDL.Rotation().EulerZYX(
                -180 * np.pi / 180, 45 * np.pi / 180, -90 * np.pi / 180
            ),
            V=PyKDL.Vector(0.0, 0.585, 0.1),
        )
        ref_frame = ref_frame.Inverse()

        ref_rot = ref_frame.M
        ref_rot_axis = ref_rot.GetRot()
        ref_tr = ref_frame.p

        rvec_ref = np.array(
            [ref_rot_axis.x(), ref_rot_axis.y(), ref_rot_axis.z()],
            dtype="double",
        )

        tvec_ref = np.array(
            [ref_tr.x(), ref_tr.y(), ref_tr.z()],
            dtype="double",
        )

        LM_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            100000000,
            0.000000001,
        )

        rotation_vector, translation_vector = cv2.solvePnPRefineLM(
            objectPoints=points_3D,
            imagePoints=points_2D,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            rvec=rvec_ref,
            tvec=tvec_ref,
            criteria=LM_criteria,
        )  # refined solvepnp using Levenberg-Marquardt iterative minimization process

        rot_vec = PyKDL.Vector(
            x=rotation_vector[0],
            y=rotation_vector[1],
            z=rotation_vector[2],
        )
        tf_ = PyKDL.Frame(
            R=PyKDL.Rotation().Rot(
                vec=rot_vec,
                angle=rot_vec.Norm(),
            ),
            V=PyKDL.Vector(
                x=translation_vector[0],
                y=translation_vector[1],
                z=translation_vector[2],
            ),
        )

        rot_vec_to_euler = tf_.M.GetEulerZYX()
        if rot_vec_to_euler[1] != 0:
            calibration_tf = self.kdl_to_transform(tf_)

            lh_to_v_camera_frame = self.kdl_to_transform(
                PyKDL.Frame(R=PyKDL.Rotation().EulerZYX(-np.pi / 2, 0, -np.pi / 2))
            )

            kdl_tf_tracker_to_camera = transform_to_kdl(calibration_tf)
            kdl_tf_tracker_to_lh = do_transform_frame(
                kdl_tf_tracker_to_camera, lh_to_v_camera_frame
            )
            kdl_tf_lh_to_tracker = kdl_tf_tracker_to_lh.Inverse()
            kdl_tf_map_to_lh = do_transform_frame(kdl_tf_lh_to_tracker, self.initial_tf)

            self.final_tf.child_frame_id = "lh_frame"
            self.final_tf.header.frame_id = "map"
            self.final_tf.header.stamp = self.get_clock().now().to_msg()
            self.final_tf.transform = self.kdl_to_transform(kdl_tf_map_to_lh).transform

            test = StaticTransformBroadcaster(self)
            test.sendTransform(self.final_tf)

    def kdl_to_transform(self, kdl_frame):
        q = kdl_frame.M.GetQuaternion()
        tf_stamped = TransformStamped()
        tf_stamped.transform.rotation.x = q[0]
        tf_stamped.transform.rotation.y = q[1]
        tf_stamped.transform.rotation.z = q[2]
        tf_stamped.transform.rotation.w = q[3]
        tf_stamped.transform.translation.x = kdl_frame.p.x()
        tf_stamped.transform.translation.y = kdl_frame.p.y()
        tf_stamped.transform.translation.z = kdl_frame.p.z()
        return tf_stamped

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


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    lh_enemies_tracker = LH_enemies_tracker()
    try:
        rclpy.spin(lh_enemies_tracker)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
