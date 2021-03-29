#!/usr/bin/env python3


"""Robot localisation node."""


import math
import numpy as np
import time
import copy
import rclpy

from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import MarkerArray
from tf2_ros import StaticTransformBroadcaster
from .vlx.vlx_readjustment import VlxReadjustment
from localisation.utils import euler_to_quaternion, is_simulation
from nav_msgs.msg import Odometry
from tf2_kdl import *
from datetime import datetime


class Localisation(rclpy.node.Node):
    """Robot localisation node."""

    def __init__(self):
        super().__init__("localisation_node")
        self.robot = self.get_namespace().strip("/")
        self.side = self.declare_parameter("side", "blue")
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.robot_pose = PoseStamped()
        (
            self.robot_pose.pose.position.x,
            self.robot_pose.pose.position.y,
            theta,
        ) = self.fetchStartPosition()
        self.robot_pose.pose.orientation = euler_to_quaternion(theta)
        self._tf_brodcaster = StaticTransformBroadcaster(self)
        self._tf = TransformStamped()
        self._tf.header.frame_id = "map"
        self._tf.child_frame_id = "odom"
        self.update_transform()
        self.subscription_ = self.create_subscription(
            MarkerArray,
            "/allies_positions_markers",
            self.allies_subscription_callback,
            10,
        )
        self.subscription_ = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
        )
        self.tf_publisher_ = self.create_publisher(
            TransformStamped, "adjust_odometry", 10
        )
        self.odom_map_relative_publisher_ = self.create_publisher(
            PoseStamped, "odom_map_relative", 10
        )
        self.last_odom_update = 0
        self.get_logger().info(f"Default side is {self.side.value}")
        self.vlx = VlxReadjustment(self)
        self.get_logger().info("Localisation node is ready")

    def _on_set_parameters(self, params):
        """Handle Parameter events especially for side."""
        result = SetParametersResult()
        try:
            for param in params:
                if param.name == "side":
                    self.get_logger().warn(f"Side changed {param.value}")
                    self.side = param
                else:
                    setattr(self, param.name, param)
            (
                self.robot_pose.pose.position.x,
                self.robot_pose.pose.position.y,
                theta,
            ) = self.fetchStartPosition()
            self.robot_pose.pose.orientation = euler_to_quaternion(theta)
            self.update_transform()
            result.successful = True
        except BaseException as e:
            result.reason = e
        return result

    def fetchStartPosition(self):
        """Fetch start position for side and robot."""
        # TODO : Calibrate the start position using VL53L1X
        if self.robot == "asterix":
            if self.side.value == "blue":
                return (0.29, 1.33, 0)
            elif self.side.value == "yellow":
                return (0.29, 1.33, 0)
        elif self.robot == "obelix":
            if self.side.value == "blue":
                return (0.29, 1.33, 0)
            elif self.side.value == "yellow":
                return (3 - 0.29, 1.33, np.pi)
        # Make it crash in case of undefined parameters
        return None

    def update_transform(self):
        """Update and publish transform odom --> map."""
        self._tf.header.stamp = self.get_clock().now().to_msg()
        self._tf.transform.translation.x = self.robot_pose.pose.position.x
        self._tf.transform.translation.y = self.robot_pose.pose.position.y
        self._tf.transform.rotation = self.robot_pose.pose.orientation
        self._initial_tf = copy.deepcopy(self._tf)
        self._tf_brodcaster.sendTransform(self._tf)

    def allies_subscription_callback(self, msg):
        """Identity the robot marker in assurancetourix marker_array detection
        publish the transformation for drive to readjust odometry
        compute base_link --> odom"""
        for ally_marker in msg.markers:
            if (
                ally_marker.text.lower() == self.robot
                and (self.get_clock().now().to_msg().sec - self.last_odom_update) > 0.75
            ):
                if (
                    is_simulation()
                ):  # simulate marker delais (image analysis from assurancetourix)
                    time.sleep(0.15)
                if self.vlx.continuous_sampling == 0:
                    self.get_logger().info(f"initial continuous_sampling == 0")
                    self.create_and_send_tf(
                        ally_marker.pose.position.x,
                        ally_marker.pose.position.y,
                        ally_marker.pose.orientation,
                        ally_marker.header.stamp,
                    )
                else:
                    self.vlx.try_to_readjust_with_vlx(
                        ally_marker.pose.position.x,
                        ally_marker.pose.position.y,
                        ally_marker.pose.orientation,
                        ally_marker.header.stamp,
                    )
                if self.vlx.continuous_sampling in [0, 1]:
                    self.vlx.start_continuous_sampling_thread(0.65, 1)

    def is_near_walls(self, pt):
        return pt.x < 0.25 or pt.y < 0.25 or pt.x > 2.75 or pt.y > 1.75

    def create_and_send_tf(self, x, y, q, stamp):
        self._tf.header.stamp = stamp
        self._tf.transform.translation.x = x
        self._tf.transform.translation.y = y
        self._tf.transform.translation.z = float(0)
        self._tf.transform.rotation = q
        self.last_odom_update = self.get_clock().now().to_msg().sec
        self.get_logger().info(f"publishing tf")
        self.tf_publisher_.publish(self._tf)

    def odom_callback(self, msg):
        """Odom callback, compute the new pose of the robot relative to map"""
        robot_tf = TransformStamped()
        robot_tf.transform.translation.x = msg.pose.pose.position.x
        robot_tf.transform.translation.y = msg.pose.pose.position.y
        robot_tf.transform.rotation = msg.pose.pose.orientation
        robot_frame = transform_to_kdl(robot_tf)
        new_robot_pose_kdl = do_transform_frame(robot_frame, self._initial_tf)
        self.robot_pose.pose.position.x = new_robot_pose_kdl.p.x()
        self.robot_pose.pose.position.y = new_robot_pose_kdl.p.y()
        q = new_robot_pose_kdl.M.GetQuaternion()
        self.robot_pose.header.stamp = msg.header.stamp
        self.robot_pose.header.frame_id = "map"
        self.robot_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.odom_map_relative_publisher_.publish(self.robot_pose)
        if self.is_near_walls(self.robot_pose.pose.position):
            if self.vlx.continuous_sampling == 2:
                self.vlx.near_wall_routine(self.robot_pose)
            else:
                self.vlx.start_near_wall_routine(self.robot_pose)
        elif self.vlx.continuous_sampling == 2:
            self.vlx.stop_near_wall_routine()


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    localisation = Localisation()
    try:
        rclpy.spin(localisation)
    except KeyboardInterrupt:
        pass
    localisation.vlx.stop_continuous_sampling_thread()
    localisation.vlx.sensors.node.destroy_node()
    localisation.destroy_node()
    rclpy.shutdown()
