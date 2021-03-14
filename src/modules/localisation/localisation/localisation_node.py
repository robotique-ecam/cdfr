#!/usr/bin/env python3


"""Robot localisation node."""


import math
import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import MarkerArray
from tf2_ros import StaticTransformBroadcaster
from transformix_msgs.srv import TransformixParametersTransformStamped
from localisation.sensors_sim import Sensors


class Localisation(rclpy.node.Node):
    """Robot localisation node."""

    def __init__(self):
        super().__init__("localisation_node")
        self.robot = self.get_namespace().strip("/")
        self.side = self.declare_parameter("side", "blue")
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._x, self._y, self._theta = self.fetchStartPosition()
        self._tf_brodcaster = StaticTransformBroadcaster(self)
        self._tf = TransformStamped()
        self._tf.header.frame_id = "map"
        self._tf.child_frame_id = "odom"
        self.update_transform()
        self.get_initial_tf_srv = self.create_service(
            TransformixParametersTransformStamped,
            "get_odom_map_tf",
            self.get_initial_tf_srv_callback,
        )
        self.subscription_ = self.create_subscription(
            MarkerArray,
            "/allies_positions_markers",
            self.allies_subscription_callback,
            10,
        )
        self.tf_publisher_ = self.create_publisher(
            TransformStamped, "adjust_odometry", 10
        )
        self.last_odom_update = 0
        self.get_logger().info(f"Default side is {self.side.value}")
        self.sensors = Sensors(self, [30, 31, 32, 33, 34, 35])
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
            self._x, self._y, self._theta = self.fetchStartPosition()
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

    def euler_to_quaternion(self, yaw, pitch=0, roll=0):
        """Conversion between euler angles and quaternions."""
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def quaternion_to_euler(self, x, y, z, w):
        """Conversion between quaternions and euler angles."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)
        return (X, Y, Z)

    def update_transform(self):
        """Update and publish transform callback."""
        self._tf.header.stamp = self.get_clock().now().to_msg()
        self._tf.transform.translation.x = float(self._x)
        self._tf.transform.translation.y = float(self._y)
        qx, qy, qz, qw = self.euler_to_quaternion(self._theta)
        self._tf.transform.rotation.x = float(qx)
        self._tf.transform.rotation.y = float(qy)
        self._tf.transform.rotation.z = float(qz)
        self._tf.transform.rotation.w = float(qw)
        self._initial_tf = self._tf
        self._tf_brodcaster.sendTransform(self._tf)

    def allies_subscription_callback(self, msg):
        """Identity the robot marker in assurancetourix marker_array detection
        publish the transformation for drive to readjust odometry"""
        for allie_marker in msg.markers:
            if (
                allie_marker.text.lower() == self.robot
                and (self.get_clock().now().to_msg().sec - self.last_odom_update) > 5
            ):
                self._x = allie_marker.pose.position.x
                self._y = allie_marker.pose.position.y
                q = allie_marker.pose.orientation
                self._theta = self.quaternion_to_euler(q.x, q.y, q.z, q.w)[2]
                self._tf.header.stamp = allie_marker.header.stamp
                self._tf.transform.translation.x = allie_marker.pose.position.x
                self._tf.transform.translation.y = allie_marker.pose.position.y
                self._tf.transform.translation.z = float(0)
                self._tf.transform.rotation = q
                self.tf_publisher_.publish(self._tf)
                self.last_odom_update = self.get_clock().now().to_msg().sec

    def get_initial_tf_srv_callback(self, request, response):
        self.get_logger().info(f"incoming request for {self.robot} odom -> map tf")
        response.transform_stamped = self._initial_tf
        return response


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    localisation = Localisation()
    try:
        rclpy.spin(localisation)
    except KeyboardInterrupt:
        pass
    localisation.destroy_node()
    rclpy.shutdown()
