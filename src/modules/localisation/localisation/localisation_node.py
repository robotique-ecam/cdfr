#!/usr/bin/env python3


""" Robot localisation node """


import math
import numpy as np
import time
import copy
import rclpy

from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import MarkerArray
from tf2_ros import StaticTransformBroadcaster
from localisation.utils import euler_to_quaternion, is_simulation
from localisation.lh_tracker_geometry import LH_tracker_geometry
from nav_msgs.msg import Odometry
from tf2_kdl import transform_to_kdl, do_transform_frame
from transformix_msgs.srv import InitialStaticTFsrv
from lh_tracker_msgs.msg import LHtracker


class Localisation(rclpy.node.Node):
    """Robot localisation node"""

    def __init__(self):
        """Init Localisation node"""
        super().__init__("localisation_node")
        self.robot = "asterix" #self.get_namespace().strip("/")

        self.lh_geometry = LH_tracker_geometry(self)

        self.createSubscribers()

        self.createPublishers()

        self.createClients()

        self.createServices()

        self.side = "blue"

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
        self.intial_tf_to_drive_request.final_set.data = False
        self.update_transform()

        self.get_logger().info(f"Default side is {self.side}")
        self.get_logger().info("Localisation node is ready")

    def createServices(self):
        self.initial_tf_service = self.create_service(
            InitialStaticTFsrv,
            "localisation_side_selection",
            self.initialSideSelectionCallback,
        )
        self.initial_tf_service_called = False
        self.lh_tf_service = self.create_service(
            InitialStaticTFsrv,
            "tf_map_lh",
            self.map_lh_tf_callback,
        )

    def createClients(self):
        self.intial_tf_to_drive_client = self.create_client(
            InitialStaticTFsrv, "intial_tf_to_drive"
        )
        self.intial_tf_to_drive_request = InitialStaticTFsrv.Request()

    def createPublishers(self):
        self.tf_publisher_ = self.create_publisher(
            TransformStamped, "adjust_odometry", 10
        )
        self.odom_map_relative_publisher_ = self.create_publisher(
            PoseStamped, "odom_map_relative", 10
        )

    def createSubscribers(self):
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

    def map_lh_tf_callback(self, request, response):
        response.acquittal.data = True
        self.lh_geometry.final_tf = request.map_odom_static_tf
        self.lh_geometry.calibration_done = True
        self.lh_geometry.update_lines_origin()
        self.get_logger().info(f"lh_map_tf received {self.lh_geometry.final_tf}")
        return response

    def initialSideSelectionCallback(self, request, response):
        """Assurancetourix send the side of the team"""
        response.acquittal.data = True
        if request.final_set.data:
            self.side = "blue"
        else:
            self.side = "yellow"
        self.robot_pose = PoseStamped()
        (
            self.robot_pose.pose.position.x,
            self.robot_pose.pose.position.y,
            theta,
        ) = self.fetchStartPosition()
        self.robot_pose.pose.orientation = euler_to_quaternion(theta)
        self.intial_tf_to_drive_request.final_set.data = True
        self.update_transform()
        self.get_logger().info(f"Final side selection is {self.side}")
        return response

    def fetchStartPosition(self):
        """Fetch start position for side and robot"""
        if self.robot == "asterix":
            if self.side == "blue":
                return (0.29, 1.33, 0)
            elif self.side == "yellow":
                return (1.5, 1.33, 0)
        elif self.robot == "obelix":
            if self.side == "blue":
                return (0.29, 1.33, 0)
            elif self.side == "yellow":
                return (3 - 0.29, 1.33, np.pi)
        return None

    def update_transform(self):
        """Update and publish odom --> map transform"""
        self._tf.header.stamp = self.get_clock().now().to_msg()
        self._tf.transform.translation.x = self.robot_pose.pose.position.x
        self._tf.transform.translation.y = self.robot_pose.pose.position.y
        self._tf.transform.rotation = self.robot_pose.pose.orientation
        self._initial_tf = copy.deepcopy(self._tf)
        self._tf_brodcaster.sendTransform(self._tf)
        self.intial_tf_to_drive_request.map_odom_static_tf = self._tf
        if self.initial_tf_service_called:
            self.get_logger().info("drive initial_tf service cancellation")
            self.destroy_timer(self.timer_check_tf_service)
            self.future_tf_service.cancel()
        self.future_tf_service = self.intial_tf_to_drive_client.call_async(
            self.intial_tf_to_drive_request
        )
        self.initial_tf_service_called = True
        self.get_logger().info("drive initial_tf service called")
        self.timer_check_tf_service = self.create_timer(2.0, self.check_tf_service)

    def check_tf_service(self):
        if self.future_tf_service.done():
            try:
                response = self.future_tf_service.result()
            except Exception as e:
                self.get_logger().info("Service call failed %r" % (e,))
                self.get_logger().warn(
                    "initial_tf sending to drive failed, retrying..."
                )
                self.future_tf_service = self.intial_tf_to_drive_client.call_async(
                    self.intial_tf_to_drive_request
                )

            else:
                if response.acquittal.data:
                    self.get_logger().info("initial_tf successfuly sent to drive")
                    self.destroy_timer(self.timer_check_tf_service)
                    self.initial_tf_service_called = False
                else:
                    self.get_logger().warn(
                        "initial_tf sending to drive failed, retrying..."
                    )
                    self.future_tf_service = self.intial_tf_to_drive_client.call_async(
                        self.intial_tf_to_drive_request
                    )
            return
        self.get_logger().info("drive initial_tf service cancellation + recall")
        self.future_tf_service.cancel()
        self.future_tf_service = self.intial_tf_to_drive_client.call_async(
            self.intial_tf_to_drive_request
        )

    def allies_subscription_callback(self, msg):
        """Identity the robot marker in assurancetourix marker_array detection,
        send the marker to correct odometry"""
        for ally_marker in msg.markers:
            if ally_marker.text.lower() == self.robot:
                self.create_and_send_tf(
                    ally_marker.pose.position.x,
                    ally_marker.pose.position.y,
                    ally_marker.pose.orientation,
                    ally_marker.header.stamp,
                )

    def create_and_send_tf(self, x, y, q, stamp):
        """Create and send tf to drive for it to re-adjust odometry"""
        self._tf.header.stamp = stamp
        self._tf.transform.translation.x = x
        self._tf.transform.translation.y = y
        self._tf.transform.translation.z = float(0)
        self._tf.transform.rotation = q
        self.last_odom_update = self.get_clock().now().to_msg().sec
        self.tf_publisher_.publish(self._tf)

    def odom_callback(self, msg):
        """Odom callback, computes the new pose of the robot relative to map,
        send this new pose on odom_map_relative topic"""
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
