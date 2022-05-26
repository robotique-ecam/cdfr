from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from PyKDL import Rotation
from tf2_kdl import transform_to_kdl, do_transform_frame
import rclpy
import numpy as np

from rclpy.action import ActionClient
from rclpy.node import Node


# Class taken from https://github.com/ros-planning/navigation2/issues/2283#issuecomment-853456150
class Navigator(Node):
    def __init__(self):
        super().__init__(node_name="navigator")
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info(
            "Navigating to goal: "
            + str(pose.pose.position.x)
            + " "
            + str(pose.pose.position.y)
            + "..."
        )
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(
                "Goal to "
                + str(pose.pose.position.x)
                + " "
                + str(pose.pose.position.y)
                + " was rejected!"
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info("Canceling current goal.")
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info("Goal failed with status code: {0}".format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info("Goal succeeded!")
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def navigate_to_point(position, orientation, z=0.0):
    navigator = Navigator()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = position[0]
    goal_pose.pose.position.y = position[1]
    goal_pose.pose.position.z = z
    q = Rotation.RotZ(np.deg2rad(orientation)).GetQuaternion()
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]
    navigator.goToPose(goal_pose)
    
    navigator.info(f"Goal objective updated to:")
    navigator.info(f"Position x: {position[0]}, y: {position[1]}")
    navigator.info(f"Orientation {orientation} degrees")

    while not navigator.isNavComplete():
        pass

def get_transformed_position(object_position, object_orientation, robot_to_actuator):
    """"Object position in (x, y), orientation in degrees and robot_to_actuator in (x, y)"""
    actuator_to_robot = TransformStamped()
    actuator_to_robot.transform.translation.x = -robot_to_actuator[0]
    actuator_to_robot.transform.translation.y = -robot_to_actuator[1]

    goal_pose = TransformStamped()
    goal_pose.transform.translation.x = object_position[0]
    goal_pose.transform.translation.y = object_position[1]

    rot = Rotation.RotZ(np.deg2rad(object_orientation))

    q = rot.GetQuaternion()

    goal_pose.transform.rotation.x = q[0]
    goal_pose.transform.rotation.y = q[1]
    goal_pose.transform.rotation.z = q[2]
    goal_pose.transform.rotation.w = q[3]

    actuator_to_robot_kdl = transform_to_kdl(actuator_to_robot)

    robot_goal_pose_kdl = do_transform_frame(actuator_to_robot_kdl, goal_pose)

    return (robot_goal_pose_kdl.p.x(), robot_goal_pose_kdl.p.y())