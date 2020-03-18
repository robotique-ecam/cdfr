#!/usr/bin/env python3


import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from cetautomatix.magic_points import elements
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Goal


class Robot(Node):
    def __init__(self):
        super().__init__(node_name='robot')
        self.objective = "ARUCO42"
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info('I heard: "%s"' % msg.data)

    def compute_best_action(self, list):
        action_coeff_list = []
        # start = timeEndOfGame.time - 100.0
        for action in list:
            for key, value in elements.items():
                if key == action:
                    distance = np.sqrt(
                        (value[0] - self.position[0])**2 + (value[1] - self.position[1])**2)
                    coeff_distance = distance * 100 / 3.6
                    action_coeff_list.append((key, coeff_distance))
                    break
        max = 0
        best_action = None
        for action in action_coeff_list:
            if action[1] > max:
                max = action[1]
                best_action = action[0]
        self.objective = best_action

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - \
            np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + \
            np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - \
            np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + \
            np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]

    def getGoalPose(self):
        msg = NavigateToPose_Goal()
        value = elements[self.objective]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.position.x = value[0]
        msg.pose.pose.position.y = value[1]
        q = self.euler_to_quaternion(value[2] if value[2] is not None else 0, 0, 0)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        return msg
