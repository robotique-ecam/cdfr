#!/usr/bin/env python3


"""Teb_obstacles localisation node."""


import rclpy
import copy

from rclpy.node import Node
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32, PoseStamped
from platform import machine


class Teb_obstacles(Node):
    def __init__(self):
        super().__init__("teb_dynamic_obstacles_node")
        self.simulation = True if machine() != "aarch64" else False

        self.ally = (
            "obelix" if self.get_namespace().strip("/") == "asterix" else "asterix"
        )

        self.allies_subscription_ = self.create_subscription(
            PoseStamped,
            f"/{self.ally}/odom_map_relative",
            self.allies_subscription_callback,
            10,
        )
        self.enemies_subscription_ = self.create_subscription(
            MarkerArray,
            "/ennemies_positions_markers",
            self.enemies_subscription_callback,
            10,
        )

        self.obstacles_publisher_ = self.create_publisher(
            ObstacleArrayMsg, "obstacles", 10
        )

        self.enemies_markers_ids = [0, 0]

        self.last_time_ally_callback = self.get_clock().now().to_msg()

        self.initObstaclesArray()

        self.create_timer(0.5, self.send_obstacles)

        self.get_logger().info("teb_dynamic_obstacles node is ready")

    def initObstaclesArray(self):
        """ObstacleArray index 0: ally, index 1-2: enemies"""
        self.obstacles = ObstacleArrayMsg()
        self.obstacles.header.frame_id = "map"
        self.obstacles.obstacles.append(ObstacleMsg())
        self.obstacles.obstacles.append(ObstacleMsg())
        self.obstacles.obstacles.append(ObstacleMsg())
        for i in range(3):
            self.obstacles.obstacles[i].header.frame_id = "map"
            self.obstacles.obstacles[i].polygon.points = [Point32()]
            self.obstacles.obstacles[i].polygon.points[0].x = -1.0
            self.obstacles.obstacles[i].polygon.points[0].y = -1.0
            self.obstacles.obstacles[i].radius = 0.15
        self.previous_obstacles = copy.deepcopy(self.obstacles)

    def get_diff_time(self, t1, t2):
        """Returns the nb of seconds between the two Time object"""
        return t1.sec - t2.sec + (t1.nanosec - t2.nanosec) * 1e-9

    def set_obstacle(self, index, marker):
        """Set the marker as obstacle in ObstacleArrayMsg at the given index,
        compute the linear velocities relative to the previous state"""
        self.previous_obstacles.obstacles[index] = copy.deepcopy(
            self.obstacles.obstacles[index]
        )
        self.obstacles.obstacles[index].header = marker.header
        self.obstacles.obstacles[index].polygon.points[0].x = marker.pose.position.x
        self.obstacles.obstacles[index].polygon.points[0].y = marker.pose.position.y

        dt = float(
            self.get_diff_time(
                marker.header.stamp,
                self.previous_obstacles.obstacles[index].header.stamp,
            )
        )

        if dt != 0.0:
            self.obstacles.obstacles[index].velocities.twist.linear.x = (
                marker.pose.position.x
                - self.previous_obstacles.obstacles[index].polygon.points[0].x
            ) / dt

            self.obstacles.obstacles[index].velocities.twist.linear.y = (
                marker.pose.position.y
                - self.previous_obstacles.obstacles[index].polygon.points[0].y
            ) / dt

    def allies_subscription_callback(self, msg):
        """Determine the pose of base_link in map
        set the dynamic obstacle for teb_local_planner"""
        if (
            self.get_diff_time(
                self.get_clock().now().to_msg(), self.last_time_ally_callback
            )
            > 0.3
        ):
            x = msg.pose.position.x
            y = msg.pose.position.y
            tmp_marker = Marker()
            tmp_marker.pose.position.x = x
            tmp_marker.pose.position.y = y
            self.set_obstacle(0, tmp_marker)
            self.last_time_ally_callback = self.get_clock().now().to_msg()

    def enemies_subscription_callback(self, msg):
        """Identify the enemy marker in assurancetourix marker_array detection
        set the dynamic obstacle for teb_local_planner"""
        for enemy_marker in msg.markers:
            if enemy_marker.id <= 10:  # >10 are predicted markers
                marker_stored = False
                for index in range(2):
                    if self.enemies_markers_ids[index] == enemy_marker.id:
                        self.set_obstacle(index + 1, enemy_marker)
                        marker_stored = True
                if not marker_stored:
                    if self.enemies_markers_ids[0] == 0:
                        self.enemies_markers_ids[0] = enemy_marker.id
                    elif self.enemies_markers_ids[1] == 0:
                        self.enemies_markers_ids[1] = enemy_marker.id
                    else:
                        self.get_logger().info(
                            "obstacleArray index limit, are there 3 enemies, or is it a bad marker detection"
                        )

    def send_obstacles(self):
        self.obstacles_publisher_.publish(self.obstacles)


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    teb_obstacles = Teb_obstacles()
    try:
        rclpy.spin(teb_obstacles)
    except KeyboardInterrupt:
        pass
    teb_obstacles.destroy_node()
    rclpy.shutdown()
