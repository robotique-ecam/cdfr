#!/usr/bin/env python3


"""Teb_obstacles localisation node."""


import rclpy

from rclpy.node import Node
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point32

class Teb_obstacles(Node):

    def __init__(self):
        super().__init__("teb_dynamic_obstacles_node")
        self.allie = "obelix" if self.get_namespace().strip("/") == "asterix" else "asterix"
        self.allies_subscription_ = self.create_subscription(
            MarkerArray, '/allies_positions_markers', self.allies_subscription_callback, 10)
        self.ennemies_subscription_ = self.create_subscription(
            MarkerArray, '/ennemies_positions_markers', self.ennemies_subscription_callback, 10)
        self.allies_subscription_
        self.ennemies_subscription_
        self.dictionary_index_id = {"0":0, "1":0, "2":0}
        self.initObstaclesArray()
        self.get_logger().info('teb_dynamic_obstacles node is ready')

    def initObstaclesArray(self):
        """ObstacleArray index 0: allie, index 1-2: ennemies"""
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
    def set_obstacle(self, index, marker):


    def allies_subscription_callback(self, msg):
        """Identity the allie marker in assurancetourix marker_array detection
           set the dynamic obstacle for teb_local_planner"""
        for allie_marker in msg.markers:
            if allie_marker.text.lower() == self.allie:
                if self.dictionary_index_id["0"] == 0:
                    self.dictionary_index_id["0"] = allie_marker.id
                    self.obstacles.obstacles[0].id = self.dictionary_index_id["0"]
                self.set_obstacle(0, allie_marker)

    def ennemies_subscription_callback(self, msg):
        """Identity the ennemie marker in assurancetourix marker_array detection
           set the dynamic obstacle for teb_local_planner"""
        for ennemie_marker in msg.markers:
            if ennemie_marker.id <= 10:
                in_dict = False
                for index in range(1,2):
                    if self.dictionary_index_id[f"{index}"] == ennemie_marker.id:
                        self.set_obstacle(index, ennemie_marker)
                        in_dict = True
                if not in_dict:
                    if self.dictionary_index_id["1"] == 0:
                        self.dictionary_index_id["1"] = ennemie_marker.id
                    elif self.dictionary_index_id["2"] == 0:
                        self.dictionary_index_id["2"] = ennemie_marker.id
                    else:
                        self.get_logger().info('obstacleArray index limit, is there 3 ennemies, or is it a bad marker detection')


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
