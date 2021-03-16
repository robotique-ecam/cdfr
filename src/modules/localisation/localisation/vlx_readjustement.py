#!/usr/bin/env python3


"""Vlx readjustement for localisation"""

from localisation.sensors_sim import Sensors


class VlxReadjustement:
    def __init__(self, parent_node):
        self.parent = parent_node
        self.sensors = Sensors(parent_node, [30, 31, 32, 33, 34, 35])
        self.parent.create_timer(1, self.testVlx)
        self.vlx_readjustement = false

    def testVlx(self):
        values = self.sensors.get_distances()
        self.parent.get_logger().info(
            f"{values[30]}, {values[31]}, {values[32]}, {values[33]}, {values[34]}, {values[35]}, {self.parent.robot_pose.pose.position.x}"
        )
