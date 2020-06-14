#!/usr/bin/env python3

from os import environ

import rclpy
from visualization_msgs.msg import Marker, MarkerArray
from webots_ros2_core.webots_node import WebotsNode
from rclpy.node import Node

environ['WEBOTS_ROBOT_NAME'] = 'game_manager'


class WebotsPositions(WebotsNode):
    """ webots_positions node """

    def __init__(self, args):
        super().__init__('webots_positions', args=None)
        self.init_parameters()

        if self.robots == None:
            self.get_logger().info(f'No elements parsed in Yaml')
        elif self.refresh_frequency != None:
            self.publisher_ = self.create_publisher(MarkerArray, 'webots_positions', 10)
            timer_period = 1 / self.refresh_frequency
            self.timer = self.create_timer(timer_period, self.positions_callback)
            self.get_logger().info('webots_positions node up')

    def init_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robots', None),
                ('refresh_frequency', None)
            ])
        self.refresh_frequency = self.get_parameter('refresh_frequency').value
        self.robots = self.get_parameter('robots').value

    def positions_callback(self):

        array = []

        for robot in self.robots:

            # get position of robot in webots
            x, _, y = self.robot.getFromDef(robot).getPosition()
            y = 2 - y

            #construct the marker and publish it
            position_marker = Marker()
            position_marker.header.frame_id = "map"
            position_marker.header.stamp = self.get_clock().now().to_msg()
            position_marker.pose.position.x = x
            position_marker.pose.position.y = y
            position_marker.text = robot
            array.append(position_marker)

        self.publisher_.publish(MarkerArray(markers = array))

def main(args = None):
    rclpy.init(args=args)

    webots_positions = WebotsPositions(args=args)
    try:
        rclpy.spin(webots_positions)
    except KeyboardInterrupt:
        pass

    webots_positions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
