#!/usr/bin/env python3


import sys

import rclpy
import py_trees_ros
import py_trees.console as console
from cetautomatix.robot import Robot
from cetautomatix.tree import create_tree
from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    root = create_tree(robot)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )
    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red + 'failed to setup the tree, aborting [{}]'.format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror('tree setup interrupted')
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=100)

    executor = SingleThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(tree.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    tree.node.destroy_node()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
