#!/usr/bin/env python3


"""Some utils functions"""

import numpy as np
import math

from geometry_msgs.msg import Quaternion
from platform import machine


def euler_to_quaternion(yaw, pitch=0, roll=0):
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
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def quaternion_to_euler(q):
    """Conversion between quaternions and euler angles.

    :param q: The quaternion to transform to euler angles
    :type q: geometry_msgs.msg.Quaternion
    :return: euler angles equivalent
    :rtype: tuple (x,y,z)
    """
    x, y, z, w = q.x, q.y, q.z, q.w
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


def in_rectangle(rect, pose):
    """Return true if the coord are inside the rect coordinates

    :param rect: The list of rect coordinates.
    :type rect: array [x_min, y_min, x_max_ y_max]
    :param pose: the pose to test
    :type coord: geometry_msgs.msg.PoseStamped
    :return: true if the coord are inside the rect coordinates, else false
    :rtype: bool
    """
    x = pose.pose.position.x
    y = pose.pose.position.y
    return rect[0] < x and rect[1] < y and rect[2] > x and rect[3] > y


def is_simulation():
    return True if machine() != "aarch64" else False
