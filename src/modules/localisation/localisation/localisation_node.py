#!/usr/bin/env python3


"""Robot localisation node."""


from transforms3d.euler import euler2quat

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import StaticTransformBroadcaster


class Localisation(rclpy.node.Node):
    """Robot localisation node."""

    def __init__(self):
        super().__init__('localisation_node')
        self.robot = self.get_namespace().strip('/')
        self.side = self.declare_parameter('side', 'blue')
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._x, self._y, self._theta = self.fetchStartPosition()
        self._tf_brodcaster = StaticTransformBroadcaster(self)
        self._tf = TransformStamped()
        self._tf.header.frame_id = 'map'
        self._tf.child_frame_id = 'odom'
        self.create_timer(1, self.update_transform)
        self.get_logger().info(f'Default side is {self.side.value}')
        self.get_logger().info('Localisation node is ready')

    def _on_set_parameters(self, params):
        """Handle Parameter events especially for side."""
        result = SetParametersResult()
        try:
            for param in params:
                if param.name == 'side':
                    self.get_logger().warn(f'Side changed {param.value}')
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
        print(self.robot, self.side)
        if self.robot == 'asterix':
            if self.side.value == 'blue':
                return (0.29, 1.33, 0)
            elif self.side.value == 'yellow':
                return (0.29, 1.33, 0)
        elif self.robot == 'obelix':
            if self.side.value == 'blue':
                return (0.29, 1.33, 0)
            elif self.side.value == 'yellow':
                return (0.29, 1.33, 0)
        # Make it crash in case of undefined parameters
        return None

    def update_transform(self):
        """Update and publish transform callback."""
        self._tf.header.stamp = self.get_clock().now().to_msg()
        self._tf.transform.translation.x = float(self._x)
        self._tf.transform.translation.y = float(self._y)
        qx, qy, qz, qw = euler2quat(0, 0, self._theta, 'rxyz')
        self._tf.transform.rotation.x = float(qx)
        self._tf.transform.rotation.y = float(qy)
        self._tf.transform.rotation.z = float(qz)
        self._tf.transform.rotation.w = float(qw)
        self._tf_brodcaster.sendTransform(self._tf)


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
