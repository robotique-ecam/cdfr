import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info('I heard: "%s"' % msg.data)
