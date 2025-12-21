#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Hearer1_660610840(Node):

    def __init__(self):
        super().__init__('hearer1_660610840')
        self.subscription = self.create_subscription(
            String,
            '/gossip_660610840',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('SUB1 get: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    hearer1_660610840 = Hearer1_660610840()

    rclpy.spin(hearer1_660610840)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hearer1_660610840.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
