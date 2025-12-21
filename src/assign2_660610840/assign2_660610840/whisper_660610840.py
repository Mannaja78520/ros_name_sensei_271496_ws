#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Whisper_660610840(Node):

    def __init__(self):
        super().__init__('whisper_660610840')
        self.publisher_ = self.create_publisher(String, '/gossip_660610840', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Oh My ROS, I am 660610840: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('PUB whisper: "%s"' % msg.data)
        self.i += 2


def main(args=None):
    rclpy.init(args=args)

    whisper_660610840 = Whisper_660610840()

    rclpy.spin(whisper_660610840)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    whisper_660610840.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()