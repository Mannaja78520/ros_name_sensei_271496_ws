#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Whisper_660610840(Node):

    def __init__(self):
        super().__init__('whisper_660610840')

        # ===== Declare parameters with default values =====
        self.declare_parameter('who', 'mannaja')
        self.declare_parameter('speaking', 'shoutout')
        self.declare_parameter('spk_msg', 'Successful assignment')

        # ===== Get parameter values =====
        self.who = self.get_parameter('who').get_parameter_value().string_value
        self.speaking = self.get_parameter('speaking').get_parameter_value().string_value
        self.spk_msg = self.get_parameter('spk_msg').get_parameter_value().string_value

        # ===== Publisher =====
        self.publisher = self.create_publisher(
            String,
            '/gossip_660610840',
            10
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.who = self.get_parameter('who').get_parameter_value().string_value
        self.speaking = self.get_parameter('speaking').get_parameter_value().string_value
        self.spk_msg = self.get_parameter('spk_msg').get_parameter_value().string_value
        
        msg = String()

        if (self.who == 'mannaja' and self.speaking == 'shoutout' and self.spk_msg == 'Successful assignment'):
            self.spk_msg = 'Oh My ROS, I am 660610840'
            self.get_logger().info(f'PUB whisper: {self.spk_msg}: {self.i}')
        
        else:                
            self.get_logger().info(
                f'{self.who} {self.speaking} {self.spk_msg}: {self.i}'
            )

        msg.data = f'{self.spk_msg}: {self.i}'

        self.publisher.publish(msg)

        self.i += 2


def main(args=None):
    rclpy.init(args=args)

    node = Whisper_660610840()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
