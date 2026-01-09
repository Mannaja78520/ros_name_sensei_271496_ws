#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import math

class Lidar_scan_660610840(Node):

    def __init__(self):
        super().__init__('lidar_scan_660610840')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.yaw_deg = 180.0
        
    def rotate_ranges(self, msg, yaw_deg):
        ranges = list(msg.ranges)
        yaw_rad = math.radians(yaw_deg)

        # คำนวณจำนวน index ที่ต้อง shift
        shift = int(yaw_rad / msg.angle_increment)

        # ทำให้ shift อยู่ในช่วง 0..N-1
        N = len(ranges)
        shift = shift % N

        rotated_ranges = ranges[-shift:] + ranges[:-shift]
        return rotated_ranges

    def scan_callback(self, msg):
        
        rotated_ranges = self.rotate_ranges(msg, self.yaw_deg)

        # ช่วงมุมที่ต้องการ (0 ± 5°)
        angle_start = math.radians(-5.0)
        angle_end   = math.radians(5.0)

        start_index = int((angle_start - msg.angle_min) / msg.angle_increment)
        end_index   = int((angle_end   - msg.angle_min) / msg.angle_increment)

        start_index = max(0, start_index)
        end_index   = min(len(rotated_ranges)-1, end_index)

        selected_ranges = rotated_ranges[start_index:end_index+1]
        valid_ranges = [r for r in selected_ranges if math.isfinite(r) and r > 0.0]


        msg_out:String = f''
            
        if valid_ranges:
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            msg_out = f"Average distance (0° ± 5°): {avg_distance:.3f} m"
        else:
            msg_out = f"No valid distance values in (0° ± 5°)"
        
        self.get_logger().info('LIDAR SCAN DATA: \n'
            f'angle minimum: {msg.angle_min} \n'
            f'angle maximum: {msg.angle_max} \n'
            f'range minimum: {msg.range_min} \n'
            f'range maximum: {msg.range_max} \n'
            f'{msg_out}'
            )


def main(args=None):
    rclpy.init(args=args)
    lidar_scan_660610840 = Lidar_scan_660610840()
    rclpy.spin(lidar_scan_660610840)
    lidar_scan_660610840.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
