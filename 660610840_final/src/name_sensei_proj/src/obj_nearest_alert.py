#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from name_sensei_proj.msg import MecanumCmd
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import math

class ObjNearestAlert(Node):
    def __init__(self):
        super().__init__('obj_nearest_alert')
        
        # --- Settings ---
        self.target_dist = 0.12     # 120mm: ระยะที่หุ่นจะหยุดถอยและรักษาตำแหน่งไว้
        self.alert_dist = 0.15      # 150mm: ระยะเริ่มแจ้งเตือน
        self.stop_threshold = 0.05  # 50mm: ระยะหยุดฉุกเฉิน
        self.kp = 0.5               # ปรับ P-Gain เล็กน้อยเพื่อความนุ่มนวล
        self.alpha = 0.6            
        
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0

        self.self_filter_dist = 0.018 
        self.half_length = 0.311 / 2 
        self.half_width = 0.190 / 2  
        self.lidar_offset_x = 0.070
        self.lidar_offset_y = 0.000

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(
            MecanumCmd, '/cmd_collision', qos_profile_sensor_data)
        self.alert_pub = self.create_publisher(
            String, '/obstacle_alert', 10)

    def scan_callback(self, msg):
        num_readings = len(msg.ranges)
        raw_vx = 0.0
        raw_vy = 0.0
        
        left_min = float('inf')
        right_min = float('inf')
        min_eff_dist = float('inf')
        min_angle_deg = 0.0

        for i in range(num_readings):
            raw_dist = msg.ranges[i]
            if not math.isfinite(raw_dist) or raw_dist <= 0.05:
                continue

            angle_lidar = msg.angle_min + (i * msg.angle_increment)
            lx = raw_dist * math.cos(angle_lidar)
            ly = raw_dist * math.sin(angle_lidar)
            bx = lx + self.lidar_offset_x
            by = ly + self.lidar_offset_y

            dist_from_center = math.sqrt(bx**2 + by**2)
            angle_from_center = math.atan2(by, bx)
            cos_a = abs(math.cos(angle_from_center))
            sin_a = abs(math.sin(angle_from_center))
            dist_to_edge = min(self.half_length / max(cos_a, 1e-6), self.half_width / max(sin_a, 1e-6))
            effective_dist = dist_from_center - dist_to_edge

            if effective_dist < self.self_filter_dist:
                continue

            deg_center = (math.degrees(angle_from_center) + 360) % 360

            if effective_dist < min_eff_dist:
                min_eff_dist = effective_dist
                min_angle_deg = (-math.degrees(angle_from_center) + 360) % 360

            # เก็บค่าด้านซ้ายและขวา
            if 80.0 <= deg_center <= 100.0:
                left_min = min(left_min, effective_dist)
            elif 260.0 <= deg_center <= 280.0:
                right_min = min(right_min, effective_dist)

            # --- แก้ไขจุดนี้: แรงผลักจะทำงานเฉพาะเมื่อระยะน้อยกว่า target_dist เท่านั้น ---
            # เมื่อระยะห่างเท่ากับ 120mmพอดี error จะเป็น 0 ทำให้ raw_vx/vy เป็น 0 (หยุดถอย)
            if effective_dist < self.target_dist:
                error = self.target_dist - effective_dist
                raw_vx -= math.cos(angle_from_center) * error * self.kp
                raw_vy -= math.sin(angle_from_center) * error * self.kp

        # --- Logic การประคองกลาง (Centering) ---
        side_vel_y = 0.0
        in_side_control = False

        # ถ้ามีด้านใดด้านหนึ่งใกล้กว่า 120mm
        if left_min < self.target_dist or right_min < self.target_dist:
            in_side_control = True
            # ใช้ค่า 120mm เป็นตัวตั้งต้น ถ้าฝั่งไหนไกลกว่า 120mm ให้มองว่าปกติ
            l_val = left_min if left_min < self.target_dist else self.target_dist
            r_val = right_min if right_min < self.target_dist else self.target_dist
            
            # ผลักออกจากฝั่งที่ใกล้กว่า เพื่อไปหาจุด 120mm
            side_vel_y = (l_val - r_val) * self.kp
            raw_vx = 0.0 

        target_vx = 0.0 if in_side_control else raw_vx
        target_vy = side_vel_y if in_side_control else raw_vy

        # --- Low-pass Filter ---
        self.smooth_vx = (self.alpha * target_vx) + ((1.0 - self.alpha) * self.smooth_vx)
        self.smooth_vy = (self.alpha * target_vy) + ((1.0 - self.alpha) * self.smooth_vy)

        # --- Alert & Emergency Stop ---
        if min_eff_dist < self.alert_dist:
            alert_msg = String()
            alert_msg.data = f"WARNING! Object {min_eff_dist*1000:.0f}mm at {min_angle_deg:.1f} deg"
            self.alert_pub.publish(alert_msg)

        if (left_min < self.stop_threshold and right_min < self.stop_threshold):
             self.get_logger().warn("PINCHED! Emergency Stop.")
             self.control_mecanum(0.0, 0.0)
        else:
             # ถ้าค่าความเร็วน้อยมาก (ใกล้ถึง 120mm แล้ว) ให้ส่ง 0 ไปเลยเพื่อความนิ่ง
             if abs(self.smooth_vx) < 0.005: self.smooth_vx = 0.0
             if abs(self.smooth_vy) < 0.005: self.smooth_vy = 0.0
             self.control_mecanum(self.smooth_vx, self.smooth_vy)

    def control_mecanum(self, vx, vy):
        move_cmd = MecanumCmd()
        limit = 0.115 
        move_cmd.x = max(min(vx, limit), -limit)
        move_cmd.y = max(min(vy, limit), -limit)
        self.cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObjNearestAlert()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()