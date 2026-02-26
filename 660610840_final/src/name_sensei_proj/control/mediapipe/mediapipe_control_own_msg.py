#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np
import mediapipe as mp
import time

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from name_sensei_proj.msg import MecanumCmd

import os
from ament_index_python.packages import get_package_share_directory

class DualHandSmoothControl(Node):

    def __init__(self):
        super().__init__('dual_hand_smooth_control')

        # ==========================================================
        # ⚙️ CONFIGURABLE VARIABLES (คงเดิมทั้งหมด)
        # ==========================================================
        self.min_linear = 0.1
        self.max_linear = 1.0
        self.linear_speed = 0.15
        self.min_angular = 0.2
        self.max_angular = 2.0
        self.angular_speed = 1.0 
        self.rot_gain = 12.0           
        self.rot_ui_gain = 20.0        
        self.alpha = 0.25              
        self.deadzone = 0.15           
        self.joy_radius = 0.13         
        self.latch_duration = 0.135    
        self.latch_speed = 0.18        
        self.latch_trigger_dist = 0.4  
        self.lin_adj_sensitivity = 0.1 
        self.ang_adj_sensitivity = 0.2 
        self.pinch_start_time_cooldown = 1.0
        self.claw_start_time_cooldown = 1.0
        self.pinch_threshold = 0.1
        self.pinch_tolerance = 1.7     
        self.pinch_joint_dist = 0.16
        self.pinch_middle_gap = 0.05
        self.claw_threshold = 0.125
        self.claw_tolerance = 0.4      
        self.claw_index_mid_gap = 0.08
        self.claw_thumb_idx_gap = 0.10
        self.fold_trigger = 0.025      

        # UI Colors
        self.ui_bg_color = (40, 40, 40)
        self.ui_telemetry_bg = (20, 20, 20)
        self.ui_lin_color = (0, 255, 255)
        self.ui_ang_color = (255, 255, 0)
        self.ui_turn_left_color = (255, 0, 255)
        self.ui_turn_right_color = (0, 255, 255)
        self.ui_joy_edge_color = (255, 255, 0)
        self.ui_anchor_color = (255, 255, 255)
        self.ui_bar_width = 150

        # ==========================================================
        # 🛠️ INTERNAL SYSTEM
        # ==========================================================
        self.cmd_pub = self.create_publisher(MecanumCmd, '/cmd_vel_control', qos_profile_sensor_data)
        self.alert_sub = self.create_subscription(String, '/obstacle_alert', self.alert_callback, 10)
        self.mode_sub = self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        
        self.last_alert_msg = ""
        self.last_alert_time = 0
        self.current_system_mode = "INITIALIZING..."
        self.filtered_x = self.filtered_y = self.filtered_z = 0.0
        self.joy_center = None
        self.pinch_start_time = self.claw_start_time = self.latch_start_time = None
        self.speed_adjust_enabled = self.ang_speed_adjust_enabled = False
        self.last_pinch_y = self.last_claw_y = None
        self.draw_pinch_ref = self.draw_pinch_curr = None
        self.draw_claw_ref = self.draw_claw_curr = None
        
        package_share_directory = get_package_share_directory('name_sensei_proj')
        model_path = os.path.join(package_share_directory, 'models', 'hand_landmarker.task')
        base_options = python.BaseOptions(model_asset_path=model_path, delegate=python.BaseOptions.Delegate.CPU)
        options = vision.HandLandmarkerOptions(base_options=base_options, num_hands=2, running_mode=vision.RunningMode.VIDEO)
        self.detector = vision.HandLandmarker.create_from_options(options)
        
        self.cap = cv2.VideoCapture(0)
        self.get_logger().info("Mirror Mode: Fixed Left Hand UI & Original Boxes Position")

    def alert_callback(self, msg):
        self.last_alert_msg = msg.data
        self.last_alert_time = time.time()

    def mode_callback(self, msg):
        self.current_system_mode = msg.data

    def smooth(self, target, current):
        return self.alpha * target + (1 - self.alpha) * current

    def is_pinch(self, hand):
        dist_tips = np.sqrt((hand[4].x - hand[8].x)**2 + (hand[4].y - hand[8].y)**2)
        dist_joints = np.sqrt((hand[3].x - hand[6].x)**2 + (hand[3].y - hand[6].y)**2)
        dist_middle = np.sqrt((hand[8].x - hand[12].x)**2 + (hand[8].y - hand[12].y)**2)
        thresh = self.pinch_threshold * self.pinch_tolerance if self.speed_adjust_enabled else self.pinch_threshold
        return dist_tips < thresh and dist_joints < self.pinch_joint_dist and dist_middle > self.pinch_middle_gap
    
    def is_claw(self, hand):
        thresh = self.claw_threshold * self.claw_tolerance if self.ang_speed_adjust_enabled else self.claw_threshold
        dist_stretch = np.sqrt((hand[4].x - hand[20].x)**2 + (hand[4].y - hand[20].y)**2)
        dist_index_middle = np.sqrt((hand[8].x - hand[12].x)**2 + (hand[8].y - hand[12].y)**2)
        dist_thumb_index = np.sqrt((hand[4].x - hand[8].x)**2 + (hand[4].y - hand[8].y)**2)
        return dist_stretch > thresh and dist_index_middle > self.claw_index_mid_gap and dist_thumb_index > self.claw_thumb_idx_gap

    def is_fist(self, hand):
        return (hand[8].y > hand[6].y and hand[12].y > hand[10].y and hand[16].y > hand[14].y)

    def run(self):
        while rclpy.ok() and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret: break
            self.process_frame(frame)
            rclpy.spin_once(self, timeout_sec=0)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    def process_frame(self, frame):
        timestamp_ms = int(time.time() * 1000)
        
        # 1. Flip Frame ทันที
        frame = cv2.flip(frame, 1)
        
        small_w, small_h = 640, 480
        small_frame = cv2.resize(frame, (small_w, small_h)) 
        rgb_small = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_small)
        result = self.detector.detect_for_video(mp_image, timestamp_ms)

        frame = cv2.resize(frame, (1280, 720))
        h, w, _ = frame.shape
        target_x, target_y, target_z = 0.0, 0.0, 0.0
        right_hand, left_hand = None, None

        if result.hand_landmarks:
            for idx, hand in enumerate(result.hand_landmarks):
                handedness = result.handedness[idx][0].category_name
                # สลับ Logic รับค่ามือ (Mirror)
                if handedness == "Left": right_hand = hand
                elif handedness == "Right": left_hand = hand

        # --- RIGHT HAND (Joystick) ---
        if right_hand:
            if self.is_fist(right_hand):
                target_x = target_y = target_z = 0.0
                self.filtered_x = self.filtered_y = self.filtered_z = 0.0
                self.joy_center = self.latch_start_time = None
                cv2.putText(frame, "EMERGENCY STOP", (int(w/2)-180, h-60), 0, 1.2, (0, 0, 255), 3)
            else:
                idx_tip_r = right_hand[8]
                current_time = time.time()
                if self.joy_center is None: self.joy_center = (idx_tip_r.x, idx_tip_r.y)
                
                dx_raw = -(idx_tip_r.x - self.joy_center[0])
                dy_raw = idx_tip_r.y - self.joy_center[1]
                dist_raw = np.sqrt(dx_raw**2 + dy_raw**2)

                fold_l = right_hand[12].y - right_hand[9].y
                fold_r = right_hand[16].y - right_hand[13].y
                is_turning = False
                if fold_l > self.fold_trigger and fold_r > self.fold_trigger: target_z = 0.0
                elif fold_l > self.fold_trigger: target_z, is_turning = fold_l * self.angular_speed * self.rot_gain, True
                elif fold_r > self.fold_trigger: target_z, is_turning = -fold_r * self.angular_speed * self.rot_gain, True

                if is_turning:
                    if self.latch_start_time is None: self.latch_start_time = current_time
                    if (current_time - self.latch_start_time) < self.latch_duration and dist_raw < (self.joy_radius * self.latch_trigger_dist):
                        self.joy_center = (self.joy_center[0] * (1-self.latch_speed) + idx_tip_r.x * self.latch_speed, self.joy_center[1] * (1-self.latch_speed) + idx_tip_r.y * self.latch_speed)
                        cv2.putText(frame, "LATCHING...", (int(idx_tip_r.x*w)+20, int(idx_tip_r.y*h)), 0, 0.5, (0, 255, 255), 1)
                else: self.latch_start_time = None

                dx, dy = -(idx_tip_r.x - self.joy_center[0]), idx_tip_r.y - self.joy_center[1]
                dist = np.sqrt(dx**2 + dy**2)
                if dist > self.joy_radius: scale = self.joy_radius / dist; dx *= scale; dy *= scale
                norm_x, norm_y = dx/self.joy_radius, dy/self.joy_radius
                if abs(norm_x) < self.deadzone: norm_x = 0
                if abs(norm_y) < self.deadzone: norm_y = 0
                target_x, target_y = -norm_y * self.linear_speed, -norm_x * self.linear_speed

                # Draw Joystick
                cx, cy = int(self.joy_center[0]*w), int(self.joy_center[1]*h)
                joy_px_radius = int(self.joy_radius * w)
                cv2.circle(frame, (cx, cy), joy_px_radius, self.ui_joy_edge_color, 1)
                if is_turning:
                    start_a = -90; end_a = start_a - (target_z * self.rot_ui_gain)
                    color = self.ui_turn_left_color if target_z > 0 else self.ui_turn_right_color
                    cv2.ellipse(frame, (cx, cy), (joy_px_radius+10, joy_px_radius+10), 0, start_a, end_a, color, 5)
                tip_px = (int(idx_tip_r.x*w), int(idx_tip_r.y*h))
                cv2.line(frame, (cx, cy), tip_px, (0, 255, 0), 2); cv2.circle(frame, tip_px, 10, (0, 255, 0), -1)
        else: self.joy_center = None

        # --- LEFT HAND (Adjustments) ---
        if left_hand:
            current_time = time.time()
            if self.is_fist(left_hand):
                self.pinch_start_time = self.claw_start_time = None
                self.speed_adjust_enabled = self.ang_speed_adjust_enabled = False
                self.draw_pinch_ref = self.draw_pinch_curr = self.draw_claw_ref = self.draw_claw_curr = None
                cv2.putText(frame, "LEFT HAND LOCKED", (20, h-80), 0, 0.6, (0, 0, 255), 2)
                
            px_l, py_l = int(left_hand[8].x * w), int(left_hand[8].y * h)
            px_w, py_w = int(left_hand[0].x * w), int(left_hand[0].y * h)

            if self.is_pinch(left_hand) and not self.is_claw(left_hand):
                if self.pinch_start_time is None: self.pinch_start_time, self.last_pinch_y = current_time, left_hand[8].y
                self.draw_pinch_ref, self.draw_pinch_curr = (px_l, int(self.last_pinch_y * h)), (px_l, py_l)
                if (current_time - self.pinch_start_time) < self.pinch_start_time_cooldown:
                    prog = (current_time - self.pinch_start_time) / self.pinch_start_time_cooldown
                    cv2.ellipse(frame, (px_l, py_l), (20, 20), 0, 0, int(prog * 360), self.ui_anchor_color, 2)
                else:
                    self.speed_adjust_enabled = True
                    dy = self.last_pinch_y - left_hand[8].y
                    self.linear_speed = np.clip(self.linear_speed + dy * self.lin_adj_sensitivity, self.min_linear, self.max_linear)
            else: self.pinch_start_time = None; self.speed_adjust_enabled = False; self.draw_pinch_ref = self.draw_pinch_curr = None

            if self.is_claw(left_hand):
                if self.claw_start_time is None: self.claw_start_time, self.last_claw_y = current_time, left_hand[0].y
                self.draw_claw_ref, self.draw_claw_curr = (px_w, int(self.last_claw_y * h)), (px_w, py_w)
                if (current_time - self.claw_start_time) < self.claw_start_time_cooldown:
                    prog = (current_time - self.claw_start_time) / self.claw_start_time_cooldown
                    cv2.ellipse(frame, (px_w, py_w), (25, 25), 0, 0, int(prog * 360), self.ui_ang_color, 2)
                else:
                    self.ang_speed_adjust_enabled = True
                    dy = self.last_claw_y - left_hand[0].y
                    self.angular_speed = np.clip(self.angular_speed + dy * self.ang_adj_sensitivity, self.min_angular, self.max_angular)
            else: self.claw_start_time = None; self.ang_speed_adjust_enabled = False; self.draw_claw_ref = self.draw_claw_curr = None

        # --- Smoothing & Command ---
        self.filtered_x, self.filtered_y, self.filtered_z = self.smooth(target_x, self.filtered_x), self.smooth(-target_y, self.filtered_y), self.smooth(target_z, self.filtered_z)
        cmd = MecanumCmd(); cmd.x, cmd.y, cmd.omega_z = float(self.filtered_x), float(self.filtered_y), float(self.filtered_z)
        self.cmd_pub.publish(cmd)

        # ==========================================================
        # 🎨 BOXES RE-POSITIONED TO ORIGINAL SIDES
        # ==========================================================
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 60), (20, 20, 20), -1) 
        # กล่องซ้าย (Settings) กลับมาอยู่ที่เดิม
        cv2.rectangle(overlay, (15, 75), (260, 230), self.ui_bg_color, -1) 
        # กล่องขวา (Telemetry) กลับมาอยู่ที่เดิม
        cv2.rectangle(overlay, (w - 275, 75), (w - 15, 230), self.ui_telemetry_bg, -1) 
        frame = cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)

        # Status
        mode_color = (0, 255, 0) # เขียว (Manual)
        if "AUTO" in self.current_system_mode: mode_color = (0, 165, 255) # ส้ม (LIDAR)
        if "LOCKED" in self.current_system_mode: mode_color = (0, 0, 255) # แดง (Lock)
        cv2.putText(frame, f"STATUS: {self.current_system_mode}", (25, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)

        # 1. Left Side: Control Settings (เหมือนโค้ดเดิม)
        ty_speed = 105
        cv2.putText(frame, "CONTROL SETTINGS", (30, ty_speed), 0, 0.6, (200, 200, 200), 2)
        cv2.putText(frame, f"LIN Limit: {self.linear_speed:.2f}", (30, ty_speed + 35), 0, 0.5, self.ui_lin_color, 1)
        l_bar = int((self.linear_speed / self.max_linear) * self.ui_bar_width)
        cv2.rectangle(frame, (30, ty_speed + 45), (30 + self.ui_bar_width, ty_speed + 55), (100, 100, 100), 1)
        cv2.rectangle(frame, (30, ty_speed + 45), (30 + l_bar, ty_speed + 55), self.ui_lin_color, -1)
        cv2.putText(frame, f"ANG Limit: {self.angular_speed:.2f}", (30, ty_speed + 85), 0, 0.5, self.ui_ang_color, 1)
        a_bar = int((self.angular_speed / self.max_angular) * self.ui_bar_width)
        cv2.rectangle(frame, (30, ty_speed + 95), (30 + self.ui_bar_width, ty_speed + 105), (100, 100, 100), 1)
        cv2.rectangle(frame, (30, ty_speed + 95), (30 + a_bar, ty_speed + 105), self.ui_ang_color, -1)

        # 2. Right Side: Live Telemetry (เหมือนโค้ดเดิม)
        tx_tele = w - 260
        cv2.putText(frame, "LIVE TELEMETRY", (tx_tele, 105), 0, 0.6, (200, 200, 200), 2)
        cv2.putText(frame, f"X (Linear):  {self.filtered_x:+.3f}", (tx_tele, 145), 0, 0.6, (100, 255, 100), 2)
        cv2.putText(frame, f"Y (Lateral): {self.filtered_y:+.3f}", (tx_tele, 180), 0, 0.6, (100, 255, 255), 2)
        cv2.putText(frame, f"Z (Angular): {self.filtered_z:+.3f}", (tx_tele, 215), 0, 0.6, (150, 150, 255), 2)

        # 3. Hand Guides (Fixed for Mirror)
        if self.draw_pinch_ref and self.pinch_start_time:
            prx, pry = self.draw_pinch_ref; pcx, pcy = self.draw_pinch_curr
            if self.speed_adjust_enabled:
                cv2.line(frame, (pcx - 50, pry), (pcx + 50, pry), (0, 255, 0), 3)
                cv2.line(frame, (pcx, pcy), (pcx, pry), (255, 255, 255), 1)
                cv2.circle(frame, (pcx, pcy), 12, (255, 255, 255), -1)
                cv2.putText(frame, "LIN ADJ", (pcx - 30, pcy - 25), 0, 0.5, (0, 255, 0), 2)
            else: cv2.circle(frame, (prx, pry), 4, self.ui_anchor_color, -1)

        if self.draw_claw_ref and self.claw_start_time:
            rx, ry = self.draw_claw_ref; cx, cy = self.draw_claw_curr
            if self.ang_speed_adjust_enabled:
                cv2.line(frame, (rx - 50, ry), (rx + 50, ry), (255, 150, 0), 3)
                cv2.line(frame, (rx, ry), (cx, cy), (255, 255, 255), 1)
                cv2.circle(frame, (cx, cy), 12, (255, 255, 0), -1)
                cv2.putText(frame, "ANG ADJ", (cx - 30, cy - 25), 0, 0.5, (255, 255, 0), 2)
            else: cv2.circle(frame, (rx, ry), 4, self.ui_ang_color, -1)

        # 4. Alert
        if time.time() - self.last_alert_time < 1.5:
            cv2.rectangle(frame, (0, h-50), (w, h), (0, 0, 180), -1)
            cv2.putText(frame, f"SENSOR ALERT: {self.last_alert_msg}", (40, h-18), 0, 0.7, (0,0,0), 3)
            cv2.putText(frame, f"SENSOR ALERT: {self.last_alert_msg}", (40, h-18), 0, 0.7, (255,255,255), 2)

        cv2.imshow("Dual Hand Control - Fixed Mirror UI", frame)

def main():
    rclpy.init(); node = DualHandSmoothControl()
    try: node.run()
    except KeyboardInterrupt: pass
    finally: node.cap.release(); cv2.destroyAllWindows(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()