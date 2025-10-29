#!/usr/bin/env python3
"""
Professional Gate Mission Navigator - State Machine Version

This node implements the progressive-approach state machine defined in
the 'professional_gate_navigator' section of the params file.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
import time
import math
import numpy as np
from enum import Enum

class MissionState(Enum):
    SUBMERGING = 1
    SEARCHING = 2
    ACQUISITION = 3
    COARSE_ALIGN = 4
    APPROACH = 5
    FINE_ALIGN = 6
    FINAL_APPROACH = 7
    PASSING = 8
    COMPLETE = 9
    FAILED = 10

class ProfessionalGateNavigator(Node):
    
    def __init__(self):
        # Initialize node with the name that matches the YAML file
        super().__init__('professional_gate_navigator')
        
        # --- Declare ALL parameters from the YAML file ---
        self.declare_parameter('target_depth', -1.5)
        self.declare_parameter('depth_tolerance', 0.15)
        self.declare_parameter('depth_correction_gain', 1.5)
        
        self.declare_parameter('search_speed', 0.25)
        self.declare_parameter('acquisition_speed', 0.20)
        self.declare_parameter('approach_speed', 0.35)
        self.declare_parameter('fine_approach_speed', 0.20)
        self.declare_parameter('passing_speed', 0.50)
        
        self.declare_parameter('acquisition_distance', 10.0)
        self.declare_parameter('coarse_alignment_distance', 6.0)
        self.declare_parameter('approach_distance', 4.0)
        self.declare_parameter('fine_alignment_distance', 2.5)
        self.declare_parameter('final_approach_distance', 1.5)
        self.declare_parameter('passing_distance', 0.8)
        
        self.declare_parameter('coarse_alignment_threshold', 0.35)
        self.declare_parameter('fine_alignment_threshold', 0.12)
        self.declare_parameter('passing_alignment_threshold', 0.08)
        
        self.declare_parameter('yaw_gain_far', 1.2)
        self.declare_parameter('yaw_gain_medium', 1.8)
        self.declare_parameter('yaw_gain_near', 2.5)
        self.declare_parameter('yaw_gain_fine', 3.5)
        
        self.declare_parameter('gate_lost_timeout', 5.0)
        self.declare_parameter('max_search_time', 45.0)
        self.declare_parameter('passing_duration', 6.0)
        self.declare_parameter('min_detection_confidence', 0.6)

        # --- Get all parameters ---
        self.target_depth = self.get_parameter('target_depth').value
        self.depth_tolerance = self.get_parameter('depth_tolerance').value
        self.depth_gain = self.get_parameter('depth_correction_gain').value

        self.search_speed = self.get_parameter('search_speed').value
        self.acquisition_speed = self.get_parameter('acquisition_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.fine_approach_speed = self.get_parameter('fine_approach_speed').value
        self.passing_speed = self.get_parameter('passing_speed').value

        self.d_acquisition = self.get_parameter('acquisition_distance').value
        self.d_coarse = self.get_parameter('coarse_alignment_distance').value
        self.d_approach = self.get_parameter('approach_distance').value
        self.d_fine = self.get_parameter('fine_alignment_distance').value
        self.d_final = self.get_parameter('final_approach_distance').value
        self.d_pass = self.get_parameter('passing_distance').value

        self.a_coarse = self.get_parameter('coarse_alignment_threshold').value
        self.a_fine = self.get_parameter('fine_alignment_threshold').value
        self.a_pass = self.get_parameter('passing_alignment_threshold').value

        self.k_yaw_far = self.get_parameter('yaw_gain_far').value
        self.k_yaw_medium = self.get_parameter('yaw_gain_medium').value
        self.k_yaw_near = self.get_parameter('yaw_gain_near').value
        self.k_yaw_fine = self.get_parameter('yaw_gain_fine').value
        
        self.gate_lost_timeout = self.get_parameter('gate_lost_timeout').value
        self.max_search_time = self.get_parameter('max_search_time').value
        self.passing_duration = self.get_parameter('passing_duration').value
        self.min_confidence = self.get_parameter('min_detection_confidence').value
        
        # --- State Variables ---
        self.state = MissionState.SUBMERGING
        self.current_depth = 0.0
        self.gate_detected = False
        self.gate_distance = 999.0
        self.alignment_error = 0.0
        self.detection_confidence = 0.0
        self.last_detection_time = 0.0
        self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        
        # --- Subscriptions ---
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        self.gate_detected_sub = self.create_subscription(
            Bool, '/gate/detected', self.gate_detected_callback, 10)
        self.alignment_sub = self.create_subscription(
            Float32, '/gate/alignment_error', self.alignment_callback, 10)
        self.distance_sub = self.create_subscription(
            Float32, '/gate/estimated_distance', self.distance_callback, 10)
        self.confidence_sub = self.create_subscription(
            Float32, '/gate/detection_confidence', self.confidence_callback, 10)
        
        # --- Mission Timer ---
        self.mission_timer = self.create_timer(0.1, self.update_mission) # 10 Hz loop
        
        self.get_logger().info('='*50)
        self.get_logger().info('PROFESSIONAL Gate Navigator Initialized.')
        self.get_logger().info(f'Target Depth: {self.target_depth}m, Search Timeout: {self.max_search_time}s')
        self.get_logger().info('='*50)

    # --- Sensor Callbacks ---
    def odom_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z

    def gate_detected_callback(self, msg: Bool):
        self.gate_detected = msg.data
        if self.gate_detected:
            self.last_detection_time = self.get_clock().now().seconds_nanoseconds()[0]

    def alignment_callback(self, msg: Float32):
        self.alignment_error = msg.data

    def distance_callback(self, msg: Float32):
        self.gate_distance = msg.data

    def confidence_callback(self, msg: Float32):
        self.detection_confidence = msg.data

    # --- Helper Functions ---
    def publish_twist(self, x=0.0, y=0.0, z=0.0, az=0.0):
        cmd = Twist()
        if self.state != MissionState.SUBMERGING:
            depth_error = self.target_depth - self.current_depth
            cmd.linear.z = np.clip(depth_error * self.depth_gain, -0.5, 0.5)
        else:
            cmd.linear.z = z
        cmd.linear.x = x
        cmd.linear.y = y
        cmd.angular.z = az
        self.cmd_vel_pub.publish(cmd)

    def publish_status(self, msg: str):
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(msg, throttle_duration_sec=1.0)
        
    def set_state(self, new_state: MissionState):
        self.state = new_state
        self.state_start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info(f"--- Changing state to {new_state.name} ---")

    def get_time_in_state(self):
        # Return time in state in seconds (float)
        return (self.get_clock().now().seconds_nanoseconds()[0] - self.state_start_time) / 1e9

    def is_gate_lost(self):
        time_since_seen = (self.get_clock().now().seconds_nanoseconds()[0] - self.last_detection_time) / 1e9
        return not self.gate_detected and time_since_seen > self.gate_lost_timeout
        
    def get_adaptive_yaw_cmd(self):
        """Selects yaw gain based on distance"""
        if self.gate_distance > self.d_coarse:
            gain = self.k_yaw_far
        elif self.gate_distance > self.d_approach:
            gain = self.k_yaw_medium
        elif self.gate_distance > self.d_fine:
            gain = self.k_yaw_near
        else:
            gain = self.k_yaw_fine
            
        yaw_cmd = -self.alignment_error * gain
        return np.clip(yaw_cmd, -0.8, 0.8)

    # --- Main State Machine Loop (10 Hz) ---
    def update_mission(self):
        
        # Universal gate-lost check
        if self.state not in [MissionState.SUBMERGING, MissionState.SEARCHING, MissionState.COMPLETE, MissionState.FAILED, MissionState.PASSING]:
            if self.is_gate_lost():
                self.get_logger().warn("Gate LOST. Returning to SEARCHING.")
                self.publish_twist()
                self.set_state(MissionState.SEARCHING)
                return

        # State logic
        if self.state == MissionState.SUBMERGING:
            self.publish_status(f"State: SUBMERGING (Depth: {self.current_depth:.2f}m)")
            depth_error = self.target_depth - self.current_depth
            if abs(depth_error) < self.depth_tolerance:
                self.publish_twist()
                self.set_state(MissionState.SEARCHING)
            else:
                z_vel = np.clip(depth_error * self.depth_gain, -0.5, 0.5)
                self.publish_twist(z=z_vel, x=0.1) # Move forward slightly
        
        elif self.state == MissionState.SEARCHING:
            self.publish_status(f"State: SEARCHING (Time: {self.get_time_in_state():.1f}s)")
            if self.gate_detected and self.detection_confidence >= self.min_confidence:
                self.publish_twist()
                self.set_state(MissionState.ACQUISITION)
            elif self.get_time_in_state() > self.max_search_time:
                self.get_logger().error("Search timeout. Mission FAILED.")
                self.publish_twist()
                self.set_state(MissionState.FAILED)
            else:
                self.publish_twist(az=self.search_speed) # Rotate

        elif self.state == MissionState.ACQUISITION:
            self.publish_status(f"State: ACQUISITION (Dist: {self.gate_distance:.2f}m)")
            if self.gate_distance < self.d_acquisition:
                self.set_state(MissionState.COARSE_ALIGN)
            else:
                yaw_cmd = self.get_adaptive_yaw_cmd()
                self.publish_twist(x=self.acquisition_speed, az=yaw_cmd)
        
        elif self.state == MissionState.COARSE_ALIGN:
            self.publish_status(f"State: COARSE ALIGN (Dist: {self.gate_distance:.2f}m, Align: {abs(self.alignment_error):.2f} rad)")
            if self.gate_distance < self.d_coarse and abs(self.alignment_error) < self.a_coarse:
                self.set_state(MissionState.APPROACH)
            else:
                yaw_cmd = self.get_adaptive_yaw_cmd()
                self.publish_twist(x=self.approach_speed, az=yaw_cmd)

        elif self.state == MissionState.APPROACH:
            self.publish_status(f"State: APPROACH (Dist: {self.gate_distance:.2f}m, Align: {abs(self.alignment_error):.2f} rad)")
            if self.gate_distance < self.d_approach:
                self.set_state(MissionState.FINE_ALIGN)
            else:
                yaw_cmd = self.get_adaptive_yaw_cmd()
                self.publish_twist(x=self.approach_speed, az=yaw_cmd)
                
        elif self.state == MissionState.FINE_ALIGN:
            self.publish_status(f"State: FINE ALIGN (Dist: {self.gate_distance:.2f}m, Align: {abs(self.alignment_error):.2f} rad)")
            if self.gate_distance < self.d_fine and abs(self.alignment_error) < self.a_fine:
                self.set_state(MissionState.FINAL_APPROACH)
            else:
                yaw_cmd = self.get_adaptive_yaw_cmd()
                self.publish_twist(x=self.fine_approach_speed, az=yaw_cmd)

        elif self.state == MissionState.FINAL_APPROACH:
            self.publish_status(f"State: FINAL APPROACH (Dist: {self.gate_distance:.2f}m, Align: {abs(self.alignment_error):.2f} rad)")
            if self.gate_distance < self.d_pass and abs(self.alignment_error) < self.a_pass:
                self.set_state(MissionState.PASSING)
            else:
                yaw_cmd = self.get_adaptive_yaw_cmd()
                self.publish_twist(x=self.fine_approach_speed, az=yaw_cmd)

        elif self.state == MissionState.PASSING:
            elapsed = self.get_time_in_state()
            self.publish_status(f"State: PASSING (Time: {elapsed:.1f}s / {self.passing_duration}s)")
            if elapsed > self.passing_duration:
                self.get_logger().info("Gate pass complete. MISSION COMPLETE.")
                self.publish_twist()
                self.set_state(MissionState.COMPLETE)
            else:
                self.publish_twist(x=self.passing_speed)
        
        elif self.state == MissionState.COMPLETE or self.state == MissionState.FAILED:
            status = "COMPLETE" if self.state == MissionState.COMPLETE else "FAILED"
            self.publish_status(f"State: {status}. Halting robot.")
            self.publish_twist()
            self.mission_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = ProfessionalGateNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_twist() # E-Stop
        node.get_logger().info("Shutting down professional gate navigator.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()