#!/usr/bin/env python3
"""
Complete Autonomous Mission Controller with Integrated Gate Navigation
Run ONLY this file (plus gate_detector_node.py for detection)
NO need to run gate_navigator_node.py separately
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
import time
import math
import numpy as np


class AutonomousMissionController(Node):
    
    def __init__(self):
        super().__init__('autonomous_mission_controller')
        
        # ============================================
        # MISSION STATES
        # ============================================
        self.STATE_INIT = 0
        self.STATE_SUBMERGING = 1
        self.STATE_SEARCHING_GATE = 2
        self.STATE_APPROACHING_GATE = 3  # UNIFIED approach state (no separate aligning)
        self.STATE_PASSING_GATE = 4
        self.STATE_GATE_PASSED_CONFIRMATION = 5
        self.STATE_FLARE_EMERGENCY_AVOIDANCE = 6
        self.STATE_SEARCHING_FLARES = 7
        self.STATE_ALIGNING_FLARE = 8
        self.STATE_BUMPING_FLARE = 9
        self.STATE_SEARCHING_DRUMS = 10
        self.STATE_ALIGNING_DRUM = 11
        self.STATE_DROPPING_MARKER = 12
        self.STATE_COMPLETED = 13
        
        self.state = self.STATE_INIT
        self.state_start_time = time.time()
        self.mission_start_time = time.time()
        
        # ============================================
        # GATE NAVIGATION PARAMETERS (from gate_navigator_node.py)
        # ============================================
        self.declare_parameter('gate_target_depth', -1.5)
        self.declare_parameter('gate_depth_tolerance', 0.15)
        self.declare_parameter('gate_depth_gain', 1.5)
        
        # Speed parameters - Optimized for shortest path
        self.declare_parameter('gate_search_speed', 0.3)
        self.declare_parameter('gate_approach_speed_far', 0.6)     # Fast when far
        self.declare_parameter('gate_approach_speed_near', 0.4)    # Slower for sharp turn
        self.declare_parameter('gate_passing_speed', 0.7)
        
        # CRITICAL: Sharp turn distance threshold
        self.declare_parameter('gate_sharp_turn_distance', 1.5)
        
        # Alignment thresholds
        self.declare_parameter('gate_passing_alignment_threshold', 0.15)  # ~8.5 degrees
        
        # Control gains - Two-phase approach
        self.declare_parameter('gate_yaw_gain_far', 0.8)           # Gentle when far
        self.declare_parameter('gate_yaw_gain_sharp', 6.0)         # Aggressive when close
        
        # Flare avoidance
        self.declare_parameter('gate_flare_avoidance_gain', 1.5)
        self.declare_parameter('gate_flare_critical_distance', 3.0)
        self.declare_parameter('gate_emergency_avoidance_duration', 4.0)
        
        # Safety and recovery
        self.declare_parameter('gate_lost_timeout', 5.0)
        self.declare_parameter('gate_max_search_time', 45.0)
        self.declare_parameter('gate_passing_duration', 6.0)
        self.declare_parameter('gate_min_detection_confidence', 0.6)
        
        # Load gate parameters
        self.gate_target_depth = self.get_parameter('gate_target_depth').value
        self.gate_depth_tolerance = self.get_parameter('gate_depth_tolerance').value
        self.gate_depth_gain = self.get_parameter('gate_depth_gain').value
        self.gate_search_speed = self.get_parameter('gate_search_speed').value
        self.gate_approach_speed_far = self.get_parameter('gate_approach_speed_far').value
        self.gate_approach_speed_near = self.get_parameter('gate_approach_speed_near').value
        self.gate_passing_speed = self.get_parameter('gate_passing_speed').value
        self.gate_sharp_turn_distance = self.get_parameter('gate_sharp_turn_distance').value
        self.gate_passing_alignment_threshold = self.get_parameter('gate_passing_alignment_threshold').value
        self.gate_yaw_gain_far = self.get_parameter('gate_yaw_gain_far').value
        self.gate_yaw_gain_sharp = self.get_parameter('gate_yaw_gain_sharp').value
        self.gate_flare_avoidance_gain = self.get_parameter('gate_flare_avoidance_gain').value
        self.gate_flare_critical_distance = self.get_parameter('gate_flare_critical_distance').value
        self.gate_emergency_avoidance_duration = self.get_parameter('gate_emergency_avoidance_duration').value
        self.gate_lost_timeout = self.get_parameter('gate_lost_timeout').value
        self.gate_max_search_time = self.get_parameter('gate_max_search_time').value
        self.gate_passing_duration = self.get_parameter('gate_passing_duration').value
        self.gate_min_confidence = self.get_parameter('gate_min_detection_confidence').value
        
        # ============================================
        # STATE TRACKING
        # ============================================
        self.current_depth = 0.0
        self.current_position = None
        
        # Gate detection state
        self.gate_detected = False
        self.gate_center = None
        self.gate_distance = 999.0
        self.alignment_error = 0.0
        self.detection_confidence = 0.0
        self.gate_last_seen_time = None
        
        # Orange Flare detection state (for avoidance)
        self.orange_flare_detected = False
        self.flare_avoidance_direction = 0.0
        self.flare_warning_level = ""
        self.emergency_avoidance_start_time = None
        
        # Flare bumping task state
        self.target_flare_color = 'red'
        self.flare_bump_order = ['red', 'yellow', 'blue']
        self.flares_bumped = []
        self.red_flare_detected = False
        self.red_flare_pose = PoseStamped()
        self.yellow_flare_detected = False
        self.yellow_flare_pose = PoseStamped()
        self.blue_flare_detected = False
        self.blue_flare_pose = PoseStamped()
        self.flare_detected = False
        self.flare_pose = PoseStamped()
        self.flare_alignment_threshold_px = 25
        
        # Drum task state
        self.blue_drum_detected = False
        self.blue_drum_pose = PoseStamped()
        self.drum_alignment_threshold_px = 20
        self.drop_duration = 5.0
        
        # Search pattern state
        self.search_mode = 'rotating'
        self.search_mode_start_time = time.time()
        self.search_rotate_duration = 10.0
        self.search_forward_duration = 5.0
        
        # ============================================
        # PUBLISHERS
        # ============================================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ============================================
        # SUBSCRIPTIONS
        # ============================================
        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Gate detection (from gate_detector_node.py)
        self.gate_detected_sub = self.create_subscription(
            Bool, '/gate/detected', self.gate_detected_callback, 10)
        self.gate_center_sub = self.create_subscription(
            Point, '/gate/center', self.gate_center_callback, 10)
        self.gate_alignment_sub = self.create_subscription(
            Float32, '/gate/alignment_error', self.gate_alignment_callback, 10)
        self.gate_distance_sub = self.create_subscription(
            Float32, '/gate/estimated_distance', self.gate_distance_callback, 10)
        self.gate_confidence_sub = self.create_subscription(
            Float32, '/gate/detection_confidence', self.gate_confidence_callback, 10)
        
        # Orange flare detection (obstacle avoidance)
        self.flare_detected_sub = self.create_subscription(
            Bool, '/flare/detected', self.orange_flare_detected_callback, 10)
        self.flare_avoidance_sub = self.create_subscription(
            Float32, '/flare/avoidance_direction', self.orange_flare_avoidance_callback, 10)
        self.flare_warning_sub = self.create_subscription(
            String, '/flare/warning', self.flare_warning_callback, 10)
            
        # Bumping Flare Detectors (Task 4)
        self.red_flare_bool_sub = self.create_subscription(
            Bool, '/flare/red_detected_bool', self.red_flare_bool_callback, 10)
        self.red_flare_pose_sub = self.create_subscription(
            PoseStamped, '/flare/red_detected', self.red_flare_pose_callback, 10)
        self.yellow_flare_bool_sub = self.create_subscription(
            Bool, '/flare/yellow_detected_bool', self.yellow_flare_bool_callback, 10)
        self.yellow_flare_pose_sub = self.create_subscription(
            PoseStamped, '/flare/yellow_detected', self.yellow_flare_pose_callback, 10)
        self.blue_flare_bool_sub = self.create_subscription(
            Bool, '/flare/blue_detected_bool', self.blue_flare_bool_callback, 10)
        self.blue_flare_pose_sub = self.create_subscription(
            PoseStamped, '/flare/blue_detected', self.blue_flare_pose_callback, 10)
            
        # Drum Detectors (Task 2)
        self.blue_drum_bool_sub = self.create_subscription(
            Bool, '/drum/blue_detected', self.blue_drum_bool_callback, 10)
        self.blue_drum_pose_sub = self.create_subscription(
            PoseStamped, '/drum/blue_pose', self.blue_drum_pose_callback, 10)

        # Control loop timer
        self.control_loop_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 Complete Autonomous Mission Controller Started!')
        self.get_logger().info('   Gate Navigation: INTEGRATED (No separate node needed)')
        self.get_logger().info('   Detection: Using gate_detector_node.py')
        self.get_logger().info('='*70)

    # ============================================
    # CALLBACKS
    # ============================================
    
    def odom_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = msg.pose.pose.position
    
    # Gate detection callbacks
    def gate_detected_callback(self, msg: Bool):
        prev_detected = self.gate_detected
        self.gate_detected = msg.data
        if self.gate_detected:
            self.gate_last_seen_time = time.time()
            if not prev_detected:
                self.get_logger().info('✓ Gate acquired!')
    
    def gate_center_callback(self, msg: Point):
        self.gate_center = (msg.x, msg.y)
    
    def gate_alignment_callback(self, msg: Float32):
        self.alignment_error = msg.data
    
    def gate_distance_callback(self, msg: Float32):
        self.gate_distance = msg.data
    
    def gate_confidence_callback(self, msg: Float32):
        self.detection_confidence = msg.data
    
    # Orange flare (obstacle) callbacks
    def orange_flare_detected_callback(self, msg: Bool):
        prev_flare = self.orange_flare_detected
        self.orange_flare_detected = msg.data
        if self.orange_flare_detected and not prev_flare:
            self.get_logger().error('🚨 ORANGE FLARE DETECTED - INITIATING AVOIDANCE!')
    
    def orange_flare_avoidance_callback(self, msg: Float32):
        self.flare_avoidance_direction = msg.data
    
    def flare_warning_callback(self, msg: String):
        self.flare_warning_level = msg.data
        if "CRITICAL" in msg.data:
            self.get_logger().error(f'🚨 {msg.data}')
    
    # Flare bumping callbacks
    def get_flare_data(self):
        if self.target_flare_color == 'red':
            self.flare_detected = self.red_flare_detected
            self.flare_pose = self.red_flare_pose
        elif self.target_flare_color == 'yellow':
            self.flare_detected = self.yellow_flare_detected
            self.flare_pose = self.yellow_flare_pose
        elif self.target_flare_color == 'blue':
            self.flare_detected = self.blue_flare_detected
            self.flare_pose = self.blue_flare_pose
        else:
            self.flare_detected = False

    def red_flare_bool_callback(self, msg: Bool): 
        self.red_flare_detected = msg.data
    def red_flare_pose_callback(self, msg: PoseStamped): 
        self.red_flare_pose = msg
    def yellow_flare_bool_callback(self, msg: Bool): 
        self.yellow_flare_detected = msg.data
    def yellow_flare_pose_callback(self, msg: PoseStamped): 
        self.yellow_flare_pose = msg
    def blue_flare_bool_callback(self, msg: Bool): 
        self.blue_flare_detected = msg.data
    def blue_flare_pose_callback(self, msg: PoseStamped): 
        self.blue_flare_pose = msg

    # Drum callbacks
    def blue_drum_bool_callback(self, msg: Bool): 
        self.blue_drum_detected = msg.data
    def blue_drum_pose_callback(self, msg: PoseStamped): 
        self.blue_drum_pose = msg
    
    # ============================================
    # MAIN CONTROL LOOP
    # ============================================
    
    def control_loop(self):
        """Main 20Hz control loop with integrated gate navigation"""
        
        # Update flare data for bumping task
        if self.state in [self.STATE_SEARCHING_FLARES, self.STATE_ALIGNING_FLARE]:
            self.get_flare_data()
        
        # Emergency override: Flare avoidance during gate approach
        if self.should_emergency_avoid_flare():
            if self.state != self.STATE_FLARE_EMERGENCY_AVOIDANCE:
                self.get_logger().error('🚨 EMERGENCY FLARE AVOIDANCE ACTIVATED!')
                self.transition_to(self.STATE_FLARE_EMERGENCY_AVOIDANCE)

        # State machine execution
        if self.state == self.STATE_INIT:
            self.handle_init()
        elif self.state == self.STATE_SUBMERGING:
            self.handle_submerging()
        elif self.state == self.STATE_SEARCHING_GATE:
            self.handle_searching_gate()
        elif self.state == self.STATE_APPROACHING_GATE:
            self.handle_approaching_gate()  # PROFESSIONAL NAVIGATION INTEGRATED HERE
        elif self.state == self.STATE_PASSING_GATE:
            self.handle_passing_gate()
        elif self.state == self.STATE_FLARE_EMERGENCY_AVOIDANCE:
            self.handle_emergency_avoidance()
        elif self.state == self.STATE_SEARCHING_FLARES:
            self.handle_searching_flares()
        elif self.state == self.STATE_ALIGNING_FLARE:
            self.handle_aligning_flare()
        elif self.state == self.STATE_BUMPING_FLARE:
            self.handle_bumping_flare()
        elif self.state == self.STATE_SEARCHING_DRUMS:
            self.handle_searching_drums()
        elif self.state == self.STATE_ALIGNING_DRUM:
            self.handle_aligning_drum()
        elif self.state == self.STATE_DROPPING_MARKER:
            self.handle_dropping_marker()
        elif self.state == self.STATE_COMPLETED:
            self.handle_completed()

    # ============================================
    # STATE HANDLERS
    # ============================================
    
    def handle_init(self):
        self.get_logger().info('📍 INIT → Starting autonomous mission...')
        time.sleep(0.5)
        self.transition_to(self.STATE_SUBMERGING)
    
    def handle_submerging(self):
        """Submerge to gate depth"""
        depth_error = self.gate_target_depth - self.current_depth
        elapsed = time.time() - self.state_start_time
        
        if int(elapsed) % 2 == 0:
            self.get_logger().info(
                f'⬇️  Submerging to Gate Depth: {self.current_depth:.2f}m / {self.gate_target_depth:.2f}m'
            )
        
        # Check if depth reached
        if abs(depth_error) < self.gate_depth_tolerance:
            self.get_logger().info('✅ Gate depth reached!')
            self.transition_to(self.STATE_SEARCHING_GATE)
            return
        
        # Timeout check
        if elapsed > 30.0:
            self.get_logger().warn('⚠️  Submerging timeout - continuing')
            self.transition_to(self.STATE_SEARCHING_GATE)
            return
        
        # Depth control with gentle forward motion
        cmd = Twist()
        cmd.linear.z = depth_error * self.gate_depth_gain
        cmd.linear.z = np.clip(cmd.linear.z, -0.5, 0.5)
        cmd.linear.x = -0.6
        self.cmd_vel_pub.publish(cmd)
    
    def handle_searching_gate(self):
        """Search for gate with rotation + forward pattern"""
        elapsed = time.time() - self.state_start_time
        
        # Check if gate detected
        if self.gate_detected and self.detection_confidence >= self.gate_min_confidence:
            self.get_logger().info('🎯 Gate detected! → APPROACHING')
            self.transition_to(self.STATE_APPROACHING_GATE)
            return
        
        # Timeout check
    #    if elapsed > self.gate_max_search_time:
    #        self.get_logger().error('❌ Search timeout - gate not found')
    #        Could transition to recovery or restart
    #        return
        
        # Search pattern
        cmd = self.get_search_command(self.gate_target_depth)
        self.cmd_vel_pub.publish(cmd)
    
    def handle_approaching_gate(self):
        """
        PROFESSIONAL GATE NAVIGATION - INTEGRATED FROM gate_navigator_node.py
        Two-phase approach for shortest path:
        1. FAR (>1.5m): Fast straight approach with minimal turning
        2. NEAR (<1.5m): Sharp turn to align, then pass
        """
        
        # Check gate still visible
        if not self.gate_detected or self.detection_confidence < self.gate_min_confidence:
            if self.time_since_gate_seen() > self.gate_lost_timeout:
                self.get_logger().warn('⚠️ Gate lost - returning to search')
                self.transition_to(self.STATE_SEARCHING_GATE)
                return
        
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(self.gate_target_depth)
        
        # === TWO-PHASE SHORTEST PATH LOGIC ===
        
        if self.gate_distance < self.gate_sharp_turn_distance:
            # ===== PHASE 2: SHARP TURN MODE (Distance < 1.5m) =====
            
            # Check if aligned enough to pass
            if abs(self.alignment_error) < self.gate_passing_alignment_threshold:
                self.get_logger().info('✅ PERFECTLY ALIGNED! → PASSING THROUGH GATE')
                self.transition_to(self.STATE_PASSING_GATE)
                return
            
            # Sharp turn control
            cmd.linear.x = self.gate_approach_speed_near  # Slower for tight turn
            
            # AGGRESSIVE yaw correction for sharp, efficient turns
            cmd.angular.z = -self.alignment_error * self.gate_yaw_gain_sharp
            cmd.angular.z = np.clip(cmd.angular.z, -0.8, 0.8)  # Allow fast rotation
            
            # Apply flare avoidance if needed
            if self.orange_flare_detected and self.gate_distance > self.gate_flare_critical_distance:
                cmd.linear.y = self.flare_avoidance_direction * 0.15
            
            self.get_logger().info(
                f'🔪 SHARP TURN: Dist={self.gate_distance:.2f}m, '
                f'Align={math.degrees(self.alignment_error):.1f}°, '
                f'Yaw_cmd={cmd.angular.z:.2f}',
                throttle_duration_sec=0.3
            )
        
        else:
            # ===== PHASE 1: STRAIGHT APPROACH (Distance > 1.5m) =====
            
            cmd.linear.x = self.gate_approach_speed_far  # FAST straight approach
            
            # MINIMAL yaw correction - just point roughly toward gate
            cmd.angular.z = -self.alignment_error * self.gate_yaw_gain_far
            cmd.angular.z = np.clip(cmd.angular.z, -0.25, 0.25)  # Small turns only
            
            # Apply flare avoidance if needed
            if self.orange_flare_detected and self.gate_distance > self.gate_flare_critical_distance:
                cmd.linear.y = self.flare_avoidance_direction * 0.15
            
            self.get_logger().info(
                f'→→ STRAIGHT APPROACH: Dist={self.gate_distance:.2f}m, '
                f'Align={math.degrees(self.alignment_error):.1f}°, '
                f'Speed={cmd.linear.x:.2f}',
                throttle_duration_sec=0.5
            )
        
        self.cmd_vel_pub.publish(cmd)
    
    def handle_passing_gate(self):
        """Pass through gate at full speed"""
        elapsed = time.time() - self.state_start_time
        
        if elapsed > self.gate_passing_duration:
            total_time = time.time() - self.mission_start_time
            self.get_logger().info('='*70)
            self.get_logger().info('✅ GATE PASSED SUCCESSFULLY!')
            self.get_logger().info(f'   Mission time so far: {total_time:.2f} seconds')
            self.get_logger().info('='*70)
            self.transition_to(self.STATE_SEARCHING_FLARES)
            return
        
        # Full speed ahead with depth hold
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(self.gate_target_depth)
        cmd.linear.x = self.gate_passing_speed  # MAXIMUM SPEED
        
        # Minimal yaw correction
        if self.gate_detected:
            cmd.angular.z = -self.alignment_error * self.gate_yaw_gain_far * 0.5
            cmd.angular.z = np.clip(cmd.angular.z, -0.3, 0.3)
        
        progress = (elapsed / self.gate_passing_duration) * 100
        self.get_logger().info(
            f'🚀 PASSING THROUGH: {progress:.0f}% complete',
            throttle_duration_sec=0.5
        )
        
        self.cmd_vel_pub.publish(cmd)
    
    def handle_emergency_avoidance(self):
        """Emergency flare avoidance - highest priority"""
        
        if self.emergency_avoidance_start_time is None:
            self.emergency_avoidance_start_time = time.time()
        
        elapsed = time.time() - self.emergency_avoidance_start_time
        
        # Check if avoidance complete
        if not self.orange_flare_detected or elapsed > self.gate_emergency_avoidance_duration:
            self.get_logger().info('✓ Emergency avoidance complete - resuming approach')
            self.emergency_avoidance_start_time = None
            self.transition_to(self.STATE_APPROACHING_GATE)
            return
        
        # Emergency maneuver: Strong lateral motion + slow forward
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(self.gate_target_depth)
        cmd.linear.y = self.flare_avoidance_direction * self.gate_flare_avoidance_gain
        cmd.linear.x = self.gate_search_speed * 0.5  # Slow forward
        cmd.angular.z = 0.0  # No rotation during emergency
        
        direction_text = "LEFT" if self.flare_avoidance_direction < 0 else "RIGHT"
        self.get_logger().error(
            f'🚨 EMERGENCY AVOIDANCE: Moving {direction_text} '
            f'({elapsed:.1f}s / {self.gate_emergency_avoidance_duration:.1f}s)',
            throttle_duration_sec=0.5
        )
        
        self.cmd_vel_pub.publish(cmd)

    def handle_searching_flares(self):
        """Search for flares (Task 4)"""
        target_depth = -2.5  # Flare depth
        
        if self.flare_detected:
            self.get_logger().info(f'🎯 Found target flare: {self.target_flare_color}! → ALIGNING')
            self.transition_to(self.STATE_ALIGNING_FLARE)
            return
        
        cmd = self.get_search_command(target_depth)
        self.cmd_vel_pub.publish(cmd)

    def handle_aligning_flare(self):
        """Align with target flare for bumping"""
        if not self.flare_detected:
            self.get_logger().warn(f'⚠️  Lost flare {self.target_flare_color}! → SEARCHING')
            self.transition_to(self.STATE_SEARCHING_FLARES)
            return

        img_center_x = 800 / 2
        img_center_y = 600 / 2
        
        err_x = self.flare_pose.pose.position.x - img_center_x
        err_y = self.flare_pose.pose.position.y - img_center_y
        
        if abs(err_x) < self.flare_alignment_threshold_px and abs(err_y) < self.flare_alignment_threshold_px:
             self.get_logger().info(f'✅ Aligned on {self.target_flare_color}! → BUMPING')
             self.transition_to(self.STATE_BUMPING_FLARE)
             return
             
        cmd = Twist()
        Kp_depth = 0.005
        cmd.linear.z = -err_y * Kp_depth
        Kp_yaw = 0.003
        cmd.angular.z = -err_x * Kp_yaw
        cmd.linear.x = -0.1
        self.cmd_vel_pub.publish(cmd)

    def handle_bumping_flare(self):
        """Bump the flare"""
        elapsed = time.time() - self.state_start_time
        
        if elapsed > 4.0:
            self.get_logger().info(f'✅ Bumped {self.target_flare_color}!')
            self.flares_bumped.append(self.target_flare_color)
            
            if len(self.flares_bumped) == len(self.flare_bump_order):
                self.get_logger().info('✅ All flares bumped! → SEARCHING DRUMS')
                self.transition_to(self.STATE_SEARCHING_DRUMS)
            else:
                self.target_flare_color = self.flare_bump_order[len(self.flares_bumped)]
                self.get_logger().info(f'Next flare target: {self.target_flare_color} → SEARCHING')
                self.transition_to(self.STATE_SEARCHING_FLARES)
            return
            
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(-2.5)
        cmd.linear.x = -0.6
        self.cmd_vel_pub.publish(cmd)

    def handle_searching_drums(self):
        """Search for drums (Task 2)"""
        target_depth = -2.7
        
        if self.blue_drum_detected:
            self.get_logger().info(f'🎯 Found blue drum! → ALIGNING')
            self.transition_to(self.STATE_ALIGNING_DRUM)
            return
        
        cmd = self.get_search_command(target_depth)
        self.cmd_vel_pub.publish(cmd)

    def handle_aligning_drum(self):
        """Align over drum"""
        if not self.blue_drum_detected:
            self.get_logger().warn(f'⚠️  Lost blue drum! → SEARCHING')
            self.transition_to(self.STATE_SEARCHING_DRUMS)
            return
            
        img_center_x = 640 / 2
        img_center_y = 480 / 2
        
        err_x = self.blue_drum_pose.pose.position.x - img_center_x
        err_y = self.blue_drum_pose.pose.position.y - img_center_y
        
        if abs(err_x) < self.drum_alignment_threshold_px and abs(err_y) < self.drum_alignment_threshold_px:
             self.get_logger().info(f'✅ Aligned over blue drum! → DROPPING')
             self.transition_to(self.STATE_DROPPING_MARKER)
             return
             
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(-2.7)
        Kp_sway = 0.003
        Kp_surge = 0.003
        cmd.linear.y = -err_x * Kp_sway
        cmd.linear.x = err_y * Kp_surge
        self.cmd_vel_pub.publish(cmd)

    def handle_dropping_marker(self):
        """Drop marker in drum"""
        elapsed = time.time() - self.state_start_time
        
        if elapsed > self.drop_duration:
            self.get_logger().info('✅ Marker dropped! → MISSION COMPLETE')
            self.transition_to(self.STATE_COMPLETED)
            return
            
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(-2.7)
        self.cmd_vel_pub.publish(cmd)
        
        if int(elapsed) == 0:
            self.get_logger().info('Simulating marker drop...')

    def handle_completed(self):
        """Mission complete"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        if int(time.time() - self.state_start_time) == 0:
            total_time = time.time() - self.mission_start_time
            self.get_logger().info('='*70)
            self.get_logger().info('✅ MISSION COMPLETED!')
            self.get_logger().info(f'   Total time: {total_time:.2f} seconds')
            self.get_logger().info('='*70)

    # ============================================
    # HELPER FUNCTIONS
    # ============================================
    
    def get_depth_control(self, target_depth):
        """Get depth control command"""
        depth_error = target_depth - self.current_depth
        cmd_z = depth_error * self.gate_depth_gain
        return np.clip(cmd_z, -0.6, 0.6)
    
    def should_emergency_avoid_flare(self):
        """Check if emergency flare avoidance is needed"""
        if not self.orange_flare_detected:
            return False
        
        # Emergency avoidance during gate approach when flare is close
        if self.state == self.STATE_APPROACHING_GATE:
            if "CRITICAL" in self.flare_warning_level:
                return True
            if self.gate_distance < self.gate_flare_critical_distance:
                return True
        
        return False
    
    def time_since_gate_seen(self):
        """Time since gate was last seen"""
        if self.gate_last_seen_time is None:
            return 999.0
        return time.time() - self.gate_last_seen_time

    def get_search_command(self, target_depth):
        """Returns a Twist command for the sequential rotate-then-move pattern"""
        elapsed = time.time() - self.state_start_time
        if int(elapsed) % 3 == 0:
             self.get_logger().info(
                 f'🔍 Searching... (Mode: {self.search_mode}, Depth: {self.current_depth:.2f}m)'
             )
        
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(target_depth)
        
        search_elapsed = time.time() - self.search_mode_start_time
        
        if self.search_mode == 'rotating':
            if search_elapsed > self.search_rotate_duration:
                self.search_mode = 'forward'
                self.search_mode_start_time = time.time()
                self.get_logger().info('🔍 Search: → FORWARD')
            else:
                cmd.angular.z = 0.25
        
        elif self.search_mode == 'forward':
            if search_elapsed > self.search_forward_duration:
                self.search_mode = 'rotating'
                self.search_mode_start_time = time.time()
                self.get_logger().info('🔍 Search: → ROTATING')
            else:
                cmd.linear.x = -0.3
                
        return cmd

    def transition_to(self, new_state):
        """Transitions the state machine to a new state"""
        state_names = {
            0: 'INIT', 1: 'SUBMERGING', 2: 'SEARCHING_GATE', 3: 'APPROACHING_GATE',
            4: 'PASSING_GATE', 5: 'GATE_PASSED_CONFIRMATION', 6: 'FLARE_EMERGENCY_AVOIDANCE',
            7: 'SEARCHING_FLARES', 8: 'ALIGNING_FLARE', 9: 'BUMPING_FLARE',
            10: 'SEARCHING_DRUMS', 11: 'ALIGNING_DRUM', 12: 'DROPPING_MARKER', 
            13: 'COMPLETED'
        }
        
        self.get_logger().info(
            f'🔄 STATE CHANGE: {state_names[self.state]} → {state_names[new_state]}'
        )
        
        # Stop rotation when entering approaching state
        if new_state == self.STATE_APPROACHING_GATE:
            stop_cmd = Twist()
            stop_cmd.linear.z = self.get_depth_control(self.gate_target_depth)
            for _ in range(3):
                self.cmd_vel_pub.publish(stop_cmd)
                time.sleep(0.01)
        
        self.state = new_state
        self.state_start_time = time.time()
        
        # Reset search pattern when entering search states
        if self.state in [self.STATE_SEARCHING_GATE, self.STATE_SEARCHING_FLARES, 
                         self.STATE_SEARCHING_DRUMS]:
            self.search_mode = 'rotating'
            self.search_mode_start_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMissionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Emergency stop
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()