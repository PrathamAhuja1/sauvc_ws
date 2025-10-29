#!/usr/bin/env python3
"""
Action Nodes for the SAUVC Gate Mission Behavioral Tree.

This file contains the Python classes that implement the "how-to"
for each action the robot can take (e.g., how to submerge, how to search).
These nodes are subscribed to ROS topics for state and publish to /cmd_vel.
"""

import rclpy
import time
import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String

import py_trees
from py_trees.behaviour import Behaviour
from py_trees_ros.subscribers import ToBlackboard

# ============================================
# Helper Class for Shared ROS Subscriptions
# ============================================

class AUVKnowledgeBase(Node):
    """
    A shared ROS2 node to hold all subscriptions and data.
    It writes subscribed data to the py_trees Blackboard.
    """
    def __init__(self):
        super().__init__('auv_knowledge_base')
        self.blackboard = py_trees.blackboard.Blackboard()
        
        # --- Blackboard Variables ---
        # Odom
        self.blackboard.register_variable("current_depth", 0.0)
        
        # Gate Detection
        self.blackboard.register_variable("gate_detected", False)
        self.blackboard.register_variable("gate_distance", 999.0)
        self.blackboard.register_variable("alignment_error", 0.0)
        self.blackboard.register_variable("detection_confidence", 0.0)
        self.blackboard.register_variable("gate_last_seen", 0.0)
        
        # Flare (Obstacle) Detection
        self.blackboard.register_variable("orange_flare_detected", False)
        self.blackboard.register_variable("flare_avoidance_direction", 0.0)
        self.blackboard.register_variable("flare_warning_level", "")

        # --- Subscriptions ---
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        self.gate_detected_sub = self.create_subscription(
            Bool, '/gate/detected', self.gate_detected_callback, 10)
        self.gate_alignment_sub = self.create_subscription(
            Float32, '/gate/alignment_error', self.gate_alignment_callback, 10)
        self.gate_distance_sub = self.create_subscription(
            Float32, '/gate/estimated_distance', self.gate_distance_callback, 10)
        self.gate_confidence_sub = self.create_subscription(
            Float32, '/gate/detection_confidence', self.gate_confidence_callback, 10)
        
        self.flare_detected_sub = self.create_subscription(
            Bool, '/flare/detected', self.orange_flare_detected_callback, 10)
        self.flare_avoidance_sub = self.create_subscription(
            Float32, '/flare/avoidance_direction', self.orange_flare_avoidance_callback, 10)
        self.flare_warning_sub = self.create_subscription(
            String, '/flare/warning', self.flare_warning_callback, 10)

    # --- Callbacks ---
    def odom_callback(self, msg: Odometry):
        self.blackboard.current_depth = msg.pose.pose.position.z

    def gate_detected_callback(self, msg: Bool):
        self.blackboard.gate_detected = msg.data
        if msg.data:
            self.blackboard.gate_last_seen = time.time()

    def gate_alignment_callback(self, msg: Float32):
        self.blackboard.alignment_error = msg.data

    def gate_distance_callback(self, msg: Float32):
        self.blackboard.gate_distance = msg.data

    def gate_confidence_callback(self, msg: Float32):
        self.blackboard.detection_confidence = msg.data
        
    def orange_flare_detected_callback(self, msg: Bool):
        self.blackboard.orange_flare_detected = msg.data
        
    def orange_flare_avoidance_callback(self, msg: Float32):
        self.blackboard.flare_avoidance_direction = msg.data
        
    def flare_warning_callback(self, msg: String):
        self.blackboard.flare_warning_level = msg.data

# ============================================
# Base Action Class
# ============================================

class ROSAction(Behaviour):
    """Base class for ROS Actions that publishes to /cmd_vel."""
    def __init__(self, name: str, node: rclpy.node.Node):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.node = node
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()

    def setup(self):
        self.node.get_logger().info(f"Setting up {self.name}")

    def initialise(self):
        self.node.get_logger().info(f"Initialising {self.name}")
        self.cmd_vel = Twist()

    def terminate(self, new_status: py_trees.common.Status):
        self.node.get_logger().info(f"Terminating {self.name} with status {new_status}")
        # Stop motion on termination
        self.cmd_vel_pub.publish(Twist())

    def get_depth_control(self, target_depth, gain=1.5):
        """Helper to get Z velocity command for depth hold."""
        depth_error = target_depth - self.blackboard.current_depth
        cmd_z = depth_error * gain
        return np.clip(cmd_z, -0.6, 0.6)

# ============================================
# Mission-Specific Action Nodes
# ============================================

class Submerge(ROSAction):
    """Action to submerge to a target depth."""
    def __init__(self, name: str, node: rclpy.node.Node, target_depth: float, tolerance: float = 0.15):
        super().__init__(name, node)
        self.target_depth = target_depth
        self.tolerance = tolerance
        self.start_time = 0.0

    def initialise(self):
        super().initialise()
        self.start_time = time.time()
        self.node.get_logger().info(f"Submerging to {self.target_depth}m...")

    def update(self) -> py_trees.common.Status:
        depth_error = self.target_depth - self.blackboard.current_depth
        
        # 1. Check for success
        if abs(depth_error) < self.tolerance:
            self.node.get_logger().info("Depth reached!")
            return py_trees.common.Status.SUCCESS
        
        # 2. Check for failure (timeout)
        if (time.time() - self.start_time) > 30.0:
            self.node.get_logger().warn("Submerging timeout!")
            return py_trees.common.Status.FAILURE
            
        # 3. If still running, execute logic
        self.cmd_vel.linear.z = self.get_depth_control(self.target_depth)
        self.cmd_vel.linear.x = -0.6 # Gentle forward motion (as in BT.py)
        self.cmd_vel_pub.publish(self.cmd_vel)
        
        return py_trees.common.Status.RUNNING

class SearchForGate(ROSAction):
    """Action to search for the gate using a rotate/forward pattern."""
    def __init__(self, name: str, node: rclpy.node.Node, target_depth: float, min_confidence: float = 0.6):
        super().__init__(name, node)
        self.target_depth = target_depth
        self.min_confidence = min_confidence
        self.search_mode = 'rotating'
        self.mode_start_time = 0.0
        self.rotate_duration = 10.0
        self.forward_duration = 5.0

    def initialise(self):
        super().initialise()
        self.mode_start_time = time.time()
        self.search_mode = 'rotating'
        self.node.get_logger().info("Searching for gate...")

    def update(self) -> py_trees.common.Status:
        # 1. Check for success
        if self.blackboard.gate_detected and self.blackboard.detection_confidence >= self.min_confidence:
            self.node.get_logger().info("Gate found!")
            return py_trees.common.Status.SUCCESS
        
        # 2. Check for failure (e.g., mission timeout, not implemented here)
        
        # 3. If still running, execute search pattern
        self.cmd_vel.linear.z = self.get_depth_control(self.target_depth)
        
        search_elapsed = time.time() - self.mode_start_time
        
        if self.search_mode == 'rotating':
            if search_elapsed > self.rotate_duration:
                self.search_mode = 'forward'
                self.mode_start_time = time.time()
            else:
                self.cmd_vel.angular.z = 0.25 # Rotate
                self.cmd_vel.linear.x = 0.0
        
        elif self.search_mode == 'forward':
            if search_elapsed > self.forward_duration:
                self.search_mode = 'rotating'
                self.mode_start_time = time.time()
            else:
                self.cmd_vel.linear.x = -0.3 # Move forward
                self.cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(self.cmd_vel)
        return py_trees.common.Status.RUNNING

class ApproachGate(ROSAction):
    """
    Action to approach the gate using the 2-phase logic from BT.py.
    This node will run until the AUV is aligned and ready to pass.
    It includes flare avoidance logic.
    """
    def __init__(self, name: str, node: rclpy.node.Node, target_depth: float,
                 min_confidence: float = 0.6,
                 sharp_turn_dist: float = 1.5,
                 passing_align_thresh: float = 0.15,
                 lost_timeout: float = 5.0):
        super().__init__(name, node)
        # Gate params
        self.target_depth = target_depth
        self.min_confidence = min_confidence
        self.sharp_turn_dist = sharp_turn_dist
        self.passing_align_thresh = passing_align_thresh
        self.lost_timeout = lost_timeout
        # Speeds
        self.speed_far = 0.6
        self.speed_near = 0.4
        # Gains
        self.yaw_gain_far = 0.8
        self.yaw_gain_sharp = 6.0
        # Flare params
        self.flare_avoid_gain = 1.5
        self.flare_critical_dist = 3.0
        self.emergency_avoid_duration = 4.0
        
        self.in_emergency_avoidance = False
        self.emergency_start_time = 0.0

    def update(self) -> py_trees.common.Status:
        
        # --- Emergency Flare Avoidance Check ---
        if self.should_emergency_avoid():
            self.in_emergency_avoidance = True
            self.emergency_start_time = time.time()
        
        if self.in_emergency_avoidance:
            return self.execute_emergency_avoidance()

        # --- Standard Approach Logic ---
        
        # 1. Check for failure (gate lost)
        time_since_seen = time.time() - self.blackboard.gate_last_seen
        if (not self.blackboard.gate_detected or self.blackboard.detection_confidence < self.min_confidence) and \
           (time_since_seen > self.lost_timeout):
            self.node.get_logger().warn("Gate lost during approach!")
            return py_trees.common.Status.FAILURE
            
        # Get latest data
        dist = self.blackboard.gate_distance
        align_err = self.blackboard.alignment_error
        
        # 2. Check for success (aligned and ready to pass)
        if dist < self.sharp_turn_dist and abs(align_err) < self.passing_align_thresh:
            self.node.get_logger().info("Aligned and ready to pass!")
            return py_trees.common.Status.SUCCESS
            
        # 3. If still running, execute logic
        self.cmd_vel.linear.z = self.get_depth_control(self.target_depth)
        
        if dist < self.sharp_turn_dist:
            # --- Phase 2: SHARP TURN (Near) ---
            self.cmd_vel.linear.x = self.speed_near
            self.cmd_vel.angular.z = -align_err * self.yaw_gain_sharp
            self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z, -0.8, 0.8)
        else:
            # --- Phase 1: STRAIGHT APPROACH (Far) ---
            self.cmd_vel.linear.x = self.speed_far
            self.cmd_vel.angular.z = -align_err * self.yaw_gain_far
            self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z, -0.25, 0.25)
            
        # Non-emergency flare avoidance
        if self.blackboard.orange_flare_detected and dist > self.flare_critical_dist:
            self.cmd_vel.linear.y = self.blackboard.flare_avoidance_direction * 0.15

        self.cmd_vel_pub.publish(self.cmd_vel)
        return py_trees.common.Status.RUNNING

    def should_emergency_avoid(self):
        """Check if emergency flare avoidance is needed."""
        if not self.blackboard.orange_flare_detected:
            return False
        
        if "CRITICAL" in self.blackboard.flare_warning_level:
            return True
        if self.blackboard.gate_distance < self.flare_critical_dist:
            return True
        return False

    def execute_emergency_avoidance(self):
        """Perform emergency avoidance maneuver."""
        elapsed = time.time() - self.emergency_start_time
        
        # Check if avoidance complete
        if not self.blackboard.orange_flare_detected or elapsed > self.emergency_avoid_duration:
            self.node.get_logger().info("Emergency avoidance complete.")
            self.in_emergency_avoidance = False
            return py_trees.common.Status.RUNNING # Return RUNNING to resume approach
            
        # Emergency maneuver
        self.cmd_vel.linear.z = self.get_depth_control(self.target_depth)
        self.cmd_vel.linear.y = self.blackboard.flare_avoidance_direction * self.flare_avoid_gain
        self.cmd_vel.linear.x = self.speed_near * 0.5  # Slow forward
        self.cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(self.cmd_vel)
        return py_trees.common.Status.RUNNING

class PassThroughGate(ROSAction):
    """Action to move forward at speed for a fixed duration."""
    def __init__(self, name: str, node: rclpy.node.Node, target_depth: float, duration: float, speed: float):
        super().__init__(name, node)
        self.target_depth = target_depth
        self.duration = duration
        self.speed = speed
        self.start_time = 0.0

    def initialise(self):
        super().initialise()
        self.start_time = time.time()
        self.node.get_logger().info(f"Passing through gate for {self.duration}s...")

    def update(self) -> py_trees.common.Status:
        elapsed = time.time() - self.start_time
        
        # 1. Check for success
        if elapsed > self.duration:
            self.node.get_logger().info("Assumed passed through gate.")
            return py_trees.common.Status.SUCCESS
        
        # 2. Check for failure (not applicable here)
        
        # 3. If still running, execute logic
        self.cmd_vel.linear.z = self.get_depth_control(self.target_depth)
        self.cmd_vel.linear.x = self.speed
        
        # Minimal correction if gate still visible
        if self.blackboard.gate_detected:
            self.cmd_vel.angular.z = -self.blackboard.alignment_error * 0.8
            self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z, -0.3, 0.3)
            
        self.cmd_vel_pub.publish(self.cmd_vel)
        return py_trees.common.Status.RUNNING