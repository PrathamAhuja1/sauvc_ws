#!/usr/bin/env python3
"""
Behavioral Tree Mission Planner for Gate Detection Task
Uses py_trees library for clean BT implementation
Coordinates with gate_detector_node.py and implements navigation logic
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Point

import py_trees
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.decorators import Retry, Timeout
from py_trees import common, blackboard
import time
import math
import numpy as np


# ============================================
# BEHAVIORAL TREE BEHAVIORS (Leaf Nodes)
# ============================================

class SubmergeToDepth(py_trees.behaviour.Behaviour):
    """Submerge AUV to target depth for gate navigation"""
    
    def __init__(self, name, target_depth=-1.5, tolerance=0.15):
        super().__init__(name)
        self.target_depth = target_depth
        self.tolerance = tolerance
        self.bb = py_trees.blackboard.Client(name=self.__class__.__name__)
        self.bb.register_key(key='current_depth', access=py_trees.common.Access.READ)
        self.bb.register_key(key='cmd_vel_pub', access=py_trees.common.Access.READ)
        
    def update(self):
        current_depth = self.bb.get('current_depth')
        cmd_vel_pub = self.bb.get('cmd_vel_pub')
        
        if current_depth is None:
            return py_trees.common.Status.RUNNING
        
        depth_error = self.target_depth - current_depth
        
        # Check if target depth reached
        if abs(depth_error) < self.tolerance:
            self.feedback_message = f"Target depth {self.target_depth:.2f}m reached"
            return py_trees.common.Status.SUCCESS
        
        # Depth control command
        cmd = Twist()
        cmd.linear.z = depth_error * 1.5  # Proportional control
        cmd.linear.z = np.clip(cmd.linear.z, -0.5, 0.5)
        cmd.linear.x = 0.2  # Gentle forward motion while submerging
        
        cmd_vel_pub.publish(cmd)
        
        self.feedback_message = f"Submerging: {current_depth:.2f}m / {self.target_depth:.2f}m"
        return py_trees.common.Status.RUNNING


class SearchForGate(py_trees.behaviour.Behaviour):
    """Search for gate using rotation + forward pattern"""
    
    def __init__(self, name, search_speed=0.25, rotation_speed=0.15, max_time=45.0):
        super().__init__(name)
        self.search_speed = search_speed
        self.rotation_speed = rotation_speed
        self.max_time = max_time
        self.start_time = None
        
        self.bb = py_trees.blackboard.Client(name=self.__class__.__name__)
        self.bb.register_key(key='gate_detected', access=py_trees.common.Access.READ)
        self.bb.register_key(key='detection_confidence', access=py_trees.common.Access.READ)
        self.bb.register_key(key='current_depth', access=py_trees.common.Access.READ)
        self.bb.register_key(key='cmd_vel_pub', access=py_trees.common.Access.READ)
        
    def initialise(self):
        self.start_time = time.time()
        
    def update(self):
        gate_detected = self.bb.get('gate_detected')
        confidence = self.bb.get('detection_confidence')
        cmd_vel_pub = self.bb.get('cmd_vel_pub')
        current_depth = self.bb.get('current_depth')
        
        # Check if gate found with sufficient confidence
        if gate_detected and confidence >= 0.6:
            self.feedback_message = f"Gate detected! Confidence: {confidence:.2f}"
            return py_trees.common.Status.SUCCESS
        
        # Check timeout
        elapsed = time.time() - self.start_time
        if elapsed > self.max_time:
            self.feedback_message = "Search timeout - gate not found"
            return py_trees.common.Status.FAILURE
        
        # Search pattern: rotate + forward
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(current_depth)
        cmd.linear.x = self.search_speed
        cmd.angular.z = self.rotation_speed
        
        cmd_vel_pub.publish(cmd)
        
        self.feedback_message = f"Searching for gate... {elapsed:.0f}s"
        return py_trees.common.Status.RUNNING
    
    def get_depth_control(self, current_depth, target_depth=-1.5):
        if current_depth is None:
            return 0.0
        depth_error = target_depth - current_depth
        return np.clip(depth_error * 1.5, -0.6, 0.6)


class AlignWithGate(py_trees.behaviour.Behaviour):
    """Align with gate using proportional control"""
    
    def __init__(self, name, alignment_threshold=0.15, approach_distance=2.0):
        super().__init__(name)
        self.alignment_threshold = alignment_threshold  # ~8.5 degrees
        self.approach_distance = approach_distance
        self.yaw_gain = 3.5  # Aggressive for quick alignment
        
        self.bb = py_trees.blackboard.Client(name=self.__class__.__name__)
        self.bb.register_key(key='gate_detected', access=py_trees.common.Access.READ)
        self.bb.register_key(key='alignment_error', access=py_trees.common.Access.READ)
        self.bb.register_key(key='gate_distance', access=py_trees.common.Access.READ)
        self.bb.register_key(key='detection_confidence', access=py_trees.common.Access.READ)
        self.bb.register_key(key='current_depth', access=py_trees.common.Access.READ)
        self.bb.register_key(key='cmd_vel_pub', access=py_trees.common.Access.READ)
        self.bb.register_key(key='flare_detected', access=py_trees.common.Access.READ)
        self.bb.register_key(key='flare_avoidance_direction', access=py_trees.common.Access.READ)
        
    def update(self):
        gate_detected = self.bb.get('gate_detected')
        alignment_error = self.bb.get('alignment_error')
        gate_distance = self.bb.get('gate_distance')
        confidence = self.bb.get('detection_confidence')
        current_depth = self.bb.get('current_depth')
        cmd_vel_pub = self.bb.get('cmd_vel_pub')
        
        # Check if gate still visible
        if not gate_detected or confidence < 0.6:
            self.feedback_message = "Gate lost - cannot align"
            return py_trees.common.Status.FAILURE
        
        # Check if aligned and close enough to pass
        if abs(alignment_error) < self.alignment_threshold and gate_distance < self.approach_distance:
            self.feedback_message = f"Aligned! Error: {math.degrees(alignment_error):.1f}°"
            return py_trees.common.Status.SUCCESS
        
        # Alignment control
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(current_depth)
        cmd.linear.x = 0.35  # Moderate approach speed
        
        # Yaw correction
        cmd.angular.z = -alignment_error * self.yaw_gain
        cmd.angular.z = np.clip(cmd.angular.z, -0.7, 0.7)
        
        # Flare avoidance if needed
        flare_detected = self.bb.get('flare_detected')
        if flare_detected and gate_distance > 3.0:
            flare_direction = self.bb.get('flare_avoidance_direction')
            cmd.linear.y = flare_direction * 0.15
        
        cmd_vel_pub.publish(cmd)
        
        self.feedback_message = f"Aligning: {math.degrees(alignment_error):.1f}°, Dist: {gate_distance:.2f}m"
        return py_trees.common.Status.RUNNING
    
    def get_depth_control(self, current_depth, target_depth=-1.5):
        if current_depth is None:
            return 0.0
        depth_error = target_depth - current_depth
        return np.clip(depth_error * 1.5, -0.6, 0.6)


class PassThroughGate(py_trees.behaviour.Behaviour):
    """Pass through gate at full speed"""
    
    def __init__(self, name, passing_speed=0.6, passing_duration=6.0):
        super().__init__(name)
        self.passing_speed = passing_speed
        self.passing_duration = passing_duration
        self.start_time = None
        
        self.bb = py_trees.blackboard.Client(name=self.__class__.__name__)
        self.bb.register_key(key='current_depth', access=py_trees.common.Access.READ)
        self.bb.register_key(key='cmd_vel_pub', access=py_trees.common.Access.READ)
        self.bb.register_key(key='alignment_error', access=py_trees.common.Access.READ)
        self.bb.register_key(key='gate_detected', access=py_trees.common.Access.READ)
        
    def initialise(self):
        self.start_time = time.time()
        
    def update(self):
        cmd_vel_pub = self.bb.get('cmd_vel_pub')
        current_depth = self.bb.get('current_depth')
        alignment_error = self.bb.get('alignment_error')
        gate_detected = self.bb.get('gate_detected')
        
        elapsed = time.time() - self.start_time
        
        # Check if passed through
        if elapsed > self.passing_duration:
            self.feedback_message = "Successfully passed through gate!"
            return py_trees.common.Status.SUCCESS
        
        # Full speed ahead with depth hold
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(current_depth)
        cmd.linear.x = self.passing_speed  # MAXIMUM SPEED
        
        # Minimal yaw correction if gate still visible
        if gate_detected and alignment_error is not None:
            cmd.angular.z = -alignment_error * 0.8
            cmd.angular.z = np.clip(cmd.angular.z, -0.3, 0.3)
        
        cmd_vel_pub.publish(cmd)
        
        progress = (elapsed / self.passing_duration) * 100
        self.feedback_message = f"Passing through: {progress:.0f}% complete"
        return py_trees.common.Status.RUNNING
    
    def get_depth_control(self, current_depth, target_depth=-1.5):
        if current_depth is None:
            return 0.0
        depth_error = target_depth - current_depth
        return np.clip(depth_error * 1.5, -0.6, 0.6)


class EmergencyFlareAvoidance(py_trees.behaviour.Behaviour):
    """Emergency avoidance when orange flare is critically close"""
    
    def __init__(self, name, avoidance_duration=4.0):
        super().__init__(name)
        self.avoidance_duration = avoidance_duration
        self.start_time = None
        
        self.bb = py_trees.blackboard.Client(name=self.__class__.__name__)
        self.bb.register_key(key='flare_detected', access=py_trees.common.Access.READ)
        self.bb.register_key(key='flare_warning_level', access=py_trees.common.Access.READ)
        self.bb.register_key(key='flare_avoidance_direction', access=py_trees.common.Access.READ)
        self.bb.register_key(key='current_depth', access=py_trees.common.Access.READ)
        self.bb.register_key(key='cmd_vel_pub', access=py_trees.common.Access.READ)
        
    def initialise(self):
        self.start_time = time.time()
        
    def update(self):
        flare_detected = self.bb.get('flare_detected')
        warning_level = self.bb.get('flare_warning_level')
        avoidance_direction = self.bb.get('flare_avoidance_direction')
        cmd_vel_pub = self.bb.get('cmd_vel_pub')
        current_depth = self.bb.get('current_depth')
        
        elapsed = time.time() - self.start_time
        
        # Check if avoidance complete
        if not flare_detected or elapsed > self.avoidance_duration:
            self.feedback_message = "Flare avoidance complete"
            return py_trees.common.Status.SUCCESS
        
        # Emergency maneuver: Strong lateral motion
        cmd = Twist()
        cmd.linear.z = self.get_depth_control(current_depth)
        cmd.linear.y = avoidance_direction * 1.5  # Strong lateral
        cmd.linear.x = 0.15  # Slow forward
        cmd.angular.z = 0.0  # No rotation
        
        cmd_vel_pub.publish(cmd)
        
        direction_text = "LEFT" if avoidance_direction < 0 else "RIGHT"
        self.feedback_message = f"EMERGENCY AVOIDANCE: Moving {direction_text}"
        return py_trees.common.Status.RUNNING
    
    def get_depth_control(self, current_depth, target_depth=-1.5):
        if current_depth is None:
            return 0.0
        depth_error = target_depth - current_depth
        return np.clip(depth_error * 1.5, -0.6, 0.6)


# ============================================
# CONDITION CHECKS
# ============================================

class CheckFlareEmergency(py_trees.behaviour.Behaviour):
    """Check if emergency flare avoidance is needed"""
    
    def __init__(self, name):
        super().__init__(name)
        self.bb = py_trees.blackboard.Client(name=self.__class__.__name__)
        self.bb.register_key(key='flare_detected', access=py_trees.common.Access.READ)
        self.bb.register_key(key='flare_warning_level', access=py_trees.common.Access.READ)
        
    def update(self):
        flare_detected = self.bb.get('flare_detected')
        warning_level = self.bb.get('flare_warning_level')
        
        if flare_detected and "CRITICAL" in str(warning_level):
            self.feedback_message = "CRITICAL flare detected!"
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.FAILURE


# ============================================
# ROS 2 NODE WITH BEHAVIORAL TREE
# ============================================

class GateMissionBehavioralTree(Node):
    """ROS 2 Node that runs behavioral tree for gate detection mission"""
    
    def __init__(self):
        super().__init__('gate_mission_bt')
        
        # Blackboard setup
        self.blackboard = py_trees.blackboard.Client(name="GateMission")
        self.blackboard.register_key(key='current_depth', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='current_position', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='gate_detected', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='gate_center', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='gate_distance', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='alignment_error', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='detection_confidence', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='flare_detected', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='flare_avoidance_direction', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='flare_warning_level', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='cmd_vel_pub', access=py_trees.common.Access.WRITE)
        
        # Initialize blackboard values
        self.blackboard.set('current_depth', 0.0)
        self.blackboard.set('gate_detected', False)
        self.blackboard.set('gate_distance', 999.0)
        self.blackboard.set('alignment_error', 0.0)
        self.blackboard.set('detection_confidence', 0.0)
        self.blackboard.set('flare_detected', False)
        self.blackboard.set('flare_avoidance_direction', 0.0)
        self.blackboard.set('flare_warning_level', "")
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.blackboard.set('cmd_vel_pub', self.cmd_vel_pub)
        
        self.mission_status_pub = self.create_publisher(String, '/mission/status', 10)
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        self.gate_detected_sub = self.create_subscription(
            Bool, '/gate/detected', self.gate_detected_callback, 10)
        self.gate_center_sub = self.create_subscription(
            Point, '/gate/center', self.gate_center_callback, 10)
        self.alignment_sub = self.create_subscription(
            Float32, '/gate/alignment_error', self.alignment_callback, 10)
        self.distance_sub = self.create_subscription(
            Float32, '/gate/estimated_distance', self.distance_callback, 10)
        self.confidence_sub = self.create_subscription(
            Float32, '/gate/detection_confidence', self.confidence_callback, 10)
        
        self.flare_detected_sub = self.create_subscription(
            Bool, '/flare/detected', self.flare_detected_callback, 10)
        self.flare_avoidance_sub = self.create_subscription(
            Float32, '/flare/avoidance_direction', self.flare_avoidance_callback, 10)
        self.flare_warning_sub = self.create_subscription(
            String, '/flare/warning', self.flare_warning_callback, 10)
        
        # Build behavioral tree
        self.root = self.create_behavior_tree()
        
        # Tick timer (10 Hz)
        self.tick_timer = self.create_timer(0.1, self.tick_tree)
        
        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.mission_start_time = time.time()
        
        self.get_logger().info('='*70)
        self.get_logger().info('🌳 Gate Mission Behavioral Tree Started')
        self.get_logger().info('='*70)
    
    def create_behavior_tree(self):
        """Create the behavioral tree structure"""
        
        # Main mission sequence
        gate_mission = Sequence(
            name="GateMission",
            memory=False,
            children=[
                SubmergeToDepth(name="SubmergeToDepth"),
                SearchForGate(name="SearchForGate"),
                AlignWithGate(name="AlignWithGate"),
                PassThroughGate(name="PassThroughGate")
            ]
        )
        
        # Emergency handling - higher priority
        emergency_handling = Selector(
            name="EmergencyHandling",
            memory=False,
            children=[
                Sequence(
                    name="HandleFlareEmergency",
                    memory=False,
                    children=[
                        CheckFlareEmergency(name="CheckFlareEmergency"),
                        EmergencyFlareAvoidance(name="EmergencyFlareAvoidance")
                    ]
                ),
                gate_mission
            ]
        )
        
        return emergency_handling
    
    # ===== CALLBACKS =====
    
    def odom_callback(self, msg: Odometry):
        self.blackboard.set('current_depth', msg.pose.pose.position.z)
        self.blackboard.set('current_position', msg.pose.pose.position)
    
    def gate_detected_callback(self, msg: Bool):
        self.blackboard.set('gate_detected', msg.data)
    
    def gate_center_callback(self, msg: Point):
        self.blackboard.set('gate_center', (msg.x, msg.y))
    
    def alignment_callback(self, msg: Float32):
        self.blackboard.set('alignment_error', msg.data)
    
    def distance_callback(self, msg: Float32):
        self.blackboard.set('gate_distance', msg.data)
    
    def confidence_callback(self, msg: Float32):
        self.blackboard.set('detection_confidence', msg.data)
    
    def flare_detected_callback(self, msg: Bool):
        self.blackboard.set('flare_detected', msg.data)
    
    def flare_avoidance_callback(self, msg: Float32):
        self.blackboard.set('flare_avoidance_direction', msg.data)
    
    def flare_warning_callback(self, msg: String):
        self.blackboard.set('flare_warning_level', msg.data)
    
    # ===== TREE EXECUTION =====
    
    def tick_tree(self):
        """Tick the behavioral tree (10 Hz)"""
        self.root.tick_once()
        
        # Check if mission complete
        if self.root.status == py_trees.common.Status.SUCCESS:
            elapsed = time.time() - self.mission_start_time
            self.get_logger().info('='*70)
            self.get_logger().info('✅ GATE MISSION COMPLETED!')
            self.get_logger().info(f'   Total time: {elapsed:.2f} seconds')
            self.get_logger().info('='*70)
            
            # Stop robot
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            # Stop ticking
            self.tick_timer.cancel()
        
        elif self.root.status == py_trees.common.Status.FAILURE:
            self.get_logger().error('❌ MISSION FAILED!')
            
            # Stop robot
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            # Stop ticking
            self.tick_timer.cancel()
    
    def publish_status(self):
        """Publish mission status (1 Hz)"""
        status_msg = String()
        
        # Get current behavior feedback
        visitor = py_trees.visitors.DebugVisitor()
        self.root.visit(visitor)
        
        current_behavior = None
        for behaviour in visitor.visited:
            if behaviour.status == py_trees.common.Status.RUNNING:
                current_behavior = behaviour
                break
        
        if current_behavior:
            status_msg.data = f"{current_behavior.name}: {current_behavior.feedback_message}"
        else:
            status_msg.data = f"Status: {self.root.status.name}"
        
        self.mission_status_pub.publish(status_msg)
        
        # Log to console
        if current_behavior:
            self.get_logger().info(
                f"🌳 {current_behavior.name}: {current_behavior.feedback_message}",
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = GateMissionBehavioralTree()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()