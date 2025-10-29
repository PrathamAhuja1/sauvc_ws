#!/usr/bin/env python3
"""
Professional Gate Navigator for SAUVC Competition
State machine for robust gate passing with flare avoidance
Ensures smooth, collision-free navigation through the gate
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
from enum import IntEnum


class NavigationState(IntEnum):
    """Gate navigation states"""
    IDLE = 0
    SUBMERGING = 1
    SEARCHING = 2
    VISUAL_ACQUISITION = 3
    COARSE_ALIGNMENT = 4
    APPROACH = 5
    FINE_ALIGNMENT = 6
    FINAL_APPROACH = 7
    PASSING = 8
    FLARE_EMERGENCY_AVOIDANCE = 9
    COMPLETED = 10
    RECOVERY = 11


class ProfessionalGateNavigator(Node):
    def __init__(self):
        super().__init__('professional_gate_navigator')
        
        # State machine
        self.state = NavigationState.IDLE
        self.previous_state = NavigationState.IDLE
        self.state_start_time = time.time()
        self.mission_start_time = time.time()
        
        # Parameters - Tuned for optimal performance
        self.declare_parameter('target_depth', -1.5)  # Gate depth
        self.declare_parameter('depth_tolerance', 0.15)
        self.declare_parameter('depth_correction_gain', 1.5)
        
        # Speed parameters - Progressive approach
        self.declare_parameter('search_speed', 0.25)
        self.declare_parameter('acquisition_speed', 0.20)
        self.declare_parameter('approach_speed', 0.35)
        self.declare_parameter('fine_approach_speed', 0.20)
        self.declare_parameter('passing_speed', 0.50)
        
        # Distance thresholds for state transitions
        self.declare_parameter('acquisition_distance', 10.0)
        self.declare_parameter('coarse_alignment_distance', 6.0)
        self.declare_parameter('approach_distance', 4.0)
        self.declare_parameter('fine_alignment_distance', 2.5)
        self.declare_parameter('final_approach_distance', 1.5)
        self.declare_parameter('passing_distance', 0.8)
        
        # Alignment thresholds (radians)
        self.declare_parameter('coarse_alignment_threshold', 0.35)  # ~20 degrees
        self.declare_parameter('fine_alignment_threshold', 0.12)    # ~7 degrees
        self.declare_parameter('passing_alignment_threshold', 0.08) # ~4.5 degrees
        
        # Control gains - Adaptive based on distance
        self.declare_parameter('yaw_gain_far', 1.2)
        self.declare_parameter('yaw_gain_medium', 1.8)
        self.declare_parameter('yaw_gain_near', 2.5)
        self.declare_parameter('yaw_gain_fine', 3.5)
        
        # Flare avoidance
        self.declare_parameter('flare_avoidance_gain', 1.5)
        self.declare_parameter('flare_critical_distance', 3.0)
        self.declare_parameter('emergency_avoidance_duration', 4.0)
        
        # Safety and recovery
        self.declare_parameter('gate_lost_timeout', 5.0)
        self.declare_parameter('max_search_time', 45.0)
        self.declare_parameter('passing_duration', 6.0)
        self.declare_parameter('min_detection_confidence', 0.6)
        
        # Load parameters
        self.load_parameters()
        
        # Vehicle state
        self.current_depth = 0.0
        self.current_position = None
        self.current_orientation = None
        
        # Gate detection state
        self.gate_detected = False
        self.gate_center = None
        self.gate_distance = 999.0
        self.alignment_error = 0.0
        self.detection_confidence = 0.0
        self.gate_last_seen_time = None
        
        # Flare detection state
        self.flare_detected = False
        self.flare_avoidance_direction = 0.0
        self.flare_warning_level = ""
        self.emergency_avoidance_start = None
        
        # Control state
        self.current_yaw_gain = self.yaw_gain_far
        self.forward_speed = 0.0
        self.target_yaw_rate = 0.0
        
        # Performance tracking
        self.alignment_history = []
        self.distance_history = []
        
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
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/gate_nav/state', 10)
        self.status_pub = self.create_publisher(String, '/gate_nav/status', 10)
        
        # Control loop (50 Hz for smooth control)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('Professional Gate Navigator initialized')
        self.get_logger().info(f'Target depth: {self.target_depth}m')
        self.get_logger().info(f'Gate width: 1.5m, Min confidence: {self.min_confidence}')
        self.get_logger().info('='*70)
    
    def load_parameters(self):
        """Load all navigation parameters"""
        self.target_depth = self.get_parameter('target_depth').value
        self.depth_tolerance = self.get_parameter('depth_tolerance').value
        self.depth_gain = self.get_parameter('depth_correction_gain').value
        
        self.search_speed = self.get_parameter('search_speed').value
        self.acquisition_speed = self.get_parameter('acquisition_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.fine_approach_speed = self.get_parameter('fine_approach_speed').value
        self.passing_speed = self.get_parameter('passing_speed').value
        
        self.acquisition_distance = self.get_parameter('acquisition_distance').value
        self.coarse_alignment_distance = self.get_parameter('coarse_alignment_distance').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.fine_alignment_distance = self.get_parameter('fine_alignment_distance').value
        self.final_approach_distance = self.get_parameter('final_approach_distance').value
        self.passing_distance = self.get_parameter('passing_distance').value
        
        self.coarse_alignment_threshold = self.get_parameter('coarse_alignment_threshold').value
        self.fine_alignment_threshold = self.get_parameter('fine_alignment_threshold').value
        self.passing_alignment_threshold = self.get_parameter('passing_alignment_threshold').value
        
        self.yaw_gain_far = self.get_parameter('yaw_gain_far').value
        self.yaw_gain_medium = self.get_parameter('yaw_gain_medium').value
        self.yaw_gain_near = self.get_parameter('yaw_gain_near').value
        self.yaw_gain_fine = self.get_parameter('yaw_gain_fine').value
        
        self.flare_avoidance_gain = self.get_parameter('flare_avoidance_gain').value
        self.flare_critical_distance = self.get_parameter('flare_critical_distance').value
        self.emergency_avoidance_duration = self.get_parameter('emergency_avoidance_duration').value
        
        self.gate_lost_timeout = self.get_parameter('gate_lost_timeout').value
        self.max_search_time = self.get_parameter('max_search_time').value
        self.passing_duration = self.get_parameter('passing_duration').value
        self.min_confidence = self.get_parameter('min_detection_confidence').value
    
    # === CALLBACKS ===
    
    def odom_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
    
    def gate_detected_callback(self, msg: Bool):
        prev_detected = self.gate_detected
        self.gate_detected = msg.data
        
        if self.gate_detected:
            self.gate_last_seen_time = time.time()
            if not prev_detected:
                self.get_logger().info('✓ Gate acquired!')
        elif prev_detected:
            self.get_logger().warn('⚠ Gate lost from view')
    
    def gate_center_callback(self, msg: Point):
        self.gate_center = (msg.x, msg.y)
    
    def alignment_callback(self, msg: Float32):
        self.alignment_error = msg.data
        self.alignment_history.append(abs(msg.data))
        if len(self.alignment_history) > 50:
            self.alignment_history.pop(0)
    
    def distance_callback(self, msg: Float32):
        self.gate_distance = msg.data
        self.distance_history.append(msg.data)
        if len(self.distance_history) > 50:
            self.distance_history.pop(0)
    
    def confidence_callback(self, msg: Float32):
        self.detection_confidence = msg.data
    
    def flare_detected_callback(self, msg: Bool):
        prev_flare = self.flare_detected
        self.flare_detected = msg.data
        
        if self.flare_detected and not prev_flare:
            self.get_logger().error('🚨 ORANGE FLARE DETECTED - INITIATING AVOIDANCE!')
    
    def flare_avoidance_callback(self, msg: Float32):
        self.flare_avoidance_direction = msg.data
    
    def flare_warning_callback(self, msg: String):
        self.flare_warning_level = msg.data
        if "CRITICAL" in msg.data:
            self.get_logger().error(f'🚨 {msg.data}')
    
    # === MAIN CONTROL LOOP ===
    
    def control_loop(self):
        """50Hz control loop - state machine execution"""
        
        # Emergency override: Flare avoidance has highest priority
        if self.should_emergency_avoid_flare():
            if self.state != NavigationState.FLARE_EMERGENCY_AVOIDANCE:
                self.transition_to(NavigationState.FLARE_EMERGENCY_AVOIDANCE)
        
        # Execute current state
        cmd = Twist()
        
        if self.state == NavigationState.IDLE:
            cmd = self.handle_idle(cmd)
        elif self.state == NavigationState.SUBMERGING:
            cmd = self.handle_submerging(cmd)
        elif self.state == NavigationState.SEARCHING:
            cmd = self.handle_searching(cmd)
        elif self.state == NavigationState.VISUAL_ACQUISITION:
            cmd = self.handle_visual_acquisition(cmd)
        elif self.state == NavigationState.COARSE_ALIGNMENT:
            cmd = self.handle_coarse_alignment(cmd)
        elif self.state == NavigationState.APPROACH:
            cmd = self.handle_approach(cmd)
        elif self.state == NavigationState.FINE_ALIGNMENT:
            cmd = self.handle_fine_alignment(cmd)
        elif self.state == NavigationState.FINAL_APPROACH:
            cmd = self.handle_final_approach(cmd)
        elif self.state == NavigationState.PASSING:
            cmd = self.handle_passing(cmd)
        elif self.state == NavigationState.FLARE_EMERGENCY_AVOIDANCE:
            cmd = self.handle_emergency_avoidance(cmd)
        elif self.state == NavigationState.RECOVERY:
            cmd = self.handle_recovery(cmd)
        elif self.state == NavigationState.COMPLETED:
            cmd = self.handle_completed(cmd)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Publish status
        self.publish_status()
    
    # === STATE HANDLERS ===
    
    def handle_idle(self, cmd: Twist) -> Twist:
        """Idle state - waiting to start"""
        return cmd
    
    def start_mission(self):
        """Called by external controller to start gate navigation"""
        self.get_logger().info('🚀 Starting gate navigation mission')
        self.mission_start_time = time.time()
        self.transition_to(NavigationState.SUBMERGING)
    
    def handle_submerging(self, cmd: Twist) -> Twist:
        """Submerge to target depth"""
        depth_error = self.target_depth - self.current_depth
        elapsed = time.time() - self.state_start_time
        
        # Status logging
        if int(elapsed) % 2 == 0:
            self.get_logger().info(
                f'⬇ Submerging: {self.current_depth:.2f}m / {self.target_depth:.2f}m'
            )
        
        # Check if depth reached
        if abs(depth_error) < self.depth_tolerance:
            self.get_logger().info('✓ Target depth reached!')
            self.transition_to(NavigationState.SEARCHING)
            return cmd
        
        # Timeout check
        if elapsed > 30.0:
            self.get_logger().warn('⚠ Submerge timeout - proceeding')
            self.transition_to(NavigationState.SEARCHING)
            return cmd
        
        # Depth control with gentle forward motion
        cmd.linear.z = depth_error * self.depth_gain
        cmd.linear.z = np.clip(cmd.linear.z, -0.5, 0.5)
        cmd.linear.x = 0.15  # Gentle forward motion
        
        return cmd
    
    def handle_searching(self, cmd: Twist) -> Twist:
        """Search for gate with spiral pattern"""
        elapsed = time.time() - self.state_start_time
        
        # Check if gate detected
        if self.gate_detected and self.detection_confidence >= self.min_confidence:
            self.get_logger().info('✓ Gate detected during search!')
            self.transition_to(NavigationState.VISUAL_ACQUISITION)
            return cmd
        
        # Timeout check
        if elapsed > self.max_search_time:
            self.get_logger().error('❌ Search timeout - gate not found')
            self.transition_to(NavigationState.RECOVERY)
            return cmd
        
        # Spiral search pattern
        cmd.linear.z = self.get_depth_control()
        cmd.linear.x = self.search_speed
        cmd.angular.z = 0.15  # Gentle rotation
        
        if int(elapsed) % 3 == 0:
            self.get_logger().info(f'🔍 Searching for gate... {elapsed:.0f}s')
        
        return cmd
    
    def handle_visual_acquisition(self, cmd: Twist) -> Twist:
        """Initial gate acquisition - lock onto target"""
        
        # Check gate still visible
        if not self.gate_detected or self.detection_confidence < self.min_confidence:
            if self.time_since_gate_seen() > self.gate_lost_timeout:
                self.get_logger().warn('⚠ Gate lost - returning to search')
                self.transition_to(NavigationState.SEARCHING)
                return cmd
        
        # Check distance for state transition
        if self.gate_distance < self.coarse_alignment_distance:
            self.get_logger().info('→ Transitioning to coarse alignment')
            self.transition_to(NavigationState.COARSE_ALIGNMENT)
            return cmd
        
        # Acquisition behavior: gentle approach with loose alignment
        cmd.linear.z = self.get_depth_control()
        cmd.linear.x = self.acquisition_speed
        
        # Light yaw correction
        self.current_yaw_gain = self.yaw_gain_far
        cmd.angular.z = -self.alignment_error * self.current_yaw_gain
        cmd.angular.z = np.clip(cmd.angular.z, -0.3, 0.3)
        
        self.log_state_info('ACQUISITION', 5.0)
        
        return cmd
    
    def handle_coarse_alignment(self, cmd: Twist) -> Twist:
        """Coarse alignment at medium distance"""
        
        # Gate visibility check
        if not self.check_gate_visibility():
            return cmd
        
        # Distance-based transition
        if self.gate_distance < self.approach_distance:
            # Check if reasonably aligned before proceeding
            if abs(self.alignment_error) < self.coarse_alignment_threshold * 1.5:
                self.get_logger().info('→ Entering approach phase')
                self.transition_to(NavigationState.APPROACH)
            else:
                self.get_logger().warn(
                    f'⚠ Alignment needed: {math.degrees(self.alignment_error):.1f}°'
                )
        
        # Coarse alignment control
        cmd.linear.z = self.get_depth_control()
        cmd.linear.x = self.acquisition_speed * 0.8  # Slower for alignment
        
        self.current_yaw_gain = self.yaw_gain_medium
        cmd.angular.z = -self.alignment_error * self.current_yaw_gain
        cmd.angular.z = np.clip(cmd.angular.z, -0.4, 0.4)
        
        self.log_state_info('COARSE ALIGN', 3.0)
        
        return cmd
    
    def handle_approach(self, cmd: Twist) -> Twist:
        """Main approach phase with moderate alignment"""
        
        # Gate visibility check
        if not self.check_gate_visibility():
            return cmd
        
        # Distance-based transition
        if self.gate_distance < self.fine_alignment_distance:
            if abs(self.alignment_error) < self.fine_alignment_threshold:
                self.get_logger().info('→ Entering fine alignment')
                self.transition_to(NavigationState.FINE_ALIGNMENT)
            else:
                self.get_logger().warn(
                    f'⚠ Fine alignment required: {math.degrees(self.alignment_error):.1f}°'
                )
        
        # Approach control with continuous alignment correction
        cmd.linear.z = self.get_depth_control()
        cmd.linear.x = self.approach_speed
        
        self.current_yaw_gain = self.yaw_gain_near
        cmd.angular.z = -self.alignment_error * self.current_yaw_gain
        cmd.angular.z = np.clip(cmd.angular.z, -0.5, 0.5)
        
        # Apply flare avoidance if needed (non-emergency)
        if self.flare_detected and self.gate_distance > self.flare_critical_distance:
            cmd.linear.y = self.flare_avoidance_direction * 0.15
        
        self.log_state_info('APPROACH', 2.0)
        
        return cmd
    
    def handle_fine_alignment(self, cmd: Twist) -> Twist:
        """Fine alignment before final approach"""
        
        # Gate visibility check
        if not self.check_gate_visibility():
            return cmd
        
        # Check if ready for final approach
        if self.gate_distance < self.final_approach_distance:
            if abs(self.alignment_error) < self.passing_alignment_threshold:
                self.get_logger().info('→ Entering final approach')
                self.transition_to(NavigationState.FINAL_APPROACH)
            else:
                self.get_logger().warn(
                    f'⚠ Precise alignment needed: {math.degrees(self.alignment_error):.1f}°'
                )
        
        # Fine alignment control - precise corrections
        cmd.linear.z = self.get_depth_control()
        cmd.linear.x = self.fine_approach_speed
        
        self.current_yaw_gain = self.yaw_gain_fine
        cmd.angular.z = -self.alignment_error * self.current_yaw_gain
        cmd.angular.z = np.clip(cmd.angular.z, -0.6, 0.6)
        
        self.log_state_info('FINE ALIGN', 1.0)
        
        return cmd
    
    def handle_final_approach(self, cmd: Twist) -> Twist:
        """Final approach - maintain perfect alignment"""
        
        # Gate visibility check
        if not self.check_gate_visibility():
            return cmd
        
        # Check if ready to pass
        if self.gate_distance < self.passing_distance:
            if abs(self.alignment_error) < self.passing_alignment_threshold * 1.2:
                self.get_logger().info('🚀 PASSING THROUGH GATE!')
                self.transition_to(NavigationState.PASSING)
                return cmd
        
        # Final approach control - maintain alignment
        cmd.linear.z = self.get_depth_control()
        cmd.linear.x = self.fine_approach_speed * 1.2
        
        # Very high yaw gain for precise tracking
        cmd.angular.z = -self.alignment_error * self.current_yaw_gain * 1.2
        cmd.angular.z = np.clip(cmd.angular.z, -0.7, 0.7)
        
        self.log_state_info('FINAL APPROACH', 0.5)
        
        return cmd
    
    def handle_passing(self, cmd: Twist) -> Twist:
        """Pass through gate at full speed"""
        elapsed = time.time() - self.state_start_time
        
        # Check if passed through
        if elapsed > self.passing_duration:
            self.get_logger().info('✅ GATE PASSED SUCCESSFULLY!')
            self.log_performance_summary()
            self.transition_to(NavigationState.COMPLETED)
            return cmd
        
        # Full speed ahead with depth hold
        cmd.linear.z = self.get_depth_control()
        cmd.linear.x = self.passing_speed
        
        # Minimal yaw correction (momentum carries us through)
        if self.gate_detected:
            cmd.angular.z = -self.alignment_error * self.yaw_gain_medium * 0.5
            cmd.angular.z = np.clip(cmd.angular.z, -0.3, 0.3)
        
        progress = (elapsed / self.passing_duration) * 100
        self.get_logger().info(
            f'🚀 PASSING: {progress:.0f}% ({elapsed:.1f}s / {self.passing_duration:.1f}s)',
            throttle_duration_sec=0.5
        )
        
        return cmd
    
    def handle_emergency_avoidance(self, cmd: Twist) -> Twist:
        """Emergency flare avoidance - highest priority"""
        
        if self.emergency_avoidance_start is None:
            self.emergency_avoidance_start = time.time()
        
        elapsed = time.time() - self.emergency_avoidance_start
        
        # Check if avoidance complete
        if not self.flare_detected or elapsed > self.emergency_avoidance_duration:
            self.get_logger().info('✓ Emergency avoidance complete - resuming')
            self.emergency_avoidance_start = None
            self.transition_to(self.previous_state)
            return cmd
        
        # Emergency maneuver: Strong lateral motion + slow forward
        cmd.linear.z = self.get_depth_control()
        cmd.linear.y = self.flare_avoidance_direction * self.flare_avoidance_gain
        cmd.linear.x = self.search_speed * 0.5  # Slow forward
        cmd.angular.z = 0.0  # No rotation during emergency
        
        self.get_logger().error(
            f'🚨 EMERGENCY AVOIDANCE: Moving {["LEFT", "RIGHT"][int(self.flare_avoidance_direction > 0)]}',
            throttle_duration_sec=1.0
        )
        
        return cmd
    
    def handle_recovery(self, cmd: Twist) -> Twist:
        """Recovery behavior when gate is lost"""
        elapsed = time.time() - self.state_start_time
        
        # Check if gate reacquired
        if self.gate_detected and self.detection_confidence >= self.min_confidence:
            self.get_logger().info('✓ Gate reacquired!')
            # Return to appropriate state based on distance
            if self.gate_distance > self.coarse_alignment_distance:
                self.transition_to(NavigationState.VISUAL_ACQUISITION)
            elif self.gate_distance > self.approach_distance:
                self.transition_to(NavigationState.COARSE_ALIGNMENT)
            else:
                self.transition_to(NavigationState.APPROACH)
            return cmd
        
        # Recovery timeout
        if elapsed > 15.0:
            self.get_logger().error('❌ Recovery failed - returning to search')
            self.transition_to(NavigationState.SEARCHING)
            return cmd
        
        # Recovery pattern: Slow rotation and forward movement
        cmd.linear.z = self.get_depth_control()
        cmd.linear.x = self.search_speed * 0.6
        cmd.angular.z = -0.2  # Gentle reverse rotation
        
        self.get_logger().warn(f'⟲ Recovery mode... {elapsed:.1f}s', throttle_duration_sec=2.0)
        
        return cmd
    
    def handle_completed(self, cmd: Twist) -> Twist:
        """Mission complete - stop motion"""
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0
        return cmd
    
    # === HELPER FUNCTIONS ===
    
    def get_depth_control(self) -> float:
        """Get depth control command"""
        depth_error = self.target_depth - self.current_depth
        cmd_z = depth_error * self.depth_gain
        return np.clip(cmd_z, -0.6, 0.6)
    
    def check_gate_visibility(self) -> bool:
        """Check if gate is still visible"""
        if not self.gate_detected or self.detection_confidence < self.min_confidence:
            if self.time_since_gate_seen() > self.gate_lost_timeout:
                self.get_logger().warn('⚠ Gate lost - entering recovery')
                self.transition_to(NavigationState.RECOVERY)
                return False
        return True
    
    def should_emergency_avoid_flare(self) -> bool:
        """Check if emergency flare avoidance is needed"""
        if not self.flare_detected:
            return False
        
        # Check if in critical states and flare is close
        critical_states = [
            NavigationState.APPROACH,
            NavigationState.FINE_ALIGNMENT,
            NavigationState.FINAL_APPROACH
        ]
        
        if self.state in critical_states:
            if "CRITICAL" in self.flare_warning_level:
                return True
            if self.gate_distance < self.flare_critical_distance:
                return True
        
        return False
    
    def time_since_gate_seen(self) -> float:
        """Time since gate was last seen"""
        if self.gate_last_seen_time is None:
            return 999.0
        return time.time() - self.gate_last_seen_time
    
    def log_state_info(self, state_name: str, interval: float):
        """Log current state information"""
        elapsed = time.time() - self.state_start_time
        if int(elapsed * 10) % int(interval * 10) == 0:
            self.get_logger().info(
                f'{state_name}: D={self.gate_distance:.2f}m, '
                f'Align={math.degrees(self.alignment_error):.1f}°, '
                f'Conf={self.detection_confidence:.2f}, '
                f'Gain={self.current_yaw_gain:.1f}'
            )
    
    def log_performance_summary(self):
        """Log mission performance summary"""
        total_time = time.time() - self.mission_start_time
        
        avg_alignment = np.mean(self.alignment_history) if self.alignment_history else 0
        max_alignment = np.max(np.abs(self.alignment_history)) if self.alignment_history else 0
        
        self.get_logger().info('='*70)
        self.get_logger().info('📊 GATE NAVIGATION PERFORMANCE SUMMARY')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Total mission time: {total_time:.2f}s')
        self.get_logger().info(
            f'Alignment performance: Avg={math.degrees(avg_alignment):.2f}°, '
            f'Max={math.degrees(max_alignment):.2f}°'
        )
        self.get_logger().info(f'Final confidence: {self.detection_confidence:.2f}')
        self.get_logger().info('='*70)
    
    def transition_to(self, new_state: NavigationState):
        """Transition to new state"""
        state_names = {
            0: 'IDLE', 1: 'SUBMERGING', 2: 'SEARCHING', 3: 'VISUAL_ACQUISITION',
            4: 'COARSE_ALIGNMENT', 5: 'APPROACH', 6: 'FINE_ALIGNMENT',
            7: 'FINAL_APPROACH', 8: 'PASSING', 9: 'FLARE_EMERGENCY_AVOIDANCE',
            10: 'COMPLETED', 11: 'RECOVERY'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info(f'🔄 STATE: {old_name} → {new_name}')
        
        self.previous_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
    
    def publish_status(self):
        """Publish current navigation status"""
        state_names = {
            0: 'IDLE', 1: 'SUBMERGING', 2: 'SEARCHING', 3: 'VISUAL_ACQUISITION',
            4: 'COARSE_ALIGNMENT', 5: 'APPROACH', 6: 'FINE_ALIGNMENT',
            7: 'FINAL_APPROACH', 8: 'PASSING', 9: 'FLARE_EMERGENCY_AVOIDANCE',
            10: 'COMPLETED', 11: 'RECOVERY'
        }
        
        state_msg = String()
        state_msg.data = state_names.get(self.state, 'UNKNOWN')
        self.state_pub.publish(state_msg)
        
        status_msg = String()
        status_msg.data = (
            f"State: {state_names.get(self.state, 'UNKNOWN')} | "
            f"Distance: {self.gate_distance:.2f}m | "
            f"Alignment: {math.degrees(self.alignment_error):.1f}° | "
            f"Confidence: {self.detection_confidence:.2f} | "
            f"Depth: {self.current_depth:.2f}m"
        )
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProfessionalGateNavigator()
    
    # Auto-start mission after initialization
    time.sleep(1.0)
    node.start_mission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop on shutdown
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.get_logger().info('Gate Navigator shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()