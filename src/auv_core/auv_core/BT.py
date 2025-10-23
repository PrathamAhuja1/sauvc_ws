#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time


class SimplifiedMissionController(Node):
    """Simplified mission controller that focuses on basic depth control and navigation"""
    
    def __init__(self):
        super().__init__('auv_behavior_tree_node')
        
        # Mission states
        self.INIT = 0
        self.SUBMERGING = 1
        self.SEARCHING_GATE = 2
        self.COMPLETED = 3
        
        self.state = self.INIT
        
        # Target parameters
        self.target_depth = -1.5
        self.depth_tolerance = 0.15
        self.max_depth_error_time = 30.0  # Maximum time to reach depth
        
        # State tracking
        self.current_depth = 0.0
        self.state_start_time = time.time()
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('🚀 Simplified Mission Controller Started!')
        self.get_logger().info(f'📍 Initial State: INIT')
    
    def odom_callback(self, msg: Odometry):
        """Update current depth from odometry"""
        self.current_depth = msg.pose.pose.position.z
    
    def control_loop(self):
        """Main control loop"""
        
        if self.state == self.INIT:
            self.handle_init()
        elif self.state == self.SUBMERGING:
            self.handle_submerging()
        elif self.state == self.SEARCHING_GATE:
            self.handle_searching()
        elif self.state == self.COMPLETED:
            self.handle_completed()
    
    def handle_init(self):
        """Initialize and transition to submerging"""
        self.get_logger().info('📍 State: INIT → Starting mission...')
        time.sleep(0.5)  # Brief pause
        self.transition_to(self.SUBMERGING)
    
    def handle_submerging(self):
        """Actively submerge to target depth"""
        depth_error = self.target_depth - self.current_depth
        elapsed = time.time() - self.state_start_time
        
        # Log progress
        if int(elapsed) % 2 == 0:  # Every 2 seconds
            self.get_logger().info(
                f'⬇️  Submerging: Current={self.current_depth:.2f}m, '
                f'Target={self.target_depth:.2f}m, Error={depth_error:.2f}m'
            )
        
        # Check if depth reached
        if abs(depth_error) < self.depth_tolerance:
            self.get_logger().info(f'✅ Target depth reached: {self.current_depth:.2f}m')
            self.transition_to(self.SEARCHING_GATE)
            return
        
        # Check timeout
        if elapsed > self.max_depth_error_time:
            self.get_logger().error(
                f'⚠️  Submerging timeout! Still at {self.current_depth:.2f}m after {elapsed:.1f}s'
            )
            self.transition_to(self.SEARCHING_GATE)  # Continue anyway
            return
        
        # Apply STRONG downward thrust
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        
        # CRITICAL FIX: In standard AUV/ROS convention:
        # - Negative Z velocity = go DOWN
        # - Positive Z velocity = go UP
        # Since target_depth is NEGATIVE (-1.5m), and we're at 0m,
        # depth_error = -1.5 - 0 = -1.5 (negative)
        # We want to go DOWN, so we need NEGATIVE thrust
        
        # Strong downward thrust (negative Z is down)
        base_thrust = -1.5  # Increased from -1.2
        time_factor = min(2.0, 1.0 + elapsed / 15.0)  # Ramp up faster
        cmd.linear.z = base_thrust * time_factor
        
        cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
        
        # Debug logging
        if elapsed < 5.0 or int(elapsed) % 5 == 0:
            self.get_logger().info(
                f'Thrust command: vz={cmd.linear.z:.2f} (factor={time_factor:.2f})'
            )
    
    def handle_searching(self):
        """Search for gate while maintaining depth"""
        elapsed = time.time() - self.state_start_time
        
        if int(elapsed) % 5 == 0:  # Every 5 seconds
            self.get_logger().info(
                f'🔍 Searching for gate... Depth: {self.current_depth:.2f}m, '
                f'Time: {elapsed:.1f}s'
            )
        
        # Timeout after 60 seconds
        if elapsed > 60.0:
            self.get_logger().info('Search timeout - mission complete')
            self.transition_to(self.COMPLETED)
            return
        
        # Maintain depth + slow forward motion
        cmd = Twist()
        
        # Depth correction
        depth_error = self.target_depth - self.current_depth
        cmd.linear.z = depth_error * 1.0  # Proportional control
        
        # Slow forward search
        cmd.linear.x = 0.3
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
    
    def handle_completed(self):
        """Mission complete - stop all motion"""
        cmd = Twist()  # All zeros
        self.cmd_vel_pub.publish(cmd)
        
        # Log once
        if int(time.time() - self.state_start_time) == 0:
            self.get_logger().info('✅ Mission Complete!')
    
    def transition_to(self, new_state):
        """Transition to a new state"""
        state_names = {
            self.INIT: 'INIT',
            self.SUBMERGING: 'SUBMERGING',
            self.SEARCHING_GATE: 'SEARCHING_GATE',
            self.COMPLETED: 'COMPLETED'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info(f'🔄 STATE CHANGE: {old_name} → {new_name}')
        
        self.state = new_state
        self.state_start_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = SimplifiedMissionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down mission controller...')
    finally:
        # Stop all motion on shutdown
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()