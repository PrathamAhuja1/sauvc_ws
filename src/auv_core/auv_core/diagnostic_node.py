#!/usr/bin/env python3
"""
Diagnostic Node - Monitors AUV state and publishes diagnostics
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
import math

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')
        
        # State variables
        self.odom_received = False
        self.gate_detected = False
        self.cmd_vel_received = False
        self.thruster_cmds_received = [False] * 6
        
        self.last_odom = None
        self.last_cmd_vel = None
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.gt_odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.gt_odom_callback, 10)
        
        self.gate_sub = self.create_subscription(
            Bool, '/gate/detected', self.gate_callback, 10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/rp2040/cmd_vel', self.cmd_vel_callback, 10)
        
        # Monitor thruster commands
        for i in range(1, 7):
            self.create_subscription(
                Float32, f'/thruster{i}_cmd', 
                lambda msg, idx=i-1: self.thruster_callback(msg, idx), 10)
        
        # Status timer
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('Diagnostic Node started')
    
    def odom_callback(self, msg: Odometry):
        self.odom_received = True
        self.last_odom = msg
    
    def gt_odom_callback(self, msg: Odometry):
        # Ground truth always available
        pass
    
    def gate_callback(self, msg: Bool):
        if msg.data and not self.gate_detected:
            self.get_logger().info('🎯 GATE DETECTED!')
        self.gate_detected = msg.data
    
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel_received = True
        self.last_cmd_vel = msg
    
    def thruster_callback(self, msg: Float32, idx: int):
        self.thruster_cmds_received[idx] = True
    
    def print_status(self):
        self.get_logger().info('='*60)
        self.get_logger().info('🔍 AUV DIAGNOSTIC STATUS')
        self.get_logger().info('='*60)
        
        # Odometry status
        if self.odom_received and self.last_odom:
            pos = self.last_odom.pose.pose.position
            self.get_logger().info(f'✓ Odometry: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}')
        else:
            self.get_logger().warn('✗ No filtered odometry received!')
        
        # Gate detection
        status = '✓' if self.gate_detected else '✗'
        self.get_logger().info(f'{status} Gate Detected: {self.gate_detected}')
        
        # Command velocity
        if self.cmd_vel_received and self.last_cmd_vel:
            cv = self.last_cmd_vel
            magnitude = math.sqrt(cv.linear.x**2 + cv.linear.y**2 + cv.linear.z**2)
            self.get_logger().info(
                f'✓ Cmd Vel: vx={cv.linear.x:.2f}, vy={cv.linear.y:.2f}, '
                f'vz={cv.linear.z:.2f}, yaw={cv.angular.z:.2f} (mag={magnitude:.2f})'
            )
        else:
            self.get_logger().warn('✗ No cmd_vel commands received!')
        
        # Thruster commands
        active_thrusters = sum(self.thruster_cmds_received)
        if active_thrusters > 0:
            self.get_logger().info(f'✓ Thrusters active: {active_thrusters}/6')
        else:
            self.get_logger().warn('✗ No thruster commands received!')
        
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()