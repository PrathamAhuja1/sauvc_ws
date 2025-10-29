#!/usr/bin/env python3
"""
Fixed Thruster Mapper for Orca4 AUV
Converts Twist commands to individual thruster forces
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np


class FixedThrusterMapper(Node):
    def __init__(self):
        super().__init__('fixed_thruster_mapper')
        
        # Parameters
        self.declare_parameter('max_thrust', 300.0)
        self.declare_parameter('thrust_scale', 50.0)
        self.declare_parameter('vertical_boost', 3.0)
        
        self.max_thrust = self.get_parameter('max_thrust').value
        self.thrust_scale = self.get_parameter('thrust_scale').value
        self.vertical_boost = self.get_parameter('vertical_boost').value
        
        # CORRECTED Thruster Allocation Matrix
        # Based on BlueROV2 Heavy vectored configuration
        # Format: [Surge, Sway, Heave, Roll, Pitch, Yaw]
        # T1: Front-Left (45° angle)
        # T2: Front-Right (45° angle)
        # T3: Back-Left (45° angle)
        # T4: Back-Right (45° angle)
        # T5: Vertical-Left
        # T6: Vertical-Right
        
        self.TAM = np.array([
            # Surge, Sway,  Heave, Roll,  Pitch, Yaw
            [  0.707, -0.707, 0,     0,     0,    -1],  # T1: FL
            [  0.707,  0.707, 0,     0,     0,     1],  # T2: FR
            [ -0.707,  0.707, 0,     0,     0,    -1],  # T3: BL
            [ -0.707, -0.707, 0,     0,     0,     1],  # T4: BR
            [  0,      0,     1,     0.3,   0.2,   0],  # T5: VL
            [  0,      0,     1,    -0.3,  -0.2,   0],  # T6: VR
        ], dtype=np.float64)
        
        # Pseudo-inverse for allocation
        self.TAM_pinv = np.linalg.pinv(self.TAM)
        
        # Command tracking
        self.last_cmd_time = self.get_clock().now()
        
        # Subscriber
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10)
        
        # Publishers for each thruster
        self.thruster_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(
                Float64, 
                f'/thruster{i}_cmd', 
                10
            )
            self.thruster_pubs.append(pub)
        
        # Watchdog timer to detect if commands stop
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)
        
        self.get_logger().info('='*70)
        self.get_logger().info('Fixed Thruster Mapper initialized')
        self.get_logger().info(f'Max thrust: {self.max_thrust}')
        self.get_logger().info(f'Thrust scale: {self.thrust_scale}')
        self.get_logger().info(f'Vertical boost: {self.vertical_boost}')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):
        """Convert Twist to thruster commands"""
        
        self.last_cmd_time = self.get_clock().now()
        
        # Build wrench vector from Twist
        wrench = np.array([
            msg.linear.x * self.thrust_scale,      # Surge
            msg.linear.y * self.thrust_scale,      # Sway
            msg.linear.z * self.thrust_scale * self.vertical_boost,  # Heave
            0.0,                                    # Roll
            0.0,                                    # Pitch
            msg.angular.z * self.thrust_scale * 0.5,  # Yaw
        ])
        
        # Allocate to thrusters
        raw_thrusts = self.TAM_pinv @ wrench
        
        # Apply saturation
        final_thrusts = np.clip(raw_thrusts, -self.max_thrust, self.max_thrust)
        
        # Publish to each thruster
        for i, (pub, thrust) in enumerate(zip(self.thruster_pubs, final_thrusts)):
            thrust_msg = Float64()
            thrust_msg.data = float(thrust)
            pub.publish(thrust_msg)
        
        # Log significant commands
        if np.any(np.abs(final_thrusts) > 5.0):
            self.get_logger().info(
                f'CMD: [{msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}] '
                f'→ T: [{final_thrusts[0]:.0f}, {final_thrusts[1]:.0f}, '
                f'{final_thrusts[2]:.0f}, {final_thrusts[3]:.0f}, '
                f'{final_thrusts[4]:.0f}, {final_thrusts[5]:.0f}]',
                throttle_duration_sec=0.5
            )
    
    def watchdog_check(self):
        """Check if commands are being received"""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_cmd > 2.0:
            self.get_logger().warn(
                f'⚠️  No commands received for {time_since_cmd:.1f}s',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = FixedThrusterMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()