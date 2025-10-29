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
        self.declare_parameter('max_thrust', 150.0)
        self.declare_parameter('thrust_scale', 10.0)
        self.declare_parameter('vertical_boost', 1.5) 
        
        self.max_thrust = self.get_parameter('max_thrust').value
        self.thrust_scale = self.get_parameter('thrust_scale').value
        self.vertical_boost = self.get_parameter('vertical_boost').value

        
        # For a VECTORED thruster configuration:
        # - All 4 horizontal thrusters contribute to BOTH surge AND yaw
        # - Diagonal thrust vectors must be properly decomposed
        

        # Format: [Surge, Sway, Heave, Roll, Pitch, Yaw]
        self.TAM = np.array([

            [  0.707,  -0.707,   0,    0,    0,    -0.163],

            [  0.707,   0.707,   0,    0,    0,     0.163],

            [ -0.707,   0.707,   0,    0,    0,    -0.034],

            [ -0.707,  -0.707,   0,    0,    0,     0.034],

            [  0,       0,       1,    0,    0.109,  0],
            [  0,       0,       1,    0,   -0.109,  0],
        ], dtype=np.float64)
        
        # Pseudo-inverse for allocation
        self.TAM_pinv = np.linalg.pinv(self.TAM)
        
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
        
        self.get_logger().info('Fixed Thruster Mapper initialized')
        self.get_logger().info(f'Max thrust: {self.max_thrust}')
        self.get_logger().info(f'Thrust scale: {self.thrust_scale}')
    
    def twist_callback(self, msg: Twist):
        """Convert Twist to thruster commands"""
        
        # Build wrench vector from Twist
        # Apply scaling to convert velocities to forces
        wrench = np.array([
            msg.linear.x * self.thrust_scale,      # Surge
            msg.linear.y * self.thrust_scale,      # Sway
            msg.linear.z * self.thrust_scale * self.vertical_boost,  # Heave (boosted)
            0.0,                                    # Roll (not used)
            0.0,                                    # Pitch (not used)
            msg.angular.z * self.thrust_scale*0.1,     # Yaw
        ])
        
        # Allocate to thrusters
        raw_thrusts = self.TAM_pinv @ wrench
        
        # Apply saturation
        final_thrusts = np.clip(raw_thrusts, -self.max_thrust, self.max_thrust)
        
        # Publish to each thruster
        for i, (pub, thrust) in enumerate(zip(self.thruster_pubs, final_thrusts)):
            msg = Float64()
            msg.data = float(thrust)
            pub.publish(msg)
        
        # Log for debugging
        if np.any(np.abs(final_thrusts) > 1.0):
            self.get_logger().info(
                f'Thrusters: T1={final_thrusts[0]:.1f} T2={final_thrusts[1]:.1f} '
                f'T3={final_thrusts[2]:.1f} T4={final_thrusts[3]:.1f} '
                f'T5={final_thrusts[4]:.1f} T6={final_thrusts[5]:.1f}',
                throttle_duration_sec=1.0
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