#!/usr/bin/env python3
"""
Simple Thruster Mapper Node - FIXED VERSION
Converts Twist commands (cmd_vel) to individual thruster commands for 6-thruster configuration
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np


class SimpleThrusterMapper(Node):
    def __init__(self):
        super().__init__('simple_thruster_mapper')
        
        # Parameters
        self.declare_parameter('max_thrust', 10.0)
        self.declare_parameter('thrust_scale', 3.0)  # Scaling factor to increase thrust
        
        self.max_thrust = self.get_parameter('max_thrust').value
        self.thrust_scale = self.get_parameter('thrust_scale').value
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/rp2040/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers for each thruster
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/thruster{i}_cmd', 10)
            for i in range(1, 7)
        ]
        
        # Stats
        self.cmd_count = 0
        self.timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('Simple Thruster Mapper initialized')
        self.get_logger().info(f'Max thrust: {self.max_thrust}, Scale: {self.thrust_scale}')
    
    def print_stats(self):
        self.get_logger().info(f'Thruster Mapper: Processed {self.cmd_count} cmd_vel messages')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Map Twist to thruster commands
        Twist frame: X=forward, Y=left, Z=up, Yaw=rotation about Z
        """
        self.cmd_count += 1
        
        # Extract velocities
        vx = msg.linear.x   # Forward/backward
        vy = msg.linear.y   # Left/right (sway)
        vz = msg.linear.z   # Up/down
        yaw = msg.angular.z # Rotation
        
        # Apply scaling
        vx *= self.thrust_scale
        vy *= self.thrust_scale
        vz *= self.thrust_scale
        yaw *= self.thrust_scale
        
        # Thruster allocation matrix for BlueROV2/Orca4 configuration
        # Thrusters 1-4 are at 45-degree angles
        # Simplified model based on URDF configuration:
        # Thruster 1: Front-left 
        # Thruster 2: Front-right
        # Thruster 3: Back-left 
        # Thruster 4: Back-right
        # Thrusters 5-6: Vertical
        
        thrust = np.zeros(6)
        
        # Horizontal thrusters (1-4) - Vectored configuration
        # Each thruster contributes to surge, sway, and yaw
        
        # Thruster 1 (Front-Left, -45 deg)
        thrust[0] = vx - vy + yaw
        
        # Thruster 2 (Front-Right, -135 deg) 
        thrust[1] = vx + vy - yaw
        
        # Thruster 3 (Back-Left, 45 deg) - reversed in URDF
        thrust[2] = -vx + vy + yaw
        
        # Thruster 4 (Back-Right, 135 deg) - reversed in URDF
        thrust[3] = -vx - vy - yaw
        
        # Vertical thrusters (5-6)
        thrust[4] = vz
        thrust[5] = vz
        
        # Apply limits
        for i in range(6):
            thrust[i] = np.clip(thrust[i], -self.max_thrust, self.max_thrust)
        
        # Publish commands
        for i, pub in enumerate(self.thruster_pubs):
            cmd_msg = Float64()
            cmd_msg.data = float(thrust[i])
            pub.publish(cmd_msg)
        
        # Log periodically (every 20 messages)
        if self.cmd_count % 20 == 0:
            self.get_logger().info(
                f'Thrust: T1={thrust[0]:.2f}, T2={thrust[1]:.2f}, T3={thrust[2]:.2f}, '
                f'T4={thrust[3]:.2f}, T5={thrust[4]:.2f}, T6={thrust[5]:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleThrusterMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()