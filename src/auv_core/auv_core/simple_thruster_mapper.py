#!/usr/bin/env python3
"""
Simple Thruster Mapper Node
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
        self.max_thrust = self.get_parameter('max_thrust').value
        
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
        
        # Thruster configuration for BlueROV2/Orca4
        # Thrusters 1-4: Horizontal (vectored thrust)
        # Thrusters 5-6: Vertical
        
        self.get_logger().info('Simple Thruster Mapper initialized')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Map Twist to thruster commands
        Twist frame: X=forward, Y=left, Z=up, Yaw=rotation about Z
        """
        # Extract velocities
        vx = msg.linear.x   # Forward/backward
        vy = msg.linear.y   # Left/right (sway)
        vz = msg.linear.z   # Up/down
        yaw = msg.angular.z # Rotation
        
        # Thruster allocation matrix for BlueROV2/Orca4 configuration
        # Thrusters 1-4 are at 45-degree angles
        # Simplified model:
        
        # Horizontal thrusters (1-4)
        # Thruster 1: Front-left (135 deg)
        # Thruster 2: Front-right (45 deg)
        # Thruster 3: Back-left (-135 deg)
        # Thruster 4: Back-right (-45 deg)
        
        thrust = np.zeros(6)
        
        # Forward/Backward (surge)
        thrust[0] += vx  # Thruster 1
        thrust[1] += vx  # Thruster 2
        thrust[2] -= vx  # Thruster 3 (reverse)
        thrust[3] -= vx  # Thruster 4 (reverse)
        
        # Left/Right (sway)
        thrust[0] += vy  # Thruster 1
        thrust[1] -= vy  # Thruster 2
        thrust[2] -= vy  # Thruster 3
        thrust[3] += vy  # Thruster 4
        
        # Yaw (rotation)
        thrust[0] += yaw  # Thruster 1
        thrust[1] -= yaw  # Thruster 2
        thrust[2] += yaw  # Thruster 3
        thrust[3] -= yaw  # Thruster 4
        
        # Vertical thrusters (5-6)
        thrust[4] = vz  # Thruster 5
        thrust[5] = vz  # Thruster 6
        
        # Scale and limit thrust
        max_horizontal = max(abs(thrust[0]), abs(thrust[1]), abs(thrust[2]), abs(thrust[3]))
        if max_horizontal > self.max_thrust:
            scale = self.max_thrust / max_horizontal
            thrust[0:4] *= scale
        
        max_vertical = max(abs(thrust[4]), abs(thrust[5]))
        if max_vertical > self.max_thrust:
            scale = self.max_thrust / max_vertical
            thrust[4:6] *= scale
        
        # Publish commands
        for i, pub in enumerate(self.thruster_pubs):
            msg = Float64()
            msg.data = float(thrust[i])
            pub.publish(msg)


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