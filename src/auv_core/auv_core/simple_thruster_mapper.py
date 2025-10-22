#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np


class SimpleThrusterMapper(Node):
    def __init__(self):
        super().__init__('simple_thruster_mapper')
        
        # Parameters - INCREASED for stronger thrust
        self.declare_parameter('max_thrust', 10.0)
        self.declare_parameter('thrust_scale', 5.0)  # INCREASED from 3.0 to 5.0
        self.declare_parameter('vertical_thrust_boost', 2.0)  # NEW: Extra boost for vertical
        
        self.max_thrust = self.get_parameter('max_thrust').value
        self.thrust_scale = self.get_parameter('thrust_scale').value
        self.vertical_boost = self.get_parameter('vertical_thrust_boost').value
        
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
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('Simple Thruster Mapper initialized')
        self.get_logger().info(f'Max thrust: {self.max_thrust}, Scale: {self.thrust_scale}, Vertical Boost: {self.vertical_boost}')
    
    def print_stats(self):
        self.get_logger().info(f'Thruster Mapper: Processed {self.cmd_count} cmd_vel messages')
        
        # Check if we're receiving commands
        time_since_last = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_last > 2.0:
            self.get_logger().warn(f'No cmd_vel received for {time_since_last:.1f}s')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Map Twist to thruster commands
        Twist frame: X=forward, Y=left, Z=up, Yaw=rotation about Z
        """
        self.cmd_count += 1
        self.last_cmd_time = self.get_clock().now()
        
        # Extract velocities
        vx = msg.linear.x   # Forward/backward
        vy = msg.linear.y   # Left/right (sway)
        vz = msg.linear.z   # Up/down
        yaw = msg.angular.z # Rotation
        
        # Apply scaling
        vx *= self.thrust_scale
        vy *= self.thrust_scale
        vz *= self.thrust_scale * self.vertical_boost  # EXTRA BOOST for vertical
        yaw *= self.thrust_scale
        
        # Initialize thrust array
        thrust = np.zeros(6)
        
        # Horizontal thrusters (1-4) - Vectored configuration
        # BlueROV2/Orca4 thruster layout:
        # Thruster 1: Front-Left (-45°)
        # Thruster 2: Front-Right (-135°) 
        # Thruster 3: Back-Left (45°) - reversed in URDF
        # Thruster 4: Back-Right (135°) - reversed in URDF
        
        # Thruster 1 (Front-Left, -45 deg)
        thrust[0] = vx - vy + yaw
        
        # Thruster 2 (Front-Right, -135 deg) 
        thrust[1] = vx + vy - yaw
        
        # Thruster 3 (Back-Left, 45 deg) - reversed in URDF
        thrust[2] = -vx + vy + yaw
        
        # Thruster 4 (Back-Right, 135 deg) - reversed in URDF
        thrust[3] = -vx - vy - yaw
        
        # Vertical thrusters (5-6) - APPLY DIRECTLY
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
        
        # Log periodically (every 20 messages) or if vertical thrust is active
        if self.cmd_count % 20 == 0 or abs(vz) > 0.1:
            self.get_logger().info(
                f'Thrust: T1={thrust[0]:.2f}, T2={thrust[1]:.2f}, T3={thrust[2]:.2f}, '
                f'T4={thrust[3]:.2f}, T5={thrust[4]:.2f}, T6={thrust[5]:.2f} | '
                f'Input vz={msg.linear.z:.2f}'
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