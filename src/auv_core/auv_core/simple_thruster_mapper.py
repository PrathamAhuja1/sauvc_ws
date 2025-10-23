#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np


class SimpleThrusterMapper(Node):
    def __init__(self):
        super().__init__('simple_thruster_mapper')
        
        # Parameters
        self.declare_parameter('max_thrust', 100.0)
        self.declare_parameter('thrust_scale', 10.0)
        self.declare_parameter('vertical_thrust_boost', 3.0)
        
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
        
        # Publishers for each thruster (1-6)
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/thruster{i}_cmd', 10)
            for i in range(1, 7)
        ]
        
        # Stats
        self.cmd_count = 0
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_vel = None
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.print_stats)
        
        self.get_logger().info('='*60)
        self.get_logger().info('FIXED Simple Thruster Mapper initialized')
        self.get_logger().info(f'Max thrust: {self.max_thrust}')
        self.get_logger().info(f'Thrust scale: {self.thrust_scale}')
        self.get_logger().info(f'Vertical boost: {self.vertical_boost}')
        self.get_logger().info('='*60)
    
    def print_stats(self):
        time_since_last = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last > 3.0:
            self.get_logger().warn(f'⚠️  No cmd_vel received for {time_since_last:.1f}s')
        else:
            self.get_logger().info(f'✓ Processed {self.cmd_count} commands (last: {time_since_last:.1f}s ago)')
        
        if self.last_cmd_vel:
            self.get_logger().info(
                f'Last cmd: vx={self.last_cmd_vel.linear.x:.2f}, '
                f'vy={self.last_cmd_vel.linear.y:.2f}, '
                f'vz={self.last_cmd_vel.linear.z:.2f}, '
                f'yaw={self.last_cmd_vel.angular.z:.2f}'
            )
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Map Twist to thruster commands
        
        CRITICAL SIGN CONVENTION:
        - cmd_vel.linear.z: NEGATIVE = go DOWN (standard AUV convention)
        - Gazebo thruster: POSITIVE angular velocity = UPWARD thrust
        - Therefore: NO SIGN INVERSION needed for vertical thrusters!
        """
        self.cmd_count += 1
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_vel = msg
        
        # Extract velocities
        vx = msg.linear.x   # Forward/backward
        vy = msg.linear.y   # Left/right (sway)
        vz = msg.linear.z   # Up/down (NEGATIVE = DOWN in AUV convention)
        yaw = msg.angular.z # Rotation
        
        # Apply scaling
        vx_scaled = vx * self.thrust_scale
        vy_scaled = vy * self.thrust_scale
        vz_scaled = vz * self.thrust_scale * self.vertical_boost
        yaw_scaled = yaw * self.thrust_scale
        
        # Initialize thrust array
        thrust = np.zeros(6)
        
        # ========== HORIZONTAL THRUSTERS (1-4) ==========
        # BlueROV2/Orca4 thruster layout (vectored at 45°):
        # T1: Front-Left  (affects +X, -Y, +Yaw)
        # T2: Front-Right (affects +X, +Y, -Yaw)
        # T3: Back-Left   (affects -X, +Y, +Yaw) - REVERSED in URDF
        # T4: Back-Right  (affects -X, -Y, -Yaw) - REVERSED in URDF
        
        thrust[0] = vx_scaled - vy_scaled + yaw_scaled    # T1: Front-Left
        thrust[1] = vx_scaled + vy_scaled - yaw_scaled    # T2: Front-Right
        thrust[2] = -vx_scaled + vy_scaled + yaw_scaled   # T3: Back-Left (reversed)
        thrust[3] = -vx_scaled - vy_scaled - yaw_scaled   # T4: Back-Right (reversed)
        
        # ========== VERTICAL THRUSTERS (5-6) ==========
        # CRITICAL FIX: Do NOT invert sign!
        # cmd_vel.linear.z convention: negative = down
        # Gazebo expects: positive angular velocity for upward thrust
        # The thruster plugin and hydrodynamics handle the actual physics
        
        # Direct mapping (no inversion):
        thrust[4] = vz_scaled  # T5: Vertical-Left
        thrust[5] = vz_scaled  # T6: Vertical-Right
        
        # Apply limits
        thrust = np.clip(thrust, -self.max_thrust, self.max_thrust)
        
        # Publish commands
        for i, pub in enumerate(self.thruster_pubs):
            cmd_msg = Float64()
            cmd_msg.data = float(thrust[i])
            pub.publish(cmd_msg)
        
        # Enhanced logging (every command when vertical thrust is active)
        if abs(vz) > 0.01 or self.cmd_count % 50 == 0:
            self.get_logger().info(
                f'🚀 CMD #{self.cmd_count}: '
                f'Input(vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw={yaw:.2f}) -> '
                f'Thrust[T1={thrust[0]:.1f}, T2={thrust[1]:.1f}, T3={thrust[2]:.1f}, '
                f'T4={thrust[3]:.1f}, T5={thrust[4]:.1f}, T6={thrust[5]:.1f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleThrusterMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down thruster mapper...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()