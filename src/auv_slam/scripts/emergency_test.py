#!/usr/bin/env python3
"""
Test script to verify autonomous movement pipeline
Tests: Twist → Thruster Mapper → Bridge → Gazebo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import time


class MovementTester(Node):
    def __init__(self):
        super().__init__('movement_tester')
        
        # State tracking
        self.initial_position = None
        self.current_position = None
        self.thruster_values = [0.0] * 6
        self.test_phase = 0
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        for i in range(6):
            self.create_subscription(
                Float64, f'/thruster{i+1}_cmd',
                lambda msg, idx=i: self.thruster_callback(msg, idx), 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_input', 10)
        
        # Test timer (every 5 seconds)
        self.timer = self.create_timer(5.0, self.run_test)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🧪 MOVEMENT TEST STARTED')
        self.get_logger().info('='*70)
    
    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        
        if self.initial_position is None:
            self.initial_position = (pos.x, pos.y, pos.z)
            self.get_logger().info(
                f'📍 Initial: X={pos.x:.3f}, Y={pos.y:.3f}, Z={pos.z:.3f}'
            )
        
        self.current_position = (pos.x, pos.y, pos.z)
    
    def thruster_callback(self, msg: Float64, idx: int):
        self.thruster_values[idx] = msg.data
    
    def run_test(self):
        """Run systematic tests"""
        
        if self.test_phase == 0:
            self.get_logger().info('='*70)
            self.get_logger().info('TEST 1: Downward Thrust')
            self.get_logger().info('='*70)
            cmd = Twist()
            cmd.linear.z = -1.0  # DOWN
            self.cmd_vel_pub.publish(cmd)
            
        elif self.test_phase == 1:
            self.check_movement('Downward')
            
            self.get_logger().info('='*70)
            self.get_logger().info('TEST 2: Upward Thrust')
            self.get_logger().info('='*70)
            cmd = Twist()
            cmd.linear.z = 1.0  # UP
            self.cmd_vel_pub.publish(cmd)
            
        elif self.test_phase == 2:
            self.check_movement('Upward')
            
            self.get_logger().info('='*70)
            self.get_logger().info('TEST 3: Forward Thrust')
            self.get_logger().info('='*70)
            cmd = Twist()
            cmd.linear.x = 0.5  # FORWARD
            cmd.linear.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            
        elif self.test_phase == 3:
            self.check_movement('Forward')
            
            self.get_logger().info('='*70)
            self.get_logger().info('TEST 4: Yaw Rotation')
            self.get_logger().info('='*70)
            cmd = Twist()
            cmd.angular.z = 0.5  # YAW
            self.cmd_vel_pub.publish(cmd)
            
        elif self.test_phase == 4:
            self.check_movement('Yaw')
            
            self.get_logger().info('='*70)
            self.get_logger().info('TEST 5: Stop')
            self.get_logger().info('='*70)
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
        elif self.test_phase == 5:
            self.print_final_report()
            self.timer.cancel()
            return
        
        self.test_phase += 1
    
    def check_movement(self, test_name: str):
        """Check if movement occurred"""
        if self.initial_position and self.current_position:
            dx = abs(self.current_position[0] - self.initial_position[0])
            dy = abs(self.current_position[1] - self.initial_position[1])
            dz = abs(self.current_position[2] - self.initial_position[2])
            total = (dx**2 + dy**2 + dz**2)**0.5
            
            if total > 0.05:
                self.get_logger().info(
                    f'✅ {test_name}: MOVED {total:.3f}m '
                    f'(ΔX={dx:.3f}, ΔY={dy:.3f}, ΔZ={dz:.3f})'
                )
            else:
                self.get_logger().error(
                    f'❌ {test_name}: NO MOVEMENT! Distance: {total:.3f}m'
                )
        
        # Show thruster values
        non_zero = [f'T{i+1}={v:.1f}' for i, v in enumerate(self.thruster_values) if abs(v) > 0.1]
        if non_zero:
            self.get_logger().info(f'  Thrusters: {", ".join(non_zero)}')
        else:
            self.get_logger().warn('  ⚠️  No thruster commands!')
    
    def print_final_report(self):
        """Print comprehensive report"""
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('📊 FINAL TEST REPORT')
        self.get_logger().info('='*70)
        
        if self.initial_position and self.current_position:
            dx = self.current_position[0] - self.initial_position[0]
            dy = self.current_position[1] - self.initial_position[1]
            dz = self.current_position[2] - self.initial_position[2]
            total = (dx**2 + dy**2 + dz**2)**0.5
            
            self.get_logger().info(f'Initial: {self.initial_position}')
            self.get_logger().info(f'Final:   {self.current_position}')
            self.get_logger().info(f'Total displacement: {total:.3f}m')
            
            if total > 0.1:
                self.get_logger().info('✅ MOVEMENT CONFIRMED - System working!')
            else:
                self.get_logger().error('❌ NO MOVEMENT - Check configuration!')
                self.get_logger().error('')
                self.get_logger().error('TROUBLESHOOTING STEPS:')
                self.get_logger().error('1. Check if bridge is running: ros2 topic list | grep thruster')
                self.get_logger().error('2. Check thruster mapper: ros2 topic echo /thruster1_cmd')
                self.get_logger().error('3. Check Gazebo topics: gz topic -l')
                self.get_logger().error('4. Verify URDF thruster plugin configuration')
        
        self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    node = MovementTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop on exit
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()