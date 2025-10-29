#!/usr/bin/env python3
"""
Complete System Diagnostic - Tests ALL components
Run this FIRST to identify issues
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import time


class SystemDiagnostic(Node):
    def __init__(self):
        super().__init__('system_diagnostic')
        
        # State tracking
        self.topics_found = {
            '/ground_truth/odom': False,
            '/camera_forward/image_raw': False,
            '/thruster1_cmd': False,
            '/thruster2_cmd': False,
            '/thruster3_cmd': False,
            '/thruster4_cmd': False,
            '/thruster5_cmd': False,
            '/thruster6_cmd': False,
        }
        
        self.initial_position = None
        self.current_position = None
        self.images_received = 0
        self.thrusters_active = [False] * 6
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, '/camera_forward/image_raw', self.image_callback, 10)
        
        for i in range(6):
            self.create_subscription(
                Float64, f'/thruster{i+1}_cmd',
                lambda msg, idx=i: self.thruster_callback(msg, idx), 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Test sequence
        self.test_phase = 0
        self.phase_start_time = time.time()
        self.timer = self.create_timer(0.1, self.run_diagnostic)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🔍 SYSTEM DIAGNOSTIC STARTED')
        self.get_logger().info('='*70)
        self.get_logger().info('Testing all components...')
    
    def odom_callback(self, msg: Odometry):
        self.topics_found['/ground_truth/odom'] = True
        pos = msg.pose.pose.position
        
        if self.initial_position is None:
            self.initial_position = (pos.x, pos.y, pos.z)
        
        self.current_position = (pos.x, pos.y, pos.z)
    
    def image_callback(self, msg: Image):
        self.topics_found['/camera_forward/image_raw'] = True
        self.images_received += 1
    
    def thruster_callback(self, msg: Float64, idx: int):
        topic = f'/thruster{idx+1}_cmd'
        self.topics_found[topic] = True
        
        if abs(msg.data) > 1.0:
            self.thrusters_active[idx] = True
    
    def run_diagnostic(self):
        elapsed = time.time() - self.phase_start_time
        
        if self.test_phase == 0:
            # Phase 0: Check topic availability (5 seconds)
            if elapsed > 5.0:
                self.print_topic_status()
                self.test_phase = 1
                self.phase_start_time = time.time()
        
        elif self.test_phase == 1:
            # Phase 1: Test downward thrust
            self.get_logger().info('🧪 TEST 1: DOWNWARD THRUST', throttle_duration_sec=2.0)
            cmd = Twist()
            cmd.linear.z = -1.0
            self.cmd_vel_pub.publish(cmd)
            
            if elapsed > 3.0:
                self.check_movement('Downward')
                self.test_phase = 2
                self.phase_start_time = time.time()
        
        elif self.test_phase == 2:
            # Phase 2: Test upward thrust
            self.get_logger().info('🧪 TEST 2: UPWARD THRUST', throttle_duration_sec=2.0)
            cmd = Twist()
            cmd.linear.z = 1.0
            self.cmd_vel_pub.publish(cmd)
            
            if elapsed > 3.0:
                self.check_movement('Upward')
                self.test_phase = 3
                self.phase_start_time = time.time()
        
        elif self.test_phase == 3:
            # Phase 3: Test forward thrust
            self.get_logger().info('🧪 TEST 3: FORWARD THRUST', throttle_duration_sec=2.0)
            cmd = Twist()
            cmd.linear.x = 0.5
            self.cmd_vel_pub.publish(cmd)
            
            if elapsed > 3.0:
                self.check_movement('Forward')
                self.test_phase = 4
                self.phase_start_time = time.time()
        
        elif self.test_phase == 4:
            # Phase 4: Test yaw
            self.get_logger().info('🧪 TEST 4: YAW ROTATION', throttle_duration_sec=2.0)
            cmd = Twist()
            cmd.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd)
            
            if elapsed > 3.0:
                self.check_movement('Yaw')
                self.test_phase = 5
                self.phase_start_time = time.time()
        
        elif self.test_phase == 5:
            # Phase 5: Stop and report
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            
            if elapsed > 1.0:
                self.print_final_report()
                self.timer.cancel()
    
    def print_topic_status(self):
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('📊 TOPIC AVAILABILITY CHECK')
        self.get_logger().info('='*70)
        
        for topic, found in self.topics_found.items():
            status = '✅' if found else '❌'
            self.get_logger().info(f'{status} {topic}')
        
        self.get_logger().info(f'📷 Images received: {self.images_received}')
        self.get_logger().info('')
    
    def check_movement(self, test_name: str):
        if self.initial_position and self.current_position:
            dx = abs(self.current_position[0] - self.initial_position[0])
            dy = abs(self.current_position[1] - self.initial_position[1])
            dz = abs(self.current_position[2] - self.initial_position[2])
            total = (dx**2 + dy**2 + dz**2)**0.5
            
            if total > 0.1:
                self.get_logger().info(
                    f'✅ {test_name}: MOVEMENT DETECTED {total:.2f}m '
                    f'(ΔX={dx:.2f}, ΔY={dy:.2f}, ΔZ={dz:.2f})'
                )
            else:
                self.get_logger().error(
                    f'❌ {test_name}: NO MOVEMENT! Distance: {total:.3f}m'
                )
            
            # Show active thrusters
            active = [i+1 for i, active in enumerate(self.thrusters_active) if active]
            if active:
                self.get_logger().info(f'   Active thrusters: {active}')
            else:
                self.get_logger().error('   ⚠️  NO THRUSTERS ACTIVE!')
            
            # Reset for next test
            self.thrusters_active = [False] * 6
    
    def print_final_report(self):
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('📋 FINAL DIAGNOSTIC REPORT')
        self.get_logger().info('='*70)
        
        all_topics_ok = all(self.topics_found.values())
        
        if all_topics_ok:
            self.get_logger().info('✅ All topics are available')
        else:
            self.get_logger().error('❌ Some topics are missing!')
            missing = [t for t, found in self.topics_found.items() if not found]
            for topic in missing:
                self.get_logger().error(f'   Missing: {topic}')
        
        if self.images_received > 10:
            self.get_logger().info(f'✅ Camera working ({self.images_received} images)')
        else:
            self.get_logger().error(f'❌ Camera issue ({self.images_received} images)')
        
        if self.initial_position and self.current_position:
            total_movement = sum([
                abs(self.current_position[i] - self.initial_position[i])
                for i in range(3)
            ])
            
            if total_movement > 0.5:
                self.get_logger().info(f'✅ Movement system working ({total_movement:.2f}m total)')
            else:
                self.get_logger().error(f'❌ Movement system FAILED ({total_movement:.2f}m total)')
                self.get_logger().error('')
                self.get_logger().error('🔧 TROUBLESHOOTING STEPS:')
                self.get_logger().error('1. Check if gz_bridge is running:')
                self.get_logger().error('   ros2 topic list | grep thruster')
                self.get_logger().error('2. Check if simple_thruster_mapper is running:')
                self.get_logger().error('   ros2 node list | grep thruster')
                self.get_logger().error('3. Manually test thrusters:')
                self.get_logger().error('   ros2 topic pub /thruster1_cmd std_msgs/msg/Float64 "{data: 100.0}"')
                self.get_logger().error('4. Check Gazebo simulation:')
                self.get_logger().error('   gz topic -l | grep thruster')
        
        self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    node = SystemDiagnostic()
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