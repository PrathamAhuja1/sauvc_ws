#!/usr/bin/env python3
"""
EMERGENCY THRUSTER TEST
This bypasses ALL control nodes and sends commands directly
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import time

class EmergencyThrusterTest(Node):
    def __init__(self):
        super().__init__('emergency_thruster_test')
        
        self.current_depth = 0.0
        self.start_depth = None
        
        # Subscribe to ground truth
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Publishers for ALL thrusters
        self.thruster_pubs = {
            1: self.create_publisher(Float64, '/thruster1_cmd', 10),
            2: self.create_publisher(Float64, '/thruster2_cmd', 10),
            3: self.create_publisher(Float64, '/thruster3_cmd', 10),
            4: self.create_publisher(Float64, '/thruster4_cmd', 10),
            5: self.create_publisher(Float64, '/thruster5_cmd', 10),
            6: self.create_publisher(Float64, '/thruster6_cmd', 10),
        }
        
        self.test_phase = 0
        self.phase_start = time.time()
        
        # Wait for initial odom
        self.get_logger().info('⏳ Waiting for odometry...')
        time.sleep(2.0)
        
        # Control loop
        self.timer = self.create_timer(0.1, self.test_loop)
        
        self.get_logger().info('='*60)
        self.get_logger().info('🚨 EMERGENCY THRUSTER TEST STARTING')
        self.get_logger().info('='*60)
    
    def odom_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z
        if self.start_depth is None:
            self.start_depth = self.current_depth
            self.get_logger().info(f'📍 Starting depth: {self.start_depth:.2f}m')
    
    def send_thrust(self, t1, t2, t3, t4, t5, t6):
        """Send thrust values to all thrusters"""
        values = [t1, t2, t3, t4, t5, t6]
        for i, val in enumerate(values, 1):
            cmd = Float64()
            cmd.data = float(val)
            self.thruster_pubs[i].publish(cmd)
    
    def test_loop(self):
        elapsed = time.time() - self.phase_start
        
        # Phase 0: Test vertical thrusters MAXIMUM DOWN (10 seconds)
        if self.test_phase == 0:
            if elapsed < 10.0:
                # MAXIMUM DOWNWARD THRUST
                self.send_thrust(0, 0, 0, 0, -100, -100)
                
                if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
                    depth_change = self.current_depth - (self.start_depth or 0)
                    self.get_logger().info(
                        f'Phase 0 [{elapsed:.1f}s]: VERTICAL DOWN | '
                        f'Depth: {self.current_depth:.2f}m | '
                        f'Change: {depth_change:.3f}m | '
                        f'T5=-100, T6=-100'
                    )
            else:
                self.get_logger().info('Phase 0 COMPLETE')
                self.test_phase = 1
                self.phase_start = time.time()
        
        # Phase 1: Test vertical thrusters UPWARD (10 seconds)
        elif self.test_phase == 1:
            if elapsed < 10.0:
                self.send_thrust(0, 0, 0, 0, 100, 100)
                
                if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
                    depth_change = self.current_depth - (self.start_depth or 0)
                    self.get_logger().info(
                        f'Phase 1 [{elapsed:.1f}s]: VERTICAL UP | '
                        f'Depth: {self.current_depth:.2f}m | '
                        f'Change: {depth_change:.3f}m | '
                        f'T5=100, T6=100'
                    )
            else:
                self.get_logger().info('Phase 1 COMPLETE')
                self.test_phase = 2
                self.phase_start = time.time()
        
        # Phase 2: Test horizontal thrusters FORWARD (10 seconds)
        elif self.test_phase == 2:
            if elapsed < 10.0:
                # Forward thrust on T1 and T2
                self.send_thrust(50, 50, -50, -50, 0, 0)
                
                if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
                    self.get_logger().info(
                        f'Phase 2 [{elapsed:.1f}s]: FORWARD | '
                        f'T1=50, T2=50, T3=-50, T4=-50'
                    )
            else:
                self.get_logger().info('Phase 2 COMPLETE')
                self.test_phase = 3
                self.phase_start = time.time()
        
        # Phase 3: STOP and report
        elif self.test_phase == 3:
            self.send_thrust(0, 0, 0, 0, 0, 0)
            
            self.get_logger().info('='*60)
            self.get_logger().info('🏁 TEST COMPLETE')
            self.get_logger().info('='*60)
            
            if self.start_depth is not None:
                total_change = self.current_depth - self.start_depth
                self.get_logger().info(f'Start depth: {self.start_depth:.2f}m')
                self.get_logger().info(f'Final depth: {self.current_depth:.2f}m')
                self.get_logger().info(f'Total change: {total_change:.3f}m')
                
                if abs(total_change) < 0.01:
                    self.get_logger().error('❌ NO MOVEMENT DETECTED!')
                    self.get_logger().error('Problem is in Gazebo physics or URDF!')
                else:
                    self.get_logger().info('✅ Movement detected - thrusters work!')
            
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyThrusterTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop all thrusters
        for i in range(1, 7):
            cmd = Float64()
            cmd.data = 0.0
            node.thruster_pubs[i].publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()