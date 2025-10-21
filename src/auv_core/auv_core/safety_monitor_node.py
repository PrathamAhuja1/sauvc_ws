#!/usr/bin/env python3
"""
Safety Monitor Node - Monitors critical parameters and triggers emergency actions
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
import time
from tf_transformations import euler_from_quaternion

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')
        
        # Parameters
        self.declare_parameter('max_depth', -3.5)
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_tilt_angle', 0.785)  # 45 degrees
        self.declare_parameter('watchdog_timeout', 5.0)
        self.declare_parameter('max_mission_time', 900.0)
        
        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_tilt = self.get_parameter('max_tilt_angle').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.max_mission_time = self.get_parameter('max_mission_time').value
        
        # State
        self.mission_start_time = time.time()
        self.last_odom_time = None
        self.emergency_triggered = False
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Publishers
        self.emergency_pub = self.create_publisher(Bool, '/safety/emergency_stop', 10)
        self.emergency_cmd_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.status_pub = self.create_publisher(Float32, '/safety/status', 10)
        
        # Watchdog timer
        self.timer = self.create_timer(1.0, self.check_safety)
        
        self.get_logger().info('Safety Monitor initialized')
    
    def odom_callback(self, msg: Odometry):
        self.last_odom_time = time.time()
        
        # Check depth limits
        depth = msg.pose.pose.position.z
        if depth < self.max_depth or depth > self.min_depth:
            self.trigger_emergency(f"Depth violation: {depth:.2f}m")
        
        # Check tilt (pitch/roll)
        q = msg.pose.pose.orientation
        roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self.trigger_emergency(f"Excessive tilt: R={roll:.2f}, P={pitch:.2f}")
    
    def check_safety(self):
        # Mission timeout
        elapsed = time.time() - self.mission_start_time
        if elapsed > self.max_mission_time:
            self.trigger_emergency("Mission timeout")
        
        # Watchdog timeout
        if self.last_odom_time:
            time_since_odom = time.time() - self.last_odom_time
            if time_since_odom > self.watchdog_timeout:
                self.trigger_emergency("Odometry timeout")
        
        # Publish status
        status = Float32()
        status.data = 1.0 if not self.emergency_triggered else 0.0
        self.status_pub.publish(status)
    
    def trigger_emergency(self, reason: str):
        if self.emergency_triggered:
            return
        
        self.emergency_triggered = True
        self.get_logger().error(f"🚨 EMERGENCY STOP: {reason}")
        
        # Publish emergency flag
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        
        # Command surface
        cmd = Twist()
        cmd.linear.z = -1.0  # Maximum upward thrust
        self.emergency_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()