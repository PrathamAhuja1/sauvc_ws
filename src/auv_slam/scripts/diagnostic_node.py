#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class GateDiagnostic(Node):
    def __init__(self):
        super().__init__('gate_diagnostic')
        
        self.bridge = CvBridge()
        
        # Track detection status
        self.gate_detected = False
        self.alignment_error = 0.0
        self.distance = 0.0
        self.gate_center = None
        self.last_image_time = None
        self.image_count = 0
        
        # Subscriptions
        self.gate_detected_sub = self.create_subscription(
            Bool, '/gate/detected', self.gate_detected_cb, 10)
        self.alignment_sub = self.create_subscription(
            Float32, '/gate/alignment_error', self.alignment_cb, 10)
        self.distance_sub = self.create_subscription(
            Float32, '/gate/estimated_distance', self.distance_cb, 10)
        self.center_sub = self.create_subscription(
            Point, '/gate/center', self.center_cb, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera_forward/image_raw', self.image_cb, 10)
        self.debug_sub = self.create_subscription(
            Image, '/gate/debug_image', self.debug_cb, 10)
        
        # Status timer
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🔍 GATE DETECTION DIAGNOSTIC TOOL')
        self.get_logger().info('='*70)
        self.get_logger().info('Monitoring gate detection topics...')
        self.get_logger().info('')
    
    def gate_detected_cb(self, msg: Bool):
        self.gate_detected = msg.data
    
    def alignment_cb(self, msg: Float32):
        self.alignment_error = msg.data
    
    def distance_cb(self, msg: Float32):
        self.distance = msg.data
    
    def center_cb(self, msg: Point):
        self.gate_center = (msg.x, msg.y)
    
    def image_cb(self, msg: Image):
        self.last_image_time = time.time()
        self.image_count += 1
    
    def debug_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Gate Detection Debug", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            pass
    
    def print_status(self):
        self.get_logger().info('-'*70)
        
        # Camera status
        if self.last_image_time and (time.time() - self.last_image_time) < 2.0:
            self.get_logger().info(f'✓ Camera: WORKING ({self.image_count} images)')
        else:
            self.get_logger().error('✗ Camera: NO IMAGES - Check camera/bridge!')
        
        # Gate detection status
        if self.gate_detected:
            self.get_logger().info(f'✓ Gate: DETECTED')
            self.get_logger().info(f'  • Distance: {self.distance:.2f}m')
            self.get_logger().info(f'  • Alignment: {self.alignment_error:.3f} rad ({self.alignment_error*57.3:.1f}°)')
            if self.gate_center:
                self.get_logger().info(f'  • Center: ({self.gate_center[0]:.0f}, {self.gate_center[1]:.0f}) px')
        else:
            self.get_logger().warn('⚠ Gate: NOT DETECTED')
        
        self.get_logger().info('')
        
        # Recommendations
        if not self.gate_detected and self.last_image_time:
            self.get_logger().warn('TROUBLESHOOTING:')
            self.get_logger().warn('1. Check if gate is visible in camera view')
            self.get_logger().warn('2. Verify HSV color ranges in gate_detector_node.py')
            self.get_logger().warn('3. Look at /gate/debug_image topic (shown in window)')
            self.get_logger().warn('4. Check gate position in world file')


def main(args=None):
    rclpy.init(args=args)
    node = GateDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()