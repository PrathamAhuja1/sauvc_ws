#!/usr/bin/env python3
"""
Enhanced Gate Detector Node for SAUVC Competition

** UPDATED VERSION **
- Now reads nested HSV parameters (e.g., 'red_hsv.lower1')
  from the gate_params.yaml file.
- Relies on 'min_contour_area' from YAML (set it to ~150!)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PoseStamped, Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class EnhancedGateDetector(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_matrix = None
        self.image_width = 800
        self.image_height = 600
        self.fx = 474.896
        self.fy = 474.896
        self.cx = 400.0
        self.cy = 300.0
        
        # Gate specifications (from rulebook)
        self.GATE_WIDTH = 1.5  # meters
        
        # Detection parameters (defaults, will be overridden by YAML)
        self.declare_parameter('min_contour_area', 150)
        self.declare_parameter('aspect_ratio_min', 1.5)
        self.declare_parameter('aspect_ratio_max', 15.0)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('detection_confidence_threshold', 0.5)
        
        # --- NEW: Declare nested HSV parameters ---
        self.declare_parameter('red_hsv.lower1', [0, 70, 70])
        self.declare_parameter('red_hsv.upper1', [15, 255, 255])
        self.declare_parameter('red_hsv.lower2', [155, 70, 70])
        self.declare_parameter('red_hsv.upper2', [180, 255, 255])
        self.declare_parameter('green_hsv.lower', [45, 70, 70])
        self.declare_parameter('green_hsv.upper', [85, 255, 255])
        self.declare_parameter('orange_hsv.lower', [10, 100, 100])
        self.declare_parameter('orange_hsv.upper', [25, 255, 255])

        # --- Get all parameters ---
        self.min_area = self.get_parameter('min_contour_area').value
        self.aspect_min = self.get_parameter('aspect_ratio_min').value
        self.aspect_max = self.get_parameter('aspect_ratio_max').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.confidence_threshold = self.get_parameter('detection_confidence_threshold').value
        
        self.red_lower1 = np.array(self.get_parameter('red_hsv.lower1').value)
        self.red_upper1 = np.array(self.get_parameter('red_hsv.upper1').value)
        self.red_lower2 = np.array(self.get_parameter('red_hsv.lower2').value)
        self.red_upper2 = np.array(self.get_parameter('red_hsv.upper2').value)
        self.green_lower = np.array(self.get_parameter('green_hsv.lower').value)
        self.green_upper = np.array(self.get_parameter('green_hsv.upper').value)
        self.orange_lower = np.array(self.get_parameter('orange_hsv.lower').value)
        self.orange_upper = np.array(self.get_parameter('orange_hsv.upper').value)
        
        self.get_logger().info(f"Gate Detector initialized. IMPORTANT: 'min_contour_area' is {self.min_area}")

        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera_forward/image_raw', self.image_callback, qos_sensor)
        
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera_forward/camera_info', self.cam_info_callback, qos_reliable)
        
        # Publishers
        self.gate_detected_pub = self.create_publisher(Bool, '/gate/detected', 10)
        self.gate_center_pub = self.create_publisher(Point, '/gate/center', 10)
        self.alignment_pub = self.create_publisher(Float32, '/gate/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/gate/estimated_distance', 10)
        self.confidence_pub = self.create_publisher(Float32, '/gate/detection_confidence', 10)
        
        self.flare_detected_pub = self.create_publisher(Bool, '/flare/detected', 10)
        self.flare_avoidance_pub = self.create_publisher(Float32, '/flare/avoidance_direction', 10)
        self.flare_warning_pub = self.create_publisher(String, '/flare/warning', 10)
        
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, '/gate/debug_image', 10)
            self.debug_mask_pub = self.create_publisher(Image, '/gate/debug_mask', 10)
    
    def cam_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.get_logger().info(
                f'Camera calibration received: fx={self.fx:.1f}, fy={self.fy:.1f}'
            )
            self.destroy_subscription(self.cam_info_sub)
    
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        if cv_image.shape[:2] != (self.image_height, self.image_width):
            cv_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        
        processed = self.preprocess_image(cv_image)
        hsv_image = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)
        
        debug_img = cv_image.copy() if self.publish_debug else None
        
        gate_result = self.detect_gate(hsv_image, debug_img)
        flare_result = self.detect_orange_flare(hsv_image, debug_img)
        
        self.publish_gate_detection(gate_result, msg.header)
        self.publish_flare_detection(flare_result)
        
        if self.publish_debug and debug_img is not None:
            self.publish_debug_images(debug_img, hsv_image, msg.header)
    
    def preprocess_image(self, image):
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        return blurred
    
    def detect_gate(self, hsv_image, debug_img):
        red_mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
        
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        
        red_post = self.find_gate_post(red_mask, "RED", debug_img)
        green_post = self.find_gate_post(green_mask, "GREEN", debug_img)
        
        if red_post is None or green_post is None:
            return None
        
        return self.validate_gate_geometry(red_post, green_post, debug_img)
    
    def find_gate_post(self, mask, color_name, debug_img):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_post = None
        best_score = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / w if w > 0 else 0
            
            if aspect_ratio < self.aspect_min or aspect_ratio > self.aspect_max:
                continue
            
            ideal_aspect = 8.0
            aspect_score = 1.0 - abs(aspect_ratio - ideal_aspect) / ideal_aspect
            aspect_score = max(0, aspect_score)
            size_score = min(1.0, area / 5000.0)
            score = 0.6 * aspect_score + 0.4 * size_score
            
            if score > best_score:
                best_score = score
                M = cv2.moments(cnt)
                cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
                cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else y + h // 2
                
                best_post = {
                    'center': (cx, cy), 'bbox': (x, y, w, h),
                    'area': area, 'aspect_ratio': aspect_ratio,
                    'score': score, 'color': color_name
                }
        
        if best_post and debug_img is not None:
            x, y, w, h = best_post['bbox']
            color = (0, 0, 255) if color_name == "RED" else (0, 255, 0)
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(debug_img, f"{color_name} POST", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return best_post
    
    def validate_gate_geometry(self, red_post, green_post, debug_img):
        red_cx, red_cy = red_post['center']
        green_cx, green_cy = green_post['center']
        
        horizontal_separation = abs(green_cx - red_cx)
        vertical_difference = abs(green_cy - red_cy)
        
        if horizontal_separation < 20:  # Min pixel separation
            return None
        
        max_vertical_diff = self.image_height * 0.4 # Allow 40% height difference
        if vertical_difference > max_vertical_diff:
            return None
        
        gate_width_pixels = horizontal_separation
        estimated_distance = (self.GATE_WIDTH * self.fx) / gate_width_pixels
        
        if estimated_distance < 0.5 or estimated_distance > 30.0: # Increased max dist
            return None
        
        gate_cx = (red_cx + green_cx) // 2
        gate_cy = (red_cy + green_cy) // 2
        
        alignment_error_px = gate_cx - self.cx
        alignment_error_rad = np.arctan2(alignment_error_px, self.fx)
        
        geometry_score = 1.0 - (vertical_difference / max_vertical_diff)
        post_quality = (red_post['score'] + green_post['score']) / 2.0
        confidence = 0.5 * geometry_score + 0.5 * post_quality
        
        gate_info = {
            'center': (gate_cx, gate_cy),
            'distance': estimated_distance,
            'alignment_error': alignment_error_rad,
            'confidence': confidence
        }
        
        if debug_img is not None and confidence > self.confidence_threshold:
            cv2.line(debug_img, (red_cx, red_cy), (green_cx, green_cy), (255, 255, 0), 2)
            cv2.circle(debug_img, (gate_cx, gate_cy), 8, (255, 0, 255), -1)
            
            info_y = 30
            cv2.putText(debug_img, f"Dist: {estimated_distance:.2f}m", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_y += 30
            cv2.putText(debug_img, f"Align: {np.degrees(alignment_error_rad):.1f} deg", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_y += 30
            cv2.putText(debug_img, f"Conf: {confidence:.2f}", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return gate_info
    
    def detect_orange_flare(self, hsv_image, debug_img):
        orange_mask = cv2.inRange(hsv_image, self.orange_lower, self.orange_upper)
        kernel = np.ones((7, 7), np.uint8)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        flare_info = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300:
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / w if w > 0 else 0
            
            if aspect_ratio < 3.0 or aspect_ratio > 15.0:
                continue
            
            if area > max_area:
                max_area = area
                M = cv2.moments(cnt)
                cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
                
                flare_offset = cx - self.cx
                avoidance_direction = -1.0 if flare_offset > 0 else 1.0
                danger_level = 1.0 - (abs(flare_offset) / self.cx)
                
                flare_info = {
                    'avoidance_direction': avoidance_direction,
                    'danger_level': danger_level
                }
                
                if debug_img is not None:
                    cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 165, 255), 3)
                    cv2.putText(debug_img, "ORANGE FLARE!", (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        
        return flare_info
    
    def publish_gate_detection(self, gate_result, header):
        gate_detected = gate_result is not None and gate_result['confidence'] >= self.confidence_threshold
        
        self.gate_detected_pub.publish(Bool(data=gate_detected))
        
        if gate_detected:
            self.gate_center_pub.publish(Point(
                x=float(gate_result['center'][0]), 
                y=float(gate_result['center'][1]), z=0.0))
            
            self.alignment_pub.publish(Float32(data=float(gate_result['alignment_error'])))
            self.distance_pub.publish(Float32(data=float(gate_result['distance'])))
            self.confidence_pub.publish(Float32(data=float(gate_result['confidence'])))
        
        # We can publish pose even if confidence is low, for debugging
        if gate_result is not None:
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.header.frame_id = 'camera_forward'
            pose_msg.pose.position.x = gate_result['distance']
            pose_msg.pose.position.y = -float(gate_result['alignment_error']) * gate_result['distance']
            pose_msg.pose.orientation.w = 1.0
            self.gate_pose_pub.publish(pose_msg)
    
    def publish_flare_detection(self, flare_result):
        flare_detected = flare_result is not None
        
        self.flare_detected_pub.publish(Bool(data=flare_detected))
        
        if flare_detected:
            self.flare_avoidance_pub.publish(Float32(data=float(flare_result['avoidance_direction'])))
            
            danger = flare_result['danger_level']
            if danger > 0.7: warning = "CRITICAL: Orange flare directly ahead!"
            elif danger > 0.4: warning = "WARNING: Orange flare detected in path"
            else: warning = "CAUTION: Orange flare nearby"
            self.flare_warning_pub.publish(String(data=warning))

    def publish_debug_images(self, debug_img, hsv_image, header):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
            
            red_mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
            red_mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
            
            mask_viz = cv2.cvtColor(red_mask + green_mask, cv2.COLOR_GRAY2BGR)
            
            mask_msg = self.bridge.cv2_to_imgmsg(mask_viz, "bgr8")
            mask_msg.header = header
            self.debug_mask_pub.publish(mask_msg)
            
        except CvBridgeError as e:
            self.get_logger().error(f'Debug image conversion error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedGateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()