#!/usr/bin/env python3
"""
Enhanced Gate Detector Node for SAUVC Competition
Detects navigation gate with red/green striped posts
Provides robust detection with multiple validation checks
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PoseStamped, Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class EnhancedGateDetector(Node):
    def __init__(self):
        super().__init__('enhanced_gate_detector')
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_matrix = None
        self.image_width = 800
        self.image_height = 600
        self.fx = 474.896  # From config
        self.fy = 474.896
        self.cx = 400.0
        self.cy = 300.0
        
        # Gate specifications (from rulebook)
        self.GATE_WIDTH = 1.5  # meters
        self.GATE_HEIGHT = 1.5  # meters
        self.POST_WIDTH = 0.08  # meters
        
        # Detection parameters
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('aspect_ratio_min', 2.0)
        self.declare_parameter('aspect_ratio_max', 12.0)
        self.declare_parameter('stripe_detection_enabled', True)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('detection_confidence_threshold', 0.7)
        self.declare_parameter('temporal_smoothing_window', 5)
        
        self.min_area = self.get_parameter('min_contour_area').value
        self.aspect_min = self.get_parameter('aspect_ratio_min').value
        self.aspect_max = self.get_parameter('aspect_ratio_max').value
        self.stripe_detection = self.get_parameter('stripe_detection_enabled').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.confidence_threshold = self.get_parameter('detection_confidence_threshold').value
        self.smoothing_window = self.get_parameter('temporal_smoothing_window').value
        
        # HSV color ranges (optimized for underwater)
        # Red stripes (port side) - handle wraparound
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])
        
        # Green stripes (starboard side)
        self.green_lower = np.array([50, 120, 100])
        self.green_upper = np.array([70, 255, 255])
        
        # Orange flare (danger - must avoid)
        self.orange_lower = np.array([10, 100, 100])
        self.orange_upper = np.array([25, 255, 255])
        
        # Detection state
        self.gate_detected = False
        self.gate_center = None
        self.gate_distance = 0.0
        self.alignment_error = 0.0
        self.detection_confidence = 0.0
        
        # Temporal smoothing buffers
        self.center_buffer = deque(maxlen=self.smoothing_window)
        self.distance_buffer = deque(maxlen=self.smoothing_window)
        self.alignment_buffer = deque(maxlen=self.smoothing_window)
        
        # Orange flare tracking
        self.flare_detected = False
        self.flare_position = None
        
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
            Image,
            '/camera_forward/image_raw',
            self.image_callback,
            qos_sensor
        )
        
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_forward/camera_info',
            self.cam_info_callback,
            qos_reliable
        )
        
        # Publishers
        self.gate_detected_pub = self.create_publisher(Bool, '/gate/detected', 10)
        self.gate_center_pub = self.create_publisher(Point, '/gate/center', 10)
        self.alignment_pub = self.create_publisher(Float32, '/gate/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/gate/estimated_distance', 10)
        self.confidence_pub = self.create_publisher(Float32, '/gate/detection_confidence', 10)
        self.gate_pose_pub = self.create_publisher(PoseStamped, '/gate/pose', 10)
        
        # Orange flare publishers
        self.flare_detected_pub = self.create_publisher(Bool, '/flare/detected', 10)
        self.flare_avoidance_pub = self.create_publisher(Float32, '/flare/avoidance_direction', 10)
        self.flare_warning_pub = self.create_publisher(String, '/flare/warning', 10)
        
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, '/gate/debug_image', 10)
            self.debug_mask_pub = self.create_publisher(Image, '/gate/debug_mask', 10)
        
        self.get_logger().info('Enhanced Gate Detector initialized')
        self.get_logger().info(f'Gate specifications: {self.GATE_WIDTH}m x {self.GATE_HEIGHT}m')
    
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
        
        # Resize if necessary
        if cv_image.shape[:2] != (self.image_height, self.image_width):
            cv_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        
        # Pre-process image
        processed = self.preprocess_image(cv_image)
        hsv_image = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)
        
        debug_img = cv_image.copy() if self.publish_debug else None
        
        # Detect gate posts
        gate_result = self.detect_gate(hsv_image, debug_img)
        
        # Detect orange flare
        flare_result = self.detect_orange_flare(hsv_image, debug_img)
        
        # Update state and publish
        self.update_detection_state(gate_result, flare_result)
        self.publish_detections(msg.header)
        
        # Publish debug visualization
        if self.publish_debug and debug_img is not None:
            self.publish_debug_images(debug_img, hsv_image, msg.header)
    
    def preprocess_image(self, image):
        """Apply preprocessing for better underwater detection"""
        # Apply CLAHE for contrast enhancement
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        
        # Slight Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        
        return blurred
    
    def detect_gate(self, hsv_image, debug_img):
        """Detect gate posts using color and geometry"""
        
        # Detect red post (port side)
        red_mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # Detect green post (starboard side)
        green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        
        # Find posts
        red_post = self.find_gate_post(red_mask, "RED", debug_img)
        green_post = self.find_gate_post(green_mask, "GREEN", debug_img)
        
        if red_post is None or green_post is None:
            return None
        
        # Validate gate geometry
        gate_info = self.validate_gate_geometry(red_post, green_post, debug_img)
        
        return gate_info
    
    def find_gate_post(self, mask, color_name, debug_img):
        """Find a gate post from color mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_post = None
        best_score = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / w if w > 0 else 0
            
            # Check aspect ratio (posts should be tall and thin)
            if aspect_ratio < self.aspect_min or aspect_ratio > self.aspect_max:
                continue
            
            # Calculate score based on size and aspect ratio
            # Prefer taller posts with better aspect ratios
            ideal_aspect = 8.0  # Tall post
            aspect_score = 1.0 - abs(aspect_ratio - ideal_aspect) / ideal_aspect
            aspect_score = max(0, aspect_score)
            
            size_score = min(1.0, area / 5000.0)
            score = 0.6 * aspect_score + 0.4 * size_score
            
            if score > best_score:
                best_score = score
                
                # Calculate moments for centroid
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx = x + w // 2
                    cy = y + h // 2
                
                best_post = {
                    'contour': cnt,
                    'center': (cx, cy),
                    'bbox': (x, y, w, h),
                    'area': area,
                    'aspect_ratio': aspect_ratio,
                    'score': score,
                    'color': color_name
                }
        
        # Draw best post on debug image
        if best_post and debug_img is not None:
            x, y, w, h = best_post['bbox']
            cx, cy = best_post['center']
            color = (0, 0, 255) if color_name == "RED" else (0, 255, 0)
            
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), color, 2)
            cv2.circle(debug_img, (cx, cy), 5, color, -1)
            cv2.putText(debug_img, f"{color_name} POST", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(debug_img, f"AR:{aspect_ratio:.1f}", (x, y + h + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return best_post
    
    def validate_gate_geometry(self, red_post, green_post, debug_img):
        """Validate that two posts form a valid gate"""
        
        red_cx, red_cy = red_post['center']
        green_cx, green_cy = green_post['center']
        
        # Calculate horizontal separation
        horizontal_separation = abs(green_cx - red_cx)
        
        # Calculate vertical alignment (posts should be at similar heights)
        vertical_difference = abs(green_cy - red_cy)
        
        # Check if posts are horizontally separated
        if horizontal_separation < 50:  # Minimum pixel separation
            return None
        
        # Check vertical alignment (should be roughly same height)
        max_vertical_diff = self.image_height * 0.3  # Allow 30% height difference
        if vertical_difference > max_vertical_diff:
            return None
        
        # Estimate distance using known gate width
        gate_width_pixels = horizontal_separation
        estimated_distance = (self.GATE_WIDTH * self.fx) / gate_width_pixels
        
        # Sanity check on distance (gate should be 1-15 meters away)
        if estimated_distance < 0.5 or estimated_distance > 20.0:
            return None
        
        # Calculate gate center
        gate_cx = (red_cx + green_cx) // 2
        gate_cy = (red_cy + green_cy) // 2
        
        # Calculate alignment error (pixels from image center)
        alignment_error_px = gate_cx - self.cx
        
        # Convert to angular error (radians)
        alignment_error_rad = np.arctan2(alignment_error_px, self.fx)
        
        # Calculate confidence based on post quality and geometry
        geometry_score = 1.0 - (vertical_difference / max_vertical_diff)
        post_quality = (red_post['score'] + green_post['score']) / 2.0
        confidence = 0.5 * geometry_score + 0.5 * post_quality
        
        gate_info = {
            'center': (gate_cx, gate_cy),
            'distance': estimated_distance,
            'alignment_error': alignment_error_rad,
            'confidence': confidence,
            'red_post': red_post,
            'green_post': green_post,
            'width_pixels': gate_width_pixels,
            'vertical_alignment': vertical_difference
        }
        
        # Draw gate visualization
        if debug_img is not None:
            # Draw line connecting posts
            cv2.line(debug_img, (red_cx, red_cy), (green_cx, green_cy), (255, 255, 0), 2)
            
            # Draw gate center
            cv2.circle(debug_img, (gate_cx, gate_cy), 8, (255, 0, 255), -1)
            cv2.circle(debug_img, (gate_cx, gate_cy), 15, (255, 0, 255), 2)
            
            # Draw alignment crosshair
            img_center_x = int(self.cx)
            cv2.line(debug_img, (img_center_x, 0), (img_center_x, self.image_height), 
                    (0, 255, 255), 1)
            cv2.line(debug_img, (0, gate_cy), (self.image_width, gate_cy), 
                    (0, 255, 255), 1)
            
            # Draw info text
            info_y = 30
            cv2.putText(debug_img, f"Distance: {estimated_distance:.2f}m", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_y += 30
            cv2.putText(debug_img, f"Alignment: {np.degrees(alignment_error_rad):.1f} deg", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_y += 30
            cv2.putText(debug_img, f"Confidence: {confidence:.2f}", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_y += 30
            cv2.putText(debug_img, f"Width: {gate_width_pixels}px", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return gate_info
    
    def detect_orange_flare(self, hsv_image, debug_img):
        """Detect dangerous orange flare obstacle"""
        
        orange_mask = cv2.inRange(hsv_image, self.orange_lower, self.orange_upper)
        
        # Morphological operations
        kernel = np.ones((7, 7), np.uint8)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        flare_info = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300:  # Minimum area for flare
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = h / w if w > 0 else 0
            
            # Flare should be tall and thin (1.5m tall, 0.15m diameter)
            if aspect_ratio < 3.0 or aspect_ratio > 15.0:
                continue
            
            if area > max_area:
                max_area = area
                
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx = x + w // 2
                    cy = y + h // 2
                
                # Calculate avoidance direction (move away from flare)
                flare_offset = cx - self.cx
                avoidance_direction = -1.0 if flare_offset > 0 else 1.0
                
                # Calculate danger level based on proximity to center
                danger_ratio = abs(flare_offset) / self.cx
                danger_level = 1.0 - danger_ratio  # Higher when closer to center
                
                flare_info = {
                    'center': (cx, cy),
                    'bbox': (x, y, w, h),
                    'area': area,
                    'avoidance_direction': avoidance_direction,
                    'danger_level': danger_level
                }
                
                if debug_img is not None:
                    cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 165, 255), 3)
                    cv2.circle(debug_img, (cx, cy), 10, (0, 165, 255), -1)
                    cv2.putText(debug_img, "ORANGE FLARE - AVOID!", (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                    cv2.putText(debug_img, f"DANGER: {danger_level:.2f}", (x, y + h + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        
        return flare_info
    
    def update_detection_state(self, gate_result, flare_result):
        """Update detection state with temporal smoothing"""
        
        # Update gate detection
        if gate_result and gate_result['confidence'] >= self.confidence_threshold:
            self.gate_detected = True
            
            # Add to smoothing buffers
            self.center_buffer.append(gate_result['center'])
            self.distance_buffer.append(gate_result['distance'])
            self.alignment_buffer.append(gate_result['alignment_error'])
            
            # Calculate smoothed values
            if len(self.center_buffer) > 0:
                centers = np.array(list(self.center_buffer))
                self.gate_center = tuple(np.mean(centers, axis=0).astype(int))
                
                self.gate_distance = np.mean(list(self.distance_buffer))
                self.alignment_error = np.mean(list(self.alignment_buffer))
                self.detection_confidence = gate_result['confidence']
        else:
            self.gate_detected = False
            self.gate_center = None
            self.gate_distance = 0.0
            self.alignment_error = 0.0
            self.detection_confidence = 0.0
            
            # Clear buffers if detection lost
            self.center_buffer.clear()
            self.distance_buffer.clear()
            self.alignment_buffer.clear()
        
        # Update flare detection
        if flare_result:
            self.flare_detected = True
            self.flare_position = flare_result
        else:
            self.flare_detected = False
            self.flare_position = None
    
    def publish_detections(self, header):
        """Publish all detection results"""
        
        # Gate detection
        detected_msg = Bool()
        detected_msg.data = self.gate_detected
        self.gate_detected_pub.publish(detected_msg)
        
        if self.gate_detected:
            # Gate center
            center_msg = Point()
            center_msg.x = float(self.gate_center[0])
            center_msg.y = float(self.gate_center[1])
            center_msg.z = 0.0
            self.gate_center_pub.publish(center_msg)
            
            # Alignment error
            alignment_msg = Float32()
            alignment_msg.data = float(self.alignment_error)
            self.alignment_pub.publish(alignment_msg)
            
            # Distance
            distance_msg = Float32()
            distance_msg.data = float(self.gate_distance)
            self.distance_pub.publish(distance_msg)
            
            # Confidence
            confidence_msg = Float32()
            confidence_msg.data = float(self.detection_confidence)
            self.confidence_pub.publish(confidence_msg)
            
            # Pose
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.header.frame_id = 'camera_forward'
            pose_msg.pose.position.x = self.gate_distance
            pose_msg.pose.position.y = float(self.gate_center[0] - self.cx) / self.fx
            pose_msg.pose.position.z = float(self.cy - self.gate_center[1]) / self.fy
            pose_msg.pose.orientation.w = 1.0
            self.gate_pose_pub.publish(pose_msg)
        
        # Flare detection
        flare_detected_msg = Bool()
        flare_detected_msg.data = self.flare_detected
        self.flare_detected_pub.publish(flare_detected_msg)
        
        if self.flare_detected:
            avoidance_msg = Float32()
            avoidance_msg.data = float(self.flare_position['avoidance_direction'])
            self.flare_avoidance_pub.publish(avoidance_msg)
            
            # Warning message
            danger = self.flare_position['danger_level']
            if danger > 0.7:
                warning = "CRITICAL: Orange flare directly ahead!"
            elif danger > 0.4:
                warning = "WARNING: Orange flare detected in path"
            else:
                warning = "CAUTION: Orange flare nearby"
            
            warning_msg = String()
            warning_msg.data = warning
            self.flare_warning_pub.publish(warning_msg)
    
    def publish_debug_images(self, debug_img, hsv_image, header):
        """Publish debug visualization images"""
        
        try:
            # Main debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = header
            self.debug_pub.publish(debug_msg)
            
            # Color mask visualization
            red_mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
            red_mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
            orange_mask = cv2.inRange(hsv_image, self.orange_lower, self.orange_upper)
            
            # Combine masks with different colors
            mask_viz = np.zeros_like(debug_img)
            mask_viz[:, :, 2] = red_mask  # Red in red channel
            mask_viz[:, :, 1] = green_mask  # Green in green channel
            mask_viz[:, :, 0] = orange_mask  # Orange in blue channel (will appear cyan)
            
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