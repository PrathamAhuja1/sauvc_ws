# auv_core/perception_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped # Example output
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # --- Parameters ---
        self.declare_parameter('camera_topic', '/camera_forward/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_forward/camera_info')
        self.declare_parameter('publish_debug', True)

        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_debug = self.get_parameter('publish_debug').value

        # --- QoS ---
        qos_profile_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.VOLATILE)
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.VOLATILE)

        # --- Subscriptions ---
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, qos_profile_sensor_data)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.cam_info_callback, qos_profile_reliable)

        # --- Publishers ---
        # TODO: Define custom message types or use standard ones
        self.gate_pub = self.create_publisher(PointStamped, '/perception/gate_center', 10) # Example: publish center point
        self.red_flare_pub = self.create_publisher(PointStamped, '/perception/red_flare', 10)
        # Add publishers for drums, blue/yellow flares, mat, etc.

        if self.publish_debug:
            self.debug_img_pub = self.create_publisher(Image, '/perception/debug_image', 10)

        self.get_logger().info(f"Perception Node started. Listening on {camera_topic}")

    def cam_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera info received.")
            self.destroy_subscription(self.cam_info_sub) # Unsubscribe

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            # self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=5.0)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            debug_img = cv_image.copy() if self.publish_debug else None
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        now = msg.header.stamp # Use image timestamp

        # --- Gate Detection (Example: Red/Green Vertical Bars) ---
        # TODO: Define HSV ranges more accurately
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask_red = mask_red1 | mask_red2

        lower_green = np.array([35, 100, 50])
        upper_green = np.array([85, 255, 255])
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        gate_centers = []
        min_contour_area = 500 # Tune this
        aspect_ratio_threshold = 3.0 # Expect tall bars

        for cnt in contours_red + contours_green:
             area = cv2.contourArea(cnt)
             if area > min_contour_area:
                 x, y, w, h = cv2.boundingRect(cnt)
                 aspect_ratio = float(h) / w if w > 0 else 0
                 if aspect_ratio > aspect_ratio_threshold:
                     M = cv2.moments(cnt)
                     if M["m00"] != 0:
                         cX = int(M["m10"] / M["m00"])
                         cY = int(M["m01"] / M["m00"])
                         gate_centers.append((cX, cY))
                         if self.publish_debug:
                             cv2.rectangle(debug_img, (x, y), (x + w, y + h), (255, 0, 255), 2)
                             cv2.circle(debug_img, (cX, cY), 5, (255, 0, 255), -1)

        # If two bars detected, estimate center
        if len(gate_centers) == 2:
            gate_center_x = (gate_centers[0][0] + gate_centers[1][0]) // 2
            gate_center_y = (gate_centers[0][1] + gate_centers[1][1]) // 2
            if self.publish_debug:
                cv2.circle(debug_img, (gate_center_x, gate_center_y), 7, (0, 255, 255), -1)

            # Publish Gate Center (Pixel Coordinates for now)
            point_msg = PointStamped()
            point_msg.header.stamp = now
            point_msg.header.frame_id = msg.header.frame_id # Camera frame
            point_msg.point.x = float(gate_center_x)
            point_msg.point.y = float(gate_center_y)
            point_msg.point.z = 0.0 # Indicate pixel coordinates
            self.gate_pub.publish(point_msg)
            # TODO: Add logic to estimate 3D pose using solvePnP if possible

        # --- Red Flare Detection (Example: Tall Red Object) ---
        # Use mask_red from above
        contours_red_flare, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_flare_area = 300 # Tune
        flare_aspect_ratio_min = 4.0 # Tune

        for cnt in contours_red_flare:
            area = cv2.contourArea(cnt)
            if area > min_flare_area:
                 x, y, w, h = cv2.boundingRect(cnt)
                 aspect_ratio = float(h) / w if w > 0 else 0
                 if aspect_ratio > flare_aspect_ratio_min:
                     M = cv2.moments(cnt)
                     if M["m00"] != 0:
                         cX = int(M["m10"] / M["m00"])
                         cY = int(M["m01"] / M["m00"])
                         if self.publish_debug:
                            cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                            cv2.circle(debug_img, (cX, cY), 5, (0, 0, 255), -1)

                         # Publish Red Flare Center
                         point_msg = PointStamped()
                         point_msg.header.stamp = now
                         point_msg.header.frame_id = msg.header.frame_id
                         point_msg.point.x = float(cX)
                         point_msg.point.y = float(cY)
                         point_msg.point.z = 0.0
                         self.red_flare_pub.publish(point_msg)
                         # TODO: Estimate 3D pose
                         break # Assume only one red flare for now

        # --- TODO: Add Detection Logic for ---
        # - Blue Flare
        # - Yellow Flare
        # - Drums (Blue/Red circles/ellipses on green mat)
        # - Green Mat (Large green rectangle)
        # - Black Line (Using downward camera - might need separate node or camera input)

        # --- Publish Debug Image ---
        if self.publish_debug and debug_img is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
                debug_msg.header = msg.header
                self.debug_img_pub.publish(debug_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge error for debug image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()