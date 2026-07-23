import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_frame,
            10
        )

        self.publisher = self.create_publisher(Point, '/target/position', 10)

        # CvBridge is the translator between ROS2 Image messages and OpenCV numpy arrays
        # We create one instance here and reuse it every frame — creating it fresh
        # each frame would be wasteful since it's a heavy object to initialize
        self.bridge = CvBridge()

        # MOG2 background subtractor — same as your laptop script
        # Watches frames over time and builds a model of what background looks like
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500,
            varThreshold=80,
            detectShadows=False
        )

        # Morphological kernel for erosion and dilation
        # Used to clean up the noisy mask after background subtraction
        self.kernel = np.ones((5, 5), np.uint8)

        # Minimum contour area to be considered a real detection
        # Anything smaller is noise and gets ignored
        self.min_contour_area = 1500

        self.get_logger().info('Vision node online. Waiting for frames...')

    def process_frame(self, msg):
        # Step 1 — Convert ROS2 Image message to OpenCV numpy array
        # 'bgr8' is the color encoding — Blue Green Red, 8 bits per channel
        # This is OpenCV's native color format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Step 2 — Apply background subtraction to get foreground mask
        # Returns a black/white image where white = moving pixels
        fg_mask = self.bg_subtractor.apply(frame)

        # Step 3 — Clean up the mask with erosion then dilation
        # Erosion kills small noise speckles, dilation restores real object size
        fg_mask = cv2.erode(fg_mask, self.kernel, iterations=1)
        fg_mask = cv2.dilate(fg_mask, self.kernel, iterations=2)

        # Step 4 — Find contours (outlines of white blobs in the mask)
        contours, _ = cv2.findContours(
            fg_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Step 5 — If no contours found, nothing is moving, skip this frame
        if not contours:
            return

        # Step 6 — Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)

        # Step 7 — Ignore it if it's too small (noise that survived the mask cleanup)
        if cv2.contourArea(largest_contour) < self.min_contour_area:
            return

        # Step 8 — Get bounding box around the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Step 9 — Calculate center point of detected object
        center_x = x + w // 2
        center_y = y + h // 2

        # Step 10 — Publish the target position as a Point message
        # The turret controller node subscribes to this and runs PID on these coordinates
        target = Point()
        target.x = float(center_x)
        target.y = float(center_y)
        target.z = float(cv2.contourArea(largest_contour))
        self.publisher.publish(target)

        self.get_logger().info(
            f'Target detected at ({center_x}, {center_y}) '
            f'| Area: {cv2.contourArea(largest_contour):.0f}px'
        )

def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()