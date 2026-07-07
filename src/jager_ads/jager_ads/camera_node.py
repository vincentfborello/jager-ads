import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import subprocess
import threading
import numpy as np

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.width = 640
        self.height = 480
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        self.get_logger().info('Camera node online. Streaming frames...')

    def capture_loop(self):
        cmd = [
            'rpicam-vid',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', '30',
            '--codec', 'yuv420',
            '--timeout', '0',
            '--nopreview',
            '-o', '-'
        ]

        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )

        # Each YUV420 frame is width * height * 1.5 bytes
        # YUV420 uses 1.5 bytes per pixel — 1 byte for brightness, 0.5 for color
        frame_size = int(self.width * self.height * 1.5)

        while self.running:
            # Read exactly one frame worth of bytes from the pipe
            raw_frame = process.stdout.read(frame_size)

            # If we didn't get a full frame skip it
            if len(raw_frame) != frame_size:
                self.get_logger().warn('Incomplete frame, skipping...')
                continue

            # Convert raw bytes to numpy array and reshape into YUV420 format
            yuv_frame = np.frombuffer(raw_frame, dtype=np.uint8)
            yuv_frame = yuv_frame.reshape((int(self.height * 1.5), self.width))

            # Convert YUV420 to BGR — OpenCV's native color format
            bgr_frame = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2BGR_I420)

            # Convert OpenCV frame to ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(bgr_frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'

            # Publish to /camera/image_raw for vision node to consume
            self.publisher.publish(msg)

        process.terminate()

    def destroy_node(self):
        self.running = False
        self.capture_thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()