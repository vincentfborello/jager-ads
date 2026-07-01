import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Subscriber that listens for raw frames from the camera node
        # Every time a new frame arrives, it automatically calls self.process_frame
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_frame,
            10
        )

        # Publisher that sends the detected target's position to the turret controller
        # Point is a ROS2 message type that holds x, y, z coordinates
        # We'll use x and y for pixel coordinates, z we'll leave as 0 for now
        self.publisher = self.create_publisher(Point, '/target/position', 10)

        self.get_logger().info('Vision node online. Waiting for frames...')

    def process_frame(self, msg):
        # This function gets called automatically every time a new frame
        # arrives on the /camera/image_raw topic
        # msg is the raw Image message from the camera node
        # When the camera arrives we'll add MOG2 detection logic here
        self.get_logger().info('Frame received.')

def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()