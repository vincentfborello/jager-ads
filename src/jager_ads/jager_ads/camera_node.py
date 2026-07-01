import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Publisher that will send raw camera frames to the /camera/image_raw topic
        # Other nodes subscribe to this topic to receive frames
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.get_logger().info('Camera node online. Waiting for hardware...')

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()