import rclpy
from rclpy.node import Node

class ADSNode(Node):
    def __init__(self):
        super().__init__('ads_test')
        self.get_logger().info('Jager ADS Online')
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        self.get_logger().info('System nominal.')
    
def main():
    rclpy.init()
    node = ADSNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()