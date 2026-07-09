import sys
sys.path.insert(0, '/usr/local/lib/python3.12/dist-packages')

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoHardwareNode(Node):
    def __init__(self):
        super().__init__('servo_hardware_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/servo/commands',
            self.move_servos,
            10
        )

        # Initialize I2C bus using the Pi's hardware I2C pins
        i2c = busio.I2C(board.SCL, board.SDA)

        # Initialize PCA9685 and set PWM frequency to 50Hz for servos
        pca = PCA9685(i2c)
        pca.frequency = 50

        # Create servo objects on channels 0 and 1
        # min_pulse and max_pulse define the PWM range for MG996R servos
        self.pan_servo = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
        self.tilt_servo = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)

        self.get_logger().info('Servo hardware node online.')

    def move_servos(self, msg):
        # Extract pan and tilt angles from the Float32MultiArray message
        pan_angle = msg.data[0]
        tilt_angle = msg.data[1]

        # Clamp angles to safe physical range just in case
        pan_angle = max(0.0, min(180.0, pan_angle))
        tilt_angle = max(0.0, min(180.0, tilt_angle))

        # Send angles directly to physical servos
        self.pan_servo.angle = pan_angle
        self.tilt_servo.angle = tilt_angle

        self.get_logger().info(
            f'Pan: {pan_angle:.1f} Tilt: {tilt_angle:.1f}'
        )


def main():
    rclpy.init()
    node = ServoHardwareNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()