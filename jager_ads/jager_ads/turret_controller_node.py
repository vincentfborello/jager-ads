import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        # The three tuning constants
        # kp = how hard to push proportional to current error
        # ki = how hard to push based on accumulated past error
        # kd = how hard to brake based on how fast error is changing
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error = 0      # Error from last frame
        self.accumulated_error = 0   # Running sum of all past errors
        self.last_time = time.time() # Timestamp of last update

    def compute(self, error):
        current_time = time.time()
        # dt = time elapsed since last update
        # We need this to make the derivative and integral time-accurate
        dt = current_time - self.last_time

        # Proportional term — scaled current error
        proportional = self.kp * error

        # Integral term — accumulate error over time
        # Multiplying by dt makes it time-accurate regardless of loop speed
        self.accumulated_error += error * dt
        integral = self.ki * self.accumulated_error

        # Derivative term — rate of change of error
        # Dividing by dt gives us error per second
        derivative = self.kd * ((error - self.previous_error) / dt) if dt > 0 else 0

        # Store values for next iteration
        self.previous_error = error
        self.last_time = current_time

        # Final output is the sum of all three terms
        return proportional + integral + derivative


class TurretControllerNode(Node):
    def __init__(self):
        super().__init__('turret_controller')

        # Subscribe to target position from vision node
        self.subscription = self.create_subscription(
            Point,
            '/target/position',
            self.track_target,
            10
        )

        # Publisher that sends pan and tilt servo angles to the firing node
        # Float32MultiArray lets us send a list of floats — [pan_angle, tilt_angle]
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/servo/commands',
            10
        )

        # Frame center — this is our target "setpoint"
        # We want the detected object to always be at the center of the frame
        # These will update when we know the real camera resolution
        self.frame_center_x = 320  # Half of 640px wide frame
        self.frame_center_y = 240  # Half of 480px tall frame

        # Current servo angles — start centered at 90 degrees
        self.pan_angle = 90.0
        self.tilt_angle = 90.0

        # Servo angle limits — MG996R can physically move 0-180 degrees
        self.min_angle = 30.0
        self.max_angle = 150.0

        # How much each PID output unit moves the servo in degrees
        # We'll tune this once hardware is connected
        self.angle_scale = 0.15

        # Two PID controllers — one for pan (left/right) one for tilt (up/down)
        # These Kp, Ki, Kd values are starting guesses — we'll tune them with real hardware
        self.pan_pid = PIDController(kp=0.08, ki=0.0, kd=0.0)
        self.tilt_pid = PIDController(kp=0.08, ki=0.0, kd=0.0)

        self.get_logger().info('Turret controller online. Waiting for target...')

    def track_target(self, msg):
        pan_error = msg.x - self.frame_center_x
        tilt_error = msg.y - self.frame_center_y

        # Dead zone — ignore tiny errors caused by detection noise
        # Only move if error is larger than 15 pixels
        if abs(pan_error) < 15:
            pan_error = 0
        if abs(tilt_error) < 15:
            tilt_error = 0

        # If both errors are zero don't move at all
        if pan_error == 0 and tilt_error == 0:
            return

        pan_correction = self.pan_pid.compute(pan_error)
        tilt_correction = self.tilt_pid.compute(tilt_error)

        self.pan_angle += pan_correction * self.angle_scale
        self.tilt_angle += tilt_correction * self.angle_scale

        self.pan_angle = max(self.min_angle, min(self.max_angle, self.pan_angle))
        self.tilt_angle = max(self.min_angle, min(self.max_angle, self.tilt_angle))

        cmd = Float32MultiArray()
        cmd.data = [self.pan_angle, self.tilt_angle]
        self.publisher.publish(cmd)

        self.get_logger().info(
            f'Pan: {self.pan_angle:.1f}° Tilt: {self.tilt_angle:.1f}° '
            f'| Error: ({pan_error:.0f}, {tilt_error:.0f})'
        )

def main():
    rclpy.init()
    node = TurretControllerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()