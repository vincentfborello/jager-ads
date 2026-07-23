import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import time

class FiringNode(Node):
    def __init__(self):
        super().__init__('firing_node')

        # Subscribe to target position from vision node
        # This gives us x, y (position) and z (contour area) every frame
        self.subscription = self.create_subscription(
            Point,
            '/target/position',
            self.evaluate_target,
            10
        )

        # Publisher that sends a fire command to the physical firing mechanism
        # True = fire, False = don't fire
        # When hardware arrives this gets wired to the servo trigger
        self.fire_publisher = self.create_publisher(Bool, '/firing/trigger', 10)

        # --- Tunable constants --- adjust these during real testing ---

        # How many consecutive frames the target must be detected before firing
        # Higher = more reliable but slower reaction time
        self.lock_on_frames_required = 5

        # How close to frame center the target must be (in pixels) to be considered locked
        # Frame is 640x480 so center is (320, 240)
        # 40px means the target must be within a 80x80px box around center
        self.center_threshold_px = 40

        # Contour area limits — filters out non-projectile sized objects
        # Minimum: anything smaller than a tennis ball at range gets ignored
        # Maximum: anything larger than a tennis ball (people, large objects) gets ignored
        self.min_projectile_area = 800
        self.max_projectile_area = 3000

        # Cooldown between shots in seconds — prevents mag dump on one target
        self.fire_cooldown_seconds = 2.0

        # Frame center — matches what turret controller uses
        self.frame_center_x = 320
        self.frame_center_y = 240

        # --- Internal state ---

        # Running count of consecutive frames where lock-on conditions are met
        self.consecutive_lock_frames = 0

        # Timestamp of last shot fired — used to enforce cooldown
        self.last_fire_time = 0.0

        # Whether the system is currently in cooldown
        self.in_cooldown = False

        self.get_logger().info('Firing node online. Waiting for lock-on...')

    def evaluate_target(self, msg):
        # Extract target data from the Point message
        target_x = msg.x
        target_y = msg.y
        contour_area = msg.z

        # --- Condition 1: Size filter ---
        # Is this object the right size to be a projectile?
        # Rejects people, large moving objects, and tiny noise that slipped through
        if not (self.min_projectile_area <= contour_area <= self.max_projectile_area):
            self.consecutive_lock_frames = 0
            self.get_logger().debug(
                f'Target rejected — area {contour_area:.0f}px outside projectile range'
            )
            return

        # --- Condition 2: Center threshold ---
        # Is the turret actually aimed at the target?
        # Calculate pixel distance from target to frame center
        x_error = abs(target_x - self.frame_center_x)
        y_error = abs(target_y - self.frame_center_y)

        if x_error > self.center_threshold_px or y_error > self.center_threshold_px:
            self.consecutive_lock_frames = 0
            self.get_logger().debug(
                f'Target not centered — error ({x_error:.0f}, {y_error:.0f})px'
            )
            return

        # --- Condition 3: Consecutive frame counter ---
        # Both conditions passed — increment lock-on counter
        self.consecutive_lock_frames += 1
        self.get_logger().info(
            f'Lock-on progress: {self.consecutive_lock_frames}/{self.lock_on_frames_required} frames'
        )

        # Not enough consecutive frames yet — keep waiting
        if self.consecutive_lock_frames < self.lock_on_frames_required:
            return

        # --- Condition 4: Cooldown check ---
        # All lock-on conditions met — check if we're still in cooldown
        current_time = time.time()
        time_since_last_fire = current_time - self.last_fire_time

        if time_since_last_fire < self.fire_cooldown_seconds:
            remaining = self.fire_cooldown_seconds - time_since_last_fire
            self.get_logger().info(f'Locked on but in cooldown — {remaining:.1f}s remaining')
            return

        # --- Fire! ---
        # All conditions met and cooldown expired — send fire command
        fire_msg = Bool()
        fire_msg.data = True
        self.fire_publisher.publish(fire_msg)

        # Reset state
        self.last_fire_time = current_time
        self.consecutive_lock_frames = 0

        self.get_logger().info('FIRE — target locked, projectile size confirmed, centered')

def main():
    rclpy.init()
    node = FiringNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()