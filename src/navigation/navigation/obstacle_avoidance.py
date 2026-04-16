import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
import time

# Obstacle-avoidance arbiter between waypoint_nav and motor_controller.
# Normally passes waypoint_nav motor commands straight through. When the
# camera reports an obstacle in the boat's path, it overrides with a
# timed stop followed by a gentle right turn until the obstacle clears.
#
# Publishers:  'set_motor_speeds_safe'
# Subscribers: 'obstacle_detected', 'set_motor_speeds'

STOP_DURATION_S = 2.0         # seconds to hold full stop before turning
TURN_LEFT_SPEED = 0.2         # gentle right turn: left forward, right reverse
TURN_RIGHT_SPEED = -0.2
TICK_PERIOD_S = 0.1           # 10 Hz arbitration loop

# Finite-state machine states
STATE_CLEAR = 'clear'   # passthrough from waypoint_nav
STATE_STOP = 'stop'     # holding (0.0, 0.0) for STOP_DURATION_S
STATE_TURN = 'turn'     # gentle right turn until obstacle clears


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.state = STATE_CLEAR
        self.stop_started_at = None
        self.obstacle_present = False
        # Latest command received from waypoint_nav; default to stopped
        # so we never forward stale or uninitialized data.
        self.latest_nav_cmd = [0.0, 0.0]

        self.obstacle_sub = self.create_subscription(
            Bool, 'obstacle_detected', self.obstacle_callback, 10)

        self.nav_sub = self.create_subscription(
            Float32MultiArray, 'set_motor_speeds', self.nav_callback, 10)

        self.safe_pub = self.create_publisher(
            Float32MultiArray, 'set_motor_speeds_safe', 10)

        self.timer = self.create_timer(TICK_PERIOD_S, self.tick)

        self.get_logger().info('ObstacleAvoidance started in passthrough mode.')

    # Track the most recent obstacle reading and kick off the stop/turn
    # sequence on the rising edge while we are otherwise clear.
    def obstacle_callback(self, msg: Bool):
        self.obstacle_present = bool(msg.data)
        if self.obstacle_present and self.state == STATE_CLEAR:
            self.state = STATE_STOP
            self.stop_started_at = time.monotonic()
            self.get_logger().info('Obstacle detected — stopping for '
                                   f'{STOP_DURATION_S:.1f}s.')

    # Cache the latest waypoint_nav command for passthrough.
    def nav_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.latest_nav_cmd = [float(msg.data[0]), float(msg.data[1])]

    # Arbitration loop: publishes the safe command every tick based on
    # the current state machine state.
    def tick(self):
        if self.state == STATE_CLEAR:
            left, right = self.latest_nav_cmd
        elif self.state == STATE_STOP:
            left, right = 0.0, 0.0
            if time.monotonic() - self.stop_started_at >= STOP_DURATION_S:
                self.state = STATE_TURN
                self.get_logger().info('Stop complete — turning right until '
                                       'obstacle clears.')
                left, right = TURN_LEFT_SPEED, TURN_RIGHT_SPEED
        elif self.state == STATE_TURN:
            if not self.obstacle_present:
                self.state = STATE_CLEAR
                self.stop_started_at = None
                self.get_logger().info('Obstacle cleared — resuming '
                                       'waypoint navigation.')
                left, right = self.latest_nav_cmd
            else:
                left, right = TURN_LEFT_SPEED, TURN_RIGHT_SPEED
        else:
            left, right = 0.0, 0.0

        out = Float32MultiArray()
        out.data = [left, right]
        self.safe_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down obstacle_avoidance.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
