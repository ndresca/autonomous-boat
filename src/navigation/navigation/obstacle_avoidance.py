import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
import time

# Obstacle-avoidance override for waypoint_nav. Shares the 'set_motor_speeds'
# topic with waypoint_nav: when the camera reports an obstacle in the boat's
# path, this node publishes override commands (stop for STOP_DURATION_S, then
# a gentle right turn until the obstacle clears). While clear it publishes
# nothing, so waypoint_nav's commands reach motor_controller unmodified.
# Arbitration is last-write-wins on the shared topic — obstacle_avoidance
# ticks at the same rate as waypoint_nav, so its overrides win during
# obstacle events.
#
# Only active in the mission launch (not teleop).
#
# Publishers:  'set_motor_speeds'
# Subscribers: 'obstacle_detected', 'set_motor_speeds'

STOP_DURATION_S = 2.0         # seconds to hold full stop before turning
TURN_LEFT_SPEED = 0.2         # gentle right turn: left forward, right reverse
TURN_RIGHT_SPEED = -0.2
TICK_PERIOD_S = 0.1           # 10 Hz override loop

# Finite-state machine states
STATE_CLEAR = 'clear'   # no override, waypoint_nav drives the motors
STATE_STOP = 'stop'     # publishing (0.0, 0.0) for STOP_DURATION_S
STATE_TURN = 'turn'     # publishing gentle right turn until obstacle clears


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.state = STATE_CLEAR
        self.stop_started_at = None
        self.obstacle_present = False
        # Latest command observed on 'set_motor_speeds' (whoever published it).
        # Cached for logging/debugging; not used to drive output.
        self.latest_nav_cmd = [0.0, 0.0]

        self.obstacle_sub = self.create_subscription(
            Bool, 'obstacle_detected', self.obstacle_callback, 10)

        self.nav_sub = self.create_subscription(
            Float32MultiArray, 'set_motor_speeds', self.nav_callback, 10)

        self.cmd_pub = self.create_publisher(
            Float32MultiArray, 'set_motor_speeds', 10)

        self.timer = self.create_timer(TICK_PERIOD_S, self.tick)

        self.get_logger().info('ObstacleAvoidance started — monitoring for obstacles.')

    # Track the most recent obstacle reading and kick off the stop/turn
    # sequence on the rising edge while we are otherwise clear.
    def obstacle_callback(self, msg: Bool):
        self.obstacle_present = bool(msg.data)
        if self.obstacle_present and self.state == STATE_CLEAR:
            self.state = STATE_STOP
            self.stop_started_at = time.monotonic()
            self.get_logger().info('Obstacle detected — stopping for '
                                   f'{STOP_DURATION_S:.1f}s.')

    # Observe 'set_motor_speeds' traffic. Note: this also receives our own
    # override publishes; harmless since we don't act on latest_nav_cmd.
    def nav_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.latest_nav_cmd = [float(msg.data[0]), float(msg.data[1])]

    # Override loop: publishes overriding commands on 'set_motor_speeds'
    # during STOP and TURN; stays silent during CLEAR so waypoint_nav's
    # commands reach motor_controller (last-write-wins on the shared topic).
    def tick(self):
        if self.state == STATE_CLEAR:
            return

        if self.state == STATE_STOP:
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
                self.get_logger().info('Obstacle cleared — yielding to '
                                       'waypoint_nav.')
                return
            left, right = TURN_LEFT_SPEED, TURN_RIGHT_SPEED
        else:
            return

        out = Float32MultiArray()
        out.data = [left, right]
        self.cmd_pub.publish(out)


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
