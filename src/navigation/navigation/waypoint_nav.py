import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32MultiArray
import math
import time

# Waypoint navigation node for autonomous supply delivery.
# Steers the vessel toward a sequence of GPS waypoints using a
# proportional controller on bearing error and distance.
#
# Publishers:  'set_motor_speeds'
# Subscribers: 'get_GPS', 'emergency_stop'

# ── Mission waypoints ──────────────────────────────────────────
# Replace these lat/lon pairs with your actual target coordinates.
WAYPOINTS = [
    (34.01234, -117.98765),   # Waypoint 1
    (34.01300, -117.98700),   # Waypoint 2
    (34.01400, -117.98650),   # Waypoint 3
]

ACCEPTANCE_RADIUS_M = 2.0   # metres — how close counts as "reached"
MAX_SPEED          = 0.5    # motor speed cap  (-1 … 1 scale)
KP_BEARING         = 0.008  # proportional gain for turning
KP_DISTANCE        = 0.003  # proportional gain for forward speed


class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        self.current_wp_index = 0
        self.current_lat = None
        self.current_lon = None

        self.speed_publisher = self.create_publisher(
            Float32MultiArray, 'set_motor_speeds', 10)

        self.gps_subscriber = self.create_subscription(
            NavSatFix, 'get_GPS', self.gps_callback, 10)

        self.emergency_stop = self.create_subscription(
            Bool, 'emergency_stop', self.destroy_node, 10)

        self.nav_timer = self.create_timer(0.1, self.navigate)

        self.get_logger().info('WaypointNav started. '
                               f'First target: {WAYPOINTS[0]}')

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def navigate(self):
        if self.current_lat is None:
            self.get_logger().info('Waiting for GPS fix…', throttle_duration_sec=5)
            return

        if self.current_wp_index >= len(WAYPOINTS):
            self.get_logger().info('All waypoints reached. Stopping.')
            self.publish_speed(0.0, 0.0)
            self.nav_timer.cancel()
            return

        target_lat, target_lon = WAYPOINTS[self.current_wp_index]

        distance = self.haversine(
            self.current_lat, self.current_lon,
            target_lat, target_lon)

        bearing_error = self.bearing_error(
            self.current_lat, self.current_lon,
            target_lat, target_lon)

        if distance < ACCEPTANCE_RADIUS_M:
            self.get_logger().info(
                f'Waypoint {self.current_wp_index + 1} reached!')
            self.current_wp_index += 1
            return

        turn_speed    = KP_BEARING  * bearing_error
        forward_speed = KP_DISTANCE * distance

        heading_factor = max(0.0, 1.0 - abs(bearing_error) / 90.0)
        forward_speed  = forward_speed * heading_factor

        left_speed  = forward_speed + turn_speed
        right_speed = forward_speed - turn_speed

        left_speed  = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

        self.publish_speed(left_speed, right_speed)

        self.get_logger().info(
            f'WP {self.current_wp_index + 1}/{len(WAYPOINTS)} | '
            f'dist={distance:.1f}m  bearing_err={bearing_error:.1f}°  '
            f'L={left_speed:.2f}  R={right_speed:.2f}',
            throttle_duration_sec=2)

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2) -> float:
        R = 6_371_000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi  = math.radians(lat2 - lat1)
        dlam  = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2
             + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def bearing_error(lat1, lon1, lat2, lon2) -> float:
        dlam   = math.radians(lon2 - lon1)
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)

        x = math.sin(dlam) * math.cos(lat2_r)
        y = (math.cos(lat1_r) * math.sin(lat2_r)
             - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlam))

        bearing = math.degrees(math.atan2(x, y))

        if bearing > 180:
            bearing -= 360

        return bearing

    def publish_speed(self, left: float, right: float):
        msg = Float32MultiArray()
        msg.data = [left, right]
        self.speed_publisher.publish(msg)

    def destroy_node(self, msg=None):
        self.publish_speed(0.0, 0.0)
        time.sleep(0.1)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt — stopping motors.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
