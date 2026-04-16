import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import serial
import struct
import time

# This node handles the transmission of motor speed commands to the motor controller (ESP32) via USB Serial.
# Scaled speed control values from either teleoperation or autonomous control are sent as two
# signed bytes (int8, -100..100). The ESP32 maps these to ESC PWM microseconds (1000..2000).
# Publishers:
# Subscribers: 'set_motor_speeds', 'emergency_stop'
class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Serial port configuration
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.ser = None
        self.open_serial()

        # Subscription to the 'set_motor_speeds' topic. Multiple publishers
        # (teleop, waypoint_nav, obstacle_avoidance) share this topic;
        # arbitration is last-write-wins.
        self.speed_subscription = self.create_subscription(
            Float32MultiArray,
            'set_motor_speeds',
            self.set_motor_speeds,
            10)

        # Rate limiting: enforce 10 Hz max
        self.min_write_interval = 0.1  # 10 Hz
        self.last_write_time = 0.0

        # Subscription to emergency stop topic
        # If emergency stop is triggered, the node will be destroyed and both motors will be stopped
        self.emergency_stop = self.create_subscription(
            Bool,
            'emergency_stop',
            self.destroy_node,
            10
        )

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'Serial connected on {self.serial_port} @ {self.baud_rate}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {self.serial_port}: {e}')
            self.ser = None

    # Convert a speed in [-1.0, 1.0] to a signed int8 in [-100, 100]
    def convert_speed(self, speed):
        scaled = int(round(speed * 100))
        if scaled > 100:
            scaled = 100
        elif scaled < -100:
            scaled = -100
        return scaled

    # Processes the incoming speed command from the 'set_motor_speeds' topic
    # Input: [Float32MultiArray] msg containing the speed values for left and right motors
    def set_motor_speeds(self, msg):
        if len(msg.data) < 2:
            return
        # Check if the message is correctly formatted and scaled speed values are valid in the range -1 to 1
        if (-1 <= msg.data[0] <= 1 and -1 <= msg.data[1] <= 1):
            left = self.convert_speed(msg.data[0])
            right = self.convert_speed(msg.data[1])
            self.send_value(left, right)

    # Send the scaled speed values to the motor controller via USB Serial
    # Input: [int] left_value, [int] right_value in range [-100, 100]
    # Rate limited to 10 Hz max
    def send_value(self, left_value, right_value):
        if self.ser is None:
            self.open_serial()
            if self.ser is None:
                return

        # Rate limit: skip if called faster than 10 Hz
        now = time.monotonic()
        if now - self.last_write_time < self.min_write_interval:
            return
        self.last_write_time = now

        # Pack as two signed bytes (int8)
        data = struct.pack('bb', left_value, right_value)
        try:
            self.ser.write(data)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self.ser = None

    # Stop the motors by sending zero speed to both
    def stop_motors(self):
        self.send_value(0, 0)

    # Destroy the node when the emergency stop is triggered
    # Stops the motors before shutting down
    def destroy_node(self, msg=None):
        self.stop_motors()
        if self.ser is not None:
            self.ser.close()
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorControllerNode()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info("Stopping motors")
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
