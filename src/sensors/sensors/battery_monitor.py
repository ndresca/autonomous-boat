import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import smbus
import time

# This node monitors battery/solar voltage via an ADS1115 ADC over I2C.
# Publishes voltage to 'battery_voltage' and triggers 'emergency_stop'
# if voltage drops below a safe threshold.
#
# Publishers:  'battery_voltage', 'emergency_stop'
# Subscribers: ''
#
# Hardware assumed: ADS1115 ADC module connected to Pi I2C bus (0x48)
# Voltage divider on AIN0: R1=37.5kΩ, R2=7.5kΩ → scale factor = 5.0
# Adjust VOLTAGE_DIVIDER_RATIO and LOW_VOLTAGE_THRESHOLD for your battery.

# ── Configuration ──────────────────────────────────────────────
ADS1115_ADDRESS       = 0x48   # default I2C address of ADS1115
ADS1115_CONVERSION    = 0x00   # conversion register
ADS1115_CONFIG        = 0x01   # config register

# Config: single-shot, AIN0/GND, ±4.096V FSR, 128 SPS
ADS1115_CONFIG_VALUE  = 0xC385

VOLTAGE_DIVIDER_RATIO = 5.0    # (R1+R2)/R2 — adjust to match your resistors
ADC_FULL_SCALE_V      = 4.096  # volts at full ADS1115 range
ADC_MAX_COUNTS        = 32767  # 15-bit positive range

LOW_VOLTAGE_THRESHOLD = 12.0   # volts — trigger RTL below this (3S LiPo cutoff)
WARN_VOLTAGE          = 12.8   # volts — log warning but keep running
CHECK_INTERVAL        = 5.0    # seconds between voltage reads


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Publisher for raw voltage — other nodes can subscribe for logging
        self.voltage_publisher = self.create_publisher(Float32, 'battery_voltage', 10)

        # Emergency stop — same topic used by water_sensor
        self.estop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)

        # I2C bus (bus 1 on Raspberry Pi)
        try:
            self.bus = smbus.SMBus(1)
            self.get_logger().info('Battery monitor initialized on I2C bus 1.')
        except Exception as e:
            self.get_logger().error(f'I2C init failed: {e}')
            self.bus = None

        # Timer — check voltage every CHECK_INTERVAL seconds
        self.timer = self.create_timer(CHECK_INTERVAL, self.timer_callback)

        # Track previous warning state to avoid log spam
        self.warning_issued = False

    def read_voltage(self) -> float:
        """
        Reads raw ADC counts from ADS1115 and converts to battery voltage.
        Returns -1.0 on read failure.
        """
        if self.bus is None:
            return -1.0
        try:
            # Write config to trigger single-shot conversion
            config_msb = (ADS1115_CONFIG_VALUE >> 8) & 0xFF
            config_lsb = ADS1115_CONFIG_VALUE & 0xFF
            self.bus.write_i2c_block_data(
                ADS1115_ADDRESS, ADS1115_CONFIG, [config_msb, config_lsb])

            # Wait for conversion to complete (~8ms at 128 SPS)
            time.sleep(0.01)

            # Read 2 bytes from conversion register
            data = self.bus.read_i2c_block_data(ADS1115_ADDRESS, ADS1115_CONVERSION, 2)
            raw = (data[0] << 8) | data[1]

            # Convert unsigned to signed 16-bit
            if raw > 32767:
                raw -= 65536

            # Convert counts → ADC voltage → battery voltage
            adc_voltage     = (raw / ADC_MAX_COUNTS) * ADC_FULL_SCALE_V
            battery_voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO
            return round(battery_voltage, 2)

        except Exception as e:
            self.get_logger().error(f'Voltage read failed: {e}')
            return -1.0

    def timer_callback(self):
        voltage = self.read_voltage()

        if voltage < 0:
            self.get_logger().warn('Could not read battery voltage.')
            return

        # Publish raw voltage for telemetry/logging
        msg = Float32()
        msg.data = voltage
        self.voltage_publisher.publish(msg)

        self.get_logger().info(f'Battery voltage: {voltage}V')

        # ── Low voltage warning ───────────────────────────────
        if voltage <= WARN_VOLTAGE and not self.warning_issued:
            self.get_logger().warn(
                f'Battery low: {voltage}V — approaching cutoff threshold.')
            self.warning_issued = True

        # ── Critical — trigger emergency stop ─────────────────
        if voltage <= LOW_VOLTAGE_THRESHOLD:
            self.get_logger().error(
                f'CRITICAL: Battery at {voltage}V — triggering emergency stop.')
            estop_msg = Bool()
            estop_msg.data = True
            self.estop_publisher.publish(estop_msg)


def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
