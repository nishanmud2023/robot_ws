#!/usr/bin/env python3
"""
MPU-6050 IMU driver node for ROS2 Jazzy.

Reads accelerometer + gyroscope data over I2C from an MPU-6050 (HW-123 breakout)
and publishes sensor_msgs/Imu on the 'imu/data_raw' topic.

Wiring (Raspberry Pi):
    VCC -> 3.3V or 5V (board has onboard regulator, either works)
    GND -> GND
    SCL -> Pi SCL (GPIO3 / pin 5)
    SDA -> Pi SDA (GPIO2 / pin 3)

Before running, enable I2C on the Pi (raspi-config -> Interface Options -> I2C),
then confirm the chip is detected:
    sudo i2cdetect -y 1
You should see address 0x68 (default) or 0x69 (if AD0 pin is pulled high).

Install dependency:
    pip install smbus2 --break-system-packages

Run:
    ros2 run serial_interface mpu6050_node
    (or directly: python3 mpu6050_node.py)

Check output:
    ros2 topic echo /imu/data_raw
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import smbus2 as smbus
except ImportError:
    import smbus


# ---- MPU-6050 register map ----
MPU6050_ADDR = 0x68          # default I2C address; use 0x69 if AD0 is high
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Sensitivity scale factors for default full-scale ranges (+/-2g, +/-250 deg/s)
ACCEL_SCALE = 16384.0   # LSB per g
GYRO_SCALE = 131.0      # LSB per deg/s
GRAVITY = 9.80665       # m/s^2 per g
DEG_TO_RAD = math.pi / 180.0


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', MPU6050_ADDR)
        self.declare_parameter('frame_id', 'imu_sensor_mount')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('calibrate_on_start', True)
        self.declare_parameter('calibration_samples', 200)

        self.i2c_bus_num = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        self.calibrate_on_start = self.get_parameter('calibrate_on_start').value
        self.calibration_samples = self.get_parameter('calibration_samples').value

        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_bias = [0.0, 0.0, 0.0]

        self.bus = smbus.SMBus(self.i2c_bus_num)
        self._init_sensor()

        if self.calibrate_on_start:
            self.get_logger().info(
                'Calibrating IMU bias — keep the robot still and level...'
            )
            self._calibrate()
            self.get_logger().info(
                f'Calibration done. Gyro bias (deg/s): {self.gyro_bias}'
            )

        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_imu)

        self.get_logger().info(
            f'MPU-6050 node started on bus {self.i2c_bus_num}, '
            f'address 0x{self.address:02X}, publishing at {rate_hz} Hz'
        )

    def _init_sensor(self):
        # Wake up the sensor (it starts in sleep mode)
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # Sample rate divider: 1kHz / (1 + 7) = 125 Hz internal sample rate
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 0x07)
        # Digital low pass filter
        self.bus.write_byte_data(self.address, CONFIG, 0x03)
        # Gyro full scale range: +/-250 deg/s (default, matches GYRO_SCALE above)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)
        # Accel full scale range: +/-2g (default, matches ACCEL_SCALE above)
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)
        time.sleep(0.1)

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value -= 0x10000
        return value

    def _read_raw(self):
        ax = self._read_word(ACCEL_XOUT_H)
        ay = self._read_word(ACCEL_XOUT_H + 2)
        az = self._read_word(ACCEL_XOUT_H + 4)
        gx = self._read_word(GYRO_XOUT_H)
        gy = self._read_word(GYRO_XOUT_H + 2)
        gz = self._read_word(GYRO_XOUT_H + 4)
        return (ax, ay, az, gx, gy, gz)

    def _calibrate(self):
        """Average N samples while assumed stationary to compute gyro/accel bias.
        NOTE: this only zeroes out constant offset error, it does NOT fix
        long-term drift, which is inherent to MEMS gyros. Fuse with wheel
        odometry (robot_localization ekf_node) rather than trusting IMU alone.
        """
        gx_sum = gy_sum = gz_sum = 0.0
        ax_sum = ay_sum = az_sum = 0.0
        n = self.calibration_samples

        for _ in range(n):
            ax, ay, az, gx, gy, gz = self._read_raw()
            ax_sum += ax / ACCEL_SCALE
            ay_sum += ay / ACCEL_SCALE
            az_sum += az / ACCEL_SCALE
            gx_sum += gx / GYRO_SCALE
            gy_sum += gy / GYRO_SCALE
            gz_sum += gz / GYRO_SCALE
            time.sleep(0.005)

        self.gyro_bias = [gx_sum / n, gy_sum / n, gz_sum / n]
        # Only bias gravity out of X/Y (assume robot resting flat, Z should read ~1g)
        self.accel_bias = [ax_sum / n, ay_sum / n, (az_sum / n) - 1.0]

    def _publish_imu(self):
        try:
            ax, ay, az, gx, gy, gz = self._read_raw()
        except OSError as e:
            self.get_logger().warn(f'I2C read failed: {e}', throttle_duration_sec=5.0)
            return

        # Convert to physical units and remove calibrated bias
        accel_x = (ax / ACCEL_SCALE - self.accel_bias[0]) * GRAVITY
        accel_y = (ay / ACCEL_SCALE - self.accel_bias[1]) * GRAVITY
        accel_z = (az / ACCEL_SCALE - self.accel_bias[2]) * GRAVITY

        gyro_x = (gx / GYRO_SCALE - self.gyro_bias[0]) * DEG_TO_RAD
        gyro_y = (gy / GYRO_SCALE - self.gyro_bias[1]) * DEG_TO_RAD
        gyro_z = (gz / GYRO_SCALE - self.gyro_bias[2]) * DEG_TO_RAD

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        # MPU-6050 has no onboard magnetometer/fusion -> no absolute orientation.
        # Mark orientation as unknown per REP-103 convention.
        msg.orientation_covariance[0] = -1.0

        # Reasonable starting covariances for a cheap MEMS IMU — tune later
        # against real static/dynamic noise if EKF results look off.
        accel_cov = 0.04
        gyro_cov = 0.02
        for i in range(3):
            msg.linear_acceleration_covariance[i * 3 + i] = accel_cov
            msg.angular_velocity_covariance[i * 3 + i] = gyro_cov

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
