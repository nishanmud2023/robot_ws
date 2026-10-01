#!/usr/bin/env python3
"""
Odometry diagnostic logger.

Records wheel odometry, raw IMU, and EKF-filtered odometry side by side to a
CSV file so you can analyze sensor behavior offline (after leaving the lab).

Run this in its own terminal WHILE the robot bring-up is running, then perform
your physical tests (straight 1m, 90-degree rotation, etc.). Note the wall-clock
time when you start/stop each physical maneuver so you can find it in the CSV.

Usage:
    python3 odom_diagnostic.py                 # logs to ./odom_diagnostic.csv
    python3 odom_diagnostic.py my_test.csv     # custom filename

Stop with Ctrl-C. The CSV is flushed continuously so nothing is lost.

Columns:
    t_wall        : wall-clock seconds since script start (use this to align
                    with your physical maneuvers)
    wheel_x/y/yaw : integrated pose from wheel odometry (/wheel/odometry)
    wheel_vx/vy/wz: body velocities reported by wheel odometry
    imu_wz        : IMU yaw rate (rad/s) from /imu/data_raw angular_velocity.z
    ekf_x/y/yaw   : fused pose from EKF (/odometry/filtered)
    ekf_vx/vy/wz  : fused body velocities from EKF
"""

import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_from_quat(q):
    """Extract yaw (rotation about z) from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomDiagnostic(Node):
    def __init__(self, csv_path):
        super().__init__('odom_diagnostic')

        # Latest values from each source (updated by callbacks)
        self.wheel = None   # (x, y, yaw, vx, vy, wz)
        self.imu_wz = None  # yaw rate rad/s
        self.ekf = None     # (x, y, yaw, vx, vy, wz)

        self.create_subscription(Odometry, '/wheel/odometry', self.wheel_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)

        self.start_wall = time.time()

        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            't_wall',
            'wheel_x', 'wheel_y', 'wheel_yaw', 'wheel_vx', 'wheel_vy', 'wheel_wz',
            'imu_wz',
            'ekf_x', 'ekf_y', 'ekf_yaw', 'ekf_vx', 'ekf_vy', 'ekf_wz',
        ])

        # Log a synchronized row at 20 Hz regardless of individual topic rates
        self.timer = self.create_timer(0.05, self.log_row)
        self.get_logger().info(f"Logging to {csv_path} — Ctrl-C to stop.")

    def wheel_cb(self, msg):
        self.wheel = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )

    def imu_cb(self, msg):
        self.imu_wz = msg.angular_velocity.z

    def ekf_cb(self, msg):
        self.ekf = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quat(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )

    def log_row(self):
        t = time.time() - self.start_wall
        w = self.wheel if self.wheel else (None,) * 6
        e = self.ekf if self.ekf else (None,) * 6
        self.writer.writerow([
            f"{t:.3f}",
            *[f"{v:.5f}" if v is not None else "" for v in w],
            f"{self.imu_wz:.5f}" if self.imu_wz is not None else "",
            *[f"{v:.5f}" if v is not None else "" for v in e],
        ])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else 'odom_diagnostic.csv'
    rclpy.init()
    node = OdomDiagnostic(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
