#!/usr/bin/env python3
"""
Real-hardware bringup for the omniwheel (mecanum) robot.

Runs on the Raspberry Pi. Replaces everything Gazebo used to provide:
    Gazebo /cmd_vel consumer  ->  serial_node (sends wheel cmds to Arduino Mega)
    Gazebo /odom publisher    ->  serial_node (encoder ticks -> odometry)
    Gazebo /imu publisher     ->  mpu6050_node (real MPU-6050 over I2C)
    Gazebo /scan publisher    ->  LiDAR driver (commented out below, add later)
    Gazebo /tf                ->  robot_state_publisher + ekf_node

Does NOT launch nav2, slam_toolbox, or RViz — those run on the laptop via
navigation_with_slam.launch.py with use_sim_time:=False.

Sim is unaffected by this file. It reads config/ekf_real.yaml, a copy of
ekf.yaml with the IMU inputs adjusted for the MPU-6050. Gazebo keeps using
the original ekf.yaml.

Usage on Pi:
    ros2 launch omniwheel_complete bringupomni_real.launch.py

Optional args:
    serial_port:=/dev/ttyACM0      Arduino serial device
    use_ekf:=false                 skip sensor fusion, publish raw wheel odom only
    calibrate_imu:=false           skip IMU startup bias calibration
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_omniwheel_complete = get_package_share_directory('omniwheel_complete')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    model_arg = DeclareLaunchArgument(
        'model', default_value='omniwheel.urdf.xacro',
        description='URDF/xacro description file to load'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Serial device for the Arduino Mega'
    )

    baud_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Serial baud rate — must match the Arduino firmware'
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='true',
        description='Fuse wheel odom + IMU with robot_localization ekf_node'
    )

    calibrate_imu_arg = DeclareLaunchArgument(
        'calibrate_imu', default_value='true',
        description='Run IMU bias calibration at startup (robot must be still)'
    )

    # Real hardware: never use simulated clock.
    use_sim_time = False

    urdf_file_path = PathJoinSubstitution([
        pkg_omniwheel_complete,
        'urdf',
        LaunchConfiguration('model')
    ])

    # ------------------------------------------------------------------
    # robot_state_publisher — publishes the TF tree from the URDF
    # (base_link -> wheels, base_link -> imu_sensor_mount,
    #  base_link -> lidar_sensor_mount)
    # ------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', urdf_file_path]),
            'use_sim_time': use_sim_time,
        }],
    )

    # ------------------------------------------------------------------
    # serial_node — the Arduino bridge.
    # Subscribes /cmd_vel, does inverse kinematics, sends wheel commands.
    # Reads encoder ticks back, publishes /odom (+ odom->base_link TF).
    #
    # The four geometry values below are commented out so the node's own
    # declared defaults apply (wheel_radius 0.1, wheelbase 0.3,
    # wheel_separation 0.3, ticks_per_rev 468). Measure the real robot with
    # calipers/tape, then uncomment and set the true values here rather than
    # editing serial_node.py — that keeps sim and real on one code path.
    # ------------------------------------------------------------------
    serial_node = Node(
        package='serial_interface',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            # 'wheel_radius': 0.1,
            # 'wheelbase': 0.3,
            # 'wheel_separation': 0.3,
            # 'ticks_per_rev': 468,
            'use_sim_time': use_sim_time,
        }],
    )

    # ------------------------------------------------------------------
    # mpu6050_node — real IMU over I2C, publishes imu/data_raw
    #
    # frame_id must match the URDF link name, which the sensor_mount macro
    # builds as "${prefix}_sensor_mount" -> imu_sensor_mount.
    #
    # The URDF mounts it with rpy="0 0 0", so bolt the chip down with its
    # X axis pointing robot-forward, or the gyro axes won't match what the
    # EKF expects.
    # ------------------------------------------------------------------
    imu_node = Node(
        package='serial_interface',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[{
            'i2c_bus': 1,
            'i2c_address': 0x68,
            'frame_id': 'imu_sensor_mount',
            'publish_rate_hz': 50.0,
            'calibrate_on_start': LaunchConfiguration('calibrate_imu'),
            'use_sim_time': use_sim_time,
        }],
    )

    # ------------------------------------------------------------------
    # ekf_node — fuses wheel odom + IMU -> /odometry/filtered
    #
    # Uses ekf_real.yaml (NOT ekf.yaml, which sim still needs). That file
    # should have imu0: imu/data_raw, and roll/pitch/yaw set false in
    # imu0_config — the MPU-6050 has no magnetometer and cannot provide
    # absolute orientation, only angular velocity.
    # ------------------------------------------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        parameters=[
            os.path.join(pkg_omniwheel_complete, 'config', 'ekf_real.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # LiDAR driver — ADD LATER, once you know the model.
    #
    # Uncomment and adjust once the driver package is installed. Example
    # shown for RPLidar; YDLidar/Hokuyo use different package/executable
    # names and parameters, so treat this as a shape, not a recipe.
    #
    # What matters regardless of model:
    #   - frame_id must be lidar_sensor_mount (from your URDF macro)
    #   - it must publish sensor_msgs/LaserScan on /scan
    #   - set a udev rule so the port is stable (/dev/rplidar not ttyUSB0)
    #   - the URDF mounts it at xyz="0.1 0.0 0.28" rpy="0 0 0" — if the
    #     physical mount ends up rotated, fix the URDF rpy rather than
    #     compensating in software
    #
    # lidar_node = Node(
    #     package='rplidar_ros',
    #     executable='rplidar_composition',
    #     name='rplidar_node',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': '/dev/ttyUSB0',
    #         'serial_baudrate': 115200,
    #         'frame_id': 'lidar_sensor_mount',
    #         'angle_compensate': True,
    #         'use_sim_time': use_sim_time,
    #     }],
    # )
    # ------------------------------------------------------------------

    ld = LaunchDescription()

    ld.add_action(model_arg)
    ld.add_action(serial_port_arg)
    ld.add_action(baud_arg)
    ld.add_action(use_ekf_arg)
    ld.add_action(calibrate_imu_arg)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(serial_node)
    ld.add_action(imu_node)
    ld.add_action(ekf_node)
    # ld.add_action(lidar_node)   # <- uncomment when LiDAR is added

    return ld
