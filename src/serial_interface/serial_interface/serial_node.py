import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import serial
import math
import time


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheelbase', 0.47)
        self.declare_parameter('wheel_separation', 0.52) #changed from 24 as physically measured
        self.declare_parameter('ticks_per_rev', 936) # changed from 468(in datasheet) to match firm wware and as only A channel is to iSR 4x not needed
        self.declare_parameter('max_rpm', 85.0)
        self.declare_parameter('control_period', 0.02)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # --- Serial setup ---
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {port}: {e}")
            raise

        # --- cmd_vel subscription ---
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # --- Odometry publisher ---
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)

        # --- Per-wheel publishers (for rosbag / plotting) ---
        # commands/states: m and m/s | pwm: signed duty in effort | pd_error: m/s in velocity
        self.wheel_names = ['fl', 'fr', 'bl', 'br']
        self.wheel_cmd_pub = self.create_publisher(JointState, '/wheel/commands', 10)
        self.wheel_state_pub = self.create_publisher(JointState, '/wheel/states', 10)
        self.wheel_pwm_pub = self.create_publisher(JointState, '/wheel/pwm', 10)
        self.wheel_err_pub = self.create_publisher(JointState, '/wheel/pd_error', 10)

        # --- Odometry integration state (dead reckoning) ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous cumulative tick counts per wheel (None until first reading)
        self.prev_ticks = None
        self.last_odom_time = self.get_clock().now()

        # --- Read buffer for parsing incoming serial lines ---
        self.read_buf = ""

        # Poll serial for incoming encoder feedback at ~50Hz alongside cmd_vel
        self.read_timer = self.create_timer(0.02, self.read_serial)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        wheel_radius = self.get_parameter('wheel_radius').value
        wheelbase = self.get_parameter('wheelbase').value
        wheel_separation = self.get_parameter('wheel_separation').value
        k = (wheelbase + wheel_separation) / 2.0

        fl = (vx - vy - k * wz) / wheel_radius
        fr = (vx + vy + k * wz) / wheel_radius
        bl = (vx + vy - k * wz) / wheel_radius
        br = (vx - vy + k * wz) / wheel_radius

        rpm_fl = fl * 60.0 / (2.0 * math.pi)
        rpm_fr = fr * 60.0 / (2.0 * math.pi)
        rpm_bl = bl * 60.0 / (2.0 * math.pi)
        rpm_br = br * 60.0 / (2.0 * math.pi)

        max_rpm = self.get_parameter('max_rpm').value
        rpm_fl = max(min(rpm_fl, max_rpm), -max_rpm)
        rpm_fr = max(min(rpm_fr, max_rpm), -max_rpm)
        rpm_bl = max(min(rpm_bl, max_rpm), -max_rpm)
        rpm_br = max(min(rpm_br, max_rpm), -max_rpm)

        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        control_period = self.get_parameter('control_period').value

        ticks_fl = round(rpm_fl * ticks_per_rev * control_period / 60.0)
        ticks_fr = round(rpm_fr * ticks_per_rev * control_period / 60.0)
        ticks_bl = round(rpm_bl * ticks_per_rev * control_period / 60.0)
        ticks_br = round(rpm_br * ticks_per_rev * control_period / 60.0)

        # Per-wheel commanded surface speed (m/s), from the ticks actually sent
        tick_to_ms = 2.0 * math.pi * wheel_radius / (ticks_per_rev * control_period)
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self.wheel_names
        cmd.velocity = [ticks_fl * tick_to_ms, ticks_fr * tick_to_ms,
                        ticks_bl * tick_to_ms, ticks_br * tick_to_ms]
        self.wheel_cmd_pub.publish(cmd)

        self.send_ticks(ticks_fl, ticks_fr, ticks_bl, ticks_br)

    def send_ticks(self, fl, fr, bl, br):
        packet = f"{fl},{fr},{bl},{br}\n"
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write(packet.encode())
                self.get_logger().debug(f"Sent: {packet.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def read_serial(self):
        """Drain incoming serial bytes, split into lines, parse encoder feedback."""
        try:
            if not (self.serial_port and self.serial_port.is_open):
                return

            while self.serial_port.in_waiting > 0:
                chunk = self.serial_port.read(self.serial_port.in_waiting).decode(errors='ignore')
                self.read_buf += chunk

                while '\n' in self.read_buf:
                    line, self.read_buf = self.read_buf.split('\n', 1)
                    line = line.strip()
                    if line.startswith('E,'):
                        self.process_encoder_line(line)
                    elif line.startswith('P,'):
                        self.process_control_line(line)
                    elif line:
                        self.get_logger().debug(f"Arduino: {line}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial read failed: {e}")

    def process_encoder_line(self, line):
        """Parse 'E,fl,fr,bl,br' cumulative ticks and update odometry."""
        parts = line.split(',')
        if len(parts) != 5:
            return  # malformed, drop it

        try:
            fl, fr, bl, br = (int(p) for p in parts[1:5])
        except ValueError:
            return  # non-numeric junk, drop it

        now = self.get_clock().now()

        if self.prev_ticks is None:
            # First reading: nothing to difference against yet
            self.prev_ticks = (fl, fr, bl, br)
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return  # guard against duplicate/out-of-order timestamps

        d_fl = fl - self.prev_ticks[0]
        d_fr = fr - self.prev_ticks[1]
        d_bl = bl - self.prev_ticks[2]
        d_br = br - self.prev_ticks[3]

        self.prev_ticks = (fl, fr, bl, br)
        self.last_odom_time = now

        ticks_per_rev = self.get_parameter('ticks_per_rev').value

        # ticks -> wheel angular velocity (rad/s)
        w_fl = (d_fl / ticks_per_rev) * 2.0 * math.pi / dt
        w_fr = (d_fr / ticks_per_rev) * 2.0 * math.pi / dt
        w_bl = (d_bl / ticks_per_rev) * 2.0 * math.pi / dt
        w_br = (d_br / ticks_per_rev) * 2.0 * math.pi / dt

        wheel_radius = self.get_parameter('wheel_radius').value

        # Per-wheel measured distance (m) and surface speed (m/s)
        tick_to_m = 2.0 * math.pi * wheel_radius / ticks_per_rev
        st = JointState()
        st.header.stamp = now.to_msg()
        st.name = self.wheel_names
        st.position = [fl * tick_to_m, fr * tick_to_m, bl * tick_to_m, br * tick_to_m]
        st.velocity = [w_fl * wheel_radius, w_fr * wheel_radius,
                       w_bl * wheel_radius, w_br * wheel_radius]
        self.wheel_state_pub.publish(st)

        wheelbase = self.get_parameter('wheelbase').value
        wheel_separation = self.get_parameter('wheel_separation').value
        l = (wheelbase + wheel_separation) / 2.0

        # Forward kinematics: wheel speeds -> body velocity
        vx = wheel_radius * (w_fl + w_fr + w_bl + w_br) / 4.0
        vy = wheel_radius * (-w_fl + w_fr + w_bl - w_br) / 4.0
        wz = wheel_radius * (-w_fl + w_fr - w_bl + w_br) / (4.0 * l)

        # Integrate body velocity into world-frame pose (simple Euler dead reckoning)
        dx = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        dy = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        dtheta = wz * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        self.publish_odometry(vx, vy, wz, now)

    def process_control_line(self, line):
        """Parse 'P,pwm_fl,pwm_fr,pwm_bl,pwm_br,err_fl,err_fr,err_bl,err_br'."""
        parts = line.split(',')
        if len(parts) != 9:
            return  # malformed, drop it

        try:
            pwm = [float(int(p)) for p in parts[1:5]]
            err_ticks = [float(p) for p in parts[5:9]]
        except ValueError:
            return  # non-numeric junk, drop it

        stamp = self.get_clock().now().to_msg()

        # PD error: ticks per control period -> m/s
        wheel_radius = self.get_parameter('wheel_radius').value
        ticks_per_rev = self.get_parameter('ticks_per_rev').value
        control_period = self.get_parameter('control_period').value
        tick_to_ms = 2.0 * math.pi * wheel_radius / (ticks_per_rev * control_period)

        pwm_msg = JointState()
        pwm_msg.header.stamp = stamp
        pwm_msg.name = self.wheel_names
        pwm_msg.effort = pwm
        self.wheel_pwm_pub.publish(pwm_msg)

        err_msg = JointState()
        err_msg.header.stamp = stamp
        err_msg.name = self.wheel_names
        err_msg.velocity = [e * tick_to_ms for e in err_ticks]
        self.wheel_err_pub.publish(err_msg)

    def publish_odometry(self, vx, vy, wz, stamp):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # yaw -> quaternion (rotation about z only)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        odom.twist.covariance[0] = 0.01    # vx variance
        odom.twist.covariance[7] = 0.08    # vy variance (higher — mecanum roller slip)
        odom.twist.covariance[35] = 0.02   # wz variance

        self.odom_pub.publish(odom)

    def destroy_node(self):
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                self.send_ticks(0, 0, 0, 0)
            except Exception:
                pass
            self.serial_port.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialNode()
        rclpy.spin(node)
    except (serial.SerialException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
