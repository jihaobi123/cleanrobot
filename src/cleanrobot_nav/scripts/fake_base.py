#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    # z,w for planar yaw
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class FakeBase(Node):
    """
    Offline-validation fake base for Nav2:
    cmd_vel -> (deadband + lowpass + accel/vel limits) -> wheel model -> encoder quantization -> odom + TF

    NEW:
      - subscribe both /cmd_vel_nav and /cmd_vel (configurable)
      - prefer /cmd_vel_nav by default
      - cmd timeout -> auto stop if no command
      - node name changed to avoid duplicate name conflict
    """

    def __init__(self):
        super().__init__('fake_base_sim')

        # ===== Topics / frames =====
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('prefer_nav_cmd', True)
        self.declare_parameter('cmd_timeout', 0.5)  # seconds, no cmd -> stop

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 50.0)

        # Optional initial pose in odom frame
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        # ===== Deadband + lowpass =====
        self.declare_parameter('deadband_vx', 0.02)     # m/s
        self.declare_parameter('deadband_wz', 0.05)     # rad/s
        self.declare_parameter('cmd_alpha', 0.20)       # 0..1  (smaller => smoother)

        # ===== Diff-drive geometry =====
        self.declare_parameter('wheel_base', 0.26)      # meters
        self.declare_parameter('wheel_radius', 0.035)   # meters
        self.declare_parameter('ticks_per_rev', 2048)   # encoder ticks per wheel revolution

        # ===== Limits (simulate actuation / dynamics) =====
        self.declare_parameter('max_vx', 0.40)          # m/s
        self.declare_parameter('max_wz', 1.20)          # rad/s
        self.declare_parameter('max_lin_acc', 0.80)     # m/s^2
        self.declare_parameter('max_ang_acc', 2.00)     # rad/s^2

        # publish twist from (filtered/limited) command or from quantized motion
        self.declare_parameter('twist_from_quantized', True)

        # ===== Read params =====
        self.cmd_nav_topic = self.get_parameter('cmd_vel_nav_topic').value
        self.cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.prefer_nav_cmd = bool(self.get_parameter('prefer_nav_cmd').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.deadband_vx = float(self.get_parameter('deadband_vx').value)
        self.deadband_wz = float(self.get_parameter('deadband_wz').value)
        self.alpha = float(self.get_parameter('cmd_alpha').value)

        self.L = float(self.get_parameter('wheel_base').value)
        self.R = float(self.get_parameter('wheel_radius').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').value)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.max_lin_acc = float(self.get_parameter('max_lin_acc').value)
        self.max_ang_acc = float(self.get_parameter('max_ang_acc').value)

        self.twist_from_quantized = bool(self.get_parameter('twist_from_quantized').value)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)

        # ===== Derived encoder resolution =====
        self.m_per_tick = (2.0 * math.pi * self.R) / max(1, self.ticks_per_rev)

        # ===== Command state =====
        self.cmd_nav = Twist()
        self.cmd_raw = Twist()
        self.last_cmd_nav_time = None
        self.last_cmd_raw_time = None

        # filtered/limited command states
        self.vx_f = 0.0
        self.wz_f = 0.0
        self.vx_out = 0.0
        self.wz_out = 0.0

        # wheel speed states
        self.v_l = 0.0
        self.v_r = 0.0

        self.last_time = self.get_clock().now()

        # ===== ROS I/O =====
        self.sub_nav = self.create_subscription(Twist, self.cmd_nav_topic, self.on_cmd_nav, 10)
        self.sub_raw = self.create_subscription(Twist, self.cmd_topic, self.on_cmd_raw, 10)

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_br = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / max(1e-3, rate_hz), self.on_timer)

        self.get_logger().info(
            f'FakeBase started.\n'
            f'  Sub(nav): {self.cmd_nav_topic}\n'
            f'  Sub(raw): {self.cmd_topic}\n'
            f'  Prefer nav cmd: {self.prefer_nav_cmd}, cmd_timeout={self.cmd_timeout}s\n'
            f'  Pub: {odom_topic}\n'
            f'  TF:  {self.odom_frame}->{self.base_frame}\n'
            f'  init pose (odom): x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f}\n'
            f'  wheel_base={self.L:.3f}m wheel_radius={self.R:.3f}m ticks/rev={self.ticks_per_rev}\n'
            f'  deadband vx={self.deadband_vx} wz={self.deadband_wz} alpha={self.alpha}\n'
            f'  limits max_vx={self.max_vx} max_wz={self.max_wz} max_lin_acc={self.max_lin_acc} max_ang_acc={self.max_ang_acc}'
        )

    def on_cmd_nav(self, msg: Twist):
        self.cmd_nav = msg
        self.last_cmd_nav_time = self.get_clock().now()

    def on_cmd_raw(self, msg: Twist):
        self.cmd_raw = msg
        self.last_cmd_raw_time = self.get_clock().now()

    def _deadband(self, vx: float, wz: float):
        if abs(vx) < self.deadband_vx:
            vx = 0.0
        if abs(wz) < self.deadband_wz:
            wz = 0.0
        return vx, wz

    def _lowpass(self, vx: float, wz: float):
        a = clamp(self.alpha, 0.0, 1.0)
        self.vx_f = a * vx + (1.0 - a) * self.vx_f
        self.wz_f = a * wz + (1.0 - a) * self.wz_f
        return self.vx_f, self.wz_f

    def _limit_vel(self, vx: float, wz: float):
        vx = clamp(vx, -self.max_vx, self.max_vx)
        wz = clamp(wz, -self.max_wz, self.max_wz)
        return vx, wz

    def _limit_acc(self, vx_cmd: float, wz_cmd: float, dt: float):
        dv = clamp(vx_cmd - self.vx_out, -self.max_lin_acc * dt, self.max_lin_acc * dt)
        dw = clamp(wz_cmd - self.wz_out, -self.max_ang_acc * dt, self.max_ang_acc * dt)
        self.vx_out += dv
        self.wz_out += dw
        return self.vx_out, self.wz_out

    def _cmd_to_wheels(self, vx: float, wz: float):
        v_l = vx - wz * self.L * 0.5
        v_r = vx + wz * self.L * 0.5
        return v_l, v_r

    def _wheels_to_body(self, v_l: float, v_r: float):
        vx = 0.5 * (v_l + v_r)
        wz = (v_r - v_l) / max(1e-6, self.L)
        return vx, wz

    def _quantize_wheel_distance(self, d: float):
        ticks = int(round(d / self.m_per_tick))
        return ticks * self.m_per_tick

    def _select_cmd(self, now):
        """
        Prefer /cmd_vel_nav if enabled and recent; else fallback to /cmd_vel.
        Apply timeout: if both stale -> return zeros.
        """
        def is_recent(t):
            if t is None:
                return False
            age = (now - t).nanoseconds * 1e-9
            return age <= self.cmd_timeout

        nav_ok = is_recent(self.last_cmd_nav_time)
        raw_ok = is_recent(self.last_cmd_raw_time)

        if self.prefer_nav_cmd and nav_ok:
            return self.cmd_nav, 'nav'
        if raw_ok:
            return self.cmd_raw, 'raw'
        if nav_ok:
            return self.cmd_nav, 'nav'
        # stale -> stop
        z = Twist()
        return z, 'none'

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        # ===== 1) Select command =====
        cmd_msg, src = self._select_cmd(now)
        vx = float(cmd_msg.linear.x)
        wz = float(cmd_msg.angular.z)

        # ===== 2) Deadband + lowpass + limits =====
        vx, wz = self._deadband(vx, wz)
        vx, wz = self._lowpass(vx, wz)
        vx, wz = self._limit_vel(vx, wz)
        vx, wz = self._limit_acc(vx, wz, dt)

        # ===== 3) Diff-drive model =====
        v_l_cmd, v_r_cmd = self._cmd_to_wheels(vx, wz)

        dv_l = clamp(v_l_cmd - self.v_l, -self.max_lin_acc * dt, self.max_lin_acc * dt)
        dv_r = clamp(v_r_cmd - self.v_r, -self.max_lin_acc * dt, self.max_lin_acc * dt)
        self.v_l += dv_l
        self.v_r += dv_r

        # ===== 4) Integrate with encoder quantization =====
        d_l = self.v_l * dt
        d_r = self.v_r * dt

        d_l_q = self._quantize_wheel_distance(d_l)
        d_r_q = self._quantize_wheel_distance(d_r)

        ds = 0.5 * (d_l_q + d_r_q)
        dtheta = (d_r_q - d_l_q) / max(1e-6, self.L)

        yaw_mid = self.yaw + 0.5 * dtheta
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = wrap_pi(self.yaw + dtheta)

        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        # ===== 5) TF odom -> base_link =====
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_br.sendTransform(t)

        # ===== 6) Odometry =====
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        if self.twist_from_quantized:
            vx_q, wz_q = self._wheels_to_body(d_l_q / dt, d_r_q / dt)
            odom.twist.twist.linear.x = vx_q
            odom.twist.twist.angular.z = wz_q
        else:
            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # Optional: very light debug (comment out if noisy)
        # if src == 'none':
        #     self.get_logger().debug('No recent cmd, stopping.')


def main():
    rclpy.init()
    node = FakeBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
