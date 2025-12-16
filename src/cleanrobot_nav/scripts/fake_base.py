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


class FakeBase(Node):
    def __init__(self):
        super().__init__('fake_base')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 50.0)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.cmd = Twist()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.sub = self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_br = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(
            f'FakeBase started. Sub: {cmd_topic}  Pub: {odom_topic}  TF: {self.odom_frame}->{self.base_frame}')

    def on_cmd(self, msg: Twist):
        self.cmd = msg

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        # integrate in base frame (diff-drive planar)
        vx = float(self.cmd.linear.x)
        wz = float(self.cmd.angular.z)
        self.yaw += wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # wrap
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt

        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        # TF odom -> base_link
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

        # Odometry
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
        odom.twist.twist = self.cmd
        self.odom_pub.publish(odom)


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


