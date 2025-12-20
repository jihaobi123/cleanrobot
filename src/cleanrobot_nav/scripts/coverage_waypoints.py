#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CoverageWaypoints(Node):
    def __init__(self):
        super().__init__('coverage_waypoints')

        # ===== 参数 =====
        self.declare_parameter('step_m', 0.5)          # 覆盖间距（米）
        self.declare_parameter('keepout_cells', 2)     # 离障碍最少格数
        self.declare_parameter('frame_id', 'map')

        self.step_m = self.get_parameter('step_m').value
        self.keepout = self.get_parameter('keepout_cells').value
        self.frame_id = self.get_parameter('frame_id').value

        # ✅ 使用 transient_local QoS 匹配 map_server 的 latched 地图
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos)

        self.client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses')

        self.map = None
        self.sent = False
        self.poses = None

        # 定时器：等 Nav2 ready 后发送目标
        self.timer = self.create_timer(1.0, self.try_send)

        self.get_logger().info(
            'CoverageWaypoints node started, waiting for map...')

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
        if self.poses is None:
            self.poses = self.generate_waypoints(msg)
            self.get_logger().info(f'Generated {len(self.poses)} coverage waypoints (cached).')

    def generate_waypoints(self, msg: OccupancyGrid):
        info = msg.info
        data = msg.data

        w, h = info.width, info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        step_cells = max(1, int(self.step_m / res))
        k = self.keepout

        poses = []
        direction = 1
        y = k

        while y < h - k:
            xs = range(k, w - k, step_cells)
            if direction < 0:
                xs = reversed(list(xs))

            row_has_point = False
            for x in xs:
                if self.is_safe(data, x, y, w, h, k):
                    mx = ox + (x + 0.5) * res
                    my = oy + (y + 0.5) * res

                    pose = PoseStamped()
                    pose.header.frame_id = self.frame_id
                    pose.pose.position.x = mx
                    pose.pose.position.y = my
                    pose.pose.orientation.w = 1.0

                    poses.append(pose)
                    row_has_point = True

            if row_has_point:
                direction *= -1
            y += step_cells

        # ✅ return 一定要在函数里面
        return poses

    def is_safe(self, data, x, y, w, h, k):
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    return False
                if data[ny * w + nx] != 0:
                    return False
        return True

    def try_send(self):
        if self.sent:
            return
        if self.map is None or self.poses is None or len(self.poses) == 0:
            return

        # 等 action server ready
        if not self.client.wait_for_server(timeout_sec=0.1):
            self.get_logger().debug('Waiting for /navigate_through_poses action server...')
            return

        self.get_logger().info('Nav2 ready, sending coverage goal...')
        self.send_goal(self.poses)
        self.sent = True

    def send_goal(self, poses):
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Coverage goal rejected, will retry.')
            self.sent = False
            return

        self.get_logger().info('Coverage goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Coverage finished with status={status}')
        if status != 4:  # 4=SUCCEEDED（Humble里一般这样）
            self.get_logger().warn('Coverage not succeeded, allow retry.')
            self.sent = False


def main():
    rclpy.init()
    node = CoverageWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


