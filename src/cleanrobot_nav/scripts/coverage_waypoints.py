#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
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

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses')

        self.map = None
        self.sent = False

        self.get_logger().info(
            'CoverageWaypoints node started, waiting for map...')

    def map_callback(self, msg: OccupancyGrid):
        if self.sent:
            return

        self.map = msg
        poses = self.generate_waypoints(msg)

        if not poses:
            self.get_logger().error('No valid waypoints generated.')
            return

        self.get_logger().info(f'Generated {len(poses)} coverage waypoints')
        self.send_goal(poses)
        self.sent = True

    def generate_waypoints(self, msg: OccupancyGrid):
        info = msg.info
        data = msg.data

        w, h = info.width, info.height
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

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
                    q = yaw_to_quat(0.0 if direction > 0 else math.pi)
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]

                    poses.append(pose)
                    row_has_point = True

            if row_has_point:
                direction *= -1
            y += step_cells

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

    def send_goal(self, poses):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'navigate_through_poses action not available')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        self.get_logger().info('Sending coverage path to Nav2...')
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = CoverageWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


