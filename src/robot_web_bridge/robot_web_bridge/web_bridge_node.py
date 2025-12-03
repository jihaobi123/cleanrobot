"""通过 FastAPI 和 WebSocket 暴露 ROS2 机器人状态的节点。

本模块将标准的 ROS 2 节点与 FastAPI 应用组合在一起，方便前端网页在
无需 ROS 工具的情况下获取机器人遥测数据。类和方法的注释重点说明
ROS 回调与 HTTP/WebSocket 接口之间的数据流，便于维护者快速理清逻辑。
"""
from __future__ import annotations

import asyncio
import json
import math
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn


@dataclass
class MapCache:
    """存储最新的地图数据。

    地图由 `/map` 订阅回调填充，缓存最新消息便于 HTTP 处理函数直接
    返回数据，而无需等待新的 ROS 消息。
    """

    width: int = 0
    height: int = 0
    resolution: float = 0.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    data: List[int] = field(default_factory=list)

    @property
    def has_map(self) -> bool:
        """返回是否已有有效地图数据。"""
        return self.width > 0 and self.height > 0 and len(self.data) > 0


@dataclass
class PoseState:
    """存储地图坐标系下的最新机器人位姿。

    位姿由 TF 变换更新，始终反映 `map` 坐标系下的实时位置与朝向。
    同时保存时间戳，便于客户端判断数据新鲜度。
    """

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    stamp: Optional[Header] = None

    @property
    def has_pose(self) -> bool:
        """返回位姿是否存在有效时间戳。"""
        return self.stamp is not None

class WebBridgeNode(Node):
    """使用 FastAPI 将 ROS2 话题桥接到 HTTP/WebSocket。

    节点负责 ROS 话题的订阅与发布，同时全局 FastAPI 应用通过该实例
    读取或写入共享状态。使用线程锁保护 ROS 回调线程与 asyncio Web
    服务器之间的共享数据。
    """

    def __init__(self) -> None:
        super().__init__('web_bridge_node')
        # 声明参数，便于 launch 或命令行覆盖话题与端口设置。
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('websocket_port', 8000)

        self.map_topic: str = self.get_parameter('map_topic').get_parameter_value().string_value
        self.cmd_vel_topic: str = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.goal_topic: str = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.websocket_port: int = (
            self.get_parameter('websocket_port').get_parameter_value().integer_value
        )

        # 与 FastAPI 处理函数共享的本地缓存。
        self.map_cache: MapCache = MapCache()
        self.pose_state: PoseState = PoseState()
        # 保护 ROS 回调与 FastAPI 协程同时访问的共享数据。
        self._lock = threading.Lock()

        # TF 缓存与监听器，用于查询 map → base_link 的变换。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS 接口：订阅地图，发布速度与目标。
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        # 定时器周期刷新 TF 位姿（约 10 Hz）。
        self.pose_timer = self.create_timer(0.1, self._update_pose)

    # ---------------- Map and pose handling -----------------
    def _map_callback(self, msg: OccupancyGrid) -> None:
        """收到新地图时更新缓存。"""
        with self._lock:
            self.map_cache.width = msg.info.width
            self.map_cache.height = msg.info.height
            self.map_cache.resolution = msg.info.resolution
            self.map_cache.origin_x = msg.info.origin.position.x
            self.map_cache.origin_y = msg.info.origin.position.y
            self.map_cache.data = list(msg.data)
        self.get_logger().debug('Map updated: %dx%d', msg.info.width, msg.info.height)

    def _update_pose(self) -> None:
        """基于 TF 数据刷新机器人位姿。"""
        # 当 TF 尚未可用时查询会失败，此时静默跳过避免大量警告。
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().debug('Transform lookup failed: %s', ex)
            return

        trans = transform.transform.translation
        rot = transform.transform.rotation
        # 将四元数转换为偏航角（假设平面机器人，忽略滚转与俯仰）。
        yaw = math.atan2(2.0 * (rot.w * rot.z + rot.x * rot.y), 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z))

        # 持锁复制位姿到共享缓存。
        with self._lock:
            self.pose_state.x = trans.x
            self.pose_state.y = trans.y
            self.pose_state.yaw = yaw
            header = Header()
            header.stamp = transform.header.stamp
            header.frame_id = transform.header.frame_id
            self.pose_state.stamp = header

    # ---------------- API helpers -----------------
    def get_map_payload(self) -> Dict[str, Any]:
        """获取可序列化的地图快照。"""
        # 持锁拷贝数据，避免 API 处理函数拿到可变引用；即使请求处理中有
        # 新的 ROS 消息到来也能保证响应一致。
        with self._lock:
            cache = MapCache(
                width=self.map_cache.width,
                height=self.map_cache.height,
                resolution=self.map_cache.resolution,
                origin_x=self.map_cache.origin_x,
                origin_y=self.map_cache.origin_y,
                data=list(self.map_cache.data),
            )
        return {
            'has_map': cache.has_map,
            'width': cache.width,
            'height': cache.height,
            'resolution': cache.resolution,
            'origin': {'x': cache.origin_x, 'y': cache.origin_y},
            'data': cache.data,
        }

    def get_pose_payload(self) -> Dict[str, Any]:
        """获取可序列化的位姿快照。"""
        # 同一把锁下同时读取位姿与地图有效性，保证一致性。
        with self._lock:
            pose = PoseState(
                x=self.pose_state.x,
                y=self.pose_state.y,
                yaw=self.pose_state.yaw,
                stamp=self.pose_state.stamp,
            )
            has_map = self.map_cache.has_map

        stamp = pose.stamp
        stamp_iso = ''
        if stamp is not None:
            stamp_iso = f"{stamp.stamp.sec}.{stamp.stamp.nanosec:09d}"
        return {
            'x': pose.x,
            'y': pose.y,
            'yaw': pose.yaw,
            'stamp': stamp_iso,
            'has_pose': pose.has_pose,
            'has_map': has_map,
        }

    def publish_cmd_vel(self, vx: float, vy: float, omega: float) -> None:
        """发布 Twist 消息来控制速度。"""
        # 仅使用平面线速度与偏航角速度构造 Twist。
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        self.cmd_pub.publish(msg)
        self.get_logger().info('Published cmd_vel: vx=%.2f vy=%.2f omega=%.2f', vx, vy, omega)

    def publish_goal(self, x: float, y: float, yaw: float) -> None:
        """在 map 坐标系下发布 PoseStamped 目标。"""
        # 在 map 坐标系发布目标点，方向由偏航角转换而来。
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        # 将偏航角转为四元数（仅 z-w 分量）。
        half_yaw = yaw / 2.0
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self.goal_pub.publish(msg)
        self.get_logger().info('Published goal: x=%.2f y=%.2f yaw=%.2f', x, y, yaw)


app = FastAPI(title='Robot Web Bridge')
_bridge_node: Optional[WebBridgeNode] = None


@app.get('/api/map')
async def get_map() -> Dict[str, Any]:
    """以 JSON 返回最新地图。"""
    if _bridge_node is None:
        return {}
    return _bridge_node.get_map_payload()


@app.get('/api/state')
async def get_state() -> Dict[str, Any]:
    """返回最新的机器人位姿/状态。"""
    if _bridge_node is None:
        return {}
    return _bridge_node.get_pose_payload()


@app.websocket('/ws')
async def websocket_endpoint(websocket: WebSocket) -> None:
    """用于推送位姿并接收控制指令的 WebSocket 端点。"""
    # 进入收发循环前先接受客户端连接。
    await websocket.accept()
    if _bridge_node is None:
        await websocket.close(code=1000)
        return

    try:
        # 按固定频率推送最新位姿，同时监听浏览器发来的控制指令。
        while True:
            pose_payload = {
                'type': 'pose',
                **_bridge_node.get_pose_payload(),
            }
            await websocket.send_json(pose_payload)
            # 使用超时非阻塞方式尝试接收消息，不影响位姿推送。
            try:
                message = await asyncio.wait_for(websocket.receive_text(), timeout=0.1)
            except asyncio.TimeoutError:
                continue

            # 尝试解析 JSON，并做基础校验防止格式错误。
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                _bridge_node.get_logger().warning('Invalid JSON received on websocket')
                continue

            if not isinstance(data, dict) or 'type' not in data:
                _bridge_node.get_logger().warning('Websocket message missing type field')
                continue

            if data['type'] == 'cmd_vel':
                try:
                    vx = float(data.get('vx', 0.0))
                    vy = float(data.get('vy', 0.0))
                    omega = float(data.get('omega', 0.0))
                except (TypeError, ValueError):
                    _bridge_node.get_logger().warning('Invalid cmd_vel payload')
                    continue
                _bridge_node.publish_cmd_vel(vx, vy, omega)
            elif data['type'] == 'set_goal':
                try:
                    x = float(data.get('x'))
                    y = float(data.get('y'))
                    yaw = float(data.get('yaw'))
                except (TypeError, ValueError):
                    _bridge_node.get_logger().warning('Invalid set_goal payload')
                    continue
                _bridge_node.publish_goal(x, y, yaw)
            else:
                _bridge_node.get_logger().warning('Unknown websocket message type: %s', data['type'])
    except WebSocketDisconnect:
        _bridge_node.get_logger().info('Websocket client disconnected')


def main() -> None:
    """Web 桥节点与 FastAPI 服务器的入口函数。"""
    global _bridge_node
    rclpy.init()
    _bridge_node = WebBridgeNode()

    # 在后台线程运行 ROS spin，主线程运行 FastAPI。
    ros_thread = threading.Thread(target=rclpy.spin, args=(_bridge_node,), daemon=True)
    ros_thread.start()

    try:
        uvicorn.run(app, host='0.0.0.0', port=_bridge_node.websocket_port)
    except KeyboardInterrupt:
        pass
    finally:
        if _bridge_node is not None:
            _bridge_node.destroy_node()
            _bridge_node = None
        rclpy.shutdown()


if __name__ == '__main__':
    main()
