"""robot_web_bridge 节点的启动文件。

本启动描述暴露话题名称与 FastAPI WebSocket 端口的参数，便于用户在不改
代码的情况下将桥接服务适配到现有 ROS 图。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """创建带默认参数的 LaunchDescription。"""

    # 可在启动命令行覆盖参数，例如：
    # ros2 launch robot_web_bridge web_bridge.launch.py websocket_port:=9000
    return LaunchDescription(
        [
            Node(
                package='robot_web_bridge',
                executable='web_bridge_node',
                name='web_bridge_node',
                output='screen',
                parameters=[
                    {'map_topic': '/map'},
                    {'cmd_vel_topic': '/cmd_vel'},
                    {'goal_topic': '/goal_pose'},
                    {'websocket_port': 8000},
                ],
            )
        ]
    )
