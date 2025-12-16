from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stm32_interface',
            executable='cmd_vel_to_stm32',
            name='cmd_vel_to_stm32',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                'send_rate_hz': 20.0,
                'cmd_timeout_s': 0.3,
                'max_vx_m_s': 0.6,
                'max_wz_rad_s': 2.0,
                'topic_cmd_vel': '/cmd_vel'
            }]
        )
    ])

