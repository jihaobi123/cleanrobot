from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='stm32_interface').find('stm32_interface')
    
    # 声明启动参数
    communication_type_arg = DeclareLaunchArgument(
        'communication_type',
        default_value='serial',
        description='通信类型: serial 或 udp'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='串口设备路径'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'stm32_interface.yaml'),
        description='配置文件路径'
    )

    # 创建节点
    stm32_interface_node = Node(
        package='stm32_interface',
        executable='stm32_interface_node.py',
        name='stm32_interface_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'communication_type': LaunchConfiguration('communication_type'),
                'serial_port': LaunchConfiguration('serial_port'),
            }
        ]
    )

    return LaunchDescription([
        communication_type_arg,
        serial_port_arg,
        config_file_arg,
        stm32_interface_node,
    ])

