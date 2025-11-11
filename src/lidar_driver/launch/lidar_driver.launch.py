from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='lidar_driver').find('lidar_driver')
    
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'lidar_driver.yaml'),
        description='配置文件路径'
    )

    # 创建节点
    lidar_driver_node = Node(
        package='lidar_driver',
        executable='lidar_driver_node',
        name='lidar_driver_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
            }
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        config_file_arg,
        lidar_driver_node,
    ])

