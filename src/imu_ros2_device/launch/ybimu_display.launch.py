from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os


def generate_launch_description():

    # 包路径
    package_path = get_package_share_path('imu_ros2_device')
    default_rviz_config_path = package_path / 'rviz/ybimu.rviz'
    print("config path:", default_rviz_config_path)

    # 启动参数：是否启动驱动
    start_driver_arg = DeclareLaunchArgument(
        name='start_driver',
        default_value='true',
        description='Whether to start ybimu_driver node'
    )

    # 启动参数：是否启动 RViz
    rviz_enable_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='false',
        description='Whether to launch RViz2'
    )

    # RViz 配置参数
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ✅ IMU 驱动节点：不做任何 remap（驱动自己发布 /imu/data_raw）
    device_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_driver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_driver')),
    )

    # madgwick 配置文件
    imu_filter_config = os.path.join(
        get_package_share_directory('imu_ros2_device'),
        'config',
        'imu_filter_param.yaml'
    )

    # IMU 滤波节点：订阅 /imu/data_raw，输出 /imu/data（由 yaml 决定）
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        parameters=[imu_filter_config],
        output='screen',
    )

    return LaunchDescription([
        start_driver_arg,
        rviz_enable_arg,
        rviz_arg,
        device_node,
        rviz_node,
        imu_filter_node,
    ])
