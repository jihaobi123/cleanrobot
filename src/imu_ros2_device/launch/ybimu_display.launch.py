from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os


def generate_launch_description():

    # 这里改成 imu_ros2_device
    package_path = get_package_share_path('imu_ros2_device')
    default_rviz_config_path = package_path / 'rviz/ybimu.rviz'
    print("config path:", default_rviz_config_path)

    start_driver_arg = DeclareLaunchArgument(
        name='start_driver',
        default_value='true',
        description='Whether to start ybimu_driver node'
    )

    rviz_enable_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    device_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_driver',
        remappings=[
            ('/imu/data', '/imu/data_raw'),
        ],
        condition=IfCondition(LaunchConfiguration('start_driver')),
    )

    imu_filter_config = os.path.join(
        get_package_share_directory('imu_ros2_device'),
        'config',
        'imu_filter_param.yaml'
    )

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
