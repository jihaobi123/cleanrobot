from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os


def generate_launch_description():

    # 这里改成 imu_ros2_device
    package_path = get_package_share_path('imu_ros2_device')
    default_rviz_config_path = package_path / 'rviz/ybimu.rviz'
    print("config path:", default_rviz_config_path)

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
    )

    # 🔽🔽🔽 先不要在 launch 里启动驱动节点，避免重复发布 /imu/data_raw
    # device_node = Node(
    #     package='imu_ros2_device',
    #     executable='ybimu_driver',
    #     name='ybimu_driver',
    #     remappings=[
    #         ('/imu/data', '/imu/data_raw'),
    #     ],
    # )

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
        rviz_arg,
        rviz_node,
        # device_node,  # 🔽🔽🔽 注释掉，不再由 launch 启动驱动
        imu_filter_node,
    ])
