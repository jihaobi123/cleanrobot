from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    device_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_driver',
        remappings=[
            ('/imu/data', '/imu/data_raw'),
        ],
        output='screen',
    )

    return LaunchDescription([
        device_node,
    ])
