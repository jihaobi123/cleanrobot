from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('cleanrobot_bringup')
    ekf_params = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/rf2o/odom',
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )

    return LaunchDescription([
        static_tf_imu,
        rf2o_node,
        ekf_node
    ])
