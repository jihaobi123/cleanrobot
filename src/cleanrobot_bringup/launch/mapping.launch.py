from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration('rviz')

    bringup_dir = get_package_share_directory('cleanrobot_bringup')
    slam_params = os.path.join(bringup_dir, 'config', 'slam.yaml')
    ekf_params = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'bringup.rviz')

    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar'), 'launch', 'ld14p.launch.py')
        )
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('imu_ros2_device'), 'launch', 'ybimu_display.launch.py')
        )
    )

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
            'publish_tf': False,
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        ldlidar_launch,
        imu_launch,
        static_tf_imu,
        rf2o_node,
        ekf_node,
        slam_node,
        rviz_node,
    ])
