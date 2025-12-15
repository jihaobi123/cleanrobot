from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration('rviz')

    # ================= 路径 =================
    bringup_share = get_package_share_directory('cleanrobot_bringup')
    rviz_config = os.path.join(bringup_share, 'rviz', 'bringup.rviz')

    ws_root = os.path.abspath(os.path.join(bringup_share, '..', '..', '..', '..'))
    config_dir = os.path.join(ws_root, 'config')

    slam_params = os.path.join(config_dir, 'ld14_slam.yaml')
    ekf_params  = os.path.join(config_dir, 'ekf.yaml')
    rf2o_params = os.path.join(config_dir, 'rf2o.yaml')

    # ================= LiDAR =================
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ldlidar'),
                'launch',
                'ld14p.launch.py'
            )
        )
    )

    # ================= IMU =================
    imu_driver = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_driver',
        output='screen'
    )

    imu_pkg_share = get_package_share_directory('imu_ros2_device')
    imu_filter_param = os.path.join(
        imu_pkg_share, 'config', 'imu_filter_param.yaml'
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[
            imu_filter_param,
            {'publish_tf': False}
        ]
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    # ================= RF2O =================
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='CLaserOdometry2DNode',
        output='screen',
        parameters=[rf2o_params],
    )

    # ================= EKF =================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )

    # ================= SLAM =================
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    # ================= RViz =================
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ================= TF 调试（延迟启动） =================
    tf_echo_map_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', 'odom'],
        output='screen'
    )

    tf_echo_odom_base = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'tf2_echo', 'odom', 'base_link'],
        output='screen'
    )

    tf_echo_base_laser = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'tf2_echo', 'base_link', 'base_laser'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),

        # sensors
        ldlidar_launch,
        imu_driver,
        imu_filter,

        # static TF
        static_tf_imu,

        # odom / fusion / slam
        rf2o_node,
        ekf_node,
        slam_node,

        # viz
        rviz_node,

        # ===== 延迟 5 秒启动 TF 监视 =====
        TimerAction(period=5.0, actions=[tf_echo_map_odom]),
        TimerAction(period=5.0, actions=[tf_echo_odom_base]),
        TimerAction(period=5.0, actions=[tf_echo_base_laser]),
    ])
