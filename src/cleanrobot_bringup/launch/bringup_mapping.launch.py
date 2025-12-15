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

    # cleanrobot_bringup 的 share 目录（只用来找 rviz 配置/launch 文件）
    bringup_share = get_package_share_directory('cleanrobot_bringup')
    rviz_config = os.path.join(bringup_share, 'rviz', 'bringup.rviz')

    # ========= 关键：参数文件统一从工作空间根目录 cleanrobot/config/ 读取 =========
    # 假设你的工作空间结构是：~/文档/cleanrobot/cleanrobot/config/xxx.yaml
    # 即 bringup_share = .../install/cleanrobot_bringup/share/cleanrobot_bringup
    # 向上回到 workspace 根目录再进入 config
    ws_root = os.path.abspath(os.path.join(bringup_share, '..', '..', '..', '..'))
    config_dir = os.path.join(ws_root, 'config')

    slam_params = os.path.join(config_dir, 'ld14_slam.yaml')   # 你说的 SLAM 参数文件
    ekf_params  = os.path.join(config_dir, 'ekf.yaml')
    rf2o_params = os.path.join(config_dir, 'rf2o.yaml')

    # LiDAR
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar'), 'launch', 'ld14p.launch.py')
        )
    )

    # IMU：禁用 RViz（避免一启动一堆窗口/冲突）
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('imu_ros2_device'), 'launch', 'ybimu_display.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'start_driver': 'true',
        }.items()
    )

    # base_link -> imu_link 静态 TF（如果你 IMU 安装位姿不是 0，再改这里的 xyz/rpy）
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    # RF2O（参数固定在 rf2o.yaml 里，里面记得包含 init_pose_from_topic: ""）
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='CLaserOdometry2DNode',
        output='screen',
        parameters=[rf2o_params],
    )

    # EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    # RViz（总 RViz，只开这一个）
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

