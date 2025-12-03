from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_config = PathJoinSubstitution(
        [FindPackageShare('lidar_driver'), 'config', 'lidar_driver.yaml']
    )

    slam_params = PathJoinSubstitution(
        [FindPackageShare('cleanrobot_slam'), 'config', 'slam_params.yaml']
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial device path for LD14 LiDAR'
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params,
        description='Slam Toolbox parameter file'
    )

    lidar_node = Node(
        package='lidar_driver',
        executable='lidar_driver_node',
        name='lidar_driver_node',
        output='screen',
        parameters=[
            lidar_config,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': 'base_laser',
            },
        ],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='online_async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')],
    )

    return LaunchDescription([
        serial_port_arg,
        slam_params_arg,
        lidar_node,
        slam_node,
    ])
