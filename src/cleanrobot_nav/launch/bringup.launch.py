import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # package shares
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    cleanrobot_nav_share = FindPackageShare('cleanrobot_nav')

    # ===== Launch arguments =====
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            cleanrobot_nav_share, 'maps', 'cleanrobot_map.yaml'
        ]),
        description='Full path to the map yaml file'
    )

    use_fake_base_arg = DeclareLaunchArgument(
        'use_fake_base',
        default_value='false',
        description='Whether to start fake_base (cmd_vel -> odom)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    # ===== Nav2 参数文件 =====
    params_file = PathJoinSubstitution([
        cleanrobot_nav_share,
        'config',
        'nav2_params.yaml'
    ])

    # ===== Nav2 navigation launch =====
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup_share,
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    # ===== Coverage waypoints node =====
    coverage_node = Node(
        package='cleanrobot_nav',
        executable='coverage_waypoints.py',
        name='coverage_waypoints',
        output='screen',
        parameters=[{
            'step_m': 0.25,
            'keepout_cells': 1
        }]
    )

    # ===== Fake robot (optional) =====
    fake_urdf_path = os.path.join(
        get_package_share_directory('cleanrobot_nav'),
        'config',
        'fake_robot.urdf'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(fake_urdf_path, 'r').read()
        }],
        condition=IfCondition(LaunchConfiguration('use_fake_base'))
    )

    fake_base_node = Node(
        package='cleanrobot_nav',
        executable='fake_base.py',
        name='fake_base',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_fake_base'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                cleanrobot_nav_share,
                'rviz',
                'bringup_fake.rviz'
            ])
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        map_arg,
        use_fake_base_arg,
        use_rviz_arg,

        nav2_launch,
        coverage_node,

        robot_state_publisher,
        fake_base_node,
        rviz_node,
    ])
