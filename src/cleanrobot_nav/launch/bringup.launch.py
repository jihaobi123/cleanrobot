from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    cleanrobot_nav_share = FindPackageShare('cleanrobot_nav')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to the map yaml file'
    )

    params_file = PathJoinSubstitution([
        cleanrobot_nav_share,
        'config',
        'nav2_params.yaml'
    ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup_share,
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'map': LaunchConfiguration('map')
        }.items()
    )

    return LaunchDescription([
        map_arg,
        nav2_launch
    ])
