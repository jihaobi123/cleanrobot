from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='perception_fusion').find('perception_fusion')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'perception_fusion.yaml'),
        description='配置文件路径'
    )

    # 创建节点
    perception_fusion_node = Node(
        package='perception_fusion',
        executable='perception_fusion_node.py',
        name='perception_fusion_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        perception_fusion_node,
    ])

