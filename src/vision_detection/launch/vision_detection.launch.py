from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='vision_detection').find('vision_detection')
    
    # 声明启动参数
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='摄像头设备索引'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_share, 'models', 'yolov8_dirty.rknn'),
        description='RKNN模型文件路径'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'vision_detection.yaml'),
        description='配置文件路径'
    )

    # 创建节点
    vision_detection_node = Node(
        package='vision_detection',
        executable='vision_detection_node',
        name='vision_detection_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'model_path': LaunchConfiguration('model_path'),
            }
        ]
    )

    return LaunchDescription([
        camera_index_arg,
        model_path_arg,
        config_file_arg,
        vision_detection_node,
    ])

