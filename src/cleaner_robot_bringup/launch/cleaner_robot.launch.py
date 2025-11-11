from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径
    lidar_pkg = FindPackageShare(package='lidar_driver').find('lidar_driver')
    vision_pkg = FindPackageShare(package='vision_detection').find('vision_detection')
    fusion_pkg = FindPackageShare(package='perception_fusion').find('perception_fusion')
    stm32_pkg = FindPackageShare(package='stm32_interface').find('stm32_interface')
    
    # 声明启动参数
    serial_port_lidar_arg = DeclareLaunchArgument(
        'serial_port_lidar',
        default_value='/dev/ttyUSB0',
        description='激光雷达串口设备路径'
    )
    
    serial_port_stm32_arg = DeclareLaunchArgument(
        'serial_port_stm32',
        default_value='/dev/ttyUSB1',
        description='STM32串口设备路径'
    )
    
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='摄像头设备索引'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='RKNN模型文件路径'
    )
    
    communication_type_arg = DeclareLaunchArgument(
        'communication_type',
        default_value='serial',
        description='STM32通信类型: serial 或 udp'
    )
    
    start_lidar_arg = DeclareLaunchArgument(
        'start_lidar',
        default_value='true',
        description='是否启动激光雷达节点'
    )
    
    start_vision_arg = DeclareLaunchArgument(
        'start_vision',
        default_value='true',
        description='是否启动视觉检测节点'
    )
    
    start_fusion_arg = DeclareLaunchArgument(
        'start_fusion',
        default_value='true',
        description='是否启动感知融合节点'
    )
    
    start_stm32_arg = DeclareLaunchArgument(
        'start_stm32',
        default_value='true',
        description='是否启动STM32接口节点'
    )

    # 激光雷达节点
    lidar_driver_node = Node(
        package='lidar_driver',
        executable='lidar_driver_node',
        name='lidar_driver_node',
        output='screen',
        parameters=[
            os.path.join(lidar_pkg, 'config', 'lidar_driver.yaml'),
            {
                'serial_port': LaunchConfiguration('serial_port_lidar'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('start_lidar'))
    )
    
    # 视觉检测节点
    vision_detection_node = Node(
        package='vision_detection',
        executable='vision_detection_node',
        name='vision_detection_node',
        output='screen',
        parameters=[
            os.path.join(vision_pkg, 'config', 'vision_detection.yaml'),
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'model_path': LaunchConfiguration('model_path'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('start_vision'))
    )
    
    # 感知融合节点
    perception_fusion_node = Node(
        package='perception_fusion',
        executable='perception_fusion_node.py',
        name='perception_fusion_node',
        output='screen',
        parameters=[
            os.path.join(fusion_pkg, 'config', 'perception_fusion.yaml'),
        ],
        condition=IfCondition(LaunchConfiguration('start_fusion'))
    )
    
    # STM32接口节点
    stm32_interface_node = Node(
        package='stm32_interface',
        executable='stm32_interface_node.py',
        name='stm32_interface_node',
        output='screen',
        parameters=[
            os.path.join(stm32_pkg, 'config', 'stm32_interface.yaml'),
            {
                'communication_type': LaunchConfiguration('communication_type'),
                'serial_port': LaunchConfiguration('serial_port_stm32'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('start_stm32'))
    )

    return LaunchDescription([
        serial_port_lidar_arg,
        serial_port_stm32_arg,
        camera_index_arg,
        model_path_arg,
        communication_type_arg,
        start_lidar_arg,
        start_vision_arg,
        start_fusion_arg,
        start_stm32_arg,
        lidar_driver_node,
        vision_detection_node,
        perception_fusion_node,
        stm32_interface_node,
    ])

