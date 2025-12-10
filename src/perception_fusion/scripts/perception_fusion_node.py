#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
感知融合节点
功能：融合激光雷达SLAM地图和视觉检测结果，计算脏污点在地图上的坐标，生成清洁任务
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point, TransformStamped, PoseStamped
from perception_fusion.msg import CleanTask, CleaningTaskList
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_geometry_msgs import do_transform_point
import numpy as np
from typing import List, Optional, Tuple
import math


class PerceptionFusionNode(Node):
    """感知融合节点"""
    
    def __init__(self):
        super().__init__('perception_fusion_node')
        
        # 声明参数
        # 每个参数都允许在launch文件或动态参数中配置，用于适配不同的
        # 机器人坐标系、话题名称以及任务生成策略。
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('detection_topic', '/dirty_spots')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('task_topic', '/cleaning_task')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('task_threshold', 0.5)  # 任务生成距离阈值(m)
        self.declare_parameter('max_tasks', 10)  # 最大任务数量
        
        # 获取参数
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        task_topic = self.get_parameter('task_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.task_threshold = self.get_parameter('task_threshold').get_parameter_value().double_value
        self.max_tasks = self.get_parameter('max_tasks').get_parameter_value().integer_value
        
        # 初始化TF监听器
        # TF Buffer负责缓存各个坐标系之间的变换，TransformListener持续监听
        # TF树并填充Buffer，后续的坐标转换都依赖于此。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 数据存储
        # 这些成员变量保存最新收到的传感器与检测数据，便于融合时直接取用。
        self.current_map: Optional[OccupancyGrid] = None
        self.current_odom: Optional[Odometry] = None
        self.current_detections: Optional[Detection2DArray] = None
        self.current_scan: Optional[LaserScan] = None
        self.robot_pose: Optional[PoseStamped] = None
        
        # 任务管理
        # task_list缓存尚未完成的清洁任务，task_id_counter用来给任务
        # 分配递增的唯一ID。
        self.task_list: List[CleanTask] = []
        self.task_id_counter = 0
        
        # 创建订阅者
        # 订阅地图、里程计、视觉检测结果和激光雷达数据，用于后续融合。
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, detection_topic, self.detection_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        
        # 创建发布者
        # 将生成的清洁任务列表通过task_topic发布给下游执行节点。
        self.task_publisher = self.create_publisher(CleaningTaskList, task_topic, 10)
        
        # 创建定时器，定期处理任务
        # 每秒执行一次process_tasks，用最新的检测结果生成或更新任务。
        self.timer = self.create_timer(1.0, self.process_tasks)
        
        self.get_logger().info('感知融合节点已启动')
        self.get_logger().info(f'订阅话题: {map_topic}, {odom_topic}, {detection_topic}')
        self.get_logger().info(f'发布话题: {task_topic}')
    
    def map_callback(self, msg: OccupancyGrid):
        """地图回调"""
        self.current_map = msg
        self.get_logger().debug('收到地图更新')
    
    def odom_callback(self, msg: Odometry):
        """里程计回调"""
        self.current_odom = msg
        # 更新机器人位姿
        # 通过TF获取map->base_link变换，将其转换为PoseStamped保存，便于
        # 后续把相机坐标系中的点转换到地图坐标系。
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            self.robot_pose = PoseStamped()
            self.robot_pose.header = transform.header
            self.robot_pose.pose.position.x = transform.transform.translation.x
            self.robot_pose.pose.position.y = transform.transform.translation.y
            self.robot_pose.pose.position.z = transform.transform.translation.z
            self.robot_pose.pose.orientation = transform.transform.rotation
        except TransformException as e:
            self.get_logger().warn(f'TF变换失败: {str(e)}')
    
    def detection_callback(self, msg: Detection2DArray):
        """检测结果回调"""
        self.current_detections = msg
        self.get_logger().debug(f'收到 {len(msg.detections)} 个检测结果')
    
    def scan_callback(self, msg: LaserScan):
        """激光扫描回调"""
        self.current_scan = msg
    
    def pixel_to_map_coordinate(self, pixel_x: float, pixel_y: float, 
                                image_width: int, image_height: int) -> Optional[Point]:
        """
        将图像像素坐标转换为地图坐标
        这里使用简化的相机投影模型，实际需要根据相机标定参数计算
        """
        if self.robot_pose is None or self.current_map is None:
            return None
        
        try:
            # 获取相机到地图的变换
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f'无法获取相机变换: {str(e)}')
            return None
        
        # 简化处理：假设相机视角为60度，检测区域在机器人前方一定距离。
        # 实际应用中需要使用相机内参和深度信息通过投影模型计算深度。
        camera_fov = math.radians(60.0)  # 相机视场角
        detection_distance = 1.0  # 假设检测距离1米
        
        # 计算像素到角度的映射
        normalized_x = (pixel_x - image_width / 2) / (image_width / 2)
        normalized_y = (pixel_y - image_height / 2) / (image_height / 2)
        
        # 计算相对于相机的角度
        angle_x = normalized_x * camera_fov / 2
        angle_y = normalized_y * camera_fov / 2
        
        # 计算3D点（相机坐标系）
        point_camera = Point()
        point_camera.z = detection_distance * math.cos(angle_y)
        point_camera.x = detection_distance * math.sin(angle_x) * math.cos(angle_y)
        point_camera.y = detection_distance * math.sin(angle_y)
        
        # 转换为地图坐标系
        point_map = do_transform_point(point_camera, transform)
        return point_map.point
    
    def create_cleaning_task(self, position: Point, cleaning_type: int = 0, 
                            priority: int = 128) -> CleanTask:
        """创建清洁任务"""
        task = CleanTask()
        task.task_id = self.task_id_counter
        self.task_id_counter += 1
        task.target_position = position
        task.cleaning_type = cleaning_type
        task.priority = priority
        task.status = 0  # 待执行
        task.timestamp = self.get_clock().now().to_msg()
        task.description = f"脏污清洁任务 #{task.task_id}"
        return task
    
    def process_tasks(self):
        """处理检测结果，生成清洁任务"""
        if self.current_detections is None or self.robot_pose is None:
            return
        
        # 处理新的检测结果
        # 每次定时器触发时遍历检测框，尝试为每个脏污检测生成任务。
        for detection in self.current_detections.detections:
            if len(detection.results) == 0:
                continue
            
            # 获取检测框中心
            # 仅使用中心像素来估计脏污点的方向与位置。
            bbox = detection.bbox
            pixel_x = bbox.center.position.x
            pixel_y = bbox.center.position.y
            
            # 获取图像尺寸
            if hasattr(detection, 'source_img') and detection.source_img:
                image_width = detection.source_img.width
                image_height = detection.source_img.height
            else:
                # 使用默认尺寸
                image_width = 640
                image_height = 480
            
            # 转换为地图坐标
            # 先将像素坐标转换到相机坐标系，再通过TF得到地图坐标。
            map_point = self.pixel_to_map_coordinate(
                pixel_x, pixel_y, image_width, image_height
            )
            
            if map_point is None:
                continue
            
            # 检查是否已存在相近任务
            # 若已有距离很近的任务则跳过，避免重复下发。
            task_exists = False
            for existing_task in self.task_list:
                dx = map_point.x - existing_task.target_position.x
                dy = map_point.y - existing_task.target_position.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < self.task_threshold:
                    task_exists = True
                    break
            
            # 如果不存在相近任务，创建新任务
            # 优先级根据检测置信度映射到0-255，超过阈值的使用深度清洁模式。
            if not task_exists and len(self.task_list) < self.max_tasks:
                # 根据检测置信度设置优先级
                confidence = detection.results[0].score
                priority = int(confidence * 255)
                cleaning_type = 1 if confidence > 0.8 else 0  # 高置信度使用深度清洁
                
                task = self.create_cleaning_task(map_point, cleaning_type, priority)
                self.task_list.append(task)
                self.get_logger().info(
                    f'创建新任务: ID={task.task_id}, '
                    f'位置=({map_point.x:.2f}, {map_point.y:.2f}), '
                    f'优先级={priority}'
                )
        
        # 清理已完成的任务
        # 状态>=2视为已完成或取消，从任务列表中移除。
        self.task_list = [task for task in self.task_list if task.status < 2]
        
        # 发布任务列表
        # 按列表形式发布当前待执行任务，供执行器订阅。
        if len(self.task_list) > 0:
            task_list_msg = CleaningTaskList()
            task_list_msg.tasks = self.task_list
            task_list_msg.timestamp = self.get_clock().now().to_msg()
            self.task_publisher.publish(task_list_msg)
            self.get_logger().info(f'发布 {len(self.task_list)} 个清洁任务')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

