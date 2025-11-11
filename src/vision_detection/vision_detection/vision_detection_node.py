#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视觉检测节点
功能：使用USB摄像头采集图像，通过YOLOv8 RKNN模型在NPU上推理，识别脏污区域
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, List, Tuple
import os

# RKNN相关导入（需要在RK3588平台上安装rknn-toolkit-lite）
try:
    from rknnlite.api import RKNNLite
    RKNN_AVAILABLE = True
except ImportError:
    RKNN_AVAILABLE = False
    print("警告: rknn-toolkit-lite未安装，将使用模拟模式")


class YOLOv8RKNN:
    """YOLOv8 RKNN模型推理类"""
    
    def __init__(self, model_path: str, input_size: Tuple[int, int] = (640, 640)):
        self.model_path = model_path
        self.input_size = input_size
        self.rknn = None
        
        if RKNN_AVAILABLE:
            self.init_rknn()
        else:
            print("使用模拟模式，实际部署时需要安装rknn-toolkit-lite")
    
    def init_rknn(self):
        """初始化RKNN模型"""
        if not RKNN_AVAILABLE:
            return
            
        try:
            self.rknn = RKNNLite(verbose=False)
            ret = self.rknn.load_rknn(self.model_path)
            if ret != 0:
                raise Exception(f"加载RKNN模型失败: {ret}")
            
            ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
            if ret != 0:
                raise Exception(f"初始化RKNN运行时失败: {ret}")
            
            print(f"RKNN模型加载成功: {self.model_path}")
        except Exception as e:
            print(f"RKNN初始化错误: {str(e)}")
            self.rknn = None
    
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """图像预处理"""
        # 调整大小
        img_resized = cv2.resize(image, self.input_size)
        # 转换为RGB
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        # 归一化
        img_normalized = img_rgb.astype(np.float32) / 255.0
        # 调整维度 (H, W, C) -> (1, C, H, W)
        img_transposed = np.transpose(img_normalized, (2, 0, 1))
        img_batch = np.expand_dims(img_transposed, axis=0)
        return img_batch
    
    def postprocess(self, outputs: np.ndarray, conf_threshold: float = 0.5) -> List[dict]:
        """后处理，解析检测结果"""
        # YOLOv8输出格式: (1, 84, 8400) 其中84 = 4(bbox) + 80(classes)
        # 这里简化处理，实际需要根据模型输出调整
        detections = []
        
        if outputs is None or len(outputs) == 0:
            return detections
        
        # 转置输出: (1, 84, 8400) -> (8400, 84)
        output = outputs[0].transpose((1, 0))
        
        # 解析每个检测框
        for detection in output:
            # 前4个值是bbox坐标 (cx, cy, w, h)
            cx, cy, w, h = detection[:4]
            # 后面的值是类别概率
            scores = detection[4:]
            max_score_idx = np.argmax(scores)
            max_score = scores[max_score_idx]
            
            if max_score > conf_threshold:
                # 转换为左上角坐标和宽高
                x1 = cx - w / 2
                y1 = cy - h / 2
                
                detections.append({
                    'bbox': [x1, y1, w, h],
                    'score': float(max_score),
                    'class_id': int(max_score_idx),
                    'class_name': 'dirty_spot'  # 可以根据class_id映射到具体类别
                })
        
        return detections
    
    def infer(self, image: np.ndarray, conf_threshold: float = 0.5) -> List[dict]:
        """执行推理"""
        if self.rknn is None:
            # 模拟模式，返回空结果
            return []
        
        try:
            # 预处理
            input_data = self.preprocess(image)
            
            # 推理
            outputs = self.rknn.inference(inputs=[input_data])
            
            # 后处理
            detections = self.postprocess(outputs[0], conf_threshold)
            return detections
        except Exception as e:
            print(f"推理错误: {str(e)}")
            return []


class VisionDetectionNode(Node):
    """视觉检测节点"""
    
    def __init__(self):
        super().__init__('vision_detection_node')
        
        # 声明参数
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('topic_name', '/dirty_spots')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('fps', 30.0)
        
        # 获取参数
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        input_height = self.get_parameter('input_height').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.cap: Optional[cv2.VideoCapture] = None
        self.init_camera(camera_index)
        
        # 初始化RKNN模型
        self.model: Optional[YOLOv8RKNN] = None
        if model_path and os.path.exists(model_path):
            self.model = YOLOv8RKNN(model_path, (input_width, input_height))
        else:
            self.get_logger().warn(f'模型文件不存在: {model_path}')
        
        # 创建发布者
        self.detection_publisher = self.create_publisher(Detection2DArray, topic_name, 10)
        self.image_publisher = self.create_publisher(Image, self.image_topic, 10)
        
        # 创建定时器
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.process_frame)
        
        self.get_logger().info('视觉检测节点已启动')
        self.get_logger().info(f'摄像头索引: {camera_index}, 模型路径: {model_path}')
        self.get_logger().info(f'发布话题: {topic_name}')
    
    def init_camera(self, camera_index: int):
        """初始化摄像头"""
        try:
            self.cap = cv2.VideoCapture(camera_index)
            if not self.cap.isOpened():
                raise Exception(f'无法打开摄像头 {camera_index}')
            
            # 设置摄像头参数
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            self.get_logger().info(f'摄像头 {camera_index} 初始化成功')
        except Exception as e:
            self.get_logger().error(f'摄像头初始化失败: {str(e)}')
            self.cap = None
    
    def process_frame(self):
        """处理一帧图像"""
        if self.cap is None or not self.cap.isOpened():
            return
        
        # 读取帧
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('无法读取摄像头帧')
            return
        
        # 发布原始图像
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self.frame_id
            self.image_publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'发布图像失败: {str(e)}')
        
        # 执行检测
        if self.model is not None:
            detections = self.model.infer(frame, self.conf_threshold)
            self.publish_detections(detections, frame.shape)
    
    def publish_detections(self, detections: List[dict], image_shape: Tuple[int, int, int]):
        """发布检测结果"""
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.frame_id
        
        height, width = image_shape[:2]
        input_width, input_height = self.model.input_size if self.model else (640, 640)
        scale_x = width / input_width
        scale_y = height / input_height
        
        for det in detections:
            detection = Detection2D()
            
            # 设置边界框
            bbox = BoundingBox2D()
            x1, y1, w, h = det['bbox']
            # 缩放回原图尺寸
            x1 *= scale_x
            y1 *= scale_y
            w *= scale_x
            h *= scale_y
            
            bbox.center.position.x = x1 + w / 2
            bbox.center.position.y = y1 + h / 2
            bbox.center.theta = 0.0
            bbox.size_x = w
            bbox.size_y = h
            detection.bbox = bbox
            
            # 设置检测结果
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(det['class_id'])
            hypothesis.score = det['score']
            detection.results = [hypothesis]
            
            # 设置源图像尺寸
            detection.source_img = Image()
            detection.source_img.width = width
            detection.source_img.height = height
            
            detection_array.detections.append(detection)
        
        # 发布检测结果
        self.detection_publisher.publish(detection_array)
        
        if len(detections) > 0:
            self.get_logger().info(f'检测到 {len(detections)} 个脏污区域')
    
    def destroy_node(self):
        """清理资源"""
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info('摄像头已关闭')
        if self.model is not None and self.model.rknn is not None:
            self.model.rknn.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

