# vision_detection - 视觉检测包

## 功能说明
本包使用USB摄像头采集图像，通过YOLOv8 RKNN模型在RK3588 NPU上执行推理，识别脏污区域并发布检测结果。

## 依赖
- rclpy
- sensor_msgs
- vision_msgs
- cv_bridge
- opencv-python
- rknn-toolkit-lite (RK3588平台)
- numpy

## 安装依赖
```bash
pip3 install opencv-python numpy
# 在RK3588平台上安装rknn-toolkit-lite
pip3 install rknn-toolkit-lite
```

## 使用方法

### 1. 准备RKNN模型
将训练好的YOLOv8 RKNN模型文件放在 `models/` 目录下。

### 2. 构建包
```bash
cd cleaner_robot_ws
colcon build --packages-select vision_detection
source install/setup.bash
```

### 3. 启动节点
```bash
ros2 launch vision_detection vision_detection.launch.py
```

### 4. 自定义参数启动
```bash
ros2 launch vision_detection vision_detection.launch.py camera_index:=0 model_path:=/path/to/model.rknn
```

### 5. 查看发布的话题
```bash
ros2 topic echo /dirty_spots
ros2 topic echo /camera/image_raw
```

## 参数配置
在 `config/vision_detection.yaml` 中可以配置：
- `camera_index`: 摄像头设备索引（默认: 0）
- `model_path`: RKNN模型文件路径
- `conf_threshold`: 置信度阈值（默认: 0.5）
- `input_width/height`: 模型输入尺寸（默认: 640x640）
- `topic_name`: 检测结果发布话题（默认: /dirty_spots）
- `image_topic`: 图像发布话题（默认: /camera/image_raw）
- `frame_id`: 图像坐标系（默认: camera_frame）
- `fps`: 处理帧率（默认: 30）

## 数据格式
- 发布话题: `/dirty_spots` (vision_msgs/msg/Detection2DArray)
- 图像话题: `/camera/image_raw` (sensor_msgs/msg/Image)

## 注意事项
1. 确保摄像头已正确连接
2. 需要RKNN格式的模型文件
3. 在RK3588平台上运行以使用NPU加速
4. 模型输入输出格式需要与代码中的处理逻辑匹配

