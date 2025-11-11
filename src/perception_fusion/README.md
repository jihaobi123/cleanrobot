# perception_fusion - 感知融合包

## 功能说明
本包融合激光雷达SLAM地图和视觉检测结果，将图像识别的脏污区域坐标转换为地图坐标系，生成清洁任务并发布。

## 依赖
- rclpy
- sensor_msgs
- nav_msgs
- vision_msgs
- geometry_msgs
- tf2_ros
- tf2_geometry_msgs
- numpy

## 安装依赖
```bash
pip3 install numpy
```

## 使用方法

### 1. 构建包
```bash
cd cleaner_robot_ws
colcon build --packages-select perception_fusion
source install/setup.bash
```

### 2. 启动节点
```bash
ros2 launch perception_fusion perception_fusion.launch.py
```

### 3. 查看发布的任务
```bash
ros2 topic echo /cleaning_task
```

## 参数配置
在 `config/perception_fusion.yaml` 中可以配置：
- `map_topic`: 地图话题（默认: /map）
- `odom_topic`: 里程计话题（默认: /odom）
- `detection_topic`: 检测结果话题（默认: /dirty_spots）
- `scan_topic`: 激光扫描话题（默认: /scan）
- `task_topic`: 任务发布话题（默认: /cleaning_task）
- `map_frame`: 地图坐标系（默认: map）
- `base_frame`: 机器人基座坐标系（默认: base_link）
- `camera_frame`: 相机坐标系（默认: camera_frame）
- `task_threshold`: 任务去重距离阈值（默认: 0.5m）
- `max_tasks`: 最大任务数量（默认: 10）

## 自定义消息
- `CleanTask.msg`: 单个清洁任务消息
  - task_id: 任务ID
  - target_position: 目标点位置（geometry_msgs/Point）
  - cleaning_type: 清洁类型（0: 普通, 1: 深度, 2: 定点）
  - priority: 优先级（0-255）
  - status: 任务状态（0: 待执行, 1: 执行中, 2: 已完成, 3: 失败）
  - timestamp: 时间戳
  - description: 描述信息

- `CleaningTaskList.msg`: 清洁任务列表消息
  - tasks: 任务列表（CleanTask[]）
  - timestamp: 时间戳

## 数据格式
- 订阅话题:
  - `/map` (nav_msgs/msg/OccupancyGrid): SLAM地图
  - `/odom` (nav_msgs/msg/Odometry): 里程计信息
  - `/dirty_spots` (vision_msgs/msg/Detection2DArray): 视觉检测结果
  - `/scan` (sensor_msgs/msg/LaserScan): 激光扫描数据

- 发布话题:
  - `/cleaning_task` (perception_fusion/msg/CleaningTaskList): 清洁任务列表

## 工作原理
1. 订阅地图、里程计、检测结果和激光扫描数据
2. 通过TF获取机器人在地图中的位姿
3. 将图像检测的像素坐标转换为地图坐标（使用相机投影模型）
4. 生成清洁任务，避免重复任务
5. 发布任务列表给下位机

## 注意事项
1. 需要正确配置TF树（map -> odom -> base_link -> camera_frame）
2. 相机坐标转换需要根据实际相机标定参数调整
3. 任务去重基于距离阈值，可根据实际需求调整
4. 需要SLAM系统运行以提供地图和位姿信息

