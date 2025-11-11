# 项目结构说明

## 工作空间目录结构

```
cleaner_robot_ws/
├── README.md                    # 项目主文档
├── QUICKSTART.md                # 快速开始指南
├── PROJECT_STRUCTURE.md         # 本文件
├── build.sh                     # 构建脚本
├── install_dependencies.sh      # 依赖安装脚本
├── .gitignore                   # Git忽略文件
│
├── src/                         # 源代码目录
│   ├── lidar_driver/            # 激光雷达驱动包
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── README.md
│   │   ├── resource/
│   │   │   └── lidar_driver
│   │   ├── lidar_driver/
│   │   │   ├── __init__.py
│   │   │   └── lidar_driver_node.py
│   │   ├── launch/
│   │   │   └── lidar_driver.launch.py
│   │   └── config/
│   │       └── lidar_driver.yaml
│   │
│   ├── vision_detection/        # 视觉检测包
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── README.md
│   │   ├── resource/
│   │   │   └── vision_detection
│   │   ├── vision_detection/
│   │   │   ├── __init__.py
│   │   │   └── vision_detection_node.py
│   │   ├── launch/
│   │   │   └── vision_detection.launch.py
│   │   ├── config/
│   │   │   └── vision_detection.yaml
│   │   └── models/              # RKNN模型文件目录
│   │       └── .gitkeep
│   │
│   ├── perception_fusion/       # 感知融合包
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── msg/                 # 自定义消息定义
│   │   │   ├── CleanTask.msg
│   │   │   └── CleaningTaskList.msg
│   │   ├── scripts/
│   │   │   └── perception_fusion_node.py
│   │   ├── launch/
│   │   │   └── perception_fusion.launch.py
│   │   └── config/
│   │       └── perception_fusion.yaml
│   │
│   ├── stm32_interface/         # STM32接口包
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── scripts/
│   │   │   └── stm32_interface_node.py
│   │   ├── launch/
│   │   │   └── stm32_interface.launch.py
│   │   └── config/
│   │       └── stm32_interface.yaml
│   │
│   └── cleaner_robot_bringup/   # 统一启动包
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── launch/
│           └── cleaner_robot.launch.py
│
├── build/                       # 构建目录（构建后生成）
├── install/                     # 安装目录（构建后生成）
└── log/                         # 日志目录（构建后生成）
```

## 包说明

### 1. lidar_driver (Python包)
- **类型**: ament_python
- **功能**: LD14P激光雷达驱动
- **主要文件**: 
  - `lidar_driver_node.py`: 主节点实现
  - `lidar_driver.yaml`: 配置参数
  - `lidar_driver.launch.py`: 启动文件

### 2. vision_detection (Python包)
- **类型**: ament_python
- **功能**: 视觉检测（YOLOv8 + RKNN）
- **主要文件**:
  - `vision_detection_node.py`: 主节点实现
  - `vision_detection.yaml`: 配置参数
  - `vision_detection.launch.py`: 启动文件
  - `models/`: RKNN模型文件目录

### 3. perception_fusion (CMake包)
- **类型**: ament_cmake
- **功能**: 感知融合，生成清洁任务
- **主要文件**:
  - `perception_fusion_node.py`: 主节点实现
  - `CleanTask.msg`: 清洁任务消息定义
  - `CleaningTaskList.msg`: 任务列表消息定义
  - `perception_fusion.yaml`: 配置参数
  - `perception_fusion.launch.py`: 启动文件

### 4. stm32_interface (CMake包)
- **类型**: ament_cmake
- **功能**: STM32通信接口
- **主要文件**:
  - `stm32_interface_node.py`: 主节点实现
  - `stm32_interface.yaml`: 配置参数
  - `stm32_interface.launch.py`: 启动文件

### 5. cleaner_robot_bringup (CMake包)
- **类型**: ament_cmake
- **功能**: 统一启动所有节点
- **主要文件**:
  - `cleaner_robot.launch.py`: 统一启动文件

## 消息类型

### CleanTask.msg
- `uint32 task_id`: 任务ID
- `geometry_msgs/Point target_position`: 目标位置
- `uint8 cleaning_type`: 清洁类型
- `uint8 priority`: 优先级
- `uint8 status`: 任务状态
- `builtin_interfaces/Time timestamp`: 时间戳
- `string description`: 描述

### CleaningTaskList.msg
- `CleanTask[] tasks`: 任务列表
- `builtin_interfaces/Time timestamp`: 时间戳

## 话题通信

### 发布话题
- `/scan` (sensor_msgs/msg/LaserScan): 激光扫描数据
- `/camera/image_raw` (sensor_msgs/msg/Image): 摄像头图像
- `/dirty_spots` (vision_msgs/msg/Detection2DArray): 脏污检测结果
- `/cleaning_task` (perception_fusion/msg/CleaningTaskList): 清洁任务列表
- `/stm32_status` (std_msgs/msg/String): STM32状态反馈

### 订阅话题
- `/map` (nav_msgs/msg/OccupancyGrid): SLAM地图
- `/odom` (nav_msgs/msg/Odometry): 里程计信息
- `/dirty_spots` (vision_msgs/msg/Detection2DArray): 视觉检测结果
- `/scan` (sensor_msgs/msg/LaserScan): 激光扫描数据
- `/cleaning_task` (perception_fusion/msg/CleaningTaskList): 清洁任务列表

## 配置文件

所有配置文件中 `config/` 目录下，使用YAML格式：
- `lidar_driver.yaml`: 激光雷达配置
- `vision_detection.yaml`: 视觉检测配置
- `perception_fusion.yaml`: 感知融合配置
- `stm32_interface.yaml`: STM32接口配置

## 构建系统

- **Python包**: 使用 `setup.py` 和 `setup.cfg`
- **CMake包**: 使用 `CMakeLists.txt`
- **构建工具**: colcon
- **构建命令**: `colcon build --symlink-install`

## 开发规范

1. **代码组织**: 每个功能包独立，通过话题通信
2. **参数配置**: 使用YAML文件，支持launch参数覆盖
3. **消息定义**: 自定义消息放在perception_fusion包中
4. **启动文件**: 每个包提供独立的launch文件，同时提供统一启动文件
5. **文档**: 每个包提供README.md说明文档

## 扩展指南

### 添加新功能包
1. 在 `src/` 目录下创建新包
2. 创建 `package.xml` 和构建文件
3. 添加节点代码、launch文件和配置文件
4. 更新 `cleaner_robot_bringup` 的启动文件

### 添加新消息类型
1. 在 `perception_fusion/msg/` 目录下创建 `.msg` 文件
2. 更新 `CMakeLists.txt` 中的消息定义
3. 重新构建包
4. 在代码中导入使用

### 添加新节点
1. 在对应包的 `scripts/` 或 `src/` 目录下创建节点文件
2. 更新 `package.xml` 和构建文件
3. 创建launch文件
4. 添加配置文件

