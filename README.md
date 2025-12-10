# Cleaner Robot Workspace - 扫地机器人ROS2工作空间

基于ROS2 Humble的感知与控制通信系统，用于RK3588平台上的扫地机器人项目。

## 工作空间结构

```
cleaner_robot_ws/
├── src/
│   ├── lidar_driver/          # LD14P激光雷达驱动包
│   ├── vision_detection/      # 视觉检测包（YOLOv8 + RKNN）
│   ├── perception_fusion/     # 感知融合包（地图+视觉融合）
│   └── stm32_interface/       # STM32通信接口包
├── build/                     # 构建目录
├── install/                   # 安装目录
└── log/                       # 日志目录
```

## 功能包说明

### 1. lidar_driver
- **功能**: 驱动LD14P激光雷达，通过串口读取数据并发布LaserScan消息
- **发布话题**: `/scan` (sensor_msgs/msg/LaserScan)
- **配置**: 支持串口设备、波特率、坐标系等参数配置

### 1.5 cleanrobot_slam
- **功能**: 启动LD14激光雷达与 slam_toolbox online_async 模式，实现 2D 建图
- **订阅话题**: `/scan` (sensor_msgs/msg/LaserScan, frame `base_laser`)
- **配置**: `cleanrobot_slam/config/slam_params.yaml`

### 1.6 cleanrobot_nav
- **功能**: 基于 Nav2 的导航启动包，加载本仓库的导航参数
- **订阅/发布**: 订阅 `/scan`、`/odom`，发布 `/cmd_vel`
- **配置**: `cleanrobot_nav/config/nav2_params.yaml`

### 2. vision_detection
- **功能**: 使用USB摄像头采集图像，通过YOLOv8 RKNN模型在NPU上推理识别脏污
- **发布话题**: 
  - `/dirty_spots` (vision_msgs/msg/Detection2DArray)
  - `/camera/image_raw` (sensor_msgs/msg/Image)
- **配置**: 支持模型路径、摄像头索引、置信度阈值等参数配置

### 3. perception_fusion
- **功能**: 融合SLAM地图和视觉检测结果，生成清洁任务
- **订阅话题**: 
  - `/map` (nav_msgs/msg/OccupancyGrid)
  - `/odom` (nav_msgs/msg/Odometry)
  - `/dirty_spots` (vision_msgs/msg/Detection2DArray)
  - `/scan` (sensor_msgs/msg/LaserScan)
- **发布话题**: `/cleaning_task` (perception_fusion/msg/CleaningTaskList)
- **自定义消息**: CleanTask.msg, CleaningTaskList.msg

### 4. stm32_interface
- **功能**: 与STM32下位机通信，发送清洁任务指令，接收反馈
- **订阅话题**: `/cleaning_task` (perception_fusion/msg/CleaningTaskList)
- **发布话题**: `/stm32_status` (std_msgs/msg/String)
- **通信方式**: 支持串口和UDP两种通信方式

## 环境要求

- **ROS2版本**: Humble Hawksbill
- **Python版本**: Python 3.8+
- **平台**: RK3588 (支持NPU加速)
- **系统**: Ubuntu 22.04 (推荐)

## 依赖安装

### 1. 安装ROS2 Humble
参考官方文档: https://docs.ros.org/en/humble/Installation.html

### 2. 安装Python依赖
```bash
pip3 install pyserial opencv-python numpy
```

### 3. 安装RKNN工具包（RK3588平台）
```bash
pip3 install rknn-toolkit-lite
```

### 4. 安装ROS2依赖包
```bash
sudo apt install ros-humble-sensor-msgs \
                 ros-humble-nav-msgs \
                 ros-humble-vision-msgs \
                 ros-humble-cv-bridge \
                 ros-humble-tf2-ros \
                 ros-humble-tf2-geometry-msgs \
                 ros-humble-geometry-msgs
```

## 构建工作空间

```bash
cd cleaner_robot_ws
colcon build
source install/setup.bash
```

## 运行节点

### 方式1: 分别启动各节点

```bash
# 终端1: 启动激光雷达驱动
ros2 launch lidar_driver lidar_driver.launch.py

# 终端2: 启动视觉检测
ros2 launch vision_detection vision_detection.launch.py

# 终端3: 启动感知融合
ros2 launch perception_fusion perception_fusion.launch.py

# 终端4: 启动STM32接口
ros2 launch stm32_interface stm32_interface.launch.py
```

### 方式2: 使用统一启动文件（需要创建）

```bash
ros2 launch cleaner_robot_bringup cleaner_robot.launch.py
```

### 方式3: SLAM 建图与地图保存

```bash
# 终端1: 启动雷达 + slam_toolbox（online_async 模式，LaserScan frame 为 base_laser）
ros2 launch cleanrobot_slam ld14_slam.launch.py

# 终端2: 保存地图
ros2 run nav2_map_server map_saver_cli --ros-args -p save_map_timeout:=5.0 -p free_thresh_default:=0.25 -p occupied_thresh_default:=0.65 -p map_topic:=/map -p output_file:=~/maps/cleanrobot_map
```

### 导航启动步骤

1. 启动雷达 + SLAM 建图（如果已有地图可跳过）：
   ```bash
   ros2 launch cleanrobot_slam ld14_slam.launch.py
   ```
2. 保存地图：
   ```bash
   ros2 run nav2_map_server map_saver_cli --ros-args -p map_topic:=/map -p output_file:=~/maps/cleanrobot_map
   ```
3. 使用保存好的地图启动 Nav2：
   ```bash
   ros2 launch cleanrobot_nav bringup.launch.py map:=~/maps/cleanrobot_map.yaml
   ```

## 配置说明

各功能包的配置文件位于各包的 `config/` 目录下：
- `lidar_driver/config/lidar_driver.yaml`
- `vision_detection/config/vision_detection.yaml`
- `perception_fusion/config/perception_fusion.yaml`
- `stm32_interface/config/stm32_interface.yaml`

## 话题列表

### 发布话题
- `/scan` - 激光扫描数据
- `/camera/image_raw` - 摄像头图像
- `/dirty_spots` - 脏污检测结果
- `/cleaning_task` - 清洁任务列表
- `/stm32_status` - STM32状态反馈

### 订阅话题
- `/map` - SLAM地图（需要SLAM节点提供）
- `/odom` - 里程计信息（需要导航节点提供）

## TF树结构

```
map
 └── odom
      └── base_link
           ├── laser_frame (lidar_driver)
           └── camera_frame (vision_detection)
```

## 开发指南

### 添加新功能
1. 在对应功能包的 `src/` 或 `scripts/` 目录下添加代码
2. 更新 `package.xml` 和 `CMakeLists.txt`（如需要）
3. 重新构建: `colcon build --packages-select <package_name>`
4. 更新配置文件（如需要）

### 调试技巧
```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看话题内容
ros2 topic echo /scan

# 查看节点信息
ros2 node info /lidar_driver_node

# 查看参数
ros2 param list
ros2 param get /lidar_driver_node serial_port
```

## 故障排查

### 串口权限问题
```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 摄像头权限问题
```bash
sudo usermod -a -G video $USER
# 重新登录后生效
```

### TF变换问题
确保TF树正确发布，可以使用：
```bash
ros2 run tf2_ros tf2_echo map base_link
```

### RKNN模型问题
- 确保模型文件路径正确
- 检查模型格式是否为RKNN
- 确认模型输入输出尺寸匹配

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

如有问题，请通过Issue或邮件联系。

