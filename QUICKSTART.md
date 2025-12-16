# 快速开始指南

## 1. 环境准备

### 安装ROS2 Humble
```bash
# 参考官方文档安装ROS2 Humble
# https://docs.ros.org/en/humble/Installation.html
```

### 安装依赖
```bash
cd cleaner_robot_ws
./install_dependencies.sh
```

## 2. 构建工作空间

```bash
cd cleaner_robot_ws
./build.sh
source install/setup.bash
```

## 3. 配置设备

### 串口权限
```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 摄像头权限
```bash
sudo usermod -a -G video $USER
# 重新登录后生效
```

## 4. 配置参数

### 激光雷达配置
编辑 `src/lidar_driver/config/lidar_driver.yaml`:
```yaml
serial_port: "/dev/ttyUSB0"  # 根据实际设备修改
baud_rate: 230400
```

### 视觉检测配置
编辑 `src/vision_detection/config/vision_detection.yaml`:
```yaml
camera_index: 0  # 根据实际摄像头修改
model_path: "/path/to/your/model.rknn"  # 指定RKNN模型路径
conf_threshold: 0.5
```

### STM32接口配置
默认参数已经写在 launch 里（/dev/ttyUSB0, 115200, 20Hz, 0.3s 超时），通常无需额外配置。若需修改，可在启动时覆盖 launch 参数。

## 5. 运行节点

### 方式1: 统一启动（推荐）
```bash
ros2 launch cleaner_robot_bringup cleaner_robot.launch.py
```

### 方式2: 分别启动
```bash
# 终端1: 激光雷达
ros2 launch lidar_driver lidar_driver.launch.py

# 终端2: 视觉检测
ros2 launch vision_detection vision_detection.launch.py

# 终端3: 感知融合
ros2 launch perception_fusion perception_fusion.launch.py

# 终端4: STM32接口
ros2 launch stm32_interface stm32_bridge.launch.py
```

### 导航（已有地图）
```bash
# 使用默认地图 cleanrobot_nav/maps/cleanrobot_map.yaml
ros2 launch cleanrobot_nav bringup.launch.py
# 如需临时替换地图
ros2 launch cleanrobot_nav bringup.launch.py map:=/home/cat/maps/other_map.yaml
```

## 6. 查看数据

### 查看话题列表
```bash
ros2 topic list
```

### 查看激光扫描数据
```bash
ros2 topic echo /scan
```

### 查看检测结果
```bash
ros2 topic echo /dirty_spots
```

### 查看清洁任务
```bash
ros2 topic echo /cleaning_task
```

### 查看STM32状态
```bash
ros2 topic echo /stm32_status
```

## 7. 调试命令

### 查看节点信息
```bash
ros2 node list
ros2 node info /lidar_driver_node
```

### 查看参数
```bash
ros2 param list
ros2 param get /lidar_driver_node serial_port
```

### 查看TF树
```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

## 8. 常见问题

### 串口打开失败
- 检查设备路径是否正确: `ls -l /dev/ttyUSB*`
- 检查权限: `groups $USER` (应包含 dialout)
- 检查设备是否被占用: `lsof /dev/ttyUSB0`

### 摄像头打开失败
- 检查设备索引: `v4l2-ctl --list-devices`
- 检查权限: `groups $USER` (应包含 video)

### RKNN模型加载失败
- 检查模型路径是否正确
- 检查模型格式是否为RKNN
- 确认在RK3588平台上运行

### TF变换失败
- 确保TF树正确发布
- 检查坐标系名称是否正确
- 使用 `ros2 run tf2_ros tf2_echo` 查看变换

## 9. 开发建议

1. **模块化开发**: 每个功能包独立开发，通过话题通信
2. **参数化配置**: 使用YAML文件配置参数，便于调试
3. **日志记录**: 使用ROS2日志系统记录关键信息
4. **错误处理**: 添加适当的异常处理和错误恢复机制
5. **测试验证**: 单独测试每个节点，确保功能正常

## 10. 下一步

- 集成SLAM系统（如slam_toolbox）
- 集成导航系统（如nav2）
- 优化相机坐标转换算法
- 添加任务调度和优先级管理
- 实现错误处理和重试机制

