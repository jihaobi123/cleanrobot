# lidar_driver - LD14P激光雷达驱动包

## 功能说明
本包用于驱动LD14P激光雷达，通过串口通信读取雷达数据，解析后发布为ROS2标准的`LaserScan`消息。

## 依赖
- rclpy
- sensor_msgs
- pyserial

## 安装依赖
```bash
pip3 install pyserial
```

## 使用方法

### 1. 构建包
```bash
cd cleaner_robot_ws
colcon build --packages-select lidar_driver
source install/setup.bash
```

### 2. 启动节点
```bash
ros2 launch lidar_driver lidar_driver.launch.py
```

### 3. 自定义参数启动
```bash
ros2 launch lidar_driver lidar_driver.launch.py serial_port:=/dev/ttyUSB1
```

### 4. 查看发布的话题
```bash
ros2 topic echo /scan
```

## 参数配置
在 `config/lidar_driver.yaml` 中可以配置：
- `serial_port`: 串口设备路径（默认: /dev/ttyUSB0）
- `baud_rate`: 波特率（默认: 230400）
- `frame_id`: 激光扫描的坐标系（默认: laser_frame）
- `topic_name`: 发布的话题名称（默认: /scan）
- `range_min/max`: 检测距离范围
- `angle_min/max`: 扫描角度范围

## 数据格式
- 发布话题: `/scan` (sensor_msgs/msg/LaserScan)
- 数据包格式: 遵循LD14P雷达通信协议
- 扫描频率: 40Hz
- 角度分辨率: 1度（360个点）

## 注意事项
1. 确保串口权限正确（可能需要添加用户到dialout组）
2. 检查串口设备路径是否正确
3. 确保雷达已正确连接并上电

## 使用LDROBOT官方ROS2驱动（ldlidar_sl_ros2）
如果希望直接使用LDROBOT官方提供的LD14/LD14P驱动，可参考以下步骤。官方仓库已在 **Ubuntu 22.04 + ROS2 Humble** 环境下验证，依赖和启动流程如下：

1. 安装依赖
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-rclcpp \\
                       ros-humble-sensor-msgs \\
                       ros-humble-tf2 \\
                       ros-humble-tf2-ros \\
                       ros-humble-path-navigation-msgs
   ```

2. 克隆并构建
   ```bash
   cd ~/ros2_ws/src
   git clone https://gitee.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
   cd ..
   colcon build
   source install/setup.bash
   ```

3. 运行LD14节点
   ```bash
   ros2 launch ldlidar_sl_ros2 ld14.launch.py
   ```

4. 验证话题
   ```bash
   ros2 topic list
   ```
   预期至少能看到`/scan`和`/ldlidar_node/health`。

更多细节参考官方仓库文档：https://gitee.com/ldrobotSensorTeam/ldlidar_sl_ros2

