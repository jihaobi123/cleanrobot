# stm32_interface - STM32接口包

## 功能说明
本包提供与STM32下位机的通信接口，支持串口和UDP两种通信方式。接收清洁任务并发送给STM32，同时监听STM32的反馈状态。

## 依赖
- rclpy
- std_msgs
- geometry_msgs
- perception_fusion
- pyserial

## 安装依赖
```bash
pip3 install pyserial
```

## 通信协议

### 串口协议
帧格式: `Header(0xAA55) + CMD_ID(1字节) + Payload长度(2字节) + Payload + CRC16(2字节)`

命令ID:
- `0x01`: 发送任务 (CMD_SEND_TASK)
- `0x02`: 请求状态 (CMD_REQUEST_STATUS)
- `0x03`: 取消任务 (CMD_CANCEL_TASK)
- `0x04`: 紧急停止 (CMD_EMERGENCY_STOP)

响应ID:
- `0x81`: 任务确认 (RESP_TASK_ACK)
- `0x82`: 状态反馈 (RESP_STATUS)
- `0xFF`: 错误反馈 (RESP_ERROR)

任务数据格式:
- task_id: 4字节 (uint32, 小端)
- x: 4字节 (float, 小端)
- y: 4字节 (float, 小端)
- cleaning_type: 1字节 (uint8)
- priority: 1字节 (uint8)

### UDP协议
简化格式: `Header(0xAA55, 2字节) + CMD_ID(2字节) + Payload`

## 使用方法

### 1. 构建包
```bash
cd cleaner_robot_ws
colcon build --packages-select stm32_interface
source install/setup.bash
```

### 2. 启动节点（串口模式）
```bash
ros2 launch stm32_interface stm32_interface.launch.py communication_type:=serial serial_port:=/dev/ttyUSB1
```

### 3. 启动节点（UDP模式）
```bash
ros2 launch stm32_interface stm32_interface.launch.py communication_type:=udp
```

### 4. 查看状态反馈
```bash
ros2 topic echo /stm32_status
```

## 参数配置
在 `config/stm32_interface.yaml` 中可以配置：
- `communication_type`: 通信类型（serial 或 udp）
- `serial_port`: 串口设备路径（默认: /dev/ttyUSB1）
- `serial_baud_rate`: 串口波特率（默认: 115200）
- `udp_local_port`: UDP本地端口（默认: 8888）
- `udp_remote_addr`: UDP远程地址（默认: 192.168.1.100）
- `udp_remote_port`: UDP远程端口（默认: 8889）
- `task_topic`: 任务订阅话题（默认: /cleaning_task）
- `status_topic`: 状态发布话题（默认: /stm32_status）

## 数据格式
- 订阅话题:
  - `/cleaning_task` (perception_fusion/msg/CleaningTaskList): 清洁任务列表

- 发布话题:
  - `/stm32_status` (std_msgs/msg/String): STM32状态反馈

## 注意事项
1. 确保串口权限正确（可能需要添加用户到dialout组）
2. 检查串口设备路径是否正确
3. 确保STM32端实现对应的通信协议
4. UDP模式需要确保网络连接正常
5. CRC16校验确保数据传输的可靠性

## micro-ROS支持
本包当前实现串口和UDP通信。如需使用micro-ROS，可以：
1. 在STM32端运行micro-ROS客户端
2. 通过micro-ROS Agent与ROS2通信
3. 修改本包以支持micro-ROS话题通信

## 扩展功能
可以扩展以下功能：
- 任务执行状态跟踪
- 错误处理和重试机制
- 心跳包检测
- 任务优先级调度
- 多任务队列管理

