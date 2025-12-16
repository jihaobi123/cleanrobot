# stm32_interface - /cmd_vel → STM32 串口桥

最小可用实现：订阅 `/cmd_vel`，按协议经串口把 `vx`、`wz` 发给 STM32。20Hz 周期发送，0.3s 看门狗超时自动刹车。

## 协议（下行）
- 帧头：`AA 55`
- version：`0x01`
- msg_id：`0x10`
- payload_len：`0x08`
- payload：`vx_mm_s int32`、`wz_mrad_s int32`（小端）
- 校验：CRC-16/CCITT-FALSE（poly 0x1021, init 0xFFFF）
- 总长：15 Bytes

## 依赖
- rclpy
- geometry_msgs
- pyserial（`sudo apt install -y python3-serial`）

## 构建与运行
```bash
cd ~/文档/cleanrobot/cleanrobot
colcon build --symlink-install --packages-select stm32_interface
source install/setup.bash

# 启动桥接
ros2 launch stm32_interface stm32_bridge.launch.py
```

## 验证
```bash
# 直走
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}"
# 原地转
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.6}}"
# Ctrl+C 后 0.3s 内应自动停
```

