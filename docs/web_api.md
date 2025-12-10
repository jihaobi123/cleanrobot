# Robot Web Bridge API

## 概述
`robot_web_bridge` 包通过 FastAPI 将 ROS2 机器人内部的数据以 HTTP 与 WebSocket 的形式暴露给前端网页。接口提供地图、机器人位姿状态查询，以及通过 WebSocket 推送位姿并接收速度与导航目标指令，便于在浏览器中进行可视化和控制。

## 运行方式
1. 编译工作区：
   ```bash
   colcon build
   ```
2. 加载工作区环境：
   ```bash
   . install/setup.bash
   ```
3. 启动后端服务（默认监听 8000）：
   ```bash
   ros2 launch robot_web_bridge web_bridge.launch.py
   ```
4. 可通过参数调整端口及话题（示例）：
   ```bash
   ros2 launch robot_web_bridge web_bridge.launch.py websocket_port:=9000 map_topic:=/map cmd_vel_topic:=/cmd_vel goal_topic:=/goal_pose
   ```

## HTTP 接口
### `GET /api/map`
- **说明**：返回最新的占用栅格地图。
- **返回 JSON 字段**：
  - `has_map` (bool)：是否已收到有效地图。
  - `width` / `height` (int)：地图宽、高（栅格数）。
  - `resolution` (float)：分辨率（米/格）。
  - `origin.x`、`origin.y` (float)：地图原点在 `map` 坐标系下的平移（米）。
  - `data` (int[])：占用栅格数组，与 ROS `OccupancyGrid.data` 一致。
- **示例响应**：
  ```json
  {
    "has_map": true,
    "width": 200,
    "height": 200,
    "resolution": 0.05,
    "origin": { "x": -5.0, "y": -5.0 },
    "data": [0, 0, 100, -1, ...]
  }
  ```

### `GET /api/state`
- **说明**：返回机器人在 `map` 坐标系下的最新位姿。
- **返回 JSON 字段**：
  - `x`、`y` (float)：机器人在 `map` 坐标系下的位置（米）。
  - `yaw` (float)：朝向 yaw，弧度，逆时针为正。
  - `stamp` (string)：时间戳（`sec.nanosec` 格式）。
  - `has_pose` (bool)：是否已获取到有效位姿。
  - `has_map` (bool)：当前是否已有地图（便于前端同时判断地图和位姿状态）。
- **示例响应**：
  ```json
  {
    "x": 1.23,
    "y": -0.45,
    "yaw": 0.78,
    "stamp": "1685600000.123456789",
    "has_pose": true,
    "has_map": true
  }
  ```

## WebSocket 接口
- **地址**：`ws://<lubancat-ip>:8000/ws`
- **服务器推送**：默认每 0.1 秒发送一次位姿 JSON。
  ```json
  {
    "type": "pose",
    "x": 1.23,
    "y": -0.45,
    "yaw": 0.78,
    "stamp": "1685600000.123456789",
    "has_pose": true,
    "has_map": true
  }
  ```

- **客户端指令**：
  - 发送速度指令：
    ```json
    { "type": "cmd_vel", "vx": 0.1, "vy": 0.0, "omega": 0.3 }
    ```
    - `vx`、`vy` (m/s)：底盘线速度，坐标系遵循 `base_link`。
    - `omega` (rad/s)：角速度，绕 z 轴。

  - 设置导航目标：
    ```json
    { "type": "set_goal", "x": 2.0, "y": 1.0, "yaw": 1.57 }
    ```
    - `x`、`y` (m)：目标在 `map` 坐标系下的位置。
    - `yaw` (rad)：目标朝向，弧度。

- **错误处理**：字段缺失或格式错误会在服务器日志输出 warning，指令会被忽略。
