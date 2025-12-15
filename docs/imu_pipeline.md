# IMU Pipeline

This package wires the YbIMU driver, optional Madgwick filter, and RViz visualization. Key topics and frames:

- `/imu/data_raw`: `sensor_msgs/Imu` from `ybimu_driver` (linear acceleration in m/s^2, angular velocity in rad/s), `frame_id=imu_link`.
- `/imu/data`: Filtered `sensor_msgs/Imu` when `imu_filter_madgwick` is running; same frame id as input.
- `/imu/mag`, `/baro`, `/euler`: Additional outputs from the driver.
- `imu_link`: IMU sensor frame set by the driver and used as RViz fixed frame.
- `base_link`: Robot base frame. No static transform is provided here—publish one if you need `base_link -> imu_link` in TF.

Typical launch commands:

- Full pipeline with RViz and filter (defaults: driver + RViz on):
  ```bash
  ros2 launch imu_ros2_device ybimu_display.launch.py
  ```
- Toggle pieces:
  ```bash
  ros2 launch imu_ros2_device ybimu_display.launch.py start_driver:=false   # assume driver already running elsewhere
  ros2 launch imu_ros2_device ybimu_display.launch.py rviz:=false            # skip RViz, keep driver + filter
  ```
- Driver-only for serial debugging:
  ```bash
  ros2 launch imu_ros2_device imu_only.launch.py
  ```
- If you need TF between `base_link` and `imu_link`, publish it separately (example identity transform):
  ```bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link
  ```
