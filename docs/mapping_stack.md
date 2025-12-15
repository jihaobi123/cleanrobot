# Mapping Stack (Humble)

This bringup combines LD14 lidar, YbIMU, rf2o laser odometry, robot_localization EKF, slam_toolbox, and RViz.

- Launch: `ros2 launch cleanrobot_bringup mapping.launch.py [rviz:=true|false]`
- Nodes:
  - `ldlidar/ld14p.launch.py` → `/scan`, frame `base_laser` (check your lidar launch).
  - `imu_ros2_device/ybimu_display.launch.py` → `/imu/data_raw` (`imu_link`).
  - `rf2o_laser_odometry_node` → `/rf2o/odom` (no TF).
  - `robot_localization/ekf_node` → publishes TF `odom -> base_link` and `/odometry/filtered`.
  - `slam_toolbox/async_slam_toolbox_node` → publishes TF `map -> odom` and `/map`.
  - `tf2_ros/static_transform_publisher` → `base_link -> imu_link`.
  - `rviz2` (optional) showing map/scan/TF.

Topic/TF summary:

- `/scan` (sensor_msgs/LaserScan) in `base_laser`.
- `/imu/data_raw` (sensor_msgs/Imu) in `imu_link` with acceleration in m/s^2.
- `/rf2o/odom` (nav_msgs/Odometry) frame ids `odom`/`base_link`, no TF.
- `/odometry/filtered` (nav_msgs/Odometry) from EKF, frame ids `odom`/`base_link`, publishes TF.
- TF tree: `map -> odom -> base_link -> base_laser` and `base_link -> imu_link`.

Typical commands:

```bash
cd ~/文档/cleanrobot/cleanrobot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch cleanrobot_bringup mapping.launch.py
```

Check TF tree:

```bash
ros2 run tf2_tools view_frames  # opens frames.pdf with map->odom->base_link chain
```

Common issues:

- Message filter dropping scans/IMU: ensure clocks are synced and frame ids match (`map/odom/base_link/base_laser/imu_link`), check `use_sim_time` settings (slam_toolbox false by default).
- Failed to compute odom pose: rf2o needs valid `/scan` with correct frame; verify lidar TF (`base_link -> base_laser`) exists and ranges are non-zero.
- No TF `odom -> base_link`: confirm EKF is running and receiving `/rf2o/odom` and `/imu/data_raw`; check parameter paths in `config/ekf.yaml`.
- RViz empty map: wait for slam_toolbox to receive scans; confirm `/map` topic present and fixed frame set to `map`.
