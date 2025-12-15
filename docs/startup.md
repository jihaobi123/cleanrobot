# Startup Guide

This repository provides two primary workflows:

- `./start_all.sh` — mapping mode. Launches LiDAR, IMU pipeline (driver + filter), rf2o + EKF odometry (`odom -> base_link` TF), slam_toolbox, and RViz. Nav2 stays off to avoid interfering with mapping.
- `./start_all.sh --nav` — navigation mode. Same as mapping, plus Nav2 (requires an existing map yaml at `~/文档/cleanrobot/cleanrobot/maps/my_map.yaml`; edit `MAP_YAML` inside the script to your path).

Scripts expect the workspace at `~/文档/cleanrobot/cleanrobot` and ROS 2 Humble. Before running, ensure `ldlidar`, `imu_ros2_device`, `rf2o_laser_odometry`, `robot_localization`, and `slam_toolbox` build successfully via:

```bash
cd ~/文档/cleanrobot/cleanrobot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Notes:
- The odometry TF chain comes from rf2o + EKF; no static `odom->base_link` is published.
- `start_all.sh` uses `start_driver:=true` and `rviz:=false` for the IMU bringup; adjust if you launch the driver separately.
- For Nav2, verify the TF tree `map -> odom -> base_link -> base_laser` and `base_link -> imu_link` is present, and the map file exists before starting `--nav`.
