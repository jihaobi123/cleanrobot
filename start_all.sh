#!/bin/bash
set -e

WS=~/文档/cleanrobot/cleanrobot
MAP_YAML=$WS/maps/my_map.yaml

echo "[1/8] Sourcing ROS2 & workspace..."
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash

START_NAV=false
if [[ "$1" == "--nav" ]]; then
  START_NAV=true
fi

pids=()
cleanup() {
  echo ""
  echo "[EXIT] Stopping all launched processes..."
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  exit 0
}
trap cleanup SIGINT SIGTERM

echo "[2/8] Starting LiDAR..."
ros2 launch ldlidar ld14p.launch.py &
pids+=($!)
sleep 2

echo "[3/8] Starting IMU pipeline (driver + filter, RViz off)..."
ros2 launch imu_ros2_device ybimu_display.launch.py start_driver:=true rviz:=false &
pids+=($!)
sleep 2

echo "[4/8] Starting real odometry (rf2o + EKF)..."
ros2 launch cleanrobot_bringup odom.launch.py &
pids+=($!)
sleep 2

echo "[5/8] Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$WS/config/ld14_slam.yaml &
pids+=($!)
sleep 2

echo "[6/8] Starting RViz2..."
rviz2 &
pids+=($!)

if $START_NAV; then
  echo "[7/8] Starting Nav2..."
  if [[ ! -f "$MAP_YAML" ]]; then
    echo "[ERROR] Map yaml not found: $MAP_YAML"
    echo "        Please save a map first (map_saver) and set MAP_YAML correctly."
    cleanup
  fi
  ros2 launch nav2_bringup navigation_launch.py \
    map:=$MAP_YAML use_sim_time:=false &
  pids+=($!)
fi

echo "[8/8] All started."
echo " - Press Ctrl+C to stop everything."
wait
