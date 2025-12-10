#!/bin/bash

# 设置环境变量
source ~/文档/cleanrobot/cleanrobot/install/setup.bash

# 启动雷达节点（ldlidar）
echo "Starting LiDAR..."
ros2 launch ldlidar ld14p.launch.py &

# 等待一会儿，确保雷达节点成功启动
sleep 5

# 启动 SLAM Toolbox（在线异步模式）
echo "Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/cat/文档/cleanrobot/cleanrobot/config/ld14_slam.yaml &

# 启动虚拟里程计（odom -> base_link）
echo "Publishing static transform odom -> base_link..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &

# 启动 Nav2（导航）
echo "Starting Nav2..."
ros2 launch nav2_bringup navigation_launch.py map:=/home/cat/文档/cleanrobot/cleanrobot/maps/my_map.yaml use_sim_time:=false &

# 启动 RViz
echo "Starting RViz2..."
rviz2 &


