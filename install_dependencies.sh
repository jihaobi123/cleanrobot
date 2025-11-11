#!/bin/bash
# 依赖安装脚本

set -e

echo "=========================================="
echo "安装 Cleaner Robot Workspace 依赖"
echo "=========================================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 未检测到ROS2环境，请先source ROS2安装文件"
    echo "例如: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "检测到ROS2版本: $ROS_DISTRO"

# 安装系统依赖
echo ""
echo "安装系统依赖..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-std-msgs

# 安装Python依赖
echo ""
echo "安装Python依赖..."
pip3 install --user pyserial opencv-python numpy

# 检查是否在RK3588平台
echo ""
echo "检查RK3588平台..."
if command -v rknn_server &> /dev/null || [ -f "/usr/lib/librknn_runtime.so" ]; then
    echo "检测到RK3588平台，安装RKNN工具包..."
    pip3 install --user rknn-toolkit-lite
else
    echo "未检测到RK3588平台，跳过RKNN工具包安装"
    echo "注意: 视觉检测功能需要RKNN工具包才能在NPU上运行"
fi

echo ""
echo "=========================================="
echo "依赖安装完成！"
echo "=========================================="

