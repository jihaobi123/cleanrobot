#!/bin/bash
# 构建脚本

set -e

echo "=========================================="
echo "构建 Cleaner Robot Workspace"
echo "=========================================="

# 进入工作空间目录
cd "$(dirname "$0")"

# 构建所有包
echo "开始构建..."
colcon build --symlink-install

echo ""
echo "=========================================="
echo "构建完成！"
echo "=========================================="
echo ""
echo "请运行以下命令设置环境："
echo "  source install/setup.bash"
echo ""

