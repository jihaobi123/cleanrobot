#!/bin/bash
# GitHub 上传脚本

set -e

echo "=========================================="
echo "准备上传到 GitHub"
echo "=========================================="

# 检查是否已设置远程仓库
if git remote -v | grep -q "origin"; then
    echo "检测到远程仓库已配置"
    git remote -v
else
    echo "未检测到远程仓库，请按照以下步骤操作："
    echo ""
    echo "1. 在 GitHub 上创建新仓库"
    echo "2. 运行以下命令添加远程仓库："
    echo "   git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git"
    echo "   或使用 SSH："
    echo "   git remote add origin git@github.com:YOUR_USERNAME/YOUR_REPO_NAME.git"
    echo ""
    exit 1
fi

# 检查当前分支
CURRENT_BRANCH=$(git branch --show-current)
echo "当前分支: $CURRENT_BRANCH"

# 确认是否继续
read -p "是否推送到 GitHub? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "已取消"
    exit 0
fi

# 推送到 GitHub
echo "推送到 GitHub..."
git push -u origin $CURRENT_BRANCH

echo ""
echo "=========================================="
echo "上传完成！"
echo "=========================================="

