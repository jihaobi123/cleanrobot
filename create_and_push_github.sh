#!/bin/bash
# 使用 GitHub CLI 创建仓库并推送代码

set -e

echo "=========================================="
echo "使用 GitHub CLI 创建仓库并上传"
echo "=========================================="

# 检查是否安装了 GitHub CLI
if ! command -v gh &> /dev/null; then
    echo "错误: 未安装 GitHub CLI"
    echo ""
    echo "请先运行安装脚本:"
    echo "  ./install_github_cli.sh"
    echo ""
    echo "或手动安装: https://cli.github.com/"
    exit 1
fi

# 检查是否已登录
if ! gh auth status &> /dev/null; then
    echo "未登录 GitHub，开始登录..."
    echo ""
    gh auth login
    echo ""
fi

# 显示当前登录状态
echo "当前 GitHub 登录状态:"
gh auth status
echo ""

# 获取仓库名称
read -p "请输入仓库名称 (默认: cleaner_robot_ws): " REPO_NAME
REPO_NAME=${REPO_NAME:-cleaner_robot_ws}

# 获取仓库描述
read -p "请输入仓库描述 (默认: ROS2 Humble cleaner robot workspace): " REPO_DESC
REPO_DESC=${REPO_DESC:-"ROS2 Humble cleaner robot workspace"}

# 选择可见性
echo ""
echo "选择仓库可见性:"
echo "1) Public (公开)"
echo "2) Private (私有)"
read -p "请选择 (1 或 2, 默认: 1): " VISIBILITY_CHOICE
VISIBILITY_CHOICE=${VISIBILITY_CHOICE:-1}

if [ "$VISIBILITY_CHOICE" == "2" ]; then
    VISIBILITY="--private"
else
    VISIBILITY="--public"
fi

# 确认创建
echo ""
echo "=========================================="
echo "仓库信息:"
echo "  名称: $REPO_NAME"
echo "  描述: $REPO_DESC"
echo "  可见性: $([ "$VISIBILITY" == "--private" ] && echo "私有" || echo "公开")"
echo "=========================================="
echo ""
read -p "确认创建仓库? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "已取消"
    exit 0
fi

# 创建仓库
echo ""
echo "正在创建 GitHub 仓库..."
gh repo create "$REPO_NAME" --description "$REPO_DESC" $VISIBILITY --source=. --remote=origin --push

echo ""
echo "=========================================="
echo "✅ 仓库创建并推送成功！"
echo "=========================================="
echo ""
echo "仓库地址: https://github.com/$(gh api user --jq .login)/$REPO_NAME"
echo ""
echo "查看仓库:"
echo "  gh repo view --web"
echo ""

