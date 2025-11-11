#!/bin/bash
# 安装 GitHub CLI 脚本

set -e

echo "=========================================="
echo "安装 GitHub CLI (gh)"
echo "=========================================="

# 检测操作系统
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    echo "检测到 macOS 系统"
    
    if command -v brew &> /dev/null; then
        echo "使用 Homebrew 安装..."
        brew install gh
    else
        echo "错误: 未找到 Homebrew，请先安装 Homebrew"
        echo "访问: https://brew.sh"
        exit 1
    fi
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux
    echo "检测到 Linux 系统"
    
    if command -v apt-get &> /dev/null; then
        # Debian/Ubuntu
        echo "使用 apt 安装..."
        sudo apt update
        sudo apt install -y gh
    elif command -v yum &> /dev/null; then
        # CentOS/RHEL
        echo "使用 yum 安装..."
        sudo yum install -y gh
    elif command -v dnf &> /dev/null; then
        # Fedora
        echo "使用 dnf 安装..."
        sudo dnf install -y gh
    else
        echo "错误: 未找到支持的包管理器"
        echo "请手动安装 GitHub CLI: https://cli.github.com/"
        exit 1
    fi
else
    echo "错误: 不支持的操作系统"
    echo "请手动安装 GitHub CLI: https://cli.github.com/"
    exit 1
fi

echo ""
echo "=========================================="
echo "GitHub CLI 安装完成！"
echo "=========================================="
echo ""
echo "下一步: 运行以下命令登录 GitHub"
echo "  gh auth login"
echo ""

