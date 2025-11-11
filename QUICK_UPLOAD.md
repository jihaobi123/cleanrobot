# 🚀 快速上传到 GitHub（无需手动创建仓库）

## 三步完成上传

### 步骤 1: 安装 GitHub CLI

```bash
# macOS
brew install gh

# 或运行安装脚本
./install_github_cli.sh
```

### 步骤 2: 登录 GitHub

```bash
gh auth login
```

按照提示选择登录方式（浏览器或 token），完成后会显示登录成功。

### 步骤 3: 一键创建并上传

```bash
./create_and_push_github.sh
```

脚本会提示你输入：
- 仓库名称（默认: cleaner_robot_ws）
- 仓库描述
- 可见性（公开/私有）

然后自动完成：
✅ 创建 GitHub 仓库  
✅ 添加远程仓库  
✅ 推送所有代码  

## 完成！

上传完成后，你可以：

```bash
# 查看仓库信息
gh repo view

# 在浏览器中打开仓库
gh repo view --web
```

## 手动命令（可选）

如果你更喜欢手动控制：

```bash
# 创建公开仓库并推送
gh repo create cleaner_robot_ws \
  --description "ROS2 Humble cleaner robot workspace" \
  --public \
  --source=. \
  --remote=origin \
  --push

# 创建私有仓库并推送
gh repo create cleaner_robot_ws \
  --description "ROS2 Humble cleaner robot workspace" \
  --private \
  --source=. \
  --remote=origin \
  --push
```

## 优势

✅ **无需打开浏览器** - 全部在命令行完成  
✅ **一键操作** - 创建和上传一步完成  
✅ **自动化友好** - 可以在脚本中使用  
✅ **更快速** - 比手动操作更快  

## 需要帮助？

- 详细文档: `GITHUB_CLI_GUIDE.md`
- 快速参考: `PUSH_TO_GITHUB.md`

