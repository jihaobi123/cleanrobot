# 使用 GitHub CLI 创建仓库并上传

## 方法一：使用 GitHub CLI（推荐，无需手动创建仓库）

GitHub CLI (`gh`) 允许你在命令行直接创建仓库并上传代码，无需在网站上手动操作。

### 1. 安装 GitHub CLI

#### macOS
```bash
# 使用 Homebrew
brew install gh

# 或运行安装脚本
./install_github_cli.sh
```

#### Linux (Ubuntu/Debian)
```bash
# 使用 apt
sudo apt update
sudo apt install -y gh

# 或运行安装脚本
./install_github_cli.sh
```

#### 其他系统
访问: https://cli.github.com/

### 2. 登录 GitHub

```bash
# 交互式登录
gh auth login

# 按照提示选择:
# - GitHub.com
# - HTTPS
# - 使用浏览器登录或使用 token
```

### 3. 创建仓库并上传（一键完成）

```bash
# 运行自动创建和上传脚本
./create_and_push_github.sh
```

脚本会提示你输入：
- 仓库名称（默认: cleaner_robot_ws）
- 仓库描述
- 可见性（公开/私有）

然后自动：
1. 创建 GitHub 仓库
2. 添加远程仓库
3. 推送所有代码

### 4. 手动使用 GitHub CLI 创建仓库

如果你喜欢手动控制，可以使用以下命令：

```bash
# 创建公开仓库
gh repo create cleaner_robot_ws \
  --description "ROS2 Humble cleaner robot workspace" \
  --public \
  --source=. \
  --remote=origin \
  --push

# 创建私有仓库
gh repo create cleaner_robot_ws \
  --description "ROS2 Humble cleaner robot workspace" \
  --private \
  --source=. \
  --remote=origin \
  --push
```

### 5. 验证上传

```bash
# 查看仓库信息
gh repo view

# 在浏览器中打开仓库
gh repo view --web

# 查看远程仓库
git remote -v
```

## 方法二：手动在网站上创建（传统方法）

如果你不想使用 GitHub CLI，也可以：

1. 在 GitHub 网站上创建仓库
2. 添加远程仓库
3. 推送代码

详细步骤请参考 `PUSH_TO_GITHUB.md`

## GitHub CLI 常用命令

### 认证相关
```bash
# 登录
gh auth login

# 查看登录状态
gh auth status

# 登出
gh auth logout

# 刷新 token
gh auth refresh
```

### 仓库相关
```bash
# 创建仓库
gh repo create <name> --public --source=. --remote=origin --push

# 查看仓库
gh repo view

# 在浏览器中打开仓库
gh repo view --web

# 克隆仓库
gh repo clone <owner>/<repo>

# 列出仓库
gh repo list
```

### 其他有用的命令
```bash
# 查看 GitHub 用户信息
gh api user

# 查看 GitHub 用户名
gh api user --jq .login

# 创建 Issue
gh issue create --title "标题" --body "内容"

# 创建 Pull Request
gh pr create --title "标题" --body "内容"
```

## 优势对比

### 使用 GitHub CLI
✅ 无需打开浏览器  
✅ 一键创建并上传  
✅ 可以在脚本中自动化  
✅ 支持批量操作  

### 手动创建
✅ 可视化界面  
✅ 可以设置更多选项  
✅ 适合不熟悉命令行的用户  

## 故障排查

### 问题1: 未安装 GitHub CLI
```bash
# 检查是否安装
which gh

# 安装
./install_github_cli.sh
```

### 问题2: 未登录
```bash
# 检查登录状态
gh auth status

# 登录
gh auth login
```

### 问题3: 权限不足
```bash
# 检查 token 权限
gh auth status

# 重新登录并授权
gh auth logout
gh auth login
```

### 问题4: 仓库已存在
```bash
# 如果仓库已存在，可以直接添加远程仓库
git remote add origin https://github.com/YOUR_USERNAME/REPO_NAME.git
git push -u origin main
```

## 参考文档

- GitHub CLI 官方文档: https://cli.github.com/manual/
- GitHub CLI GitHub 仓库: https://github.com/cli/cli

