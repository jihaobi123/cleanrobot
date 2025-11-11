# GitHub 上传指南

## 1. 初始化 Git 仓库

```bash
cd cleaner_robot_ws
git init
```

## 2. 添加文件到 Git

```bash
# 添加所有文件
git add .

# 查看将要提交的文件
git status
```

## 3. 创建初始提交

```bash
git commit -m "Initial commit: ROS2 Humble cleaner robot workspace

- Add lidar_driver package (LD14P radar driver)
- Add vision_detection package (YOLOv8 + RKNN)
- Add perception_fusion package (sensor fusion)
- Add stm32_interface package (STM32 communication)
- Add cleaner_robot_bringup package (unified launch)
- Add documentation and configuration files"
```

## 4. 在 GitHub 上创建仓库

1. 登录 GitHub
2. 点击右上角的 "+" 按钮，选择 "New repository"
3. 填写仓库信息：
   - Repository name: `cleaner_robot_ws` (或你喜欢的名称)
   - Description: "ROS2 Humble cleaner robot perception and control system for RK3588 platform"
   - Visibility: 选择 Public 或 Private
   - **不要** 初始化 README、.gitignore 或 license（我们已经有了）
4. 点击 "Create repository"

## 5. 连接本地仓库到 GitHub

```bash
# 添加远程仓库（替换 YOUR_USERNAME 和 YOUR_REPO_NAME）
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git

# 或者使用 SSH（如果已配置 SSH 密钥）
# git remote add origin git@github.com:YOUR_USERNAME/YOUR_REPO_NAME.git

# 查看远程仓库
git remote -v
```

## 6. 推送代码到 GitHub

```bash
# 重命名主分支为 main（如果当前是 master）
git branch -M main

# 推送到 GitHub
git push -u origin main
```

## 7. 后续更新

```bash
# 添加更改的文件
git add .

# 提交更改
git commit -m "描述你的更改"

# 推送到 GitHub
git push
```

## 注意事项

### 大文件处理
- RKNN 模型文件（`.rknn`）通常很大，建议：
  1. 不提交到 Git（已在 .gitignore 中排除）
  2. 或使用 Git LFS（Large File Storage）
  3. 或存储在云存储服务中，在 README 中提供下载链接

### 敏感信息
- 不要提交包含密码、API密钥等敏感信息的文件
- 检查配置文件，确保没有硬编码的敏感信息
- 可以使用环境变量或配置文件模板

### 分支管理
- 主分支：`main` 或 `master`（稳定版本）
- 开发分支：`dev` 或 `develop`（开发中）
- 功能分支：`feature/功能名称`（新功能开发）

### 创建开发分支示例
```bash
# 创建并切换到开发分支
git checkout -b dev

# 推送开发分支到 GitHub
git push -u origin dev
```

## 使用 Git LFS 存储大文件（可选）

如果需要在 Git 中存储 RKNN 模型等大文件：

```bash
# 安装 Git LFS
# Ubuntu/Debian:
sudo apt install git-lfs

# macOS:
brew install git-lfs

# 初始化 Git LFS
git lfs install

# 跟踪大文件类型
git lfs track "*.rknn"
git lfs track "*.onnx"

# 提交 .gitattributes 文件
git add .gitattributes
git commit -m "Add Git LFS tracking for model files"
```

## 添加 License（可选）

如果需要添加 License：

```bash
# 创建 LICENSE 文件（例如 MIT License）
# 然后提交
git add LICENSE
git commit -m "Add MIT License"
git push
```

## 添加 GitHub Actions CI/CD（可选）

可以创建 `.github/workflows/` 目录，添加自动化测试和构建流程。

## 常见问题

### 1. 推送被拒绝
```bash
# 如果远程仓库有 README 等文件，需要先拉取
git pull origin main --allow-unrelated-histories
# 解决冲突后
git push -u origin main
```

### 2. 忘记添加 .gitignore
```bash
# 如果已经提交了应该忽略的文件
git rm -r --cached build/ install/ log/
git commit -m "Remove build directories from git"
git push
```

### 3. 更改远程仓库地址
```bash
# 查看当前远程地址
git remote -v

# 更改远程地址
git remote set-url origin https://github.com/NEW_USERNAME/NEW_REPO_NAME.git
```

## 协作开发

### Fork 和 Pull Request
1. 其他开发者可以 Fork 你的仓库
2. 在他们的 Fork 上进行开发
3. 创建 Pull Request 请求合并更改

### 添加协作者
1. 在 GitHub 仓库页面点击 "Settings"
2. 选择 "Collaborators"
3. 添加协作者的 GitHub 用户名

## 参考资料

- [Git 官方文档](https://git-scm.com/doc)
- [GitHub 帮助文档](https://help.github.com/)
- [Git LFS 文档](https://git-lfs.github.com/)

