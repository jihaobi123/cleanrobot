# 快速上传到 GitHub

## 方法一：使用 HTTPS（推荐新手）

```bash
# 1. 添加远程仓库（替换 YOUR_USERNAME 和 YOUR_REPO_NAME）
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git

# 2. 推送到 GitHub
git push -u origin main
```

## 方法二：使用 SSH（如果已配置 SSH 密钥）

```bash
# 1. 添加远程仓库（替换 YOUR_USERNAME 和 YOUR_REPO_NAME）
git remote add origin git@github.com:YOUR_USERNAME/YOUR_REPO_NAME.git

# 2. 推送到 GitHub
git push -u origin main
```

## 方法三：使用上传脚本

```bash
# 1. 先添加远程仓库
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git

# 2. 运行上传脚本
./UPLOAD_TO_GITHUB.sh
```

## 验证上传

上传完成后，访问你的 GitHub 仓库页面，应该能看到所有文件。

## 后续更新

```bash
# 添加更改
git add .

# 提交更改
git commit -m "描述你的更改"

# 推送到 GitHub
git push
```

## 注意事项

1. **大文件**: RKNN 模型文件（`.rknn`）已在 `.gitignore` 中排除，不会上传
2. **敏感信息**: 确保配置文件中没有硬编码的密码或密钥
3. **分支管理**: 主分支名为 `main`，如果 GitHub 使用 `master`，可能需要重命名

## 如果遇到问题

### 问题1: 远程仓库已存在文件
```bash
# 先拉取远程文件
git pull origin main --allow-unrelated-histories
# 解决冲突后
git push -u origin main
```

### 问题2: 认证失败
- HTTPS: 需要使用 Personal Access Token（不是密码）
- SSH: 确保 SSH 密钥已添加到 GitHub

### 问题3: 推送被拒绝
```bash
# 检查远程仓库
git remote -v

# 如果需要强制推送（谨慎使用）
git push -u origin main --force
```

## 查看远程仓库

```bash
# 查看远程仓库地址
git remote -v

# 查看分支
git branch -a
```

