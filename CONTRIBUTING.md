# Contributing

感谢您对 AStarFootstepPlanner 的贡献！

## 开发环境配置

1. Fork 并克隆仓库
```bash
git clone https://github.com/your-username/AStarFootstepPlanner.git
cd AStarFootstepPlanner
```

2. 安装依赖（见 README）
3. 构建项目进行测试
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## 代码风格

- 使用 `.clang-format` 进行格式化（基于 Google 风格）
- 缩进宽度：3 个空格
- C++17 标准
- 命名空间：`ljh::path::footstep_planner`

格式化命令：
```bash
clang-format -i src/**/*.cpp include/**/*.h
```

## 提交规范

使用 [Conventional Commits](https://conventionalcommits.org/) 格式：

```
<type>(<scope>): <description>
```

类型：`feat` `fix` `docs` `style` `refactor` `test` `chore`

示例：
```
feat(planner): add early termination for timeout
docs(readme): update build instructions
fix(constraint): correct stair boundary check
```

## Pull Request 流程

1. 从 `main` 创建功能分支：`git checkout -b feature/your-feature`
2. 提交更改并推送到您的 Fork
3. 创建 PR 到 `main` 分支
4. 确保 CI 检查通过
5. 等待代码审查和合并

## Issue 报告

报告问题时请包含：
- 操作系统和版本
- 编译器/CMake 版本
- 复现步骤
- 预期行为 vs 实际行为
- 相关日志或错误信息

---

有疑问？欢迎通过 GitHub Issue 讨论！
