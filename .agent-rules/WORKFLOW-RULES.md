# Git 工作流规范 — Agent 必读

**适用**: 所有代码项目中的 Agent 工作
**强制**: Agent 在每次 git commit 前必须执行本规范

---

## 🔴 提交前强制流程（5步）

Agent **必须**在每次 commit 前按顺序执行以下检查：

### Step 1: 预览提交内容
```bash
# 查看哪些文件将被提交
git diff --cached --stat

# 确认具体内容（关键文件）
git diff --cached -- <关键文件>
```
**Agent 必须**: 在心中（或消息中）列出所有将被提交的文件，确认每个文件都应该被提交。

### Step 2: 检查 .gitignore 覆盖
```bash
# 检查是否有不该提交的文件被 staged
git status --ignored

# 验证关键目录被忽略
git check-ignore build/ external/ *.csv .DS_Store
```
**Agent 必须**: 确认以下类型文件**不被提交**：
- 构建目录: `build/`, `build-*/`, `cmake-build-*/`, `out/`
- 依赖缓存: `external/`, `node_modules/`, `__pycache__/`, `.venv/`
- 生成文件: `*.csv`, `*.png` (demo frames), `*.gif`, `*.o`, `*.so`
- IDE 文件: `.vscode/`, `.idea/`, `.vs/`
- 系统文件: `.DS_Store`, `Thumbs.db`, `*.swp`

### Step 3: 代码规范检查
```bash
# C++ 项目: 检查 .clang-format 是否已应用
# (详见 cpp/CLAUDE.md)

# Python 项目: 检查 ruff 是否通过
# (详见 python/CLAUDE.md)
```

### Step 4: Commit Message 规范
格式: `<type>: <description>`

| Type | 用途 |
|------|------|
| `feat` | 新功能 |
| `fix` | Bug 修复 |
| `docs` | 文档变更 |
| `style` | 格式调整（不影响代码逻辑） |
| `refactor` | 重构（不是 fix 也不是 feat） |
| `test` | 添加/修改测试 |
| `chore` | 构建/工具变更 |
| `ci` | CI 配置变更 |

**要求**:
- 使用英文
- 首字母小写（除非专有名词）
- 不以句号结尾
- 描述**做了什么**，不是**为什么**

```
✅ fix: add missing copyright headers to demo files
✅ feat: implement AABB pre-filter for collision detection
✅ chore: update .gitignore to exclude build directories
❌ Fixed some stuff
❌ 修复了版权头问题
❌ 更新了文件
```

### Step 5: 本地验证（推荐）
```bash
# C++ 项目: 本地编译测试
rm -rf build-test && cmake -B build-test && cmake --build build-test && rm -rf build-test

# Python 项目: 运行测试
python -m pytest

# 通用: 检查 CI 配置是否会被触发
cat .github/workflows/*.yml | grep -A2 "on:"
```

---

## 🟡 提交后流程

### Push 前
```bash
# 确认提交成功
git log -1 --oneline

# 确认远程可达（如有代理需求）
export https_proxy=http://127.0.0.1:7890 http_proxy=http://127.0.0.1:7890 all_proxy=http://127.0.0.1:7890
```

### Push 后
```bash
# 确认推送成功
git log origin/main..HEAD --oneline  # 应为空

# 检查 CI 状态（如 gh CLI 可用）
gh run list --limit 3
```

---

## 🟢 分支策略（如适用）

- **main/master**: 仅通过 PR 合并（如果项目有此规则）
- **feature 分支**: `feature/<简短描述>`
- **fix 分支**: `fix/<简短描述>`
- **提交频率**: 每个逻辑单元提交一次，不堆积大量变更

---

## ⚠️ Agent 禁止事项

| 禁止 | 原因 |
|------|------|
| `git add -A` 或 `git add .` | 可能误提交不该提交的文件 |
| `git commit --no-verify` (常规使用) | 跳过 pre-commit hook |
| 一次提交超过 20 个文件 | 应拆分为多个逻辑提交 |
| commit message 用中文 | 国际化项目统一英文 |
| 推送后不检查 CI | 应确认 CI 通过 |

**正确做法**: `git add <明确的文件列表>` — 逐个指定要提交的文件。

---

## 📋 Agent 提交检查清单

每次 commit 前，Agent 应在内心过一遍：

- [ ] 我知道每个被 staged 的文件是什么
- [ ] 没有构建产物、缓存、生成文件
- [ ] 没有 .DS_Store / Thumbs.db 等系统文件
- [ ] 所有源文件有 copyright header（如项目要求）
- [ ] commit message 符合 `<type>: <description>` 格式
- [ ] commit message 是英文
- [ ] 如有 CI，本地已验证编译/测试通过

---

*创建: 2026-03-27*
*教训: AStarFootstepPlanner 版权头遗漏 + 误提交 build 目录导致 CI 全挂*
