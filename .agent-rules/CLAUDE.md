# C++ 项目规范 — Agent 必读

**适用**: 所有 C++ 代码项目
**前置**: 必须先读 `git/WORKFLOW-RULES.md`

---

## 🎨 代码格式化

### .clang-format 检查

进入项目后，**首先检查**是否存在 `.clang-format`：

```bash
# 检查是否存在
cat .clang-format 2>/dev/null || echo "MISSING"
```

**如果存在**: 每次修改代码后运行格式化：
```bash
# 格式化单个文件
clang-format -i src/example.cpp

# 格式化所有源文件
find src/ include/ -name "*.cpp" -o -name "*.h" -o -name "*.hpp" | xargs clang-format -i
```

**如果不存在**: 使用默认模板（见下方）或从项目根目录复制。

### 推荐 .clang-format 模板

```yaml
---
Language: Cpp
BasedOnStyle: Google
ColumnLimit: 120
IndentWidth: 4
BreakBeforeBraces: Allman
SortIncludes: Never
NamespaceIndentation: All
```

> **注意**: 不同项目风格不同，以项目现有 `.clang-format` 为准。新项目可参考上方模板。

---

## 📝 Copyright Header

### 格式

**C++ (.h, .cpp)**:
```cpp
// Copyright 2026 Junhang Lai
// SPDX-License-Identifier: Apache-2.0
```

**Python (.py)**:
```python
# Copyright 2026 Junhang Lai
# SPDX-License-Identifier: Apache-2.0
```

### 检查规则

Agent 在创建或修改文件后，**必须检查**：
```bash
# 找出没有 copyright header 的源文件
find src/ include/ demo/ test/ \( -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) -exec sh -c 'head -1 "{}" | grep -q "Copyright" || echo "{}"' \;
```

**如有遗漏**: 立即补上，不要等到 commit 时才发现。

---

## 🔨 构建系统（CMake）

### 常见 CMake 项目结构
```
project/
├── CMakeLists.txt
├── src/
├── include/
├── test/
├── demo/          (可选)
├── build/         (❌ 不提交)
├── build-*/       (❌ 不提交)
└── external/      (❌ 不提交，除非是 vendored 依赖)
```

### .gitignore 必须包含
```gitignore
# Build directories
build/
build-*/
cmake-build-*/
out/

# External dependencies (if fetched by CMake)
external/

# Compiled objects
*.o
*.obj
*.so
*.dylib
*.dll
*.a
*.lib

# Generated files
*.csv
*.png          # demo frames (如果适用)
*.gif          # demo animations (如果适用)
```

### 本地编译验证

提交前（尤其是修改了 CMakeLists.txt 或头文件后）：
```bash
# 清理并重新编译
rm -rf build-test
cmake -B build-test -DCMAKE_BUILD_TYPE=Debug
cmake --build build-test

# 运行测试（如有）
cd build-test && ctest --output-on-failure

# 清理
rm -rf build-test
```

---

## 🧪 测试规范

- 测试文件放在 `test/` 目录
- 每次修改核心逻辑后，运行测试确认没有回归
- CI 通常会在三个平台（Ubuntu/macOS/Windows）上运行，本地至少验证 Linux

---

## 📦 依赖管理

### FetchContent / ExternalProject
- 被 fetch 的依赖会放在 `build/_deps/` 下，**不提交**
- 如果项目 vendored 了依赖（`external/`），确认 `.gitignore` 策略

### find_package
- 不影响仓库结构，依赖在系统中

---

## ⚠️ C++ 项目 Agent 检查清单

- [ ] `.clang-format` 存在且已配置
- [ ] 所有源文件有 copyright header
- [ ] `.gitignore` 包含 `build-*/` 和 `external/`
- [ ] 没有 `.o`/`.so`/`.a` 等编译产物被 staged
- [ ] 本地编译通过（如有 CMake）
- [ ] 测试通过（如有 test/）

---

*创建: 2026-03-27*
*参考项目: AStarFootstepPlanner, Heuclid*
