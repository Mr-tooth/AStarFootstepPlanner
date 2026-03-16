# AStarFootstepPlanner

> 基于 A* 算法的人形机器人落脚点规划器 C++ 实现，
> 灵感来源于 [IHMC Footstep Planning](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/develop/ihmc-footstep-planning)。

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C%2B%2B-11-blue.svg)]()
[![CMake](https://img.shields.io/badge/CMake-3.22+-blue.svg)]()

## 概述

AStarFootstepPlanner 是一个用于人形机器人在复杂地形中规划最优落脚点序列的 C++ 库。它实现了 A* 图搜索算法，用于寻找从起始姿态到目标姿态的无碰撞路径。

本库是 IHMC（Indiana Humanoid Motion Control）落脚点规划框架的 C++ 重新实现，原框架使用 Java 开发。它遵循 IEEE Humanoids 2019 论文《Footstep Planning for Autonomous Walking Over Rough Terrain》中描述的算法。

主要功能包括：
- 基于参数的步扩展，支持可配置的步态参数
- 机器人可达性运动学约束检查
- 环境碰撞检测（楼梯、障碍物）
- 可配置的代价函数用于步质量评估
- 可选的基于 matplotlib 的可视化调试

## 功能特性

- **A* 图搜索**：使用 A* 算法规划最优落脚点序列
- **参数化步扩展**：支持可配置步态参数的步生成
- **运动学约束**：机器人可达性和稳定性检查
- **环境感知**：楼梯区域和障碍物碰撞检测
- **灵活代价函数**：距离、偏航角、步态转换代价
- **可视化**：基于 matplotlib 的轨迹分析绘图
- **跨平台**：支持 Ubuntu、macOS、Windows (MSVC)

## 依赖项

| 依赖 | 版本 | 必需 | 说明 |
|-----------|---------|----------|-------------|
| [Heuclid](https://github.com/Mr-tooth/Heuclid) | v2.0+ | ✅ | C++ 几何数学库（IHMC Euclid 移植版） |
| [Eigen3](https://eigen.tuxfamily.org/) | 3.x | ✅ | 线性代数库 |
| [LBlocks](https://github.com/hexb66/LBlocks) | — | ✅ | 模块化框架 |
| [matplotlib_cpp](https://github.com/hexb66/matplotlib-cpp) | — | ❌ | 可视化（可选） |

## 编译构建

### 前置条件

**Ubuntu/Debian：**
```bash
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y libeigen3-dev python3 python3-pip
pip3 install matplotlib numpy
```

**macOS：**
```bash
brew install cmake eigen git
brew install python@3.11
pip3 install matplotlib numpy
```

**Windows (MSVC)：**
1. 安装 Visual Studio 2022 并选择 C++ 工作负载
2. 从 python.org 安装 Python 3.8+（勾选"添加到 PATH"）
3. 打开 PowerShell: `pip install matplotlib numpy`
4. 设置环境变量: `Python3_ROOT_DIR=C:\Path\To\Python`

### 编译步骤

```bash
# 克隆仓库（包含子模块）
git clone --recursive https://github.com/Mr-tooth/AStarFootstepPlanner.git
cd AStarFootstepPlanner

# 构建
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/Heuclid/install
cmake --build . -j$(nproc)
```

### CMake 选项

| 选项 | 默认值 | 说明 |
|--------|---------|-------------|
| `BUILD_TESTING` | ON | 编译测试可执行文件 |
| `CMAKE_BUILD_TYPE` | Release | 构建配置 |

如果找不到 `matplotlib_cpp`，请设置 `Python3_ROOT_DIR` 指向您的 Python 安装目录。

## 使用方法

### 基础示例

```cpp
#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/Data/Footstep.h>

using namespace ljh::path::footstep_planner;

int main() {
    // 设置起始姿态和目标姿态 (x, y, z, 偏航角, 俯仰角, 横滚角)
    Pose3D<double> startPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    Pose3D<double> goalPose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    // A* 搜索使用的目标姿态 (2D: x, y, 偏航角)
    Pose2D<double> goalPose2D(2.0, 0.0, 0.0);
    
    // 创建并配置规划器
    AStarFootstepPlanner planner(goalPose2D, goalPose, startPose);
    
    // 运行 A* 搜索
    planner.doAStarSearch();
    
    // 获取结果
    auto footsteps = planner.getAccurateFootstepSeries();
    
    // footsteps 现在包含规划好的落脚点序列
    return 0;
}
```

### 带可视化

```cpp
#include <FootstepPlannerLJH/Check/PlotCheck.h>

// 规划完成后
PlotCheck plotter;
plotter.plotFootsteps(footsteps, "planned_path.png");
```

### 高级配置

```cpp
#include <FootstepPlannerLJH/Parameters/FootstepPlannerParameter.h>

FootstepPlannerParameter params;
params.setIdealFootstepLength(0.3);    // 米
params.setIdealFootstepWidth(0.2);     // 米
params.setMaximumStepYaw(0.3);         // 弧度

planner.setParameters(params);
```

## 架构设计

```
┌─────────────────────────────────────────────────────────┐
│                  AStarFootstepPlanner                   │
│                     (A* 主循环)                          │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────────────┐  │
│  │  基于参数的     │    │    IdealStepCalculator  │  │
│  │  步扩展策略     │    │   (计算理想姿态)         │  │
│  └─────────────────┘    └─────────────────────────┘  │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────────────┐  │
│  │ FootstepCost    │    │   HeuristicCalculator   │  │
│  │   Calculator    │    │     (A* 启发式函数)      │  │
│  └─────────────────┘    └─────────────────────────┘  │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────────────┐  │
│  │ StepConstraint  │    │ FootstepCompletionChecker│  │
│  │    Check        │    │    (目标验证)           │  │
│  └─────────────────┘    └─────────────────────────┘  │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────────────┐  │
│  │  Simple2DBody   │    │      PlotCheck          │  │
│  │   PathHolder    │    │    (可视化模块)          │  │
│  └─────────────────┘    └─────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## API 参考

核心类和接口：

- `AStarFootstepPlanner` - 主规划器类
- `Footstep` - 落脚点状态表示
- `FootstepPlannerParameter` - 配置参数
- `PlotCheck` - 可视化工具
- `Pose3D<T>` / `Pose2D<T>` - 姿态表示（来自 Heuclid）

完整 API 文档请参阅头文件中的 Doxygen 注释，或使用以下命令生成：
```bash
cd build
cmake .. -DBUILD_DOCS=ON
make docs
```

## 路线图

- [ ] **Snap & Wiggle** - 支持 IHMC 的 FootstepSnapAndWiggler 地形适配
- [ ] **Heightmap 支持** - 完整的高度图地形感知
- [ ] **ROS 2 集成** - 原生 ROS 2 节点和消息接口
- [ ] **身体路径规划增强** - 与外部路径规划器集成

## 引用

如果您在学术工作中使用本库，请引用：

```bibtex
@inproceedings{ihmc_footstep_2019,
  title={Footstep Planning for Autonomous Walking Over Rough Terrain},
  author={Dornbush, Alexander and V\'{a}squez, Andr\'{e}s and L\'{e}on, Beatriz and de~las~Heras, Luis and Bergasa, Luis M and Oca\~{n}a, Manuel},
  booktitle={IEEE-RAS International Conference on Humanoid Robots},
  year={2019},
  organization={IEEE}
}
```

## 许可证

Copyright 2024-2025 AStarFootstepPlanner Contributors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---

**English Documentation**: [README.md](README.md)
