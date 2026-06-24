# Atmospheric Lidar Pollution

这个仓库现在包含一套最小可运行的颗粒物大气激光雷达算法骨架，主链已经同步落了一版 C++ 实现，覆盖以下链路：

- 回波模拟
- L0 到 L2 预处理
- Fernald 近似反演
- 湿度修正
- PM2.5 和 PM10 标定
- 热点检测
- ENU 坐标映射
- 评测、消融和敏感性分析
- 静态 Demo 资产生成

## C++ 主入口

C++ 代码位于 `cpp/`，构建入口位于仓库根目录的 `CMakeLists.txt`。

如果本机有 CMake 和 C++20 编译器，可以使用下面的命令：

```powershell
cmake -S . -B build
cmake --build build --config Release
```

常用入口：

```powershell
build/lidar_build_demo_assets --config configs/default_pipeline.json --output-root .
build/lidar_run_batch --config configs/default_pipeline.json --output .
build/lidar_api_server --config configs/default_pipeline.json --once
build/lidar_api_server --config configs/default_pipeline.json --host 127.0.0.1 --port 8765
build/lidar_fetch_public_ground_data --output-root .
build/lidar_fetch_cloudnet_public_sample --config configs/cloudnet_hybrid_pipeline.json --output-root .
```

说明：当前 C++ 端已覆盖默认 simulation 主链、批处理、静态 Demo、live HTTP API、Open-Meteo 公开样例抓取，以及基于 NetCDF 的 Cloudnet hybrid 本地读取路径。要跑 `configs/cloudnet_hybrid_pipeline.json`，需要在编译时链接 NetCDF；在 Windows C++ 构建下，可以先用 `lidar_fetch_cloudnet_public_sample` 抓取样例，也可以让 Cloudnet loader 在缺文件时自动下载并生成对齐后的 Open-Meteo 资产。Python 抓取脚本和 Python API 仍保留作参考实现。

## Python 参考入口

1. 生成样例数据、L1 和 L2 结果、静态 Demo 页面：

```powershell
python scripts/build_demo_assets.py
```

1. 运行单元测试：

```powershell
python -m unittest discover tests -v
```

1. 启动参考版 Python API：

```powershell
python services/api/server.py --once
```

## 主要目录

- `cpp/` C++ 版本算法主链和入口程序
- `configs/` 默认配置
- `lidar_core/` Python 参考实现 (与 C++ 主链等价)
- `services/` API 和批处理入口 (Python)
- `scripts/` 复现实验和 Demo 资产脚本
- `tests/` 单元测试

## 第 19 章教学算例 (`cpp/examples/`)

`cpp/examples/` 目录里是与《激光颗粒物监测系统入门手册》第 19 章一一对应的教学算例,所有算法用纯 C++ 实现,不依赖 lidar_demo 库,便于学习者按章节阅读。每个程序都会:

- 自己生成逼真假数据 (用物理模型正向仿真, 含 Gaussian 羽流、LiDAR 方程、射击噪声)
- 完整实现每一步算法 (预处理 -> RCS -> Klett 反演 -> 湿度修正 -> PM 标定 -> 阈值 -> ENU -> 热点)
- 在终端打印每一步中间结果, 并输出 JSON 到 `data/examples/`

### 一键运行 (4 个 stage 串联)

```powershell
cmake -S . -B build
cmake --build build --config Release

# 一键运行 4 个 stage (一维 + PPI + RHI + 完整 pipeline)
build\lidar_run_full_demo

# 或分别运行
build\lidar_example_1d_ray
build\lidar_example_2d_ppi
build\lidar_example_3d_rhi
```

| 程序 | 对应章节 | 演示内容 |
|---|---|---|
| `lidar_example_1d_ray` | 19.1-19.15 | 单条射线: 从原始 photon counts 到热点告警 (13 步算法) |
| `lidar_example_2d_ppi` | 19.17-19.27 | 二维 PPI: 水平扫描面 + 8 邻域连通域 + 喷雾目标角 |
| `lidar_example_3d_rhi` | 19.28-19.37 | 二维 RHI: 垂直剖面 + 层顶层底 + 羽流抬升判断 |
| `lidar_run_full_demo` | 全部 | 串联上述 3 个 + 调用完整 pipeline |

输出文件:

- `data/examples/1d_ray_result.json` — 一维射线每一步中间量 + 热点事件
- `data/examples/2d_ppi_result.json` — 二维 PM 矩阵 + mask + 连通域 + 质心 + 目标角
- `data/examples/3d_rhi_result.json` — RHI 剖面 + 层底层顶 + 剖面质心 + 形态描述

预期结果 (与文档第 19 章给定值一致):
- 一维射线 120m 处 PM2.5 峰值约 67 µg/m³
- PPI 二维热点质心方位角 = 40.4°, 仰角 = 10.0°
- RHI 层底 = 26.4m, 层顶 = 51.1m, 厚度 = 24.7m

## 三层实时监测架构 (`cpp/server/`, `cpp/client/`, `cpp/protocol/`)

在原有批处理主链之上，仓库新增了一套三层实时监测架构，覆盖"仿真雷达 → 主控客户端 → UI 显示"的完整闭环。

### 架构概览

```
┌──────────────────┐    TCP (JSON line)    ┌──────────────────────────────┐
│  lidar_sim_server │ ────────────────────> │  lidar_control_client         │
│  (仿真雷达服务端)  │  status / raw / hb   │  (主控客户端)                  │
│                  │                       │  ┌────────────────────────┐  │
│  · SimDevice     │                       │  │ FrameProcessor         │  │
│  · TcpServer     │                       │  │ (预处理+反演+热点检测)  │  │
│                  │                       │  ├────────────────────────┤  │
└──────────────────┘                       │  │ HotspotTracker         │  │
                                           │  │ (跨步骤事件追踪)       │  │
                                           │  ├────────────────────────┤  │
                                           │  │ DisposalLinkage        │  │
                                           │  │ (处置设备联动)         │  │
                                           │  ├────────────────────────┤  │
                                           │  │ ReportGenerator        │  │
                                           │  │ (报表生成)             │  │
                                           │  └────────────────────────┘  │
                                           └──────────────────────────────┘
```

### 模块说明

| 目录 | 内容 |
|---|---|
| `cpp/core/` | `lidar_core` 桥接库（alias 到 `lidar_demo`），避免重复代码 |
| `cpp/protocol/` | JSON line 协议：帧类型、序列化/反序列化、profile↔JSON 转换 |
| `cpp/server/` | 仿真雷达服务端：`SimDevice` 模拟 PPI 扫描 + `TcpServer` 流式推送 |
| `cpp/client/` | 主控客户端：帧处理、TCP 通信、闭环管理（告警/追踪/处置/报表） |
| `cpp/client/ui/` | Qt6 GUI（可选，需 `-DLIDAR_ENABLE_QT=ON`） |

### 快速运行（端到端）

```bash
# 构建
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 1. 启动仿真服务器（端口 19850，10 秒间隔）
./build/cpp/server/lidar_sim_server 19850 10 &

# 2. 启动主控客户端（自动处理所有步骤并生成报表）
./build/cpp/client/lidar_control_client 127.0.0.1 19850 /tmp/output

# 生成的文件：
#   /tmp/output/final_report.json   — JSON 格式完整报表
#   /tmp/output/final_report.txt    — 文本格式报表
#   /tmp/output/step_*.json         — 每步处理结果
```

### 闭环管理组件

| 组件 | 功能 |
|---|---|
| **AlarmStateMachine** | 告警状态机：candidate → confirmed → active → mitigating → resolved |
| **HotspotTracker** | 跨步骤热点关联：贪心最近邻空间匹配（ENU 距离阈值） |
| **DisposalLinkage** | 处置设备联动：自动注册设备、PM2.5 超阈值触发、事件解除时停机 |
| **ReportGenerator** | 报表生成：步骤时间线、事件详情、处置摘要，支持 JSON + 文本 |

### Qt6 GUI（可选）

GUI 客户端在安装了 Qt6 的环境下可用：

```bash
# 安装 Qt6（Ubuntu/Debian）
sudo apt install qt6-base-dev

# 构建
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DLIDAR_ENABLE_QT=ON
cmake --build build -j$(nproc)

# 运行 GUI
./build/cpp/client/lidar_gui
```

GUI 功能：
- PPI 热力图实时显示（jet 色表 + 距离环 + 热点标注）
- 告警事件列表 + 状态变更历史详情
- 处置设备状态联动显示
- 一键生成并保存报表

## 补充文档

- `docs/project_supplement.md` 六项补充内容的正式落地说明
- `docs/12_week_plan.md` 12 周落地计划
- `docs/resume_project_entry.md` 简历项目版本
- `data/public/README.md` 公开数据资产说明
