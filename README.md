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

### 方式一：Makefile（推荐）

项目根目录提供了 `Makefile`，封装了最常用的编译和运行命令，首次运行会自动执行 cmake 配置：

```bash
make            # 编译全部目标 → bin/
make server     # 只编译仿真服务器     → bin/lidar_sim_server
make client     # 只编译主控客户端     → bin/lidar_control_client
make examples   # 编译教学算例         → bin/
make test       # 编译并运行测试
make run-server # 编译并启动服务器
make run-client # 编译并启动客户端
make clean      # 清理 build/
make pristine   # 清理 build/ + bin/
make list       # 列出所有可用目标
```

所有可执行文件统一输出到 `bin/` 目录。执行 `make list` 可查看全部目标。

### 方式二：直接 cmake

如果本机有 CMake 和 C++20 编译器，也可以直接用 cmake：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

常用入口（编译后位于 `bin/`）：

```bash
./bin/lidar_build_demo_assets --config configs/DefaultPipeline.json --output-root .
./bin/lidar_run_batch --config configs/DefaultPipeline.json --output .
./bin/lidar_api_server --config configs/DefaultPipeline.json --once
./bin/lidar_api_server --config configs/DefaultPipeline.json --host 127.0.0.1 --port 8765
./bin/lidar_fetch_public_ground_data --output-root .
./bin/lidar_fetch_cloudnet_public_sample --config configs/CloudnetHybridPipeline.json --output-root .
```

说明：当前 C++ 端已覆盖默认 simulation 主链、批处理、静态 Demo、live HTTP API、Open-Meteo 公开样例抓取，以及基于 NetCDF 的 Cloudnet hybrid 本地读取路径。要跑 `configs/CloudnetHybridPipeline.json`，需要在编译时链接 NetCDF；在 Windows C++ 构建下，可以先用 `lidar_fetch_cloudnet_public_sample` 抓取样例，也可以让 Cloudnet loader 在缺文件时自动下载并生成对齐后的 Open-Meteo 资产。Python 抓取脚本和 Python API 仍保留作参考实现。

面向工地、城市固定站和走航车的逼真化仿真配置：

```bash
./bin/lidar_run_batch --config configs/CommercialPmeyeSector.json --output .
./bin/lidar_run_batch --config configs/FieldScanningLidar.json --output .
./bin/lidar_run_batch --config configs/MobileMappingLidar.json --output .
./bin/lidar_build_demo_assets --config configs/FieldScanningLidar.json --output-root .
```

其中 `CommercialPmeyeSector.json` 用于优先还原商用 PM 扫描 LiDAR 的观测节奏：355 nm 弹性通道、20 Hz PRF、5 s 单视线积分（约 100 shots）、0-180° 扇区、2.5° 方位步进、约 365 s 一轮扇区扫描。其他配置启用更接近现场设备的参数：mJ 级脉冲能量、37.5/30 m 距离门、多仰角体扫、full-overlap、太阳背景、死时间/饱和、风驱动烟羽和 L3 `volume` 体素产品。设计说明见 `docs/realistic_lidar_simulation.md`。

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
- `scripts/` 复现实验和 Demo 资产脚本 (Python)
- `tests/` 单元测试
- `bin/` 编译产物输出目录（.gitignore 已忽略）
- `Makefile` 便捷编译入口（`make server` / `make client` / ...）

## 第 19 章教学算例 (`cpp/examples/`)

`cpp/examples/` 目录里是与《激光颗粒物监测系统入门手册》第 19 章一一对应的教学算例,所有算法用纯 C++ 实现,不依赖 lidar_demo 库,便于学习者按章节阅读。每个程序都会:

- 自己生成逼真假数据 (用物理模型正向仿真, 含 Gaussian 羽流、LiDAR 方程、射击噪声)
- 完整实现每一步算法 (预处理 -> RCS -> Klett 反演 -> 湿度修正 -> PM 标定 -> 阈值 -> ENU -> 热点)
- 在终端打印每一步中间结果, 并输出 JSON 到 `data/examples/`

### 一键运行 (4 个 stage 串联)

```bash
make examples   # 编译教学算例 → bin/

# 一键运行 4 个 stage (一维 + PPI + RHI + 完整 pipeline)
./bin/lidar_run_full_demo

# 或分别运行
./bin/lidar_example_1d_ray
./bin/lidar_example_2d_ppi
./bin/lidar_example_3d_rhi
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
# 构建（两种方式任选其一）
make server client      # Makefile 方式，输出到 bin/
# cmake --build build -j$(nproc)   # 或直接 cmake

# 1. 启动仿真服务器（零参数，默认端口 19850，帧间 50ms，步间 500ms）
./bin/lidar_sim_server &

# 2. 启动主控客户端（零参数，自动连接 127.0.0.1:19850）
./bin/lidar_control_client

# 生成的文件（默认输出到 data/client_output/）：
#   data/client_output/final_report.json   — JSON 格式完整报表
#   data/client_output/final_report.txt    — 文本格式报表
#   data/client_output/step_*.json         — 每步处理结果
```

两个程序的默认参数完全对齐，**不传任何参数即可运行**。

可选参数：

| 程序 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| `lidar_sim_server` | `[port] [step_delay_ms] [playback_time_scale]` | `19850 500 100` | 监听端口、时间步间隔、真实采集节奏播放倍率 |
| `lidar_control_client` | `[host] [port] [output_dir]` | `127.0.0.1 19850 data/client_output` | 连接地址、端口、输出目录 |

`playback_time_scale=1` 表示按设备真实积分节奏推送；默认 `100` 会把 5 s 单视线积分压缩为约 50 ms，便于本地演示。

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
./bin/lidar_gui
```

GUI 功能：
- PPI 热力图实时显示（jet 色表 + 距离环 + 热点标注）
- 告警事件列表 + 状态变更历史详情
- 处置设备状态联动显示
- 一键生成并保存报表
- C++ demo payload 额外输出 L3 `volume` 体素字段，可用于后续 3D 污染云显示

## 补充文档

- `docs/project_supplement.md` 六项补充内容的正式落地说明
- `docs/12_week_plan.md` 12 周落地计划
- `docs/resume_project_entry.md` 简历项目版本
- `data/public/README.md` 公开数据资产说明
