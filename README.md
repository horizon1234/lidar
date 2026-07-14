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
make client     # 只编译 Linux Qt 客户端 → bin/lidar_gui
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

客户端仅支持 Linux，并要求 Qt 6.5 或更高版本的 Core、Gui、Widgets 和 Network。
构建脚本会自动选择 `/opt/Qt6*/6.*/gcc_64` 中版本最高的 Qt；也可以显式指定：

```bash
cmake -S . -B build -DLIDAR_QT_ROOT=/opt/Qt6.9.3/6.9.3/gcc_64
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

说明：当前 C++ 端已覆盖默认 simulation 主链、批处理、静态 Demo、live HTTP API、Open-Meteo 公开样例抓取，以及基于 NetCDF 的 Cloudnet hybrid 本地读取路径。要跑 `configs/CloudnetHybridPipeline.json`，需要在编译时链接 NetCDF；可以先用 `lidar_fetch_cloudnet_public_sample` 抓取样例，也可以让 Cloudnet loader 在缺文件时自动下载并生成对齐后的 Open-Meteo 资产。Python 抓取脚本和 Python API 仍保留作参考实现。

面向工地、城市固定站和走航车的逼真化仿真配置：

```bash
./bin/lidar_run_batch --config configs/CommercialPmeyeSector.json --output .
./bin/lidar_run_batch --config configs/FieldScanningLidar.json --output .
./bin/lidar_run_batch --config configs/MobileMappingLidar.json --output .
./bin/lidar_build_demo_assets --config configs/FieldScanningLidar.json --output-root .
```

默认实时服务端专门还原无锡中科光电 `YLJ5 / AGHJ-I-LIDAR(MPL)`：532 nm、3.75 m 距离门、至少 20 km、双望远镜近远场和偏振四通道、同步相机能力，以及采购文件确认的水平扫描、垂直观测和锥形扫描。默认每周期先做 90 度垂直观测，再让 180 条方位射线按 0/5 度逐周期轮换；5 度和自动轮换属于有论文依据、待实机确认的仿真假设。服务端按扫描周期惰性生成，避免预缓存完整战役造成数十 GB 内存占用。PRF、脉冲能量、系统常数、接收增益和厂商私有协议同样明确标为假设或未知。证据分级和剩余差距见 `docs/ylj5_fidelity.md`。

`FieldScanningLidar.json`、`MobileMappingLidar.json` 和 `CommercialPmeyeSector.json` 仍是批处理算法的通用场景配置，不代表 YLJ5 厂家参数。其中 `CommercialPmeyeSector.json` 保留 PMeye-like 355 nm 参考方案。通用仿真设计说明见 `docs/realistic_lidar_simulation.md`。

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

在原有批处理主链之上，仓库提供“仿真雷达 → Linux Qt 主控客户端”的实时路径。客户端
不再提供无头模式；socket、JSONL 拆帧、设备状态合并、四通道处理和反演全部在专用
`QThread` 中执行，GUI 主线程只渲染不可变结果快照。

### 架构概览

```
┌──────────────────┐       TCP JSONL       ┌──────────────────────────────────┐
│ lidar_sim_server │ ────────────────────> │ lidar_gui                        │
│ (YLJ5 仿真服务端) │ status / telemetry /  │                                  │
│                  │ lidar_raw / hb        │ QThread:                         │
│                  │ <──────────────────── │ QTcpSocket -> 协议解析 -> 状态合并 │
│                  │ command              │ -> 四通道拼接 -> Fernald -> PM门控 │
│  SimDevice       │                       │                                  │
│  TcpServer       │                       │ GUI: PPI / 垂直廓线 / 热点与 QC    │
└──────────────────┘                       └──────────────────────────────────┘
```

### 模块说明

| 目录 | 内容 |
|---|---|
| `cpp/core/` | `lidar_core` 桥接库（alias 到 `lidar_demo`），避免重复代码 |
| `cpp/protocol/` | JSONL 协议：状态、遥测、四通道原始帧、相机/产品、控制结果和序列化 |
| `cpp/server/` | YLJ5 仿真服务端：公开规格、四接收通道、惰性扫描、控制状态机和 TCP 推送 |
| `cpp/client/` | Linux Qt 主控客户端：工作线程网络/处理、设备状态、PPI、垂直廓线和 QC |

### 快速运行（端到端）

```bash
# 构建，Qt 会从 /opt 自动发现
make server client

# 1. 启动仿真服务器（零参数，默认端口 19850，按真实扫描节奏）
./bin/lidar_sim_server &

# 2. 启动 GUI，并自动连接默认地址
./bin/lidar_gui --connect
```

不传 `--connect` 时，GUI 会先打开，操作员可在工具栏修改地址后连接。

可选参数：

| 程序 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| `lidar_sim_server` | `[port] [cycle_delay_ms] [playback_time_scale]` | `19850 0 1` | 监听端口、额外周期间隔、配置采集节奏的播放倍率 |
| `lidar_gui` | `--host HOST --port PORT --connect` | `127.0.0.1 19850` | 连接地址、端口和是否自动连接 |

`playback_time_scale=1` 表示按配置的采集节奏推送。当前每周期先做 30 s、90 度垂直观测，再按约 2.2 s/条推送 180 条固定仰角 profile：偶数周期为 0 度水平扫描，奇数周期为 5 度锥形扫描，单周期约 436 s。需要快速演示时，可手动传入 `100`；该倍率只缩短播放等待，不改变回波物理积分和 JSON 数据量。

控制端可发送 `start`、`resume`、`pause`、`stop`、`get_status` 和 `set_scan`。服务端统一返回 `command_result`；若新扫描计划低于公开采购边界，响应会包含具体 `violations`，当前配置不会被部分修改。协议字段和证据边界见 `docs/ylj5_fidelity.md`。

### 闭环管理组件

| 组件 | 功能 |
|---|---|
| **AlarmStateMachine** | 告警状态机：candidate → confirmed → active → mitigating → resolved |
| **HotspotTracker** | 跨步骤热点关联：贪心最近邻空间匹配（ENU 距离阈值） |
| **DisposalLinkage** | 处置设备联动：自动注册设备、PM2.5 超阈值触发、事件解除时停机 |
| **ReportGenerator** | 报表生成：步骤时间线、事件详情、处置摘要，支持 JSON + 文本 |

### Linux Qt6 GUI

当前主控客户端只有 GUI 模式。项目不再定义 `LIDAR_ENABLE_QT`，Qt 是客户端必需依赖：

```bash
# 安装 Qt6（Ubuntu/Debian）
sudo apt install qt6-base-dev

# 构建；本机 Qt 位于 /opt
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DLIDAR_QT_ROOT=/opt/Qt6.9.3/6.9.3/gcc_64
cmake --build build -j$(nproc)

# 运行 GUI
./bin/lidar_gui --host 127.0.0.1 --port 19850 --connect
```

GUI 功能：

- 工具栏连接、断开、开始、暂停和停止设备扫描；
- 显示设备型号、扫描程序、活动仰角、云台指向和传输统计；
- 未加载站点标定时显示干消光、退偏比和 QC，不生成伪 PM 或浓度告警；
- 加载有效 PM 标定 JSON 后，显示定量 PM2.5 PPI 和热点；
- 垂直观测页显示最近一条天顶干消光与退偏比廓线。

标定 JSON 必须包含标定 ID、生效时间和 PM2.5/PM10 的线性系数：

```json
{
  "calibration_id": "site-cal-001",
  "valid_from": "2026-07-01",
  "pm25_intercept_ugm3": 0.0,
  "pm25_slope_ugm3_per_km": 0.0,
  "pm10_intercept_ugm3": 0.0,
  "pm10_slope_ugm3_per_km": 0.0
}
```

系数必须来自目标站点与参考颗粒物仪器的共址比对；示例中的零值只是结构说明。

## 补充文档

- `docs/ylj5_fidelity.md` YLJ5 公开规格证据、仿真假设和实机还原缺口
- `docs/realistic_lidar_simulation.md` 通用批处理仿真与 YLJ5 实时设备的边界和协议
- `docs/cpp_architecture_refactor_plan.md` 三层架构的历史设计记录与当前接口迁移说明
- `docs/project_supplement.md` 六项补充内容的正式落地说明
- `docs/12_week_plan.md` 12 周落地计划
- `docs/resume_project_entry.md` 简历项目版本
- `data/public/README.md` 公开数据资产说明
