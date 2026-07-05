# C++ 架构重构方案：从单体验证程序到 服务端-主控客户端-UI 三层架构

> **目标**：将现有的"单进程批处理验证骨架"重构为"模拟 LiDAR 服务端 → 主控客户端（预处理 + 反演 + 闭环逻辑）→ UI 显示"三层架构，同时保留现有算法链的正确性。

---

## 1. 现状分析

### 1.1 已有代码盘点

| 路径 | 行数 | 职责 | 现状评价 |
|---|---|---|---|
| `cpp/include/LidarDemo/LidarDemo.hpp` | 418 | 全量公共 API 声明 | 所有类型/函数集中在一个头文件，include 开销大、编译耦合高 |
| `cpp/src/LidarDemo.cpp` | 4,537 | 全量实现（JSON 引擎 + 仿真 + L1 + Fernald + 湿度 + ENU + PM 标定 + 热点 + 评估 + HTTP + NetCDF + dashboard HTML） | **单编译单元 4500+ 行**，改任何一处都要全量重编 |
| `cpp/apps/api_server.cpp` | 437 | 原生 socket HTTP API（每请求跑一次完整管线） | 每次请求重跑全流程，无状态、无流式推送 |
| `cpp/apps/run_batch.cpp` | 59 | 批处理 Worker CLI | 功能正确，可保留 |
| `cpp/apps/build_demo_assets.cpp` | 64 | 静态 Demo HTML 生成 | 功能正确，可保留 |
| `cpp/apps/fetch_*.cpp` | 130 | Open-Meteo / Cloudnet 数据抓取 | 功能正确，可保留 |
| `cpp/examples/example_*.cpp` | 1,474 | 第 19 章教学算例（1D/2D/3D + 全链编排） | 教学用，STL-only，**不需要改** |
| `cpp/tests/test_pipeline.cpp` | 80 | 端到端集成断言 | 功能正确，后续应扩充 |

### 1.2 核心问题

```
当前架构（单体验证程序）：

┌──────────────────────────────────────────────────────┐
│                  单进程 (lidar_demo.cpp)               │
│                                                        │
│  JSON引擎 ─── 仿真器 ─── L1预处理 ─── Fernald反演     │
│  湿度修正 ─── ENU转换 ─── PM标定 ─── 热点检测         │
│  质量评估 ─── HTTP下载 ─── NetCDF ─── dashboard HTML   │
│                                                        │
│  ← 所有模块塞在同一个匿名命名空间里，全量重编 →       │
└──────────────────────────────────────────────────────┘
       ↑                              ↑
  api_server (每请求跑一遍)      run_batch (命令行跑一遍)
```

**问题 1：无服务端 / 客户端分离**
现有 `api_server` 每收到一个 HTTP 请求就从头跑一遍仿真 + 反演。没有真正的"LiDAR 设备 → 主控"数据流。

**问题 2：无实时数据流**
没有持续推送 PPI/RHI 帧的机制。现有模型是"一次性跑完 18 个时间步然后返回全部 JSON"。

**问题 3：4500 行单文件**
改动任何一个函数都要全量重编译 4500 行，迭代效率低。

**问题 4：无闭环逻辑**
第 15 章定义的"告警状态机 → 确认 → 处置 → 复测 → 报表"在代码中完全不存在。

**问题 5：无 UI 层**
当前只有生成静态 HTML 的能力，没有真正的客户端 UI。

---

## 2. 目标架构

```
目标架构（三层 + 共享核心库）：

┌───────────────────────┐       TCP/WebSocket        ┌──────────────────────────────────┐
│   lidar_sim_server    │ ──────────────────────────→ │      lidar_control_client         │
│   (模拟 LiDAR 服务端)  │   持续推送 L0 原始帧         │      (主控客户端)                   │
│                       │ ←────────────────────────── │                                    │
│  • 物理模型正向仿真    │      控制指令(扫描模式变更)   │  ┌──────────┐  ┌───────────────┐  │
│  • PPI / RHI / stare  │                              │  │ 处理引擎  │  │  闭环管理器    │  │
│  • 噪声注入            │                              │  │ L0→L1→L2 │  │ 告警状态机    │  │
│  • 帧打包 (JSON/binary)│                              │  │ →L3 热点  │  │ 处置联动      │  │
│  • 扫描模式调度        │                              │  └─────┬────┘  │ 报表生成      │  │
└───────────────────────┘                              │        │       └───────┬───────┘  │
                                                       │  ┌─────┴────────────────┴──────┐  │
                                                       │  │      UI 显示层 (Qt Widgets)   │  │
                                                       │  │  地图热力图 / 3D体素 / 告警列表 │  │
                                                       │  │  时间轴 / 状态机面板 / 报表    │  │
                                                       │  └─────────────────────────────┘  │
                                                       └──────────────────────────────────┘
                              ↕
              ┌──────────────────────────────────┐
              │       lidar_core (共享核心库)      │
              │  拆分自 lidar_demo.cpp 的纯算法层  │
              │  JSON / 数学 / L1 / Fernald /     │
              │  湿度 / ENU / PM / 热点 / 评估     │
              └──────────────────────────────────┘
```

### 2.1 设计原则

| 原则 | 说明 |
|---|---|
| **算法与传输解耦** | `lidar_core` 只管数学，不依赖任何网络库；服务端和客户端各自做传输 |
| **服务端可替换** | 模拟服务端的接口与真实 LiDAR 设备一致（同样的帧格式），将来换真设备只改 server |
| **帧驱动** | 数据以"帧"为单位流动（一帧 = 一个 PPI 扫描或一条 RHI 剖面），不是一次性全量 |
| **闭环可追踪** | 每个热点事件有完整生命周期（candidate → active → resolved），状态变更全部记录 |
| **UI 可降级** | 没有 Qt 时可以跑 headless 模式（只输出 JSON / 终端告警），有 Qt 时上可视化 |
| **教学示例不动** | `cpp/examples/` 是纯 STL 教学代码，保持完全独立 |

---

## 3. 新目录结构

```
cpp/
├── core/                          # 共享核心库 (从 lidar_demo.cpp 拆分)
│   ├── CMakeLists.txt
│   ├── include/lidar_core/
│   │   ├── types.hpp              # 数据载体: LidarProfile, ProcessedProfile, Hotspot, ...
│   │   ├── config.hpp             # 配置结构体: PipelineConfig, SimulationConfig, ...
│   │   ├── json.hpp               # 轻量 JSON 引擎
│   │   ├── math_utils.hpp         # 通用数学: clamp, mean, median, gaussian, deg2rad
│   │   ├── time_utils.hpp         # 时间戳解析/格式化
│   │   ├── preprocess.hpp         # L0→L1 预处理 (背景扣除/能量归一/overlap/RCS/SNR)
│   │   ├── fernald.hpp            # Fernald/Klett 反演
│   │   ├── humidity.hpp           # 湿度修正 f(RH)
│   │   ├── enu.hpp                # 极坐标→ENU 坐标转换
│   │   ├── pm_calibration.hpp     # PM 标定 (最小二乘回归)
│   │   ├── hotspot.hpp            # 热点检测 (连通域 + 质心)
│   │   ├── quality.hpp            # 质量评估 (RMSE/MAE/R²)
│   │   ├── simulation.hpp         # 正向仿真 (Gaussian plume + LiDAR 方程)
│   │   └── pipeline.hpp           # 端到端管线编排 (兼容旧 run_end_to_end)
│   └── src/
│       ├── json.cpp               # ~400 行 (Json + Parser + Serializer)
│       ├── math_utils.cpp         # ~100 行
│       ├── time_utils.cpp         # ~80 行
│       ├── preprocess.cpp         # ~120 行
│       ├── fernald.cpp            # ~150 行
│       ├── humidity.cpp           # ~60 行
│       ├── enu.cpp                # ~50 行
│       ├── pm_calibration.cpp     # ~200 行
│       ├── hotspot.cpp            # ~250 行
│       ├── quality.cpp            # ~200 行
│       ├── simulation.cpp         # ~300 行
│       ├── pipeline.cpp           # ~400 行 (编排 + JSON 序列化)
│       ├── http_client.cpp        # ~150 行 (libcurl/WinHTTP 封装)
│       ├── netcdf_reader.cpp      # ~120 行 (#ifdef LIDAR_DEMO_HAS_NETCDF)
│       └── dashboard.cpp          # ~200 行 (HTML 模板)
│
├── protocol/                      # 通信协议 (服务端/客户端共用)
│   ├── include/lidar_protocol/
│   │   ├── frame.hpp              # 帧定义: ScanFrame (一帧扫描数据)
│   │   ├── wire_format.hpp        # 序列化/反序列化 (JSON 或 二进制)
│   │   └── control.hpp            # 控制指令: SetScanMode, StartScan, StopScan
│   └── src/
│       ├── frame.cpp
│       └── wire_format.cpp
│
├── server/                        # 模拟 LiDAR 服务端
│   ├── include/lidar_server/
│   │   ├── sim_device.hpp         # 模拟 LiDAR 设备 (封装 simulation 模块)
│   │   └── tcp_server.hpp         # TCP 服务器 (原生 socket, 跨平台)
│   └── src/
│       ├── sim_device.cpp         # 持续生成帧 + 噪声注入 + 扫描调度
│       ├── tcp_server.cpp         # accept 循环 + 帧推送 + 控制指令接收
│   └── apps/
│       └── main.cpp               # lidar_sim_server 入口
│
├── client/                        # 主控客户端 (处理引擎 + 闭环管理 + UI)
│   ├── processing/                # 处理引擎 (L0→L3)
│   │   ├── include/lidar_client/
│   │   │   ├── frame_processor.hpp    # 单帧处理: L0→L1→L2→热点
│   │   │   ├── l3_builder.hpp         # 多帧累积 → L3 空间产品 (栅格/体素)
│   │   │   └── quality_gate.hpp       # 质量门控 (SNR/湿度/雨雾/参考段)
│   │   └── src/
│   │       ├── frame_processor.cpp
│   │       ├── l3_builder.cpp
│   │       └── quality_gate.cpp
│   │
│   ├── closed_loop/               # 第15章闭环逻辑
│   │   ├── include/lidar_client/
│   │   │   ├── alarm_state_machine.hpp  # 告警状态机
│   │   │   ├── hotspot_tracker.hpp      # 热点事件生命周期管理
│   │   │   ├── disposal_linkage.hpp     # 处置联动 (喷淋/雾炮)
│   │   │   └── report_generator.hpp     # 报表生成
│   │   └── src/
│   │       ├── alarm_state_machine.cpp
│   │       ├── hotspot_tracker.cpp
│   │       ├── disposal_linkage.cpp
│   │       └── report_generator.cpp
│   │
│   ├── ui/                        # UI 显示层
│   │   ├── include/lidar_client/
│   │   │   ├── ppi_widget.hpp     # PPI 热力图 (QPainter + QImage)
│   │   │   ├── map_widget.hpp     # 地图底图 + 热点标注
│   │   │   ├── alarm_panel.hpp    # 告警列表 + 状态机面板
│   │   │   └── main_window.hpp    # QMainWindow 主窗口
│   │   └── src/
│   │       ├── ppi_widget.cpp
│   │       ├── map_widget.cpp
│   │       ├── alarm_panel.cpp
│   │       └── main_window.cpp
│   │
│   └── apps/
│       ├── main_gui.cpp           # lidar_control_client (GUI 模式, 需要 Qt)
│       └── main_headless.cpp      # lidar_control_headless (无 GUI, 输出 JSON/终端)
│
├── apps/                          # 现有 CLI 工具 (保留)
│   ├── run_batch.cpp              # 批处理 Worker
│   ├── build_demo_assets.cpp      # 静态 Demo 生成
│   ├── api_server.cpp             # HTTP API (改为调用 core 库)
│   ├── fetch_public_ground_data.cpp
│   └── fetch_cloudnet_public_sample.cpp
│
├── examples/                      # 教学算例 (不动)
│   ├── example_common.hpp
│   ├── example_1d_ray.cpp
│   ├── example_2d_ppi.cpp
│   ├── example_3d_rhi.cpp
│   └── run_full_demo.cpp
│
└── tests/                         # 测试
    ├── test_pipeline.cpp          # 现有端到端测试 (保留)
    ├── test_preprocess.cpp        # L1 预处理单元测试 (新增)
    ├── test_fernald.cpp           # Fernald 反演单元测试 (新增)
    ├── test_hotspot.cpp           # 热点检测单元测试 (新增)
    ├── test_state_machine.cpp     # 告警状态机单元测试 (新增)
    └── test_protocol.cpp          # 通信协议测试 (新增)
```

---

## 4. 各组件详细设计

### 4.1 共享核心库 `lidar_core`

#### 拆分策略

将 `lidar_demo.cpp` 的 4500 行按功能模块拆分到 `core/src/*.cpp`，每个文件 100–400 行。

**拆分对照表**（旧文件章节 → 新文件）：

| 旧 `lidar_demo.cpp` 章节 | 行数范围（约） | 新文件 |
|---|---|---|
| 1. JSON 引擎 (Json + Parser + Serializer) | 135–600 | `json.cpp` |
| 2. 通用数学/时间工具 | 600–900 | `math_utils.cpp` + `time_utils.cpp` |
| 3. NetCDF 读取层 | 900–1000 | `netcdf_reader.cpp` |
| 4. 地面观测处理 | 1000–1100 | `pipeline.cpp`（合入） |
| 5. HTTP 下载 | 1100–1200 | `http_client.cpp` |
| 6. 数据→JSON 序列化 | 1619–1790 | `pipeline.cpp` |
| 7. 正向仿真 | 1796–1920 | `simulation.cpp` |
| 8a. L1 预处理 | 2515–2570 | `preprocess.cpp` |
| 8b. Fernald 反演 | 2574–2640 | `fernald.cpp` |
| 8c. 湿度校正 | 2642–2690 | `humidity.cpp` |
| 8d. ENU 转换 | 2690–2740 | `enu.cpp` |
| 8e. PM 标定 | 2740–2900 | `pm_calibration.cpp` |
| 8f. 热点检测 | 2900–3200 | `hotspot.cpp` |
| 8g. 质量评估 | 3200–3600 | `quality.cpp` |
| 9. demo payload | 3600–3900 | `pipeline.cpp` |
| 10. 公开函数 | 4010–4537 | `pipeline.cpp` |

**向后兼容**：保留 `lidar_demo.hpp` 作为"伞头文件"，内部 `#include` 各子头文件，确保现有 apps/examples 不需要改 include 路径。

#### 头文件接口示例

```cpp
// core/include/lidar_core/types.hpp
#pragma once
#include <string>
#include <vector>

namespace lidar_core {

struct LidarProfile {
    std::string site_id;
    std::string timestamp;
    std::string scan_id;
    std::string scan_mode;          // "stare" | "ppi" | "rhi"
    double azimuth_deg = 0.0;
    double elevation_deg = 0.0;
    std::vector<double> ranges_m;
    std::vector<double> raw_counts;
    double laser_energy_mj = 0.0;
    double background_counts = 0.0;
    std::vector<double> overlap;
    double relative_humidity = 0.0;
    double temperature_c = 0.0;
    // ... (与现有结构体一致)
};

struct ProcessedProfile {
    LidarProfile profile;
    std::vector<double> l1_signal;
    std::vector<double> attenuated_backscatter;
    std::vector<double> snr;
    std::vector<double> extinction;
    std::vector<double> dry_extinction;
    std::vector<double> pm25;
    std::vector<double> pm10;
    std::vector<std::vector<double>> enu_points_m;
    std::vector<std::string> qc_flags;
    double latency_ms = 0.0;
};

struct Hotspot { /* ... */ };

} // namespace lidar_core
```

```cpp
// core/include/lidar_core/preprocess.hpp
#pragma once
#include "lidar_core/types.hpp"

namespace lidar_core {

struct PreprocessResult {
    std::vector<double> l1_signal;
    std::vector<double> attenuated_backscatter;
    std::vector<double> snr;
    std::vector<std::string> qc_flags;
};

/// L0→L1 预处理：背景扣除 + 能量归一 + overlap 校正 + RCS + SNR
PreprocessResult preprocess(const LidarProfile& profile);

/// SNR 门控判断：夜间 ≥3, 日间 ≥5
bool passes_snr_gate(const std::vector<double>& snr, double min_snr);

} // namespace lidar_core
```

```cpp
// core/include/lidar_core/fernald.hpp
#pragma once
#include "lidar_core/types.hpp"
#include "lidar_core/config.hpp"

namespace lidar_core {

struct FernaldResult {
    std::vector<double> extinction;   // 含湿消光 (1/m)
    std::vector<double> backscatter;  // 总后向散射 (1/(m·sr))
};

/// Fernald/Klett 反演：远端参考点 → 近端反向递推
FernaldResult fernald_inversion(
    const std::vector<double>& attenuated_backscatter,
    const std::vector<double>& ranges_m,
    const std::vector<double>& molecular_backscatter,
    const std::vector<double>& molecular_extinction,
    double aerosol_lidar_ratio_sr,
    double reference_aerosol_backscatter
);

} // namespace lidar_core
```

---

### 4.2 通信协议 `lidar_protocol`

#### 帧定义

```cpp
// protocol/include/lidar_protocol/frame.hpp
#pragma once
#include <string>
#include <vector>

namespace lidar_protocol {

/// 一帧 LiDAR 扫描数据（服务端 → 客户端）
struct ScanFrame {
    std::string frame_id;           // 唯一帧 ID
    std::string timestamp;          // ISO8601
    std::string scan_mode;          // "ppi" | "rhi" | "stare"
    double azimuth_deg = 0.0;       // PPI: 当前方位角; RHI: 固定
    double elevation_deg = 0.0;     // RHI: 当前仰角; PPI: 固定
    std::vector<double> ranges_m;   // 距离轴
    std::vector<double> raw_counts; // L0 原始计数
    double laser_energy_mj = 0.0;
    double background_counts = 0.0;
    std::vector<double> overlap;
    double relative_humidity = 0.0;
    double temperature_c = 0.0;
    double wind_speed_ms = 0.0;
    double wind_dir_deg = 0.0;
    std::vector<double> molecular_backscatter;
    std::vector<double> molecular_extinction;
    // 以下仅在仿真模式附带（真机不送）
    std::vector<double> true_pm25;
    std::vector<double> true_hotspot_mask;
};

/// 控制指令（客户端 → 服务端）
enum class ControlCommand {
    SetScanMode,    // 切换扫描模式
    StartScan,      // 开始扫描
    StopScan,       // 停止扫描
    SetParam,       // 修改参数 (脉冲能量、方位角步长等)
};

struct ControlMessage {
    ControlCommand command;
    std::string scan_mode;      // SetScanMode 用
    std::string param_name;     // SetParam 用
    double param_value = 0.0;   // SetParam 用
};

} // namespace lidar_protocol
```

#### 线格式

第一版用 **JSON 行协议**（每帧一行 JSON，以 `\n` 分隔），简单易调试：

```
{"type":"frame","frame_id":"f_001","timestamp":"2026-06-23T10:00:00","scan_mode":"ppi","azimuth_deg":0.0,"elevation_deg":8.0,"ranges_m":[50.0,100.0,...],"raw_counts":[...],"laser_energy_mj":2.0,...}\n
{"type":"frame","frame_id":"f_002","timestamp":"2026-06-23T10:00:00","scan_mode":"ppi","azimuth_deg":30.0,...}\n
...
{"type":"control_ack","command":"SetScanMode","status":"ok"}\n
```

第二版可升级为二进制帧头 + 压缩 payload（减少带宽）。

---

### 4.3 模拟 LiDAR 服务端 `lidar_sim_server`

#### 职责

1. 持续按配置生成 PPI/RHI 扫描帧
2. 每生成一帧推送给已连接的客户端
3. 接收并执行客户端发来的控制指令（切换扫描模式等）
4. 注入真实噪声（Poisson 射击噪声 + 背景光）

#### 核心类

```cpp
// server/include/lidar_server/sim_device.hpp
#pragma once
#include "lidar_protocol/frame.hpp"
#include "lidar_core/config.hpp"
#include <functional>
#include <string>

namespace lidar_server {

/// 模拟 LiDAR 设备状态
struct DeviceState {
    std::string current_scan_mode = "ppi";   // "ppi" | "rhi" | "stare"
    double current_azimuth_deg = 0.0;
    double current_elevation_deg = 8.0;
    bool is_scanning = false;
    int frame_counter = 0;
};

/// 模拟 LiDAR 设备：帧生成器
class SimDevice {
public:
    explicit SimDevice(const lidar_core::SimulationConfig& config);

    /// 生成下一帧（内部推进方位角/仰角）
    lidar_protocol::ScanFrame generate_next_frame();

    /// 处理控制指令
    void apply_control(const lidar_protocol::ControlMessage& msg);

    const DeviceState& state() const { return state_; }

private:
    lidar_core::SimulationConfig config_;
    DeviceState state_;
    // 内部仿真状态：每个方位角的气溶胶场、噪声 RNG 等
};

} // namespace lidar_server
```

```cpp
// server/include/lidar_server/tcp_server.hpp
#pragma once
#include "lidar_server/sim_device.hpp"
#include <atomic>
#include <thread>
#include <vector>

namespace lidar_server {

/// TCP 服务器：持续推送帧 + 接收控制指令
class TcpServer {
public:
    TcpServer(SimDevice& device, int port);
    ~TcpServer();

    void start();
    void stop();

private:
    SimDevice& device_;
    int port_;
    std::atomic<bool> running_{false};
    std::thread accept_thread_;
    std::vector<std::thread> client_threads_;

    void accept_loop();
    void client_loop(int client_socket);
};

} // namespace lidar_server
```

#### 运行流程

```
lidar_sim_server --port 9876 --config configs/DefaultPipeline.json

  主线程:
    TcpServer.start()
      ├─ accept_loop()    → 等待客户端连接
      └─ 对每个连接 spawn client_loop()

  client_loop():
    while (running):
      frame = sim_device.generate_next_frame()
      json = wire_format::serialize_frame(frame)
      send(client_socket, json + "\n")

      // 非阻塞检查是否有控制指令
      if (has_control_message):
          msg = wire_format::parse_control(raw)
          sim_device.apply_control(msg)
          send_ack(client_socket, msg)

      sleep(scan_interval_ms)   // 控制帧率
```

---

### 4.4 主控客户端 — 处理引擎

#### 帧处理器

```cpp
// client/processing/include/lidar_client/frame_processor.hpp
#pragma once
#include "lidar_core/types.hpp"
#include "lidar_core/config.hpp"
#include "lidar_protocol/frame.hpp"
#include <vector>

namespace lidar_client {

/// 单帧处理结果（一帧 PPI 扫描 → 多条射线的处理结果 + 热点）
struct FrameProcessResult {
    std::string frame_id;
    std::string timestamp;
    std::vector<lidar_core::ProcessedProfile> profiles;  // 每条射线
    std::vector<lidar_core::Hotspot> hotspots;           // 本帧检测到的热点
    std::vector<std::string> quality_flags;              // 整帧质量标记
    double processing_ms = 0.0;
};

/// 帧处理器：L0 帧 → L1 → L2 → 热点
class FrameProcessor {
public:
    explicit FrameProcessor(const lidar_core::PipelineConfig& config);

    /// 处理一帧（包含多条射线）
    FrameProcessResult process(const lidar_protocol::ScanFrame& frame);

private:
    lidar_core::PipelineConfig config_;
    // PM 标定模型缓存（首次标定后复用）
};

} // namespace lidar_client
```

#### L3 空间产品构建器

```cpp
// client/processing/include/lidar_client/l3_builder.hpp
#pragma once
#include "lidar_core/types.hpp"
#include <map>
#include <vector>

namespace lidar_client {

/// L3 空间快照：PPI 栅格化结果
struct GridSnapshot {
    double origin_e = 0.0;          // ENU 原点
    double origin_n = 0.0;
    double cell_size_m = 25.0;
    int grid_w = 0;                 // 东西向格数
    int grid_h = 0;                 // 南北向格数
    std::vector<double> pm25_grid;  // 行优先 PM2.5 栅格 (grid_w × grid_h)
    std::vector<double> ext_grid;   // 消光栅格
    std::string timestamp;
};

/// L3 构建器：累积多帧 ProcessedProfile → 统一栅格
class L3Builder {
public:
    void set_grid_params(double origin_e, double origin_n, double cell_size_m,
                         int grid_w, int grid_h);

    /// 添加一组处理后的剖面，栅格化到网格
    void accumulate(const std::vector<lidar_core::ProcessedProfile>& profiles);

    /// 生成当前快照
    GridSnapshot build_snapshot() const;

    /// 清空累积（开始新一轮扫描）
    void reset();

private:
    double origin_e_ = 0.0, origin_n_ = 0.0, cell_size_m_ = 25.0;
    int grid_w_ = 0, grid_h_ = 0;
    std::vector<double> pm25_sum_;
    std::vector<int> pm25_count_;
};

} // namespace lidar_client
```

#### 质量门控

```cpp
// client/processing/include/lidar_client/quality_gate.hpp
#pragma once
#include "lidar_core/types.hpp"
#include <string>
#include <vector>

namespace lidar_client {

/// 质量门控结果
struct QualityGateResult {
    bool passed = false;
    std::vector<std::string> quality_flags;
    // ["snr_ok", "humidity_corrected", "no_rain_fog", "reference_stable"]
};

/// 质量门控：决定一帧数据是否可用于告警判断
class QualityGate {
public:
    QualityGate(double min_snr_night = 3.0, double min_snr_day = 5.0,
                double max_humidity = 0.95, double min_laser_energy_mj = 0.9);

    QualityGateResult evaluate(const std::vector<lidar_core::ProcessedProfile>& profiles,
                                bool is_daytime) const;

private:
    double min_snr_night_;
    double min_snr_day_;
    double max_humidity_;
    double min_laser_energy_mj_;
};

} // namespace lidar_client
```

---

### 4.5 主控客户端 — 闭环管理器

#### 告警状态机

```cpp
// client/closed_loop/include/lidar_client/alarm_state_machine.hpp
#pragma once
#include <functional>
#include <string>
#include <vector>

namespace lidar_client {

/// 告警状态（第 15 章 §15.4.2）
enum class AlarmState {
    Candidate,      // 初次检测到，等待确认是否持续
    Active,         // 持续超标，进入告警
    Acknowledged,   // 人工确认
    Dispatched,     // 已派单
    Mitigating,     // 正在处置
    Resolved,       // 已解决
    Archived,       // 归档
};

/// 状态转换合法性表
inline bool is_valid_transition(AlarmState from, AlarmState to) {
    switch (from) {
        case AlarmState::Candidate:    return to == AlarmState::Active || to == AlarmState::Archived;
        case AlarmState::Active:       return to == AlarmState::Acknowledged || to == AlarmState::Archived;
        case AlarmState::Acknowledged: return to == AlarmState::Dispatched || to == AlarmState::Resolved;
        case AlarmState::Dispatched:   return to == AlarmState::Mitigating || to == AlarmState::Acknowledged;
        case AlarmState::Mitigating:   return to == AlarmState::Resolved || to == AlarmState::Acknowledged;
        case AlarmState::Resolved:     return to == AlarmState::Archived;
        case AlarmState::Archived:     return false; // 终态
    }
    return false;
}

/// 单个告警事件
struct AlarmEvent {
    std::string event_id;
    std::string hotspot_id;
    AlarmState state = AlarmState::Candidate;
    std::string created_at;
    std::string last_updated;
    double peak_pm25 = 0.0;
    double mean_pm25 = 0.0;
    double area_m2 = 0.0;
    std::string acknowledged_by;
    std::string disposal_action;
    std::vector<std::string> state_history;
};

/// 告警状态机管理器
class AlarmStateMachine {
public:
    /// 添加新检测到的热点（可能产生新的 candidate）
    void on_hotspots_detected(const std::vector<lidar_core::Hotspot>& hotspots,
                              const std::string& timestamp);

    /// 人工确认告警
    bool acknowledge(const std::string& event_id, const std::string& operator_name);

    /// 派单
    bool dispatch(const std::string& event_id, const std::string& action);

    /// 标记处置中
    bool start_mitigation(const std::string& event_id);

    /// 标记已解决
    bool resolve(const std::string& event_id);

    /// 归档（定时清理或手动）
    void archive_expired(const std::string& current_timestamp, int max_age_minutes = 60);

    /// 获取当前活跃告警列表
    const std::vector<AlarmEvent>& active_events() const { return events_; }

private:
    std::vector<AlarmEvent> events_;

    bool transition(AlarmEvent& event, AlarmState new_state);
};

} // namespace lidar_client
```

#### 热点追踪器

```cpp
// client/closed_loop/include/lidar_client/hotspot_tracker.hpp
#pragma once
#include "lidar_core/types.hpp"
#include <map>
#include <string>
#include <vector>

namespace lidar_client {

/// 被追踪的热点（跨帧关联）
struct TrackedHotspot {
    std::string hotspot_id;
    std::string first_seen;
    std::string last_seen;
    int consecutive_frames = 0;     // 连续命中帧数
    std::vector<double> centroid_enu_m;
    double peak_pm25 = 0.0;
    double mean_pm25 = 0.0;
    double area_m2 = 0.0;
    double confidence = 0.0;        // 置信度评分
    bool is_active = false;         // 是否满足持续时间门控
};

/// 热点追踪器：跨帧关联热点，应用持续时间/面积门控
class HotspotTracker {
public:
    HotspotTracker(int min_consecutive_frames = 3,
                   double min_area_m2 = 100.0,
                   double spatial_match_threshold_m = 50.0);

    /// 输入新检测到的热点，输出满足门控条件的活跃热点
    std::vector<TrackedHotspot> update(const std::vector<lidar_core::Hotspot>& new_hotspots,
                                       const std::string& timestamp);

    /// 获取当前所有被追踪的热点
    const std::vector<TrackedHotspot>& tracked() const { return tracked_; }

private:
    int min_consecutive_frames_;
    double min_area_m2_;
    double spatial_match_threshold_m_;
    std::vector<TrackedHotspot> tracked_;
};

} // namespace lidar_client
```

#### 处置联动

```cpp
// client/closed_loop/include/lidar_client/disposal_linkage.hpp
#pragma once
#include "lidar_core/types.hpp"
#include <string>
#include <vector>

namespace lidar_client {

/// 处置指令
struct DisposalCommand {
    std::string event_id;
    std::string target_device;         // "sprinkler_01" / "fog_cannon_02"
    double target_azimuth_deg = 0.0;   // 目标方位角
    double target_pitch_deg = 0.0;     // 目标俯仰角
    double estimated_duration_s = 0.0; // 预计喷淋时长
    bool safety_check_passed = false;
    std::vector<std::string> safety_warnings;
};

/// 设备外参
struct DeviceExtrinsics {
    std::string device_id;
    std::vector<double> origin_enu;           // 设备原点 ENU [E, N, U]
    double max_range_m = 80.0;
    std::vector<double> azimuth_range_deg;     // [min, max]
    std::vector<double> pitch_range_deg;       // [min, max]
    // 排除区域（道路、办公区等）
    std::vector<std::map<std::string, std::string>> exclusion_zones;
};

/// 处置联动管理器
class DisposalLinkage {
public:
    void add_device(DeviceExtrinsics extrinsics);

    /// 从热点事件生成处置指令（含安全检查）
    DisposalCommand plan_disposal(const std::vector<double>& hotspot_enu,
                                   double hotspot_height_m,
                                   const std::string& event_id,
                                   const std::string& device_id);

    /// 计算俯仰角
    static double compute_pitch_deg(double ground_dist_m, double height_m);

    /// 安全检查
    bool run_safety_check(DisposalCommand& cmd, const DeviceExtrinsics& dev) const;

private:
    std::vector<DeviceExtrinsics> devices_;
};

} // namespace lidar_client
```

---

### 4.6 主控客户端 — UI 显示层

#### 架构

```
┌─────────────────────────────────────────────────────────────┐
│                    MainWindow (QMainWindow)                   │
│  ┌─────────┬──────────────────────────┬───────────────────┐  │
│  │         │                          │                   │  │
│  │  地图    │     PPI 热力图            │    告警面板       │  │
│  │ Widget  │     Widget               │    (AlarmPanel)   │  │
│  │ (QGraph │     (QPainter+QImage)     │                   │  │
│  │ icsView)│                          │  • 告警列表       │  │
│  │         │  ┌─────────────────────┐ │  • 状态机颜色     │  │
│  │ • 底图   │  │   距离环 / 方位标    │ │  • 确认/派单按钮  │  │
│  │ • 热点   │  │   PM2.5 色带         │ │                   │  │
│  │ • 设备   │  │   质量标记           │ │───────────────────│  │
│  │   位置   │  └─────────────────────┘ │  处置面板         │  │
│  │         │                          │  • 设备状态       │  │
│  │─────────│──────────────────────────│  • 推荐方位角     │  │
│  │ 时间轴  │  质量信息面板             │  • 安全检查       │  │
│  │ Widget  │  • SNR / 湿度 / 参考段    │  • 确认执行       │  │
│  │         │                          │                   │  │
│  └─────────┴──────────────────────────┴───────────────────┘  │
│                      状态栏: 连接状态 / 帧率 / 数据有效率      │
└─────────────────────────────────────────────────────────────┘
```

#### PPI 热力图 Widget（Level 1 可视化）

```cpp
// client/ui/include/lidar_client/ppi_widget.hpp
#pragma once
#include "lidar_core/types.hpp"
#include <QWidget>
#include <QImage>

namespace lidar_client {

class PpiWidget : public QWidget {
    Q_OBJECT
public:
    explicit PpiWidget(QWidget* parent = nullptr);

    /// 更新 PPI 数据（一组处理后的剖面）
    void update_profiles(const std::vector<lidar_core::ProcessedProfile>& profiles);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage canvas_;         // 离屏渲染画布
    double max_range_m_ = 1500.0;
    int canvas_size_ = 600; // 正方形画布像素

    void render_to_canvas();
    QColor pm25_to_color(double pm25) const;   // 绿→黄→红 色带
};

} // namespace lidar_client
```

#### 告警面板 Widget

```cpp
// client/ui/include/lidar_client/alarm_panel.hpp
#pragma once
#include "lidar_client/alarm_state_machine.hpp"
#include <QWidget>
#include <QTableWidget>

namespace lidar_client {

class AlarmPanel : public QWidget {
    Q_OBJECT
public:
    explicit AlarmPanel(QWidget* parent = nullptr);

    void update_events(const std::vector<AlarmEvent>& events);

signals:
    void acknowledge_requested(const std::string& event_id);
    void dispatch_requested(const std::string& event_id);
    void resolve_requested(const std::string& event_id);

private:
    QTableWidget* table_;
    QPushButton* ack_button_;
    QPushButton* dispatch_button_;
    QPushButton* resolve_button_;
};

} // namespace lidar_client
```

---

### 4.7 Headless 模式（无 Qt）

当部署环境没有 Qt 时，客户端可以跑 headless 模式：

```cpp
// client/apps/main_headless.cpp
//
// lidar_control_headless --server 127.0.0.1:9876 --config configs/DefaultPipeline.json
//
// 功能：
//   1. 连接 lidar_sim_server
//   2. 接收帧 → 处理 → 热点检测 → 状态机
//   3. 告警输出到终端（带颜色）
//   4. 定期写 JSON 报表到 data/l3/
//   5. 不需要任何 GUI 库

int main(int argc, char** argv) {
    // 解析参数: --server, --config, --output
    // 连接 TCP server
    // 主循环:
    //   receive_frame()
    //   result = frame_processor.process(frame)
    //   tracked = hotspot_tracker.update(result.hotspots, frame.timestamp)
    //   alarm_state_machine.on_hotspots_detected(tracked, frame.timestamp)
    //   print_alarms_to_terminal(alarm_state_machine.active_events())
    //   if (time_to_report): report_generator.generate_report(...)
}
```

---

## 5. CMake 构建结构

```cmake
# 顶层 CMakeLists.txt

cmake_minimum_required(VERSION 3.20)
project(lidar_system LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# 选项：是否启用 Qt UI（默认 OFF，按需开启）
option(LIDAR_ENABLE_QT "Enable Qt-based UI client" OFF)
# 选项：是否启用 NetCDF（Cloudnet 真实数据）
option(LIDAR_ENABLE_NETCDF "Enable NetCDF reader" OFF)

# ---- 核心库 ----
add_subdirectory(cpp/core)          # → lidar_core (STATIC)

# ---- 通信协议 ----
add_subdirectory(cpp/protocol)      # → lidar_protocol (STATIC, 依赖 lidar_core)

# ---- 模拟服务端 ----
add_subdirectory(cpp/server)        # → lidar_sim_server (可执行, 依赖 core + protocol)

# ---- 客户端处理引擎 ----
add_subdirectory(cpp/client/processing)    # → lidar_client_processing (STATIC)
add_subdirectory(cpp/client/closed_loop)   # → lidar_client_closed_loop (STATIC)

# ---- 客户端 UI (可选) ----
if (LIDAR_ENABLE_QT)
    find_package(Qt6 COMPONENTS Widgets REQUIRED)
    add_subdirectory(cpp/client/ui)  # → lidar_client_ui (STATIC, 依赖 Qt6)
endif()

# ---- 客户端入口 ----
add_subdirectory(cpp/client/apps)   # → lidar_control_headless (总是构建)
                                    # → lidar_control_client (仅 LIDAR_ENABLE_QT=ON)

# ---- 现有 CLI 工具 ----
add_subdirectory(cpp/apps)          # → run_batch, build_demo_assets, api_server, fetch_*

# ---- 教学示例 (不变) ----
add_subdirectory(cpp/examples)

# ---- 测试 ----
add_subdirectory(cpp/tests)
enable_testing()
```

---

## 6. 数据流（端到端）

```
服务端                          客户端
──────                          ──────

SimDevice                       FrameProcessor
.generate_next_frame()          .process(frame)
  │                               │
  │ ScanFrame (L0)                │ ├─ preprocess (L0→L1)
  │ {raw_counts, ranges, ...}     │ │  背景扣除/能量归一/overlap/RCS/SNR
  │                               │ │
  ↓ TCP send                      │ ├─ fernald_inversion (L1→L2)
  ──────────────────────→         │ │  远端参考→近端递推 → 消光/后向散射
  {"type":"frame",...}\n          │ │
                                  │ ├─ humidity_correction
                                  │ │  f(RH) 干消光
                                  │ │
                                  │ ├─ pm_calibration
                                  │ │  干消光 → PM2.5/PM10
                                  │ │
                                  │ ├─ enu_convert
                                  │ │  极坐标 → ENU
                                  │ │
                                  │ └─ hotspot_detect
                                  │     PPI 网格连通域
                                  ↓
                                FrameProcessResult
                                  │
                                  ↓
                                QualityGate.evaluate()
                                  │ {passed, quality_flags}
                                  ↓
                                HotspotTracker.update()
                                  │ 跨帧关联 + 持续时间门控
                                  ↓
                                AlarmStateMachine
                                .on_hotspots_detected()
                                  │
                                  ├─→ UI: AlarmPanel 更新
                                  │    (状态颜色、确认按钮)
                                  │
                                  ├─→ DisposalLinkage.plan_disposal()
                                  │    (外参标定 + 安全检查)
                                  │
                                  ↓
                                ReportGenerator
                                .on_event_resolved()
                                  │
                                  ↓
                                SQLite + JSON 报表归档
```

---

## 7. 迁移路径（分阶段）

### 阶段 1：拆分核心库（不改行为）

| 步骤 | 内容 | 验证方式 |
|---|---|---|
| 1.1 | 创建 `cpp/core/` 目录结构 | 目录存在 |
| 1.2 | 将 `lidar_demo.hpp` 拆分为各子头文件 | `lidar_demo.hpp` 变为伞头文件 |
| 1.3 | 将 `lidar_demo.cpp` 按章节拆分到 `core/src/*.cpp` | 每个 .cpp < 400 行 |
| 1.4 | 更新 `CMakeLists.txt` 编译 `lidar_core` 库 | `cmake --build` 通过 |
| 1.5 | 运行 `lidar_pipeline_test` | 所有断言通过 |
| 1.6 | 运行 `lidar_run_full_demo` | 输出与拆分前一致 |

**此阶段不改变任何算法行为，纯粹是文件组织重构。**

### 阶段 2：通信协议 + 模拟服务端

| 步骤 | 内容 |
|---|---|
| 2.1 | 创建 `cpp/protocol/`，定义 `ScanFrame` + `ControlMessage` + JSON 序列化 |
| 2.2 | 创建 `cpp/server/`，实现 `SimDevice`（复用 `core/simulation`） |
| 2.3 | 实现 `TcpServer`（原生 socket，跨平台） |
| 2.4 | 编写 `lidar_sim_server` 入口程序 |
| 2.5 | 手动测试：启动服务端，用 `nc` 或 Python 脚本连接验证帧推送 |

### 阶段 3：客户端处理引擎

| 步骤 | 内容 |
|---|---|
| 3.1 | 创建 `cpp/client/processing/`，实现 `FrameProcessor` |
| 3.2 | 实现 `QualityGate`（SNR/湿度/能量门控） |
| 3.3 | 实现 `L3Builder`（栅格化 + 快照） |
| 3.4 | 实现 `HotspotTracker`（跨帧关联 + 持续时间门控） |
| 3.5 | 创建 `cpp/client/apps/main_headless.cpp`（TCP 接收 → 处理 → 终端告警） |
| 3.6 | 端到端测试：服务端 + headless 客户端，验证数据流 |

### 阶段 4：闭环管理器

| 步骤 | 内容 |
|---|---|
| 4.1 | 实现 `AlarmStateMachine`（状态转换 + 状态历史） |
| 4.2 | 实现 `DisposalLinkage`（外参标定 + 安全检查） |
| 4.3 | 实现 `ReportGenerator`（事件取证报告 JSON） |
| 4.4 | 将闭环管理器集成到 headless 客户端 |
| 4.5 | 测试完整闭环：检测 → 告警 → 确认 → 处置 → 复测 → 解决 → 报表 |

### 阶段 5：UI 显示层（可选，依赖 Qt）

| 步骤 | 内容 |
|---|---|
| 5.1 | `find_package(Qt6)` 集成 |
| 5.2 | 实现 `PpiWidget`（QPainter + QImage，Level 1 可视化） |
| 5.3 | 实现 `AlarmPanel`（QTableWidget + 按钮信号） |
| 5.4 | 实现 `MainWindow`（QMainWindow + QDockWidget 布局） |
| 5.5 | 实现 `MapWidget`（简易底图 + 热点标注） |
| 5.6 | 集成处理引擎 + 闭环管理器 → GUI 客户端 |

### 阶段 6：测试 + 文档

| 步骤 | 内容 |
|---|---|
| 6.1 | 新增单元测试（preprocess / fernald / hotspot / state_machine） |
| 6.2 | 更新 README（新的构建说明 + 架构图） |
| 6.3 | 更新第 19 章文档引用（指向新的模块路径） |

---

## 8. 依赖关系总结

```
lidar_core (无外部依赖, 纯 STL + 可选 NetCDF + 可选 libcurl)
    ↑
lidar_protocol (依赖 lidar_core)
    ↑
lidar_server (依赖 lidar_core + lidar_protocol)        lidar_client_processing (依赖 lidar_core + lidar_protocol)
                                                              ↑
                                                     lidar_client_closed_loop (依赖 lidar_core + processing)
                                                              ↑
                                              ┌───────────────┴───────────────┐
                                              │                               │
                                     lidar_client_ui (Qt6)            headless client (无 Qt)
                                              │
                                       GUI client
```

| 组件 | 外部依赖 | 可独立编译 |
|---|---|---|
| `lidar_core` | STL (C++20) | ✅ |
| `lidar_core` (+NetCDF) | + netCDF-c | ✅ (LIDAR_ENABLE_NETCDF=ON) |
| `lidar_core` (+HTTP) | + libcurl / WinHTTP | ✅ (默认 ON) |
| `lidar_protocol` | `lidar_core` | ✅ |
| `lidar_sim_server` | `lidar_core` + `lidar_protocol` | ✅ |
| `lidar_client_processing` | `lidar_core` + `lidar_protocol` | ✅ |
| `lidar_client_closed_loop` | + `lidar_client_processing` | ✅ |
| `lidar_client_ui` | + **Qt6 Widgets** | ❌ 需 LIDAR_ENABLE_QT=ON |
| GUI client | 全部 | ❌ 需 Qt6 |
| Headless client | 除 UI 外全部 | ✅ |

---

## 9. 与现有代码的兼容性

| 现有功能 | 重构后去向 | 兼容性保证 |
|---|---|---|
| `lidar_demo::run_end_to_end()` | `lidar_core::run_end_to_end()` 保留 | `lidar_demo.hpp` 伞头文件 `using namespace lidar_core` |
| `lidar_demo::Json` | `lidar_core::Json` | 同上，旧代码不用改 |
| `lidar_run_batch` | 链接 `lidar_core` | 不改 |
| `lidar_build_demo_assets` | 链接 `lidar_core` | 不改 |
| `lidar_api_server` | 链接 `lidar_core` | 不改（后续可选改为代理 lidar_sim_server） |
| `lidar_example_*` | **完全不动** | 仍为 STL-only 独立算例 |
| `lidar_pipeline_test` | 链接 `lidar_core` | 断言不变 |
| `configs/*.json` | 不变 | 配置格式完全兼容 |

---

## 10. 关键设计决策

### Q1：为什么用 TCP 而不是共享内存/UDP？

- **TCP**：可靠传输、跨机器部署、调试简单（nc/telnet 可连）、首版 JSON 行协议易阅读。
- 共享内存延迟更低但仅限同机，UDP 丢包处理复杂。
- 第一版优先正确性和可调试性，后续可加共享内存 fast path。

### Q2：为什么协议用 JSON 而不是 Protobuf/FlatBuffers？

- 项目已有自研 JSON 引擎（`Json` 类），零外部依赖。
- PPI 一帧 ~30 bins × 12 射线 = 360 个 double，JSON 约 8KB，TCP 带宽完全够。
- 后续如需降低延迟可加二进制模式（frame header + raw double array）。

### Q3：为什么 UI 层用 Qt Widgets 而不是 QML/Web？

- 第 14 章明确选择 Qt Widgets + QOpenGLWidget 路线。
- QPainter + QImage 渲染 PPI 热力图最直接，不需要 QML 的声明式开销。
- QTableWidget 处理告警列表最省事。
- 后续如需 3D 体素（Level 3），再引入 QOpenGLWidget + instancing。

### Q4：为什么 headless 客户端是必需的？

- CI/CD 环境无显示器，需要跑自动化测试。
- 部署到工地服务器时不一定有桌面环境。
- headless 模式输出的 JSON 可被 Web 前端消费（保留现有 dashboard 路线）。

### Q5：为什么 PM 标定模型在客户端而不是服务端？

- 服务端只管"如实"生成原始数据（模拟真实 LiDAR 设备的行为）。
- PM 标定依赖地面站数据，属于"主控端的知识"，不应下沉到设备端。
- 这样将来换真实 LiDAR 设备时，标定逻辑不需要跟着设备走。
