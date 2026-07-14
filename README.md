# YLJ5 颗粒物激光雷达数字孪生

本仓库只维护一条工程主线：在 Linux 上仿真无锡中科光电
`YLJ5 / AGHJ-I-LIDAR(MPL)`，通过 TCP JSONL 向 Qt 6 图形客户端实时发送数据，
并在客户端工作线程完成 L0-L2 处理和显示。

当前实现属于“公开规格数字孪生”，不是厂商私有协议的逐字节克隆。532 nm、3.75 m、
不小于 20 km、双望远镜、偏振、同步相机和三种扫描能力有公开证据；PRF、脉冲能量、
接收增益、overlap、厂家命令字和原始线协议仍是明确标注的仿真假设或未知项。证据边界见
[YLJ5 保真说明](docs/ylj5_fidelity.md)。

## 环境

- Linux
- CMake 3.20+
- 支持 C++20 的 GCC 或 Clang
- Qt 6.5+：Core、Gui、Widgets、Network

CMake 会自动搜索 `/opt/Qt6*/6.*/gcc_64` 中版本最高的 Qt，也可显式传入
`LIDAR_QT_ROOT`。

## 构建与测试

```bash
make                 # 构建服务端、客户端、测试和教学算例
make server          # bin/lidar_sim_server
make client          # bin/lidar_gui
make examples        # 三个纯 C++ 教学算例
make test            # 构建并运行五项设备测试
make clean           # 删除 build/
make pristine        # 删除 build/ 和 bin/
```

直接使用 CMake：

```bash
cmake -S . -B build \
  -DLIDAR_QT_ROOT=/opt/Qt6.9.3/6.9.3/gcc_64 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

当前测试：

- `lidar_device_playback_timing_test`：采集驻留和播放倍率；
- `lidar_client_device_telemetry_test`：状态增量合并和周期完整性；
- `lidar_client_frame_processor_test`：四通道拼接、逐 bin QC、反演和组合标定边界；
- `lidar_client_sim_device_pipeline_test`：5334 距离门仿真设备到客户端的端到端处理；
- `lidar_sim_device_ylj5_test`：公开规格、帧协议、扫描轮换和控制命令。

完整规格双周期正演可单独执行：

```bash
./bin/lidar_sim_device_ylj5_test --full-smoke
```

## 运行

终端 1 启动仿真设备：

```bash
./bin/lidar_sim_server
```

终端 2 启动 GUI 并自动连接：

```bash
./bin/lidar_gui --host 127.0.0.1 --port 19850 --connect
```

服务端参数为：

```text
lidar_sim_server [port] [cycle_delay_ms] [playback_time_scale]
```

默认值为 `19850 0 1`。`playback_time_scale=1` 按配置采集节奏发送；调试时可用
`100` 加速，倍率只改变等待时间，不改变积分脉冲数和正演数据。

GUI 还支持离屏截图：

```bash
QT_QPA_PLATFORM=offscreen ./bin/lidar_gui --screenshot /tmp/ylj5.png
```

## 默认设备周期

设备按周期惰性生成数据，不在启动时预生成整场战役。新连接先发送一次 `status`，之后
每个完整周期的顺序如下：

| 顺序 | 帧 | 默认数量 | 内容 |
|---|---|---:|---|
| 1 | `telemetry` | 1 | 合成温湿度、电子学温度、云台和运行状态 |
| 2 | `lidar_raw` | 1 | 90 度垂直观测，30 秒积分 |
| 3 | `lidar_raw` | 180 | 0-360 度、2 度步进的固定仰角方位扫描 |
| 4 | `camera` | 1 | 相机能力和同步指向元数据，不伪造图像 |
| 5 | `lidar_product` | 1 | 未标定信号/退偏比快速摘要，不是定量 PM |
| 6 | `ground_obs` | 0 | 默认关闭的合成地面观测 |
| 7 | `heartbeat` | 1 | 周期封口和进度 |

方位射线默认驻留 2 秒，云台移动约 0.2 秒；180 条约 396 秒，加 30 秒垂直观测和
10 秒回扫开销，一个完整周期约 436 秒。方位扫描仰角按周期轮换：

```text
周期 0：90 度垂直观测 + 0 度水平扫描
周期 1：90 度垂直观测 + 5 度锥形扫描
周期 2：重复周期 0
```

`0/5 度 round_robin` 是有相近论文依据、但尚待实机确认的调度假设。

每条射线包含 5334 个 3.75 m 距离门，最大距离轴 20002.5 m，以及四条物理接收路径：

- `near_parallel_532nm`
- `near_perpendicular_532nm`
- `far_parallel_532nm`
- `far_perpendicular_532nm`

外层 `raw_counts` 是近、远场平行通道的拼接主通道。实时帧默认不发送仿真真值。

## 客户端处理

`LidarClientWorker` 在专用 `QThread` 中执行 socket 收包、JSONL 拆帧、设备状态合并、
扫描完整性检查、四通道处理、Fernald/Klett 反演、湿度修正、ENU 投影和显示快照生成。
GUI 主线程只绘图和响应操作。

未同时加载接收机标定和站点 PM 标定时，客户端只显示干消光、退偏比和 QC，PM2.5、
PM10 与热点保持为空。组合标定 JSON 必须包含：

```json
{
  "calibration_id": "site-cal-001",
  "valid_from": "2026-07-01",
  "pm25_intercept_ugm3": 0.0,
  "pm25_slope_ugm3_per_km": 0.0,
  "pm10_intercept_ugm3": 0.0,
  "pm10_slope_ugm3_per_km": 0.0,
  "receiver_calibration": {
    "calibration_id": "device-serial-receiver-cal-001",
    "detector_mode": "photon_counting",
    "signal_unit": "mean_counts_per_pulse",
    "range_zero_offset_m": 0.0,
    "minimum_valid_range_m": 3.75,
    "minimum_retrieval_overlap": 0.1,
    "minimum_quantitative_overlap": 0.9,
    "saturation_counts": 2000000.0,
    "saturation_guard_fraction": 0.98,
    "dead_time_loss_per_count": 0.0000025,
    "maximum_dead_time_occupancy": 0.8,
    "afterpulse_kernel": [0.02]
  }
}
```

PM 系数必须来自目标站点共址实验，接收机字段必须来自对应设备序列号的暗帧、线性度、
overlap 和距离零点标定。示例数值只匹配当前仿真器，不能作为实机标定使用。模板见
`configs/ylj5_calibration.template.json`，完整流程见
[客户端算法初学者指南与当前实现](docs/algorithm_processing_chain.md)。

## 控制命令

GUI 可发送 `start`、`resume`、`pause`、`stop`、`get_status` 和 `set_scan`。服务端返回
`command_result`。默认启用公开规格校验，低于 180 条射线、每条少于 10000 脉冲或
超过公开云台边界的配置会被拒绝，并返回 `violations`。

## 教学资料

`docs/0-21`、`docs/pdf/`、根目录教学手册、图片和 `cpp/examples/` 是知识学习内容，
不等同于当前设备的厂家实现，继续保留。

三个独立算例不依赖设备运行时：

```bash
./bin/lidar_example_1d_ray
./bin/lidar_example_2d_ppi
./bin/lidar_example_3d_rhi
```

## 目录

| 路径 | 作用 |
|---|---|
| `cpp/server/` | YLJ5 规格、正演设备、扫描时序和 POSIX TCP 服务 |
| `cpp/protocol/` | 当前仿真层 JSONL 帧和四通道序列化 |
| `cpp/client/` | Linux Qt GUI、工作线程、实时处理和可视化 |
| `cpp/include/`、`cpp/src/` | 正演与实时反演共享核心 |
| `cpp/tests/` | 五项设备回归测试 |
| `cpp/examples/` | 纯 C++ 教学算例 |
| `docs/` | 设备工程文档和知识学习资料 |
| `experiments/` | YLJ5 实验记录模板 |

工程说明继续阅读：

- [项目补充说明](docs/project_supplement.md)
- [YLJ5 正演与运行模型](docs/realistic_lidar_simulation.md)
- [厂商协议还原边界](docs/vendor_frame_reconstruction.md)
- [12 周实机落地计划](docs/12_week_plan.md)
