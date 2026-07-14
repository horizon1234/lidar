# LiDAR 正演仿真、YLJ5 设备层与 3D 污染监测

本文说明当前仓库的两类仿真用途、信号模型、实时帧协议和工程边界。项目同时保留
“便于算法实验的通用批处理场景”和“面向指定设备的实时仿真”，两者不能混为一套
厂家参数。

## 1. 两条仿真链路

| 链路 | 入口 | 目标 | 参数语义 |
|---|---|---|---|
| 通用批处理 | `lidar_run_batch`、`lidar_build_demo_assets` | 算法开发、消融、评测、L3 产品 | 面向场景的工程配置，不代表具体厂家 |
| YLJ5 实时设备 | `lidar_sim_server` | 还原 YLJ5 / AGHJ-I-LIDAR(MPL) 的公开行为 | 官网确认、采购边界、仿真假设和未知项分层管理 |

`configs/FieldScanningLidar.json`、`configs/MobileMappingLidar.json` 和
`configs/CommercialPmeyeSector.json` 属于第一条链路。它们可以使用 1064 nm、355 nm、
6 km 或其他适合算法实验的参数，但不能用来描述 YLJ5。

实时服务端只以无锡中科光电 `YLJ5 / AGHJ-I-LIDAR(MPL)` 为目标。默认采用 532 nm、
3.75 m 距离分辨率、至少 20 km、双望远镜和偏振四通道。完整证据等级、采购边界与
未知项见 [YLJ5 仿真保真说明](ylj5_fidelity.md)。

## 2. YLJ5 默认扫描计划

采购文件确认软件支持水平扫描、垂直观测和锥形扫描，但没有公开厂家默认的自动顺序。
当前设备层采用可配置的 `round_robin` 仿真调度：

- 周期 0 先做 90 度垂直观测，再以 0 度仰角做水平扫描；
- 周期 1 先做 90 度垂直观测，再以 5 度仰角做锥形扫描；
- 周期 2 回到周期 0，后续按 0/5 度逐周期轮换；
- 每圈方位范围 0-360 度、2 度步进，共 180 条射线；
- 每条方位射线驻留 2 秒；假设 PRF 为 5 kHz，因此积分 10000 个脉冲；
- 云台按 10 度/秒运动，单步运动耗时约 0.2 秒；
- 一圈固定开销 10 秒，方位扫描约 406 秒；
- 每周期的 90 度垂直观测为 30 秒，完整周期约 436 秒；
- 距离轴包含 5334 个 bin，末端为 20002.5 m。

`5°` 来自相近公开扫描系统的论文，不是 YLJ5 厂家标称角度；自动轮换顺序同样属于
等待实机配置页或控制抓包确认的仿真假设。选择逐周期单层扫描，是为了模拟云台按任务
切换仰角，并把峰值内存保持在一个 181 条射线周期，而不是一次生成两个方位圈。

正式配置默认启用 `enforce_public_spec`。控制命令若把射线数、积分脉冲数、量程、
扫描速度或周期时长改到公开边界之外，会返回失败的 `command_result`，原配置保持不变。

## 3. 正演信号模型

通用批处理和 YLJ5 实时设备共用离散 LiDAR 方程：

```text
P(r) = C * E * O(r) * beta(r) * exp(-2 * tau(r)) / r^2 + background
```

模型包含以下非理想因素：

- `pulse_energy_mj` 和 `pulse_energy_jitter`：单脉冲能量与周期抖动；
- `background_counts_mean`、`detector_dark_counts`：太阳背景和探测器暗计数；
- 小信号 Poisson 采样、大信号高斯近似：积分脉冲统计噪声；
- `dead_time_loss`、`afterpulsing_ratio`：死时间压缩和强信号后拖尾；
- `adc_saturation_counts`：极端近场强信号的饱和保护；
- `overlap`：近场发射光束与接收视场未完全重合。

每条输出 profile 是 `PRF × dwell` 个脉冲的平均值，单位为
`mean_counts_per_pulse`。读噪声按积分脉冲数平方根衰减，避免把单脉冲噪声直接叠加到
长时间积分结果。

## 4. 双望远镜和偏振通道

YLJ5 设备层生成四条物理接收路径：

| 通道 ID | 望远镜 | 偏振 |
|---|---|---|
| `near_parallel_532nm` | 近场 | 平行 |
| `near_perpendicular_532nm` | 近场 | 垂直 |
| `far_parallel_532nm` | 远场 | 平行 |
| `far_perpendicular_532nm` | 远场 | 垂直 |

每条通道保存独立的 `raw_counts`、`overlap`、相对增益、背景计数、波长和望远镜口径。
外层 profile 额外保留 `depolarization_ratio`。

为兼容现有单通道反演链，外层 `raw_counts` 是近场和远场平行偏振通道在约 150 m
附近平滑拼接后的主通道。150 m、增益和 overlap 曲线都没有实机标定证据，状态帧会把
它们列入未验证参数。

## 5. 实时 JSONL 帧协议

当前协议是项目自定义的 `jsonl-ylj5-emulator-1.0`，不是厂商私有线协议。每帧是一行
JSON 对象，`type` 和 `timestamp` 位于顶层。

| 帧类型 | 方向 | 用途 |
|---|---|---|
| `status` | 服务端 -> 客户端 | 型号、公开规格、扫描计划、校准状态和未知参数 |
| `telemetry` | 服务端 -> 客户端 | 合成温湿压、温度、功耗、云台和相机状态 |
| `lidar_raw` | 服务端 -> 客户端 | 距离轴、兼容主通道、四物理通道和采集时序 |
| `camera` | 服务端 -> 客户端 | 同步相机能力和雷达指向；当前不包含伪造图像 |
| `lidar_product` | 服务端 -> 客户端 | 未标定的信号和退偏比快速摘要 |
| `ground_obs` | 服务端 -> 客户端 | 可选的合成地面 PM 与气象观测 |
| `heartbeat` | 服务端 -> 客户端 | 周期完成进度 |
| `command` | 客户端 -> 服务端 | 启停、暂停、状态查询和扫描配置 |
| `command_result` | 服务端 -> 客户端 | 命令是否接受、失败原因和生效配置摘要 |
| `alarm` | 服务端 -> 客户端 | 周期生成失败等设备级错误 |

`telemetry` 中的温度、压力和功耗明确标记为 `synthetic_not_vendor_spec`。官网只证明存在
同步相机能力，因此 `camera` 帧当前只发能力元数据，并通过 `image_available=false`
说明没有真实图像。实时协议默认关闭分子场和仿真真值，避免把验证字段误当成观测量。
`lidar_raw` 继续使用通用的 `scan_mode=ppi/stare`，同时用 `device_scan_pattern` 明确区分
水平、垂直和锥形观测，并携带当前轮换索引及计划仰角。

## 6. 惰性生成和内存模型

默认一条射线包含 5334 个距离门和四条物理通道。一旦预生成 180 个周期，仅浮点数组
就会达到数十 GB，因此设备初始化只做规格校验，不生成回波。

`generate_synthetic_campaign` 每次只正演一个周期，不运行反演、评测和磁盘 JSON 中转。
`stream_scan_cycle` 再把该周期逐帧交给 TCP 层，单条 JSON 发送后立即释放。当前双周期
全规格烟雾测试覆盖 180 条水平、180 条锥形和 2 条垂直射线，共 362 条；每条 5334 bins，
生成约 13.74 秒，最大常驻内存约 155.4 MiB。该结果是当前开发机的工程基线，不是设备
性能指标。

## 7. Linux Qt 实时客户端

`lidar_gui` 是当前唯一主控客户端。构建要求 Linux 和 Qt 6.5+；CMake 会自动搜索
`/opt/Qt6*/6.*/gcc_64`，也可通过 `LIDAR_QT_ROOT` 指定。典型启动方式：

```bash
cmake -S . -B build -DLIDAR_QT_ROOT=/opt/Qt6.9.3/6.9.3/gcc_64
cmake --build build --target lidar_sim_server lidar_gui -j$(nproc)
./bin/lidar_sim_server &
./bin/lidar_gui --host 127.0.0.1 --port 19850 --connect
```

客户端使用专用 `QThread` 持有 socket 并执行全部数据处理，GUI 只展示 PPI、垂直廓线、
设备扫描/仰角状态、传输统计、热点和 QC。没有站点标定时默认显示干消光，不显示伪造的
定量 PM；标定合同见 [客户端实时处理链](algorithm_processing_chain.md)。

## 8. 污染场和 L3 产品

通用正演场景包含边界层背景、高架输送层、风驱动工地烟羽、城市交通源和走航车近源
扬尘。烟羽位置随风速、风向和时间平流，扩散宽度随时间增长。

批处理 demo 会把多仰角 PPI 射线聚合成 ENU 体素：

```json
{
  "coordinate_system": "ENU",
  "voxel_size_m": 100.0,
  "voxels": [
    {
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "pm25": 0.0,
      "confidence": 0.0,
      "source_ray_count": 1
    }
  ]
}
```

`confidence` 由 SNR 映射。前端应同时显示浓度、置信度、热点质心、风向和扫描覆盖范围，
不能把低可信区域绘制成确定污染云。

## 9. 工程边界

当前实现已经覆盖公开规格校验、四通道正演、逐帧协议、控制状态机和客户端兼容，但
以下内容仍需实机资料：

- 厂商帧头、校验、字节序、命令字、故障码和启动时序；
- PRF、脉冲能量、探测器类型、ADC 单位、通道增益和 overlap 标定；
- 偏振串扰矩阵、距离零点、同步相机编码和外参；
- 厂商 L1/L2 产品算法和上位机文件格式。

接入真实设备时应新增 `VendorProtocolAdapter`，把厂商帧转换为现有 `LidarProfile` 和
协议模型。`SimDevice` 应继续作为可复现的测试替身，而不是改名冒充真实驱动。
