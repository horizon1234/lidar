# lidar_raw 帧实例与字段说明

本文使用 `logs/lidar_gui.log` 中 2026-07-15 11:43:09.361 打印的一条
`[client] lidar_raw frame preview:`。这条帧属于 `ylj5-sim-01_cycle_11`，是该周期
第 0 条射线，也就是 90 度垂直观测。

## 1. 阅读前先明确三件事

1. 下面是客户端日志生成的 JSON 预览，不是厂家私有线协议。当前仿真协议是项目自定义的
   TCP JSONL，每一行是一个完整 JSON 对象。
2. `_truncated` 和 `omitted_count` 由客户端日志预览函数添加，不存在于网络原始帧中。
   每个长数组实际有 5334 项，日志保留前 12 项并说明省略了 5322 项。
3. 当前线上 JSON 是扁平结构。解析后 C++ `Frame` 把 `type` 保存为枚举，把
   `timestamp` 和其余字段保存在 `payload` JSON 对象中。

## 2. 从日志复制的完整预览

日志时间、线程 ID 和源码位置已经去掉，只保留有效 JSON 预览：

~~~json
{
  "acquisition_end_offset_s": 30,
  "acquisition_end_timestamp": "2026-07-15T11:43:30.157",
  "acquisition_start_offset_s": 0,
  "acquisition_start_timestamp": "2026-07-15T11:43:00.157",
  "azimuth_deg": 0,
  "azimuth_encoder_deg": -0.015,
  "azimuth_scan_pattern": "conical_scan",
  "background_counts": 84.84793,
  "calibration_status": "assumed_pending_real_device_capture",
  "channel_count": 4,
  "channels": [
    {
      "background_counts": 84.84793,
      "channel_id": "near_parallel_532nm",
      "device_bin_quality": [],
      "overlap": [
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        {
          "_truncated": true,
          "omitted_count": 5322
        }
      ],
      "polarization": "parallel",
      "raw_counts": [
        399879.333401,
        399559.987688,
        398978.707558,
        398169.768872,
        397127.839851,
        395867.810461,
        394376.696452,
        392670.009296,
        390750.920828,
        388616.473738,
        386282.709907,
        383755.251818,
        {
          "_truncated": true,
          "omitted_count": 5322
        }
      ],
      "relative_gain": 0.08,
      "telescope": "near",
      "telescope_aperture_mm": 40,
      "wavelength_nm": 532
    },
    {
      "background_counts": 84.84793,
      "channel_id": "near_perpendicular_532nm",
      "device_bin_quality": [],
      "overlap": [
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        1,
        {
          "_truncated": true,
          "omitted_count": 5322
        }
      ],
      "polarization": "perpendicular",
      "raw_counts": [
        399114.855479,
        396732.145592,
        392466.624069,
        386674.970072,
        379449.528769,
        370959.150244,
        361378.06269,
        350901.008638,
        339707.169907,
        327988.050212,
        315917.328798,
        303651.147803,
        {
          "_truncated": true,
          "omitted_count": 5322
        }
      ],
      "relative_gain": 0.08,
      "telescope": "near",
      "telescope_aperture_mm": 40,
      "wavelength_nm": 532
    },
    {
      "background_counts": 84.84793,
      "channel_id": "far_parallel_532nm",
      "device_bin_quality": [],
      "overlap": [
        0.058315,
        0.102949,
        0.143554,
        0.181747,
        0.218239,
        0.253432,
        0.287579,
        0.320856,
        0.353391,
        0.38528,
        0.4166,
        0.44741,
        {
          "_truncated": true,
          "omitted_count": 5322
        }
      ],
      "polarization": "parallel",
      "raw_counts": [
        399834.669009,
        399646.653867,
        399424.78598,
        399185.301008,
        398937.467047,
        398680.64008,
        398417.060033,
        398140.282739,
        397866.697243,
        397577.499064,
        397285.920468,
        396989.911707,
        {
          "_truncated": true,
          "omitted_count": 5322
        }
      ],
      "relative_gain": 1,
      "telescope": "far",
      "telescope_aperture_mm": 160,
      "wavelength_nm": 532
    },
    {
      "background_counts": 84.84793,
      "channel_id": "far_perpendicular_532nm",
      "device_bin_quality": [],
      "overlap": [
        0.058315,
        0.102949,
        0.143554,
        0.181747,
        0.218239,
        0.253432,
        0.287579,
        0.320856,
        0.353391,
        0.38528,
        0.4166,
        0.44741,
        {
          "_truncated": true,
          "omitted_count": 5322
        }
      ],
      "polarization": "perpendicular",
      "raw_counts": [
        398783.319134,
        397373.038767,
        395716.427009,
        393976.54439,
        392172.946824,
        390318.129119,
        388415.973768,
        386478.345749,
        384514.850743,
        382523.857377,
        380514.901277,
        378490.266683,
        {
          "_truncated": true,
          "omitted_count": 5322
        }
      ],
      "relative_gain": 1,
      "telescope": "far",
      "telescope_aperture_mm": 160,
      "wavelength_nm": 532
    }
  ],
  "data_provenance": "synthetic_public_spec_emulator",
  "depolarization_ratio": [
    0.133365,
    0.133368,
    0.133372,
    0.133375,
    0.133378,
    0.133381,
    0.133384,
    0.133388,
    0.133391,
    0.133394,
    0.133397,
    0.1334,
    {
      "_truncated": true,
      "omitted_count": 5322
    }
  ],
  "detector_mode": "simulated_photon_counting",
  "device_bin_quality": [],
  "device_scan_pattern": "vertical_observation",
  "elevation_deg": 90,
  "elevation_encoder_deg": 90.000044,
  "elevation_schedule_index": 1,
  "elevation_schedule_length": 2,
  "frame_id": "ylj5-sim-01_cycle_11_ray_0",
  "integrated_pulses": 150000,
  "laser_energy_mj": 0.020307,
  "line_dwell_s": 30,
  "maximum_range_m": 20002.5,
  "motion_overhead_s": 0,
  "overlap": [
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    {
      "_truncated": true,
      "omitted_count": 5322
    }
  ],
  "primary_channel_id": "stitched_parallel_532nm",
  "publish_offset_s": 30,
  "publish_timestamp": "2026-07-15T11:43:30.157",
  "pulse_repetition_hz": 5000,
  "qc_hint": {
    "background_counts": 84.84793,
    "laser_energy_mj": 0.020307,
    "near_range_overlap_limited": false
  },
  "range_resolution_m": 3.75,
  "range_unit": "m",
  "ranges_m": [
    3.75,
    7.5,
    11.25,
    15,
    18.75,
    22.5,
    26.25,
    30,
    33.75,
    37.5,
    41.25,
    45,
    {
      "_truncated": true,
      "omitted_count": 5322
    }
  ],
  "raw_counts": [
    4997515.916322,
    4993524.094904,
    4986258.093282,
    4976146.359702,
    4963122.246943,
    4947371.879571,
    4928732.954454,
    4907399.365006,
    4883410.759158,
    4856730.170531,
    4827558.12264,
    4795964.896532,
    {
      "_truncated": true,
      "omitted_count": 5322
    }
  ],
  "ray_index": 0,
  "rays_in_cycle": 181,
  "relative_humidity": 0.436014,
  "scan_cycle_id": "ylj5-sim-01_cycle_11",
  "scan_cycle_timestamp": "2026-07-15T11:43:00.157",
  "scan_id": "ylj5-sim-01_cycle_11_ray_0",
  "scan_mode": "stare",
  "scheduled_elevation_deg": 5,
  "sequence_id": 2121,
  "signal_unit": "mean_counts_per_pulse",
  "simulated_receiver_response": {
    "afterpulsing_ratio": 0.02,
    "calibration_status": "assumed_pending_real_device_capture",
    "dead_time_loss_per_count": 0.000003,
    "far_full_overlap_m": 120,
    "near_full_overlap_m": 3.75,
    "saturation_counts": 2000000,
    "stitch_range_m": 150
  },
  "site_id": "ylj5-sim-01",
  "source_kind": "ylj5_synthetic_stare",
  "temperature_c": 25.994242,
  "timestamp": "2026-07-15T11:43:00.157",
  "type": "lidar_raw",
  "vendor_wire_protocol_emulated": false,
  "wavelength_nm": 532,
  "wind_dir_deg": 140.673502,
  "wind_speed_ms": 3.528904
}
~~~

## 3. 这条帧表示什么

这不是一整个扫描周期，而是一个周期中的一条射线。当前默认周期包含：

~~~text
ray_index=0       90 度垂直观测 1 条
ray_index=1..180  固定仰角方位扫描 180 条
~~~

样例的 `rays_in_cycle=181`、`ray_index=0`、`scan_mode=stare` 和
`device_scan_pattern=vertical_observation` 共同说明它是周期开头的垂直观测。

`scheduled_elevation_deg=5` 和 `azimuth_scan_pattern=conical_scan` 描述的是本周期随后
180 条方位射线采用的 5 度锥扫计划，不表示当前垂直射线也是 5 度。当前射线自己的仰角
由 `elevation_deg=90` 表示。

## 4. 帧身份、来源与标定状态

| 字段 | 样例值 | 作用 |
|---|---|---|
| `type` | `lidar_raw` | 协议帧类型。客户端据此进入原始雷达射线处理分支。 |
| `timestamp` | `2026-07-15T11:43:00.157` | 帧的统一时间戳。当前仿真器让同一周期的所有射线共用该值，客户端也用它聚合和封口周期。 |
| `site_id` | `ylj5-sim-01` | 数据所属站点或设备实例标识，用于多站点区分和结果追溯。 |
| `sequence_id` | `2121` | 设备侧跨帧递增序号。它覆盖不同帧类型，可作为链路级跳号分析依据，不能当作周期内射线序号；当前客户端尚未用它统计缺帧。 |
| `frame_id` | `ylj5-sim-01_cycle_11_ray_0` | 单帧唯一标识。当前仿真器把它设置为射线的 `scan_id`。 |
| `scan_id` | `ylj5-sim-01_cycle_11_ray_0` | `LidarProfile` 内的单射线标识。当前值与 `frame_id` 相同，但接口上是两个不同语义的字段。 |
| `data_provenance` | `synthetic_public_spec_emulator` | 数据来源声明，明确表示来自公开规格数字孪生，不是实机采集。 |
| `source_kind` | `ylj5_synthetic_stare` | 更细的数据来源和扫描类型标签。该值表示仿真的垂直凝视射线。 |
| `calibration_status` | `assumed_pending_real_device_capture` | 标定可信度声明。表示当前接收机参数是仿真假设，仍等待真实设备采集和标定。 |
| `vendor_wire_protocol_emulated` | `false` | 明确声明当前 JSONL 没有冒充厂商私有线协议。 |

## 5. 周期、射线序号与扫描模式

| 字段 | 样例值 | 作用 |
|---|---|---|
| `scan_cycle_id` | `ylj5-sim-01_cycle_11` | 扫描周期唯一 ID。`ScanCycleMonitor` 用它归组射线并统计缺帧、重复帧。 |
| `scan_cycle_timestamp` | `2026-07-15T11:43:00.157` | 周期起始参考时间。当前值和顶层 `timestamp` 相同。 |
| `ray_index` | `0` | 当前射线在周期内的零基索引。合法范围是 0 到 `rays_in_cycle-1`。 |
| `rays_in_cycle` | `181` | 设备声明的本周期射线总数。客户端用它判断是否收齐。 |
| `scan_mode` | `stare` | 公共 `LidarProfile` 的处理模式。`stare` 进入垂直廓线，`ppi` 进入方位平面产品。 |
| `device_scan_pattern` | `vertical_observation` | 当前这一条射线的设备级扫描模式。 |
| `azimuth_scan_pattern` | `conical_scan` | 本周期方位扫描部分的模式。5 度固定仰角会形成锥形扫描。 |
| `scheduled_elevation_deg` | `5` | 本周期方位扫描计划使用的固定仰角，不是当前垂直射线的实际仰角。 |
| `elevation_schedule_index` | `1` | 当前周期在仰角轮换表中的位置。样例为第二项。 |
| `elevation_schedule_length` | `2` | 仰角轮换表长度。当前默认轮换表是 0 度和 5 度两项。 |

## 6. 指向角和编码器角

| 字段 | 样例值 | 单位 | 作用 |
|---|---:|---|---|
| `azimuth_deg` | 0 | 度 | 计划或公共模型中的方位角，约定正北为 0 度并顺时针增加。垂直指向时方位角在几何上不影响射线路径。 |
| `elevation_deg` | 90 | 度 | 计划仰角。0 度是水平，90 度是天顶。 |
| `azimuth_encoder_deg` | -0.015 | 度 | 仿真的云台方位编码器读数，加入了很小的指向误差。实机适配器应映射真实编码器值。 |
| `elevation_encoder_deg` | 90.000044 | 度 | 仿真的云台俯仰编码器读数，用于比较计划角和实际角。 |

当前算法主要从 `LidarProfile` 使用 `azimuth_deg` 和 `elevation_deg` 做 ENU 投影。
编码器字段目前作为设备遥测和未来实机适配信息保留，尚未覆盖公共模型中的计划角。

## 7. 采集、运动与发布时间

| 字段 | 样例值 | 单位 | 作用 |
|---|---:|---|---|
| `acquisition_start_offset_s` | 0 | s | 当前射线开始采集相对周期起点的偏移。 |
| `acquisition_end_offset_s` | 30 | s | 当前射线结束采集相对周期起点的偏移。 |
| `publish_offset_s` | 30 | s | 数据预计可发布的相对偏移，等于采集结束加本射线运动开销。 |
| `acquisition_start_timestamp` | `2026-07-15T11:43:00.157` | ISO8601 | 当前射线的模拟采集开始时间。 |
| `acquisition_end_timestamp` | `2026-07-15T11:43:30.157` | ISO8601 | 当前射线的模拟采集结束时间。 |
| `publish_timestamp` | `2026-07-15T11:43:30.157` | ISO8601 | 当前射线按物理采集节奏应发布的时间。 |
| `line_dwell_s` | 30 | s | 本射线积分驻留时间。垂直观测默认 30 秒，方位射线默认 2 秒。 |
| `motion_overhead_s` | 0 | s | 本射线结束后的云台运动和稳定耗时。垂直观测为 0，方位射线默认约 0.2 秒。 |

日志头时间是客户端实际打印日志的墙钟时间。内嵌采集时间描述仿真物理时间；启用播放
加速后，墙钟时间可以早于 `publish_timestamp`，两者不能混为一个时间轴。

## 8. 激光、探测器和采样合同

| 字段 | 样例值 | 单位 | 作用 |
|---|---:|---|---|
| `wavelength_nm` | 532 | nm | 当前激光和接收通道中心波长。客户端标准大气回退和分子参考需要它。 |
| `pulse_repetition_hz` | 5000 | Hz | 激光脉冲重复频率，即每秒发射脉冲数。 |
| `integrated_pulses` | 150000 | 个 | 本射线累计的脉冲数。样例满足 5000 Hz × 30 s = 150000。 |
| `laser_energy_mj` | 0.020307 | mJ | 单脉冲激光能量。客户端用它做能量归一化；非正或无效值会屏蔽反演产品。 |
| `detector_mode` | `simulated_photon_counting` | 无 | 探测模式。客户端把该仿真标签归一化为 `photon_counting`，从而应用光子计数响应合同。 |
| `signal_unit` | `mean_counts_per_pulse` | 无 | `raw_counts` 的数值单位，表示已经按积分脉冲数归一化的单脉冲平均计数。 |
| `primary_channel_id` | `stitched_parallel_532nm` | 无 | 外层兼容主通道的标识，说明它由近、远场平行偏振通道拼接得到。 |
| `channel_count` | 4 | 个 | `channels` 中物理接收路径数量。当前为双望远镜乘以两种偏振。 |

`signal_unit` 很重要。若实机输出的是累计计数，应声明 `integrated_counts`，客户端才能先除以
`integrated_pulses`。单位声明和接收机标定不一致时，处理器会拒绝该射线。

## 9. 距离轴和逐 bin 主数组

| 字段 | 样例值 | 作用 |
|---|---|---|
| `range_unit` | `m` | 距离轴单位。当前固定为米。 |
| `range_resolution_m` | 3.75 | 相邻距离门间隔。它对应光往返时间采样后的空间分辨率。 |
| `maximum_range_m` | 20002.5 | 当前仿真距离轴上限，略大于 20 km。 |
| `ranges_m` | `[3.75, 7.5, ...]` | 每个 bin 对应的距离坐标。完整数组 5334 项，从 3.75 m 到 20002.5 m。 |
| `raw_counts` | `[4997515.916322, ...]` | 近、远场平行偏振通道拼接后的兼容主通道计数，与 `ranges_m` 一一对齐。 |
| `overlap` | `[1, 1, ...]` | 兼容主通道的等效几何重叠因子，范围通常为 0 到 1，与距离轴对齐。 |
| `depolarization_ratio` | `[0.133365, ...]` | 每个距离门的体退偏比，用于辅助区分球形气溶胶和非球形颗粒物。 |
| `device_bin_quality` | `[]` | 上游设备或适配器提供的逐 bin 质量位。空数组表示当前仿真器没有提供上游屏蔽信息，不等于算法已经证明全部 bin 有效。 |
| `background_counts` | 84.84793 | 外层主通道的单脉冲等效背景计数，是背景拟合不足时的回退基线。 |

客户端数值算法以完整 `ranges_m` 为准，`range_resolution_m`、`maximum_range_m` 和
`range_unit` 当前主要用于协议自描述和诊断。真实协议适配时应校验这些声明与实际距离轴
一致，不能仅相信元数据。

逐 bin 数组在语义上应按同一索引对齐。例如索引 0 同时表示 3.75 m 处的主计数、
overlap、退偏比和质量位。当前客户端会强制检查主 `ranges_m/raw_counts/overlap`、每个
物理通道的 `raw_counts/overlap`，以及非空 `device_bin_quality` 的长度；这些数组一旦
错位就拒绝整条射线。`depolarization_ratio` 目前没有同等级的长度拒绝检查，实机适配器
仍应保证它与距离轴对齐。

外层 `raw_counts` 是为了兼容只提供单主通道的设备适配器。当前四通道数据齐全时，
`FrameProcessor` 会优先处理 `channels` 中的近、远场物理通道，再重新完成校正和拼接。
因此外层主通道不是某一个真实 ADC 的直接读数，也不应单独拿来解释探测器饱和。

## 10. 四个物理接收通道

样例的 `channels` 包含以下四种组合：

| channel_id | telescope | polarization | 口径 | relative_gain |
|---|---|---|---:|---:|
| `near_parallel_532nm` | near | parallel | 40 mm | 0.08 |
| `near_perpendicular_532nm` | near | perpendicular | 40 mm | 0.08 |
| `far_parallel_532nm` | far | parallel | 160 mm | 1 |
| `far_perpendicular_532nm` | far | perpendicular | 160 mm | 1 |

每个通道对象的成员含义如下：

| 字段 | 作用 |
|---|---|
| `channel_id` | 稳定通道 ID，编码近/远场、偏振方向和波长。算法通过 ID 和其他元数据识别接收路径。 |
| `telescope` | 望远镜路径。`near` 负责近场，`far` 负责远场。 |
| `polarization` | 偏振分量。`parallel` 是平行偏振，`perpendicular` 是垂直偏振。 |
| `wavelength_nm` | 当前通道中心波长。四个样例通道都是 532 nm。 |
| `telescope_aperture_mm` | 接收望远镜有效口径。当前 40/160 mm 是公开采购边界下的仿真假设，不是实机标定值。 |
| `relative_gain` | 相对远场平行通道的接收链增益。客户端在通道融合前用它归一化通道幅度。 |
| `background_counts` | 当前物理通道的单脉冲等效背景基线。客户端优先从远端数据拟合背景，样本不足时回退到此值。 |
| `raw_counts` | 当前物理通道的原始计数数组，与外层 `ranges_m` 等长。它才对应具体接收路径的探测器输出。 |
| `overlap` | 当前物理通道的几何重叠曲线，与外层 `ranges_m` 对齐。客户端除以 overlap 前会执行有效区门控。 |
| `device_bin_quality` | 当前通道上游提供的逐 bin 质量位。样例为空，后续算法仍会自行生成饱和、低 SNR、overlap 不足等质量位。 |

近场通道在第一个 3.75 m bin 已达到仿真设定的完整 overlap，所以预览值都是 1。远场
通道的 overlap 从约 0.058 开始随距离增加，反映发射光束和远场接收视场在近距离尚未
充分重合。

平行和垂直偏振通道的计数差异用于计算退偏比。当前 `depolarization_ratio` 已由仿真器
生成并作为兼容字段发送；真实设备接入时仍需要处理偏振串扰、增益差和标定矩阵。

## 11. simulated_receiver_response

这个对象描述仿真器生成计数时采用的接收机响应假设：

| 字段 | 样例值 | 作用 |
|---|---:|---|
| `calibration_status` | `assumed_pending_real_device_capture` | 再次声明这些响应参数尚未由实机标定。 |
| `afterpulsing_ratio` | 0.02 | 仿真后脉冲泄漏比例，表示强信号的一部分影响后续距离门。 |
| `dead_time_loss_per_count` | 0.000003 | 仿真光子计数死时间软压缩系数。计数越高，丢失效应越明显。 |
| `saturation_counts` | 2000000 | 仿真单物理计数通道的饱和上限。 |
| `near_full_overlap_m` | 3.75 | 近场通道达到完整几何重叠的假设距离。 |
| `far_full_overlap_m` | 120 | 远场通道达到完整几何重叠的假设距离。 |
| `stitch_range_m` | 150 | 仿真器生成兼容主通道时近、远场平滑切换的中心尺度。 |

该对象主要用于来源追溯和验证正演。当前客户端不会直接把这里的数值当作可信实机标定；
科学处理使用 `ReceiverCalibrationModel` 中经过加载和验证的参数。两套数值在仿真测试中
应保持匹配，在真实设备中必须由对应设备序列号的标定文件替换。

项目当前 JSON 序列化器对非整数最多输出 6 位小数。配置中的默认死时间系数
`0.0000025` 在这条网络帧和日志中被舍入成 `0.000003`。对于这类很小但敏感的标定参数，
正式协议需要提高数值序列化精度，或者使用带明确尺度的整数表示，不能依赖当前预览值
进行实机标定。

## 12. qc_hint

`qc_hint` 是设备或仿真器提供的提示，不是客户端最终 QC 结论：

| 字段 | 样例值 | 作用 |
|---|---:|---|
| `background_counts` | 84.84793 | 方便接收端快速查看设备认为的背景基线。 |
| `laser_energy_mj` | 0.020307 | 重复提供当前激光能量，便于链路诊断。 |
| `near_range_overlap_limited` | false | 表示兼容主通道的最前端 overlap 在该样例中未低于仿真提示阈值。 |

客户端仍会根据完整数组和本地标定重新检查背景、饱和、死时间、overlap、SNR 和输入有限性，
最终质量结果保存在 `ProcessedProfile.bin_quality` 和 `ProcessedProfile.qc_flags` 中。

## 13. 环境量

| 字段 | 样例值 | 单位 | 作用 |
|---|---:|---|---|
| `relative_humidity` | 0.436014 | 比例 | 相对湿度，0.436014 表示约 43.6%。客户端用它做吸湿增长修正。 |
| `temperature_c` | 25.994242 | °C | 当前射线关联的环境温度。 |
| `wind_speed_ms` | 3.528904 | m/s | 当前射线关联的风速。 |
| `wind_dir_deg` | 140.673502 | 度 | 当前射线关联的风向角。真实适配器必须同时明确风向角的气象学约定。 |

这些量在当前仿真器中属于合成环境上下文。实机系统中应说明它们来自雷达内置传感器、
外接气象站还是时间对齐后的其他设备，不能只凭字段存在就认为是同一时刻的实测值。

## 14. 日志截断标记

下面的对象不是协议成员：

~~~json
{
  "_truncated": true,
  "omitted_count": 5322
}
~~~

它表示：

- 日志已经显示当前数组前 12 项；
- 原数组还有 5322 项没有打印；
- 原数组总长度是 12 + 5322 = 5334；
- 网络原始 JSON 数组中不会混入这个对象；
- 不能把日志预览直接交给 `parse_frame()` 或算法重放，因为数组已经不完整且类型被日志标记改变。

若需要可重放样本，应在协议接收层保存完整 JSONL 原文，而不是从调试日志复制。

## 15. 客户端如何消费这条帧

~~~text
TCP 字节流
  -> LidarClientWorker 按换行拆出一条 JSONL
  -> parse_frame
       type      -> FrameType::lidar_raw
       timestamp -> Frame.timestamp
       其余字段  -> Frame.payload
  -> ScanCycleMonitor
       使用 scan_cycle_id / ray_index / rays_in_cycle 统计接收完整性
  -> FrameProcessor::handle_frame
       json_to_profile 映射公共 LidarProfile 和四个 LidarChannel
       读取 integrated_pulses / wavelength_nm / signal_unit / detector_mode
       执行接收机校正、近远场拼接、Fernald/Klett、湿度修正和 QC
  -> heartbeat 封口为 StepResult
  -> GUI 显示不可变结果快照
~~~

不是每个传入字段当前都会直接参与数值算法。协议保留部分字段是为了设备审计、时序诊断、
未来实机适配和故障追踪。实现新设备协议时，应先把厂商字段转换为这套明确单位和语义的
公共模型，再交给 `FrameProcessor`，不要让厂商字节布局进入科学算法。
