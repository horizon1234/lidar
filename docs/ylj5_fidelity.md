# YLJ5 / AGHJ-I-LIDAR(MPL) 仿真保真说明

更新日期：2026-07-14。

本项目的实时仿真设备以无锡中科光电 `YLJ5 / AGHJ-I-LIDAR(MPL)` 为唯一目标。
当前实现是“公开规格数字孪生”，不是已经过实机验收的逐字节克隆。凡是没有厂商
数据手册、原始数据或抓包证据的值，代码和协议都会标记为假设或未知。

核心实现入口：

- `cpp/server/include/lidar_server/Ylj5DeviceSpec.hpp`：公开规格、扫描计划和仿真假设；
- `cpp/server/include/lidar_server/SimDevice.hpp`：惰性设备接口和控制状态机；
- `cpp/server/src/SimDevice.cpp`：单周期生成、状态/遥测/相机/产品帧；
- `cpp/protocol/src/WireFormat.cpp`：四物理通道 JSONL 序列化；
- `cpp/client/src/LidarClientWorker.cpp`：Linux Qt 工作线程网络、拆帧和周期交付；
- `cpp/client/src/FrameProcessor.cpp`：四通道预处理、反演和 PM 标定门控；
- `cpp/tests/TestSimDeviceYlj5.cpp`：规格、协议、命令和可选全规格烟雾测试。
- `cpp/tests/TestClientFrameProcessor.cpp`：客户端分子场回退、通道拼接和 PM 边界测试。

## 证据来源

1. [无锡中科光电产品页](https://www.cas-pe.com/product/1868180499106713601.html)
   及其官网数据接口
   `https://ssl.lzynet.net:9443/API-CAS-PE/manage/outward/article/vo/1868180499106713601`。
   官网明确型号、双望远镜近远场设计、同步摄像、碳纤维全密封外壳、IP66、全天候
   自动观测，以及云台/振镜、室内/户外/车载/组网应用。
2. [四川省公共资源交易结果页](https://ggzyjy.sc.gov.cn/jyxx/002002/002002003/20250116/8a69c74e9452f4ac01946d0207260fad.html)
   及项目采购文件。成交设备型号明确为 `AGHJ-I-LIDAR(MPL)`；采购技术条款提供下表
   的数值边界；现场演示条款还明确要求软件能够执行“水平扫描、垂直观测、锥形扫描”。
   采购条款是准入门槛，不等同于该台设备的厂家默认调度或实测值。
3. Wang 等人的开放论文
   [The Determination of Aerosol Distribution by a No-Blind-Zone Scanning Lidar](https://doi.org/10.3390/rs12040626)
   给出相近的中科院环境光学双视场零盲区扫描实验：固定 5 度仰角、0-360 度方位、
   2 度步进，每圈 180 条。它不是 YLJ5 数据手册，只用于给仿真默认锥扫角提供可追溯
   的工程依据。
4. Chiang 等人的开放论文
   [A new mobile and portable scanning lidar for profiling the lower troposphere](https://doi.org/10.5194/gi-4-35-2015)
   说明便携式三维扫描雷达会按任务配置方位/仰角策略，并展示了 1 度分辨率的垂直
   RHI、水平扇扫和固定 5 度低仰角观测。该论文用于确认多模式扫描的通用工作方式，
   不用于冒充 YLJ5 厂家参数。

## 保真矩阵

| 项目 | 当前值/行为 | 证据等级 | 实现位置 |
|---|---:|---|---|
| 产品型号 | YLJ5 / AGHJ-I-LIDAR(MPL) | 官网确认 | `Ylj5DeviceSpec.hpp` |
| 工作波长 | 532 nm，允许偏差不超过 2 nm | 采购边界 | `Ylj5HardwareSpec` |
| 距离分辨率 | 3.75 m 及整数倍 | 采购边界 | 默认 3.75 m |
| 最大探测距离 | 不小于 20 km | 采购边界 | 默认 20002.5 m、5334 bins |
| 望远镜 | 至少两路近/远场 | 官网确认 + 采购边界 | 四接收通道模型 |
| 近/远口径 | 近场不大于 40 mm；远场不小于 160 mm | 采购边界 | 默认取 40/160 mm 边界值，并非厂家标称值 |
| 偏振 | 支持偏振探测 | 采购边界 | 近/远场各平行、垂直通道及退偏比 |
| SNR | 不小于 15 dB | 采购边界 | 能力元数据；尚无实机噪声曲线可验收 |
| 束散角 | 不大于 0.2 mrad | 采购边界 | 能力元数据；尚未用于空间卷积 |
| 激光功率稳定度 | 不大于 2% | 采购边界 | 默认正演抖动 1.5%，但统计定义仍待实机对齐 |
| 方位/俯仰 | 0-360 度 / 0-180 度 | 采购边界 | 扫描配置校验 |
| 云台 | 0-30 度/秒；指向分辨率不大于 0.1 度 | 采购边界 | 速度校验和编码器扰动 |
| 水平扫描 | 至少 180 条、每条至少 10000 脉冲、单圈不超过 15 分钟 | 采购边界 | 默认 2 度、2 秒、5 kHz、约 406 秒 |
| 扫描模式 | 水平扫描、垂直观测、锥形扫描 | 采购现场演示要求 | 三种模式均写入帧元数据并由测试覆盖 |
| 仰角调度 | 周期 0 为 0 度，周期 1 为 5 度，随后循环；每周期先做 90 度垂直观测 | 采购模式 + 论文角度 + 仿真假设 | `round_robin` 调度；厂家默认顺序待确认 |
| 同步相机 | 背光补偿、夜间红外 | 官网/采购能力 | 当前只发能力和同步元数据，不伪造实机图像 |
| 外壳 | 碳纤维、IP66、全天候 | 官网确认 | 状态能力元数据 |
| 接口 | 有线、无线、串口、USB | 采购边界 | 能力元数据；当前只实现 TCP JSONL |
| 走航 | 0-120 km/h，每 30 m 至少一条记录 | 采购边界 | 当前仅有平台位移场景；30 m 记录节拍尚未按实机协议验收 |

## 当前扫描与数据模型

默认不再永久停留在一个水平仰角，而是执行下面的两周期轮换：

```text
周期 0：90 度垂直观测 1 条 -> 0 度水平扫描 180 条
周期 1：90 度垂直观测 1 条 -> 5 度锥形扫描 180 条
周期 2：回到周期 0 的模式，继续循环
```

水平和锥形扫描均为 0-360 度方位、2 度步进。每条驻留 2 秒，假设 PRF 为 5 kHz，
因此积累 10000 个脉冲；云台按 10 度/秒移动，每条约 0.2 秒。单个方位扫描加 10 秒
开销约 406 秒，再加 30 秒垂直观测，一个周期约 436 秒；完整执行一次 0/5 度队列约
872 秒。逐周期只生成一个固定仰角层，不会把两圈数据同时堆进内存。

核心数据结构使用 `scan_mode=ppi/stare` 区分方位射线和垂直射线。设备协议额外提供
`device_scan_pattern=horizontal_scan/vertical_observation/conical_scan`、
`scheduled_elevation_deg`、`elevation_schedule_index` 和
`elevation_schedule_length`，避免客户端靠角度猜测设备模式。

每条 `lidar_raw` 帧包含近远场拼接主通道 `raw_counts`，以及四条物理接收路径：

- `near_parallel_532nm`
- `near_perpendicular_532nm`
- `far_parallel_532nm`
- `far_perpendicular_532nm`

`raw_counts` 是近/远场平行偏振通道在约 150 m 附近平滑拼接后的结果，现有客户端
无需修改反演入口。实时协议默认不发送分子场和仿真真值，避免把验证字段误当实测量，
也降低 5334 bins 数据的带宽。

## Linux Qt 客户端

客户端只支持 Linux Qt6 GUI，不再保留阻塞式无头客户端。`LidarClientWorker` 被移动到
专用 `QThread`，在线程内持有 `QTcpSocket`，并完成 JSONL 拆帧、状态合并、扫描完整性
监控、背景估计、近远场拼接、Fernald/Klett 反演、湿度修正、ENU 投影和 PPI 栅格
预计算。GUI 主线程只绘制 `StepResult`、`DisplaySnapshot` 和 `DeviceStatusSnapshot` 的
不可变快照。

实时协议不发送分子场时，客户端会采用 532 nm 标准大气近似并添加
`molecular-reference-standard-atmosphere`。该回退不是厂家算法，也不能替代现场温压廓线。
未加载站点 PM 标定时，PM2.5、PM10 和热点数组保持为空；服务端 `lidar_product` 中的
快速摘要不会被当成定量 PM。详细流程和标定 JSON 合同见
[客户端实时处理链](algorithm_processing_chain.md)。

设备按周期惰性生成，不再预生成并缓存 180 个周期。默认规模若一次性缓存整场四通道
数据会占用数十 GB；当前运行时只保留一个 181 条射线周期。可选的 `--full-smoke` 测试
固定验证两个周期共 362 条射线，其中 180 条水平、180 条锥形、2 条垂直，且每条都包含
5334 bins 和四通道。耗时与内存应在具体实验记录中采集，不作为厂家设备指标。

## 明确的仿真假设

以下参数没有 YLJ5 实机证据，状态帧统一标记为
`assumed_pending_real_device_capture`：

- PRF 5 kHz、单脉冲能量 0.02 mJ、系统常数和激光雷达比；
- 探测器类型、增益、暗计数、读噪声、死时间、后脉冲和饱和曲线；
- 两望远镜 overlap 曲线、相对增益和 150 m 拼接位置；
- 精确发散角点扩散、滤光片带宽、偏振串扰和通道标定矩阵；
- 功耗、压力和电子学温度等遥测数值。
- 自动按 0/5 度逐周期轮换的顺序，以及 5 度作为默认锥扫角。

这些参数用于形成统计上可用的回波，不应作为厂家参数引用。PM 快视产品也明确标记为
未经过实机定量标定。

## 仍未知的厂商行为

- 厂商私有有线/无线/串口/USB 协议、帧头、校验、字节序和通道顺序；
- 固件命令字、状态寄存器、故障码、启动和自检时序；
- 原始 ADC/光子计数单位、累积方式、背景扣除位置及无效值编码；
- 相机图像编码、时间同步精度、视场和标定参数；
- 实机脉冲参数、接收链参数、几何标定、距离零点和噪声统计；
- 产品算法、文件格式和厂商上位机的字段命名。
- 厂家实际默认扫描模式、锥扫角度列表、模式切换条件，以及是否由操作员手动选择。

因此当前 TCP JSONL 是项目自己的仿真层协议，`vendor_wire_protocol_known=false`，不会
宣称已复刻厂商线协议。

## 控制接口

客户端发送 `type=command`，payload 中使用 `command`、`action` 或 `name`。已实现：

- `start` / `resume`
- `pause`
- `stop`
- `get_status`
- `set_scan` / `configure_scan`

响应类型为 `command_result`。正式配置会拒绝低于公开采购门槛的扫描计划；测试和诊断
只能通过显式设置 `enforce_public_spec=false` 使用缩小配置。

暂停命令示例：

```json
{"type":"command","timestamp":"","request_id":"pause-1","command":"pause"}
```

成功响应会保留请求 ID，并返回当前关键配置：

```json
{
  "type": "command_result",
  "request_id": "pause-1",
  "command": "pause",
  "accepted": true,
  "result": "ok",
  "device_state": "paused",
  "violations": [],
  "status": {
    "device_model": "YLJ5",
    "regulatory_model": "AGHJ-I-LIDAR(MPL)",
    "vendor_wire_protocol_known": false
  }
}
```

## 达到实机级还原所需资料

1. 各接口协议文档，或启动、扫描、暂停、状态查询的双向抓包。
2. 一组覆盖近场、远场、白天、夜间和无目标背景的原始帧及对应上位机导出文件。
3. 脉冲能量、PRF、探测器/ADC、望远镜口径与视场、overlap、偏振标定报告。
4. 云台编码器轨迹与相机同步样本。
5. 厂商状态页、自检结果、故障码和固件版本信息。

拿到这些资料后，应新增独立 `VendorProtocolAdapter` 和标定文件，而不是把当前 JSONL
字段改名后冒充厂商协议。
