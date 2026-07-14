# 商用 LiDAR 上报帧还原策略

## 结论

仅靠公开互联网资料，无法 100% 复刻某个商用 LiDAR 的私有实时帧。完整二进制帧、校验、状态码、采集卡原始格式、SDK 回调结构通常属于厂商私有资料，通常需要购买设备、SDK、协议手册或抓包样例。

本项目采用更可落地的策略：

1. 对公开数据产品做字段级映射。
2. 对实时流做通用设备帧抽象。
3. 对目标设备单独维护公开规格、采购边界、仿真假设和未知字段。

当前实时仿真的唯一目标是无锡中科光电 `YLJ5 / AGHJ-I-LIDAR(MPL)`。其证据来源、
已实现能力和实机缺口见 [YLJ5 仿真保真说明](ylj5_fidelity.md)。下述 Vaisala、
Raymetrics 和 HALO 只用于解释通用产品字段，不是 YLJ5 参数来源。

## 公开资料能还原的部分

### Vaisala CL61-like

公开资料可确认的产品级字段包括：

- 维度：`time`、`range`、`layer`
- 信号/产品：`p_pol`、`x_pol`、`beta_att`、`linear_depol_ratio`
- 天气/云：`cloud_base_heights`、`vertical_visibility`
- 监控状态：`window_condition`、`laser_power_percent`、`background_radiance`、`internal_temperature`、`internal_humidity`、`laser_temperature`

适合映射到本项目：

- `raw_counts` -> `p_pol` / `x_pol` 的近似原始通道
- `attenuated_backscatter` -> `beta_att`
- `qc_flags` / 遥测帧 -> `window_condition`、`laser_power_percent`、`background_radiance`

### Raymetrics PMeye-like

公开资料能确认其面向 PM2.5/PM10、355 nm、3D 扫描、模拟+光子计数、full overlap、远程实时监测和污染源定位。私有帧格式未公开。

适合映射到本项目：

- `wavelength_nm = 355`
- `detector_mode = hybrid_photon_analog`
- `raw_counts` -> analog/photon-counting 合成原始信号
- `extinction`、`pm25`、`pm10` -> PM 产品
- `volume.voxels` -> 3D 污染云

### HALO HPL-like

HALO StreamLine 更偏 Doppler 风 LiDAR。公开数据常见为 HPL/ASCII 或类似 replay 产品，适合借鉴扫描调度、GPS 同步和 replay，不直接等价于 PM 弹性 LiDAR。

## 项目字段映射

实时帧：

- `status`：设备身份、公开规格、扫描计划、校准状态和未知项
- `telemetry`：明确标为合成值的设备运行遥测
- `lidar_raw`：兼容主通道和四物理接收通道的 L0 原始廓线
- `camera`：同步相机能力和雷达指向，不伪造实机图像
- `lidar_product`：未标定的信号与退偏比快速摘要
- `ground_obs`：可选的合成地面 PM/气象辅助观测
- `heartbeat`：扫描周期完成边界
- `command` / `command_result`：控制请求、规格校验结果和生效配置摘要
- `alarm`：周期生成失败等设备级错误

关键 L0 字段：

- `sequence_id`
- `frame_id`
- `scan_cycle_id`
- `primary_channel_id`
- `channels[].channel_id/telescope/polarization`
- `channels[].raw_counts/overlap`
- `depolarization_ratio`
- `detector_mode`
- `ranges_m`
- `raw_counts`
- `laser_energy_mj`
- `background_counts`
- `overlap`
- `azimuth_encoder_deg`
- `elevation_encoder_deg`
- `integrated_pulses`
- `line_dwell_s`
- `motion_overhead_s`
- `acquisition_start_timestamp/acquisition_end_timestamp/publish_timestamp`
- `calibration_status`
- `vendor_wire_protocol_emulated`

关键遥测字段：

- `device_state`
- `electronics_temperature_c`
- `ambient_temperature_c`
- `relative_humidity`
- `pressure_hpa`
- `estimated_power_w`
- `gimbal_azimuth_deg`
- `gimbal_elevation_deg`
- `camera_online`
- `laser_enabled`
- `telemetry_provenance = synthetic_not_vendor_spec`
- `calibration_status`

这里没有继续模拟窗口污染、雨刷、GPS/NTP 或厂商诊断码，因为当前没有 YLJ5 实机
证据。缺证据的遥测字段不应仅为了“看起来像设备”而加入协议。

## 客户端消费边界

当前 Linux 客户端通过 Qt `QTcpSocket` 接收上述 JSONL 帧。socket、拆帧、协议解析、
状态合并、扫描完整性统计和 L0-L2 数据处理全部位于专用 `QThread`；GUI 只消费只读
周期快照，不在主线程轮询网络。

`lidar_raw` 的近/远场平行偏振通道用于背景估计、增益/overlap 修正、SNR 门控和交叉区
拼接。垂直偏振路径和协议中的退偏比会保留，但在获得偏振串扰矩阵前不声称完成厂家
偏振标定。实时帧默认不带分子场时，客户端使用明确带 QC 的 532 nm 标准大气回退。

`lidar_product` 是服务端快速摘要，不是客户端定量 PM 输入。客户端只有加载经过站点共址
验证的 PM 标定文件后才生成 PM 和浓度热点；否则 PM 数组保持为空。完整处理顺序见
[YLJ5 客户端实时处理链](algorithm_processing_chain.md)。

批处理 Demo 的关键 L3 字段如下；它们不属于当前 YLJ5 实时厂家协议：

- `volume.coordinate_system = ENU`
- `volume.voxel_size_m`
- `volume.voxels[].x_m/y_m/z_m`
- `volume.voxels[].pm25`
- `volume.voxels[].confidence`
- `volume.voxels[].source_ray_count`

## 下一步如果要更接近 100%

需要至少一种真实输入：

- 厂商 SDK 或协议 PDF
- 真实设备抓包
- 原始 L0 文件样例
- NetCDF/HPL/Licel 文件样例

拿到样例后，应新增：

- `DeviceAdapter`：按真实协议读帧
- `VendorFrameDecoder`：二进制/ASCII 到项目 L0 结构
- `VendorProtocolAdapter`：把厂商命令、状态和错误码映射到项目控制模型
- golden sample 测试：真实样例 -> 解析结果固定断言
- replay 模式：真实文件按时间戳回放
