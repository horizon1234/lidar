# 商用 LiDAR 上报帧还原策略

## 结论

仅靠公开互联网资料，无法 100% 复刻某个商用 LiDAR 的私有实时帧。完整二进制帧、校验、状态码、采集卡原始格式、SDK 回调结构通常属于厂商私有资料，通常需要购买设备、SDK、协议手册或抓包样例。

本项目采用更可落地的策略：

1. 对公开数据产品做字段级映射。
2. 对实时流做通用设备帧抽象。
3. 在 `device_product_schema` 中明确哪些字段来自公开产品格式，哪些是项目抽象。

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

- `status`：设备能力帧和健康遥测帧
- `lidar_raw`：L0 原始廓线帧
- `ground_obs`：地面 PM/气象辅助观测
- `heartbeat`：时间步/扫描周期边界

关键 L0 字段：

- `sequence_id`
- `frame_id`
- `scan_cycle_id`
- `channel_id`
- `detector_mode`
- `ranges_m`
- `raw_counts`
- `laser_energy_mj`
- `background_counts`
- `overlap`
- `azimuth_encoder_deg`
- `elevation_encoder_deg`
- `integration_pulses`
- `accumulation_time_ms`

关键遥测字段：

- `device_state`
- `scan_scheduler_state`
- `gps_lock`
- `ntp_sync`
- `clock_offset_ms`
- `enclosure_temp_c`
- `laser_head_temp_c`
- `detector_temp_c`
- `window_transmission`
- `window_contamination_index`
- `rain_sensor_wet`
- `sun_background_counts`
- `diagnostic_flags`

关键 L3 字段：

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
- golden sample 测试：真实样例 -> 解析结果固定断言
- replay 模式：真实文件按时间戳回放
