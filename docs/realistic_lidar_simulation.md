# 逼真化 LiDAR 仿真与 3D 污染监测方案

本文说明当前项目如何从算法 demo 调整为更接近工地、城市固定站和走航车应用的激光雷达污染物监测原型。定位是高级工程师面试项目：重点展示设备抽象、物理仿真、实时处理、数据产品和工程边界，而不是声称达到真实商用设备计量精度。

## 1. 对标设备与公开参数

公开资料显示，商用大气 LiDAR 软件通常具备这些能力：

- 扫描模式：PPI、RHI、DBS/VAD、体积扫描、定点 stare。
- 数据产品：原始信号、attenuated backscatter、消光/后向散射、SNR/CNR、QC flag、NetCDF L1/L2/L3、quicklook 图像。
- 应用能力：3D 污染源定位、边界层/云/降水过滤、历史回放、告警和远程运维。
- 设备约束：full overlap、太阳背景、激光能量漂移、探测器暗计数、饱和、低 SNR 远场、雨雾云干扰。

参考来源：

- Vaisala CL61 / BL-View：910.55 nm、4.8 m 分辨率、NetCDF L1/L2/L3、云/降水过滤、live/archive 显示。
- HALO Photonics StreamLine：扫描队列、GPS 同步、多种扫描模式、长距离 Doppler LiDAR。
- Raymetrics PMeye：面向 PM2.5/PM10、3D 扫描、full overlap、远程实时监测和污染源定位。

## 2. 本项目采用的仿真策略

为了兼顾“逼真”和“本机可实时演示”，项目不直接模拟真实设备的最细粒度原始采样，而是采用工程折中：

- 固定站/工地模式：6 km 量程，37.5 m 距离门，10 度方位步进，6 个低/中仰角体扫。
- 走航车模式：3.6 km 量程，30 m 距离门，低仰角优先，车辆速度进入污染场坐标。
- 波长默认 355 nm，面向 PM 扫描 LiDAR 的弹性通道。
- 单脉冲能量、背景计数、full overlap、死时间、ADC 饱和均进入配置。

新增配置：

- `configs/field_scanning_lidar.json`：工地/城市固定扫描站。
- `configs/mobile_mapping_lidar.json`：走航车扫描。

## 3. 仿真信号模型

原始信号仍以 LiDAR 方程为主：

```text
P(r) = C * E * O(r) * beta(r) * exp(-2 tau) / r^2 + background
```

在此基础上新增真实设备近似：

- `full_overlap_m` 和 `min_overlap` 控制近场不完全重叠。
- `pulse_energy_mj` 和 `pulse_energy_jitter` 控制激光能量漂移。
- `background_counts_mean`、`background_counts_jitter`、`solar_background_scale` 控制白天户外背景。
- `detector_dark_counts` 和 `read_noise_counts` 控制探测器噪声。
- 小信号使用 Poisson 采样，大信号使用高斯近似，避免理想化连续信号。
- `dead_time_loss` 与 `adc_saturation_counts` 限制近场强回波，模拟死时间和饱和。

这使 raw_counts 不再只是“干净曲线 + 5% 高斯噪声”，而包含真实仪器常见的低信噪、近场限制和强信号非线性。

## 3.1 设备上报帧

仿真服务器按真实设备思路上报三类信息：

- `status` 能力帧：设备型号、固件版本、协议版本、通道、扫描模式、量程、分辨率、full overlap、是否支持 L3 体素。
- `status` 遥测帧：设备运行状态、扫描调度状态、GPS/NTP、时钟偏移、舱内温度、激光头温度、探测器温度、窗口透过率、窗口污染指数、雨控、太阳背景、风扇、舱门、雨刷和诊断标志。
- `lidar_raw` 原始帧：除 `ranges_m` 和 `raw_counts` 外，还包含 `sequence_id`、`frame_id`、`scan_cycle_id`、`ray_index`、`rays_in_cycle`、通道 ID、探测模式、距离分辨率、波长、full overlap、编码器角度、积分脉冲数、积分时间和 QC hint。

这些字段不是厂商私有协议复刻，而是根据公开软件/设备资料抽象出的通用上报形态。后续如果拿到真实设备样例，可把字段名映射到厂商协议，同时保留当前处理链。

## 4. 污染场模型

当前污染场包含：

- 边界层背景气溶胶。
- 高架输送层。
- 工地/工业源低空烟羽。
- 城市交通/区域输送源。
- 走航车近源扬尘。

烟羽不再固定在某个方位，而是由风速、风向和时间驱动平流；扩散宽度随时间增长；源强有脉冲变化。这比固定 Gaussian 更接近工地扬尘、城市道路和移动监测场景。

## 5. L3 体素产品

为了支持最终 3D 显示，demo payload 新增 `volume` 字段：

```json
{
  "timestamp": "...",
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

体素由最新体扫的 PPI 多仰角射线聚合得到，`confidence` 由 SNR 映射。前端 3D 显示应同时呈现 PM2.5、confidence、热点质心、风向和扫描覆盖范围，避免把低可信区域画成确定污染云。

## 6. 面试讲法

建议面试中明确说明：

1. 这不是把模拟指标包装成真实设备指标，而是构造接近真实设备误差来源的可控仿真环境。
2. 算法链路覆盖 L0 raw -> L1 预处理 -> L2 反演/PM 标定 -> L3 体素/热点事件。
3. 工程重点是设备参数化、故障和低质量数据可见、3D 产品可解释、实时链路可回放。
4. 后续接真设备时，`SimDevice` 应替换为真实 `DeviceAdapter`，但协议帧、处理器、L3 产品和 UI 可以保留。

## 7. 当前限制

真实场景配置的重点是提高设备信号和空间扫描的逼真度，不应直接用默认 hotspot F1 作为“真实性能”。原因是合成真值仍来自规则化污染场，而 PM 标定后的高值区域会受反演误差、湿度修正和低 SNR 影响。面试时应把它解释为一个可控压力测试场景，并说明下一步会用真实标注或人工复核事件重新定义 L3 热点真值。
