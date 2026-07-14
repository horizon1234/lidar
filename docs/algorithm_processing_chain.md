# YLJ5 客户端实时处理链

更新日期：2026-07-14。

本文描述 `cpp/client` 当前实际执行的实时处理路径。商用设备的厂商算法、通道标定矩阵
和 PM 系数并未公开，因此项目只采用可追溯的公开方法；无法由公开证据确定的环节会输出
QC 或保持产品为空，不用经验常数冒充厂家结果。

## 1. 线程和周期边界

Linux Qt 客户端只有 GUI 模式。`LidarClientWorker` 被移动到专用 `QThread`，线程内依次
执行：

```text
QTcpSocket 收包
  -> JSONL 拆帧和协议解析
  -> DeviceStatusModel 增量合并
  -> ScanCycleMonitor 完整性统计
  -> FrameProcessor L0-L2 处理
  -> heartbeat 或时间戳变化时封口 StepResult
  -> 预计算 PPI 色标、ENU 栅格和垂直显示快照
  -> queued signal 交给 GUI
```

GUI 线程只接收 `shared_ptr<const StepResult>`、`shared_ptr<const DisplaySnapshot>` 和设备
状态快照，不访问 socket，也不执行背景扣除、反演、标定、热点检测或百万点 PPI 栅格化。
断开连接和关闭窗口时，worker 会先封口尚未完成的周期。

## 2. 输入校验

每条 `lidar_raw` 进入处理链前检查：

- 距离轴至少 8 个 bin，严格递增且数值有限；
- 近远场拼接主通道 `raw_counts` 和 `overlap` 与距离轴等长；
- 方位角位于 `[0, 360)`，仰角位于 `[0, 180]`；
- 每条物理接收通道的计数和 overlap 与距离轴等长。

未来实机适配器若完全缺失主通道 overlap，会临时填充 1.0 并添加
`main-channel-overlap-assumed-unity`；非空但尺寸错误的数据会被拒绝。结构或数值不合法的
帧不会进入反演，周期结果记录 `malformed-or-unprocessable-lidar-frame`。

## 3. 接收通道预处理

YLJ5 仿真帧携带近场/远场与平行/垂直偏振四条路径。当前定量主信号使用
`near_parallel_532nm` 和 `far_parallel_532nm`；两条垂直偏振通道会接受形状校验并随原始
廓线保留，退偏比使用协议中的 `depolarization_ratio`。在获得实机偏振串扰矩阵前，项目
不会伪造偏振标定。

每条参与拼接的通道执行：

```text
background = 末端约 5% bin 的中位数
signal = max(raw - background, 0)
corrected = signal / relative_gain / max(overlap, overlap_min)
snr = signal * sqrt(integrated_pulses) / sqrt(raw + background)
```

在 75-300 m 默认交叉区内，只有近、远场 SNR 均达到门限的 bin 才参与比例估计。比例
样本不少于 5 个时取中位数作为远场尺度，随后按距离做线性权重拼接；某一路 SNR 不足时
自动偏向另一条路径。输出包含：

- 背景、增益和 overlap 修正后的 `l1_signal`；
- 能量归一和距离平方修正后的 `attenuated_backscatter`；
- 脉冲积分修正后的 `snr`；
- `near-far-channels-glued`、`channel-gluing-ratio-unverified` 等 QC。

这套近远场 gluing 顺序与 EARLINET Single Calculus Chain 的公开处理分层一致，但具体
交叉区、增益和 overlap 仍是待实机标定参数，不是 YLJ5 厂家值。

## 4. 分子参考场

实时协议默认不发送仿真真值和分子场。若 `molecular_backscatter` 或
`molecular_extinction` 缺失，客户端按 532 nm 标准大气近似生成分子参考：采用 8 km
指数尺度高度和波长四次方缩放，并添加 `molecular-reference-standard-atmosphere`。

该回退能保证反演有明确边界条件，但不能替代站点探空、温压廓线或标准气象模式。接入
真实设备后，应优先用同期气象廓线计算分子消光和后向散射。

## 5. Fernald/Klett 弹性反演

`FernaldInversionStep` 使用远端参考区和假设的气溶胶激光雷达比，从远端向近端反向
积分，输出消光和气溶胶后向散射。该方法建立在经典 Klett/Fernald 弹性后向散射反演上：

```text
attenuated_backscatter + molecular profile
  -> 远端参考尺度
  -> 双程透过率反向积分
  -> aerosol backscatter
  -> extinction = molecular extinction + lidar_ratio * aerosol backscatter
```

单波长弹性回波无法独立确定激光雷达比，结果必须结合参考区、天气掩膜和敏感性分析。
当前代码还没有厂商云雾降水分类，也没有逐 bin 不确定度传播。

## 6. 湿度修正和坐标投影

`HumidityCorrectionStep` 用简化的 κ-Kohler 风格增长因子把环境湿消光修正到干参考状态：

```text
dry_extinction = extinction / g(relative_humidity)
```

`CoordinateProjectionStep` 再把距离、方位和仰角转换为本地 ENU：

```text
East  = r * cos(elevation) * sin(azimuth)
North = r * cos(elevation) * cos(azimuth)
Up    = r * sin(elevation)
```

因此 0 度水平圈提供近地平面的二维分布，5 度锥扫提供随距离升高的浅锥层，90 度观测
提供垂直廓线。当前 0/5/90 度调度不是密集多仰角体扫，不能仅凭单个周期宣称完整三维
层析重建。

## 7. PM 标定门控

光学消光不能用一个通用固定倍数直接变成 PM2.5/PM10。只有加载经过目标站点共址比对的
标定 JSON 后，客户端才执行：

```text
PM2.5 = max(0, intercept25 + slope25 * dry_extinction)
PM10  = max(0, intercept10 + slope10 * dry_extinction)
```

标定文件必须包含：

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

未标定时，`pm25`、`pm10` 和 `hotspots` 保持空数组，周期添加
`pm-calibration-missing`，GUI 只显示干消光、退偏比和 QC。标定有效时才允许生成定量 PM
图层并执行浓度热点检测。当前线性模型只是明确的适配接口，正式系数仍需训练/验证切分、
漂移监控和不确定度报告。

## 8. 周期质量控制

`heartbeat` 或时间戳变化触发周期封口。`StepResult` 汇总有效/拒绝射线数、PPI/垂直
射线数、仰角层数、平均处理时延、标定 ID 和去重后的 QC。`ScanCycleMonitor` 根据
`scan_cycle_id`、`ray_index`、`rays_in_cycle` 识别重复和缺失射线；不完整周期添加
`scan-cycle-incomplete`。周期摘要交付后立即释放缓存；重新连接时清空未完成周期，避免
循环扫描和断线重连造成状态长期增长或误报重复帧。

回归测试 `TestClientFrameProcessor.cpp` 固定验证以下边界：

- 缺失分子场时标准大气回退不崩溃；
- 四通道输入执行近远场平行通道拼接；
- 未标定时 PM 数组为空；
- 加载有效标定后 PM 与距离轴等长。

## 9. 仍需实机资料

达到实机级还原还缺少：厂家原始帧/SDK、ADC 或光子计数单位、通道增益、overlap、偏振
串扰、距离零点、背景区定义、激光能量监测值、气象输入、厂家 L1/L2 对照产品以及站点
PM 共址标定数据。拿到资料后应通过独立适配器和 golden sample 测试接入，而不是修改
当前 QC 把假设隐藏起来。

## 10. 公开依据

- D'Amico et al., 2015, EARLINET Single Calculus Chain, Atmos. Meas. Tech.,
  https://doi.org/10.5194/amt-8-4891-2015
- Klett, 1981, Stable analytical inversion solution for processing lidar returns,
  https://doi.org/10.1364/AO.20.000211
- Fernald, 1984, Analysis of atmospheric lidar observations: some comments,
  https://doi.org/10.1364/AO.23.000652
- Ansmann et al., 1990, Independent measurement of extinction and backscatter profiles,
  https://doi.org/10.1364/OL.15.000746
