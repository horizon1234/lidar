# LiDAR 算法处理链拆分说明

本文说明项目中的算法处理链如何拆成类，以及每一步对应的公开论文/工程依据。商用设备的完整算法实现通常不公开，因此本项目采用公开文献中成熟的 LiDAR 处理方法，并保留可替换接口。

## 1. 类化处理链

单条 profile 的实时处理由 `SingleProfileProcessingChain` 串联：

```text
LidarProfile
  -> BackgroundPreprocessStep
  -> FernaldInversionStep
  -> HumidityCorrectionStep
  -> CoordinateProjectionStep
  -> ProcessedProfile
```

体扫热点检测由 `HotspotDetectionStep` 处理：

```text
vector<ProcessedProfile> PPI/RHI/volume profiles
  -> HotspotDetectionStep
  -> vector<Hotspot>
```

这样拆分后，每一步都可以独立替换。例如后续拿到 Raman 通道时，可以把 `FernaldInversionStep` 替换成 Raman extinction retrieval；拿到真实 ground PM 标定集时，可以替换 PM 标定模型。

## 2. L0 -> L1 预处理

类：`BackgroundPreprocessStep`

输入：

- `raw_counts`
- `background_counts`
- `laser_energy_mj`
- `overlap`
- `ranges_m`

输出：

- `l1_signal`
- `attenuated_backscatter`
- `snr`
- `qc_flags`

处理逻辑：

```text
background_corrected = max(raw_counts - background_counts, 0)
energy_normalized = background_corrected / laser_energy_mj
overlap_corrected = energy_normalized / overlap(r)
attenuated_backscatter = overlap_corrected * r^2
snr = background_corrected / sqrt(raw_counts + background_counts)
```

工程意义：

- 商用 LiDAR 软件通常会保留原始信号、range corrected signal、SNR/CNR 和 QC 标志。
- full overlap 前的近场数据必须标记为低可信，不能直接当作污染高值。

## 3. Fernald/Klett 弹性反演

类：`FernaldInversionStep`

输入：

- `attenuated_backscatter`
- 分子后向散射/消光
- aerosol lidar ratio
- 远端参考后向散射

输出：

- `extinction`
- `aerosol_backscatter`

依据：

- Klett, 1981, "Stable analytical inversion solution for processing lidar returns", Applied Optics.
- Fernald, 1984, "Analysis of atmospheric lidar observations: some comments", Applied Optics.

当前实现是后向积分的 elastic-backscatter 近似。它适合无 Raman 通道、只有单波长弹性回波的工程原型，但依赖 lidar ratio 和远端参考区，必须做敏感性分析。

## 4. 湿度修正

类：`HumidityCorrectionStep`

输入：

- 湿消光 `extinction`
- `relative_humidity`

输出：

- `dry_extinction`

处理逻辑：

```text
dry_extinction = extinction / g(RH)
```

工程意义：

- 颗粒物吸湿增长会增强散射/消光。
- 如果不做湿度修正，PM 估计会把高湿导致的光学增强误判为质量浓度升高。

当前实现采用简化 κ-Kohler 风格增长因子。真实项目中应按站点和气溶胶类型标定。

## 5. 坐标投影

类：`CoordinateProjectionStep`

输入：

- range
- azimuth
- elevation

输出：

- ENU 点 `[east, north, up]`

处理逻辑：

```text
East  = r * cos(elevation) * sin(azimuth)
North = r * cos(elevation) * cos(azimuth)
Up    = r * sin(elevation)
```

工程意义：

- 3D 污染云、热点质心、走航车轨迹和 GIS 地图都需要统一空间坐标。

## 6. PM 标定

当前 PM 标定仍在批处理主流程中完成：

```text
dry_extinction + RH + hotspot proxy + station offset -> PM2.5 / PM10
```

原因是 PM 标定不是单条 profile 能独立完成的步骤，它依赖地面 PM 站、训练/验证切分、站点偏置和漂移监控。

后续建议把它继续拆成：

- `FeatureBuilder`
- `PmCalibrationTrainer`
- `PmEstimator`
- `StationDriftMonitor`

## 7. 热点检测

类：`HotspotDetectionStep`

输入：

- 同一时间步的 PPI/volume `ProcessedProfile`

输出：

- `Hotspot`

处理逻辑：

- 绝对 PM 阈值
- 相对背景异常
- 干消光相对异常
- 连通域过滤
- PM 加权质心

工程意义：

- 工地、城市和走航车应用里，业务目标通常不是单个 bin 的浓度，而是污染团的位置、范围、趋势和事件生命周期。

## 8. 更接近商用设备的后续升级

如果要继续逼近在售设备软件，优先级如下：

1. Raman 或多波长通道：用 Raman extinction 替代固定 lidar ratio 假设。
2. 云/雾/降水分类：在反演前做 weather mask。
3. 动态 lidar ratio：按气溶胶类型、湿度、退偏振比选择。
4. 不确定度传播：每个 bin 输出 PM 置信度，而不是只输出浓度。
5. 真实文件 replay：支持 CL61-like NetCDF、HPL-like ASCII、Licel-like raw。

## 9. 参考资料

- Klett, J. D. 1981. Stable analytical inversion solution for processing lidar returns. Applied Optics. https://doi.org/10.1364/AO.20.000211
- Fernald, F. G. 1984. Analysis of atmospheric lidar observations: some comments. Applied Optics. https://doi.org/10.1364/AO.23.000652
- Ansmann, A., Riebesell, M., and Weitkamp, C. 1990. Measurement of atmospheric aerosol extinction profiles with a Raman lidar. Optics Letters. https://opg.optica.org/ol/fulltext.cfm?uri=ol-15-13-746
- Müller, D., Wandinger, U., and Ansmann, A. 1999. Microphysical particle parameters from extinction and backscatter lidar data by inversion with regularization. Applied Optics. https://opg.optica.org/ao/fulltext.cfm?uri=ao-38-12-2346
