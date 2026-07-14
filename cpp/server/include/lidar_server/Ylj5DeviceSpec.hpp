/**
 * @file Ylj5DeviceSpec.hpp
 * @brief YLJ5 / AGHJ-I-LIDAR(MPL) 公开规格与仿真配置。
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace lidar_server {

/** @brief 仿真设备部署站点信息；默认坐标只用于演示，不属于设备规格。 */
struct DeviceSiteConfig {
    std::string site_id = "ylj5-sim-01"; ///< 仿真站点唯一标识，用于帧关联和客户端分组。
    std::string site_name = "YLJ5 Public-Spec Digital Twin"; ///< 状态帧显示的站点名称。
    double latitude_deg = 39.9042;        ///< 部署纬度（WGS84，度）；默认值仅为演示站点。
    double longitude_deg = 116.4074;      ///< 部署经度（WGS84，度）；默认值仅为演示站点。
    double altitude_m = 48.0;             ///< 站点海拔（m）；默认值不代表厂商测试场地。
};

/**
 * @brief 可由官网或采购文件核验的设备规格边界。
 *
 * 带“最小/最大”含义的数值是采购准入条件，不是厂家标称值。仿真默认取边界值时，
 * 只能说明满足公开要求，不能据此推断实机的精确光机参数。
 */
struct Ylj5HardwareSpec {
    std::string manufacturer = "Wuxi CAS Photonics"; ///< 厂商英文名称，用于状态帧展示。
    std::string product_name = "Portable Micro-pulse Particulate LiDAR"; ///< 官网产品名称的英文表达。
    std::string model = "YLJ5"; ///< 官网产品型号。
    std::string regulatory_model = "AGHJ-I-LIDAR(MPL)"; ///< 采购文件中的设备型号。
    std::string specification_basis =
        "CAS-PE product 1868180499106713601; procurement N5134012024000403"; ///< 公开规格证据索引。
    std::string specification_semantics =
        "public requirement bounds; factory nominal values pending device evidence"; ///< 规格值的解释口径。

    double wavelength_nm = 532.0;          ///< 工作中心波长（nm），采购要求为 532 nm。
    double wavelength_tolerance_nm = 2.0; ///< 波长允许偏差（nm），采购要求不超过 2 nm。
    double range_resolution_m = 3.75;      ///< 距离分辨率（m），必须为 3.75 m 的整数倍。
    double maximum_range_m = 20002.5;      ///< 仿真距离轴上限（m）；5334 个 bin，满足不小于 20 km。
    double minimum_snr_db = 15.0;          ///< 采购要求的最低信噪比（dB）；尚缺实机曲线验证。
    double beam_divergence_mrad_max = 0.2; ///< 采购要求的最大束散角（mrad）。
    double laser_power_instability_fraction_max = 0.02; ///< 激光功率不稳定度上限（比例，0.02=2%）。

    double near_telescope_aperture_mm = 40.0; ///< 近场望远镜口径（mm）；默认取采购上限，不是厂家标称值。
    double far_telescope_aperture_mm = 160.0; ///< 远场望远镜口径（mm）；默认取采购下限，不是厂家标称值。
    int telescope_count = 2;                  ///< 接收望远镜数量，官网明确采用双望远镜设计。
    bool polarization_channel = true;         ///< 是否具备偏振探测能力。
    bool synchronized_camera = true;          ///< 是否具备与雷达同步的一体化摄像系统。
    bool camera_backlight_compensation = true;///< 同步相机是否支持背光补偿。
    bool camera_night_ir = true;               ///< 同步相机是否支持夜间红外成像。

    double azimuth_min_deg = 0.0;              ///< 云台方位角下限（度）。
    double azimuth_max_deg = 360.0;            ///< 云台方位角上限（度）。
    double elevation_min_deg = 0.0;            ///< 云台俯仰角下限（度）。
    double elevation_max_deg = 180.0;          ///< 云台俯仰角上限（度）。
    double pointing_resolution_deg = 0.1;      ///< 最大允许的指向分辨率（度）。
    double maximum_scan_speed_deg_s = 30.0;    ///< 云台最大扫描速度（度/秒）。
    int minimum_horizontal_rays = 180;         ///< 单圈水平扫描的最少射线数。
    int minimum_integrated_pulses_per_ray = 10000; ///< 每条水平射线的最少积分脉冲数。
    double maximum_horizontal_scan_cycle_s = 900.0; ///< 一圈水平扫描的最长允许时间（s）。

    std::string enclosure_material = "carbon_fiber"; ///< 官网明确的主机外壳材料。
    std::string ingress_protection = "IP66";         ///< 官网明确的整机防护等级。
    bool autonomous_operation = true;                ///< 是否支持全天候自动观测。
    bool supports_wired_network = true;               ///< 是否支持有线网络连接。
    bool supports_wireless_network = true;            ///< 是否支持无线网络连接。
    bool supports_serial = true;                      ///< 是否支持串口连接。
    bool supports_usb = true;                         ///< 是否支持 USB 连接。
    bool supports_vehicle_operation = true;           ///< 是否支持车载走航场景。
    double maximum_vehicle_speed_kmh = 120.0;         ///< 公开要求的最大走航速度（km/h）。
    double maximum_mobile_record_spacing_m = 30.0;    ///< 公开要求的最大走航记录间距（m）。

    /** @brief 按量程和距离分辨率计算距离门数量，向上取整以覆盖量程下限。 */
    int range_bin_count() const {
        return std::max(1, static_cast<int>(std::ceil(maximum_range_m / range_resolution_m)));
    }
};

/**
 * @brief 满足公开采购边界的水平、锥形和垂直观测计划。
 *
 * 采购文件确认上位机支持水平扫描、垂直观测和锥形扫描，但没有公开厂家默认的
 * 自动调度表。默认队列采用逐周期轮换：0 度水平扫描、5 度锥形扫描，再从头循环；
 * 每个周期开始前另做一次 90 度垂直观测。5 度来自相近公开系统论文中的扫描
 * 实验，不是 YLJ5 厂家标称值，因此必须保留为可配置仿真假设。
 */
struct Ylj5ScanProgram {
    std::string mode = "scheduled_azimuth_scan"; ///< 当前实现的逐周期固定仰角方位扫描程序。
    std::string elevation_cycle_policy = "round_robin"; ///< 仰角调度策略；当前按队列循环轮换。
    std::vector<double> elevations_deg = {0.0, 5.0}; ///< 各周期固定仰角队列（度）：0 度水平，非零为锥扫。
    double azimuth_start_deg = 0.0;             ///< 固定仰角方位扫描起始角（度）。
    double azimuth_stop_deg = 360.0;            ///< 固定仰角方位扫描终止角（度），360 表示整圈不重复 0 度。
    double azimuth_step_deg = 2.0;              ///< 相邻射线方位角步进（度），默认得到 180 条射线。
    double line_dwell_s = 2.0;                  ///< 每条方位扫描射线的积分驻留时间（s）。
    double scan_speed_deg_s = 10.0;             ///< 云台相邻点位之间的运动速度（度/秒）。
    double pointing_settle_s = 0.0;             ///< 到达点位后的额外稳定等待时间（s）。
    double scan_overhead_s = 10.0;              ///< 每圈回零、状态上报等固定开销（s）。
    bool include_vertical_stare = true;         ///< 每个周期开始前是否增加一条 90 度垂直观测射线。
    double vertical_stare_dwell_s = 30.0;       ///< 垂直观测射线的积分时间（s）。

    /** @brief 返回指定周期在仰角轮换队列中的位置。 */
    std::size_t elevation_schedule_index(int cycle_index) const {
        if (elevations_deg.empty()) {
            return 0;
        }
        const long long count = static_cast<long long>(elevations_deg.size());
        long long index = static_cast<long long>(cycle_index) % count;
        if (index < 0) {
            index += count;
        }
        return static_cast<std::size_t>(index);
    }

    /** @brief 返回指定周期实际执行方位扫描的固定仰角（度）。 */
    double elevation_for_cycle(int cycle_index) const {
        return elevations_deg.empty()
            ? 0.0
            : elevations_deg[elevation_schedule_index(cycle_index)];
    }

    /** @brief 按固定仰角判定采购文件中的水平扫描或锥形扫描模式。 */
    std::string scan_pattern_for_elevation(double elevation_deg) const {
        return std::abs(elevation_deg) <= 1e-9
            ? "horizontal_scan"
            : "conical_scan";
    }

    /** @brief 返回指定周期的设备级扫描模式。 */
    std::string scan_pattern_for_cycle(int cycle_index) const {
        return scan_pattern_for_elevation(elevation_for_cycle(cycle_index));
    }

    /** @brief 根据扫描角域和步进计算水平射线数。 */
    int horizontal_ray_count() const {
        double span = azimuth_stop_deg - azimuth_start_deg;
        if (span <= 0.0) {
            span += 360.0;
        }
        return std::max(1, static_cast<int>(std::ceil(span / std::max(azimuth_step_deg, 0.1))));
    }

    /** @brief 计算相邻水平射线之间的云台运动和稳定耗时（s）。 */
    double movement_seconds_per_ray() const {
        return azimuth_step_deg / std::max(scan_speed_deg_s, 0.1) + std::max(pointing_settle_s, 0.0);
    }

    /** @brief 按 PRF 和驻留时间计算每条射线的积分脉冲数。 */
    int integrated_pulses_per_ray(double pulse_repetition_hz) const {
        return static_cast<int>(std::llround(std::max(pulse_repetition_hz, 0.0) * std::max(line_dwell_s, 0.0)));
    }

    /** @brief 计算单个固定仰角方位扫描周期耗时，不含可选的垂直观测。 */
    double horizontal_scan_cycle_seconds() const {
        return static_cast<double>(horizontal_ray_count())
            * (std::max(line_dwell_s, 0.0) + movement_seconds_per_ray())
            + std::max(scan_overhead_s, 0.0);
    }

    /** @brief 计算包含可选垂直观测在内的完整周期耗时。 */
    double full_scan_cycle_seconds() const {
        return horizontal_scan_cycle_seconds()
            + (include_vertical_stare ? std::max(vertical_stare_dwell_s, 0.0) : 0.0);
    }

    /** @brief 计算轮换队列完整执行一遍的总采集耗时。 */
    double elevation_schedule_seconds() const {
        return static_cast<double>(std::max<std::size_t>(elevations_deg.size(), 1))
            * full_scan_cycle_seconds();
    }
};

/**
 * @brief 正演场景和未经过实机标定的光电参数。
 *
 * 本结构中的光电数值只用于生成可重复的合成回波，不属于厂家公开规格。
 */
struct SimulationSceneConfig {
    std::string application_mode = "construction_site"; ///< 污染场景类型，默认模拟工地环境。
    std::string calibration_status = "assumed_pending_real_device_capture"; ///< 标定状态：等待实机采集校准。
    int seed = 7;                              ///< 随机种子，保证逐周期仿真可复现。
    double pulse_repetition_hz = 5000.0;       ///< 假设的激光脉冲重复频率（Hz）。
    double pulse_energy_mj = 0.02;             ///< 假设的单脉冲能量（mJ）。
    double pulse_energy_jitter = 0.015;        ///< 单脉冲能量相对抖动（1 sigma 比例）。
    double system_constant = 6500000000.0;     ///< 正演 LiDAR 方程使用的系统常数。
    double aerosol_lidar_ratio_sr = 45.0;      ///< 假设的气溶胶激光雷达比（sr）。
    double background_counts_mean = 80.0;      ///< 单脉冲等效背景计数均值。
    double background_counts_jitter = 12.0;    ///< 周期间背景计数标准差。
    double detector_dark_counts = 8.0;         ///< 探测器暗计数等效均值。
    double read_noise_counts = 3.0;            ///< 探测器读出噪声标准差（计数）。
    double adc_saturation_counts = 2000000.0;  ///< ADC/计数通道的仿真饱和上限。
    double dead_time_loss = 0.0000025;         ///< 光子计数死时间软压缩系数。
    double afterpulsing_ratio = 0.02;          ///< 前一距离门信号泄漏到下一门的后脉冲比例。
    double solar_background_scale = 1.0;       ///< 太阳背景强度缩放系数。

    // 以下字段描述双接收链形状，均为等待实机标定的占位参数，故不放入硬件公开规格。
    double near_channel_gain = 0.08;           ///< 近场接收链相对增益。
    double near_full_overlap_m = 3.75;         ///< 近场通道达到完整重叠的假设距离（m）。
    double far_full_overlap_m = 120.0;         ///< 远场通道达到完整重叠的假设距离（m）。
    double far_min_overlap = 0.02;             ///< 远场通道近端的最小重叠因子。
    double channel_stitch_range_m = 150.0;     ///< 近远场主通道的平滑拼接中心尺度（m）。
    double vehicle_speed_ms = 0.0;             ///< 走航平台速度（m/s），0 表示固定站。
};

/** @brief 仿真设备的帧发送和播放策略。 */
struct DeviceStreamConfig {
    double playback_time_scale = 1.0; ///< 播放时间倍率；1 表示按配置采集节奏等待。
    int inter_frame_delay_ms = 0;     ///< 固定帧间隔覆盖值（ms）；0 表示使用帧内采集耗时。
    bool emit_truth_fields = false;   ///< 是否发送仿真真值；实时协议默认关闭，防止误当实测量。
    bool emit_ground_observation = false; ///< 是否附加合成地面 PM/气象观测帧。
    bool emit_camera_frames = true;       ///< 是否发送同步相机能力和指向元数据帧。
    bool emit_product_frames = true;      ///< 是否发送未标定的快速产品摘要帧。
    bool auto_start = true;               ///< 初始化完成后是否自动进入扫描状态。
    std::string emulator_protocol = "jsonl-ylj5-emulator-1.0"; ///< 项目自定义 JSONL 仿真协议版本。
    bool vendor_wire_protocol_known = false; ///< 是否已掌握厂家私有线协议；当前必须为 false。
};

/** @brief 一台 YLJ5 仿真设备的完整配置聚合。 */
struct SimDeviceConfig {
    DeviceSiteConfig site;          ///< 部署站点与地理信息。
    Ylj5HardwareSpec hardware;      ///< 有公开证据支撑的硬件能力和边界。
    Ylj5ScanProgram scan;           ///< 水平、锥形和垂直观测的逐周期调度计划。
    SimulationSceneConfig scene;    ///< 大气场景和待标定的光电参数。
    DeviceStreamConfig stream;      ///< 帧发送、真值开关和播放策略。
    int campaign_cycles = 180;      ///< 一轮仿真战役包含的扫描周期数量。
    int minutes_per_cycle = 8;      ///< 相邻仿真周期的场景相位间隔（分钟）。
    bool enforce_public_spec = true;///< 是否强制采购规格校验；仅测试可显式关闭。

    /** @brief 返回所有违反公开采购边界或当前实现能力的配置项。 */
    std::vector<std::string> public_spec_violations() const {
        std::vector<std::string> errors;
        if (!enforce_public_spec) {
            return errors;
        }
        if (std::abs(hardware.wavelength_nm - 532.0) > hardware.wavelength_tolerance_nm) {
            errors.emplace_back("wavelength must remain within 532+/-2 nm");
        }
        double resolution_multiple = hardware.range_resolution_m / 3.75;
        if (hardware.range_resolution_m <= 0.0
            || std::abs(resolution_multiple - std::round(resolution_multiple)) > 1e-9) {
            errors.emplace_back("range resolution must be 3.75 m or an integer multiple");
        }
        if (hardware.maximum_range_m < 20000.0) {
            errors.emplace_back("maximum range must be at least 20 km");
        }
        if (hardware.telescope_count < 2
            || hardware.near_telescope_aperture_mm > 40.0
            || hardware.far_telescope_aperture_mm < 160.0) {
            errors.emplace_back("dual telescope aperture bounds are not satisfied");
        }
        if (!hardware.polarization_channel) {
            errors.emplace_back("polarization detection is required");
        }
        if (scene.pulse_energy_jitter < 0.0
            || scene.pulse_energy_jitter > hardware.laser_power_instability_fraction_max) {
            errors.emplace_back("simulated laser power instability must remain within 2 percent");
        }
        if (scan.mode != "scheduled_azimuth_scan") {
            errors.emplace_back("only the implemented scheduled_azimuth_scan program is supported");
        }
        if (scan.elevation_cycle_policy != "round_robin") {
            errors.emplace_back("only the implemented round_robin elevation policy is supported");
        }
        if (scan.elevations_deg.empty()) {
            errors.emplace_back("elevation schedule must contain at least one angle");
        }
        if (scan.azimuth_start_deg < hardware.azimuth_min_deg
            || scan.azimuth_stop_deg > hardware.azimuth_max_deg
            || scan.azimuth_step_deg < hardware.pointing_resolution_deg) {
            errors.emplace_back("azimuth program is outside the public gimbal limits");
        }
        for (double elevation : scan.elevations_deg) {
            if (elevation < hardware.elevation_min_deg || elevation > hardware.elevation_max_deg) {
                errors.emplace_back("elevation program is outside the 0..180 degree limit");
                break;
            }
        }
        if (scan.scan_speed_deg_s <= 0.0
            || scan.scan_speed_deg_s > hardware.maximum_scan_speed_deg_s) {
            errors.emplace_back("scan speed must be within (0, 30] degree/s");
        }
        if (scan.horizontal_ray_count() < hardware.minimum_horizontal_rays) {
            errors.emplace_back("each azimuth scan must contain at least 180 rays");
        }
        if (scan.integrated_pulses_per_ray(scene.pulse_repetition_hz)
            < hardware.minimum_integrated_pulses_per_ray) {
            errors.emplace_back("each horizontal ray must integrate at least 10000 pulses");
        }
        if (scan.horizontal_scan_cycle_seconds() > hardware.maximum_horizontal_scan_cycle_s) {
            errors.emplace_back("horizontal scan must complete within 15 minutes");
        }
        if (campaign_cycles <= 0 || minutes_per_cycle <= 0) {
            errors.emplace_back("campaign cycle count and period must be positive");
        }
        return errors;
    }
};

} // namespace lidar_server
