/**
 * @file PipelineConfig.hpp
 * @brief YLJ5 正演和实时反演所需的最小配置类型。
 */
#pragma once

#include "LidarDemo/DataTypes.hpp"

#include <cmath>
#include <vector>

namespace lidar_demo {

/** @brief YLJ5 单周期四通道正演配置，由设备层集中填充。 */
struct SimulationConfig {
    std::string application_mode = "construction_site"; ///< 污染场景：construction_site/urban_grid/mobile_mapping。
    int seed = 7;                         ///< 随机种子，保证周期可复现。
    int time_steps = 1;                   ///< 本次惰性生成的周期数，设备固定为 1。
    int minutes_per_step = 8;             ///< 场景相位的周期间隔（分钟）。
    int start_step_index = 0;             ///< 当前全局周期索引。
    int phase_time_steps = 180;           ///< 一轮场景日变化使用的周期数。
    int range_bin_count = 5334;           ///< 距离门数量。
    double range_bin_m = 3.75;            ///< 距离分辨率（米）。
    std::vector<double> ppi_elevations_deg = {0.0}; ///< 当前周期固定方位扫描仰角。
    double ppi_azimuth_start_deg = 0.0;   ///< 方位扫描起点（度）。
    double ppi_azimuth_stop_deg = 360.0;  ///< 方位扫描终点（度）。
    double ppi_azimuth_step_deg = 2.0;    ///< 相邻射线方位步进（度）。
    double ppi_line_dwell_s = 2.0;        ///< 单条方位射线积分时间（秒）。
    double ppi_step_overhead_s = 0.2;     ///< 云台移动与稳定时间（秒）。
    double ppi_scan_overhead_s = 10.0;    ///< 单圈固定开销（秒）。
    bool include_stare_profile = true;    ///< 是否先生成 90 度垂直观测。
    double stare_dwell_s = 30.0;          ///< 垂直观测积分时间（秒）。
    double pulse_repetition_hz = 5000.0;  ///< 假设的激光脉冲重复频率（Hz）。
    double system_constant = 6500000000.0; ///< 正演 LiDAR 方程系统常数。
    double lidar_ratio_sr = 45.0;         ///< 仿真假设的气溶胶激光雷达比（sr）。
    double wavelength_nm = 532.0;         ///< 工作波长（nm）。
    double pulse_energy_mj = 0.02;        ///< 假设的单脉冲能量（mJ）。
    double pulse_energy_jitter = 0.015;   ///< 脉冲能量相对抖动。
    double background_counts_mean = 80.0; ///< 单脉冲等效背景计数均值。
    double background_counts_jitter = 12.0; ///< 周期间背景计数标准差。
    double detector_dark_counts = 8.0;    ///< 探测器暗计数等效均值。
    double read_noise_counts = 3.0;       ///< 读出噪声标准差。
    double adc_saturation_counts = 2000000.0; ///< ADC/计数通道仿真饱和上限。
    double dead_time_loss = 0.0000025;    ///< 光子计数死时间软压缩系数。
    double afterpulsing_ratio = 0.02;     ///< 后脉冲比例。
    double solar_background_scale = 1.0; ///< 太阳背景缩放系数。
    double vehicle_speed_ms = 0.0;       ///< 走航平台速度；固定站为 0。
    double truth_hotspot_ext_threshold = 0.025; ///< 仿真真值热点消光阈值。
    double near_telescope_aperture_mm = 40.0; ///< 近场望远镜口径（mm）。
    double far_telescope_aperture_mm = 160.0; ///< 远场望远镜口径（mm）。
    double near_channel_gain = 0.08;     ///< 近场接收链相对增益假设。
    double near_full_overlap_m = 3.75;   ///< 近场完整 overlap 假设距离（米）。
    double far_full_overlap_m = 120.0;   ///< 远场完整 overlap 假设距离（米）。
    double far_min_overlap = 0.02;       ///< 远场近端最小 overlap。
    double channel_stitch_range_m = 150.0; ///< 近远场拼接尺度（米）。

    /** @brief 返回当前周期仰角；空配置安全退化为 0 度水平扫描。 */
    std::vector<double> effective_ppi_elevations_deg() const {
        return ppi_elevations_deg.empty() ? std::vector<double>{0.0} : ppi_elevations_deg;
    }

    /** @brief 根据角域和步进生成方位角，不重复完整圆的终点。 */
    std::vector<double> effective_ppi_azimuths_deg() const {
        std::vector<double> azimuths;
        const double step = ppi_azimuth_step_deg > 0.0 ? ppi_azimuth_step_deg : 1.0;
        double span = ppi_azimuth_stop_deg - ppi_azimuth_start_deg;
        if (span <= 0.0) span += 360.0;
        const bool full_circle = span >= 360.0 - 1e-9;
        const double end = ppi_azimuth_start_deg + span;
        for (double azimuth = ppi_azimuth_start_deg;
             full_circle ? azimuth < end - 1e-9 : azimuth <= end + 1e-9;
             azimuth += step) {
            double normalized = std::fmod(azimuth, 360.0);
            if (normalized < 0.0) normalized += 360.0;
            azimuths.push_back(normalized);
        }
        return azimuths.empty() ? std::vector<double>{0.0} : azimuths;
    }

    /** @brief 计算当前周期固定仰角方位扫描时长。 */
    double ppi_scan_cycle_seconds() const {
        return static_cast<double>(
            effective_ppi_elevations_deg().size() * effective_ppi_azimuths_deg().size())
            * (ppi_line_dwell_s + ppi_step_overhead_s) + ppi_scan_overhead_s;
    }
};

/** @brief Fernald/Klett 弹性反演参数。 */
struct RetrievalConfig {
    double aerosol_lidar_ratio_sr = 45.0;          ///< 气溶胶激光雷达比（sr）。
    double reference_aerosol_backscatter = 0.0004; ///< 远端参考气溶胶后向散射。
    int reference_window_bins = 64;                ///< 从有效廓线远端选取的参考窗距离门数。
    int minimum_valid_bins = 16;                   ///< 允许执行反演的最短连续有效区间。
    double maximum_extinction_per_km = 0.45;       ///< 数值稳定保护上限（km^-1）。
};

/** @brief 湿度修正参数。 */
struct HumidityConfig {
    double dry_reference_rh = 0.45; ///< 干参考相对湿度。
    double hygroscopicity = 1.1;    ///< 吸湿性参数。
};

/** @brief 已标定 PM 热点检测参数。 */
struct HotspotConfig {
    double pm25_threshold_ugm3 = 50.0;              ///< PM2.5 绝对阈值。
    double scan_relative_pm25_threshold_ugm3 = 0.18; ///< 相对背景 PM2.5 增量阈值。
    double scan_relative_dry_ext_threshold = 0.02;  ///< 衰减后向散射相对增强阈值。
    int min_cells = 3;                              ///< 最小连通像元数。
};

/** @brief YLJ5 单周期正演聚合配置。 */
struct PipelineConfig {
    SiteInfo site;               ///< 当前设备部署站点。
    SimulationConfig simulation; ///< 四通道正演参数。
};

} // namespace lidar_demo
