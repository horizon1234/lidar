/**
 * @file PipelineConfig.hpp
 * @brief Public pipeline configuration types.
 */
#pragma once

#include "LidarDemo/DataTypes.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace lidar_demo {

// ============================================================================
// 各阶段配置
// ============================================================================

/**
 * @brief 仿真配置
 *
 * 控制合成 LiDAR 场景生成的全部参数（时间步、距离分辨率、PPI 扫描几何、
 * 系统常数、激光雷达比等）。
 *
 * PPI 多仰角：真实扫描雷达一个体积扫描（volume scan）会顺序扫描多个仰角，
 * 每个仰角各做一圈方位扫描，从而得到三维体数据。`ppi_elevations_deg`
 * 为空时退化为旧行为（仅一个仰角），保证向后兼容。
 */
struct SimulationConfig {
    std::string instrument_preset = "demo_lidar";              ///< 设备预设：demo_lidar / field_scanning_pm_lidar / ceilometer_profile / mobile_mapping_lidar
    std::string application_mode = "legacy_demo";              ///< 应用场景：legacy_demo / construction_site / urban_grid / mobile_mapping
    std::string vendor_profile = "generic_jsonl";              ///< 公开格式映射：generic_jsonl / vaisala_cl61_like / raymetrics_pmeye_like / halo_hpl_like
    int seed = 7;                          ///< 随机数种子（决定可复现性）
    int time_steps = 18;                   ///< 仿真的时间步数量
    int minutes_per_step = 20;            ///< 相邻时间步的间隔（分钟）
    int range_bin_count = 60;             ///< 每条射线的距离 bin 数量
    double range_bin_m = 100.0;           ///< 相邻距离 bin 的间距（m）
    /// PPI 体积扫描的仰角序列（°，高于水平面）。
    /// 默认 {5,15,30,45,60,75} 为接近商用气溶胶扫描雷达的 6 层体积扫描方案
    /// （如 Raymetrics / 国产扫描激光雷达的常规配置）。配套参数已按商用尺度标定：
    /// 量程 6 km（60 bin × 100 m）、边界层标高 1200 m、烟羽高度 280/380 m。
    /// 高仰角 75° 在 6 km 处达 5.8 km，仍在对流层下部，信号有意义；
    /// 烟羽 1（280 m/2200 m）所需仰角约 7°，由 15° 层捕捉；最低 5° 层覆盖近场低空源。
    /// 为空时退化为单个低仰角（兼容旧配置）。
    std::vector<double> ppi_elevations_deg = {5.0, 15.0, 30.0, 45.0, 60.0, 75.0};
    double ppi_azimuth_start_deg = 0.0;   ///< PPI/扇区扫描起始方位角（°，正北顺时针）
    double ppi_azimuth_stop_deg = 360.0;  ///< PPI/扇区扫描结束方位角（°）；360 表示完整一圈但不重复 0°
    double ppi_azimuth_step_deg = 30.0;   ///< PPI 扫描的方位角步进（°）
    double ppi_line_dwell_s = 1.0;        ///< 每条视线的积分/驻留时间（s），商用 PM 扫描雷达常见量级为秒到十秒
    double ppi_scan_overhead_s = 0.0;      ///< 一次扫描周期的转台回零、状态上报等额外耗时（s）
    double pulse_repetition_hz = 20.0;     ///< 激光脉冲重复频率（Hz），用于估算每条 profile 的积分脉冲数
    double system_constant = 260000000.0; ///< LiDAR 系统常数 C（正演发射方程用）
    double lidar_ratio_sr = 45.0;         ///< 气溶胶激光雷达比（sr，消光后向散射比）
    double wavelength_nm = 532.0;         ///< 激光波长（nm），PM 扫描雷达常用 355nm，云高仪常用 905/910nm
    double pulse_energy_mj = 1.0;         ///< 单脉冲能量均值（mJ），用于仿真每条射线的能量抖动
    double pulse_energy_jitter = 0.03;    ///< 脉冲能量相对抖动（1 sigma）
    double background_counts_mean = 10.5; ///< 背景/暗计数均值（计数）
    double background_counts_jitter = 0.5;///< 背景计数随机扰动（1 sigma）
    double full_overlap_m = 500.0;        ///< 进入完整 overlap 的距离（m）
    double min_overlap = 0.22;            ///< 近场最小 overlap
    double detector_dark_counts = 0.0;    ///< 探测器暗计数等效均值
    double read_noise_counts = 0.0;       ///< 读出噪声（计数）
    double adc_saturation_counts = 50000000.0; ///< 模拟/ADC 饱和上限
    double dead_time_loss = 0.000000018;  ///< photon-counting 死时间损失近似系数
    double solar_background_scale = 1.0;  ///< 太阳背景强度缩放，移动/白天工地可调高
    double vehicle_speed_ms = 0.0;        ///< 走航模式下平台速度（m/s），固定站为 0
    double truth_hotspot_ext_threshold = 0.025; ///< 真值热点掩膜的烟羽干消光贡献阈值，默认保留旧行为

    /**
     * @brief 取有效仰角列表（兼容旧配置）。
     *
     * 若 `ppi_elevations_deg` 为空，则返回单个低仰角占位值 {8.0}，
     * 使得旧的单仰角配置仍能运行（只是不再多仰角扫描）。
     */
    std::vector<double> effective_ppi_elevations_deg() const {
        return ppi_elevations_deg.empty()
            ? std::vector<double>{8.0}
            : ppi_elevations_deg;
    }

    /**
     * @brief 取有效 PPI 方位角列表。
     *
     * 默认 [0, 360) 保持旧的完整 PPI 行为；当 stop-start 小于 360 时按闭区间生成
     * 扇区扫描，例如 0..180 且步进 2.5° 会得到 73 条视线。
     */
    std::vector<double> effective_ppi_azimuths_deg() const {
        std::vector<double> azimuths;
        double step = ppi_azimuth_step_deg > 0.0 ? ppi_azimuth_step_deg : 1.0;
        double span = ppi_azimuth_stop_deg - ppi_azimuth_start_deg;
        if (span <= 0.0) {
            span += 360.0;
        }
        bool full_circle = span >= 360.0 - 1e-9;
        double end = ppi_azimuth_start_deg + span;
        for (double azimuth = ppi_azimuth_start_deg;
             full_circle ? (azimuth < end - 1e-9) : (azimuth <= end + 1e-9);
             azimuth += step) {
            double normalized = std::fmod(azimuth, 360.0);
            if (normalized < 0.0) {
                normalized += 360.0;
            }
            azimuths.push_back(normalized);
        }
        return azimuths.empty() ? std::vector<double>{0.0} : azimuths;
    }

    double ppi_scan_cycle_seconds() const {
        return static_cast<double>(effective_ppi_elevations_deg().size() * effective_ppi_azimuths_deg().size())
            * ppi_line_dwell_s + ppi_scan_overhead_s;
    }
};

/**
 * @brief 反演配置
 *
 * Fernald/Klett 反演所用的参数。
 */
struct RetrievalConfig {
    double aerosol_lidar_ratio_sr = 45.0;        ///< 气溶胶激光雷达比（sr）
    double reference_aerosol_backscatter = 0.0004; ///< 参考点处的气溶胶后向散射系数
};

/**
 * @brief 湿度校正配置
 *
 * 控制吸湿增长因子 f(RH) 的两个参数，用于把"湿"消光换算为"干"消光。
 */
struct HumidityConfig {
    double dry_reference_rh = 0.45;  ///< 干燥参考点的相对湿度（0~1）
    double hygroscopicity = 1.1;     ///< 吸湿性参数 κ（越大表示颗粒越易吸湿增长）
};

/**
 * @brief PM 标定配置
 *
 * 控制 PM2.5/PM10 回归标定的训练集划分与特征构造。
 */
struct PmCalibrationConfig {
    double train_ratio = 0.6;   ///< 训练集比例（其余按 val_ratio 划分验证/测试）
    double val_ratio = 0.2;     ///< 验证集比例
    int surface_bin_count = 6;  ///< 自地面起算用于特征提取的近端 bin 数
};

/**
 * @brief 热点检测配置
 *
 * 控制热点判定阈值与最小连通域规模。
 */
struct HotspotConfig {
    double pm25_threshold_ugm3 = 50.0;             ///< PM2.5 绝对阈值（µg/m³）
    double scan_relative_pm25_threshold_ugm3 = 0.18; ///< PM2.5 相对阈值（相对中位数的增量）
    double scan_relative_dry_ext_threshold = 0.02;  ///< 干消光相对阈值
    int min_cells = 3;                             ///< 热点连通域所需的最小像元数
};

/**
 * @brief 评估配置
 */
struct EvaluationConfig {
    std::vector<double> sensitivity_lidar_ratios; ///< 做灵敏度分析时要尝试的激光雷达比列表
};
/**
 * @brief 管线总配置
 *
 * 汇总上述所有配置段，以及顶层的数据源模式。通常由 parse_pipeline_config
 * 从一个 JSON 配置文件解析得到。
 */
struct PipelineConfig {
    std::string source_mode = "simulation"; ///< 数据源模式（取自 source.mode，便于快速判断）
    SourceConfig source;     ///< 数据源配置
    SiteInfo site;           ///< 站点信息
    SimulationConfig simulation;     ///< 仿真配置
    RetrievalConfig retrieval;       ///< 反演配置
    HumidityConfig humidity;         ///< 湿度校正配置
    PmCalibrationConfig pm_calibration; ///< PM 标定配置
    HotspotConfig hotspot;           ///< 热点检测配置
    EvaluationConfig evaluation;     ///< 评估配置
};

} // namespace lidar_demo
