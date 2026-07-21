/**
 * @file DataTypes.hpp
 * @brief LiDAR 公共数据载体类型。
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lidar_demo {

/** @brief 逐距离门质量位的存储类型；一个 bin 可同时具有多个原因位。 */
using BinQualityMask = std::uint32_t;

/**
 * @brief 逐距离门质量位；可同时记录多个导致无效或降级的原因。
 *
 * 这些值是协议和处理结果的一部分，位序一旦发布就不能重排。partial_overlap 是降级位：
 * 它允许定性光学反演，但禁止 PM/热点；其余列入 RetrievalInvalidQualityMask 的位会直接
 * 阻断 Fernald。上游固件或适配器也必须使用同一位定义。
 */
enum class BinQualityFlag : BinQualityMask {
    none = 0,                         ///< 当前未发现失效或降级原因。
    below_minimum_range = 1u << 0,   ///< 位于触发瞬态/距离零点标定给出的最小有效距离内。
    overlap_unusable = 1u << 1,      ///< overlap 太低，除法会不可接受地放大噪声和标定误差。
    partial_overlap = 1u << 2,       ///< 可做定性光学反演，但未达到定量 PM 所需 overlap。
    saturated = 1u << 3,             ///< ADC 或光子计数器达到饱和保护区，真实计数不可恢复。
    dead_time_uncorrectable = 1u << 4, ///< 光子计数占用率过高，死时间反函数接近发散。
    low_snr = 1u << 5,               ///< 通道 SNR 低于配置阈值。
    non_finite_input = 1u << 6,      ///< 原始计数、背景或 overlap 含 NaN/Inf。
    no_valid_channel = 1u << 7,      ///< 近、远场在该距离门均不可用。
    invalid_laser_energy = 1u << 8,  ///< 激光能量缺失/非正，无法做能量归一化。
    upstream_invalid = 1u << 9,      ///< 固件标记无效，或 afterpulse 依赖的前序 bin 不可恢复。
    retrieval_unavailable = 1u << 10, ///< 未落入 Fernald 最长连续有效区或反演输出非有限。
};

/** @brief 把强类型枚举转换为可按位组合的整数掩码。 */
constexpr BinQualityMask quality_mask(BinQualityFlag flag) {
    return static_cast<BinQualityMask>(flag);
}

/** @brief 判断一个逐 bin 掩码是否包含指定原因位。 */
constexpr bool has_quality(BinQualityMask mask, BinQualityFlag flag) {
    return (mask & quality_mask(flag)) != 0;
}

/** @brief 任何一个命中都会阻断光学反演的质量位集合；partial_overlap 不在其中。 */
constexpr BinQualityMask RetrievalInvalidQualityMask =
    quality_mask(BinQualityFlag::below_minimum_range)
    | quality_mask(BinQualityFlag::overlap_unusable)
    | quality_mask(BinQualityFlag::saturated)
    | quality_mask(BinQualityFlag::dead_time_uncorrectable)
    | quality_mask(BinQualityFlag::low_snr)
    | quality_mask(BinQualityFlag::non_finite_input)
    | quality_mask(BinQualityFlag::no_valid_channel)
    | quality_mask(BinQualityFlag::invalid_laser_energy)
    | quality_mask(BinQualityFlag::upstream_invalid)
    | quality_mask(BinQualityFlag::retrieval_unavailable);

/** @brief 判断 bin 是否可进入 Fernald/Klett 及干消光计算。 */
constexpr bool bin_is_usable_for_retrieval(BinQualityMask mask) {
    return (mask & RetrievalInvalidQualityMask) == 0;
}

/** @brief 判断 bin 是否同时满足反演和定量 overlap 要求，可进入 PM/热点。 */
constexpr bool bin_is_usable_for_quantitative_product(BinQualityMask mask) {
    return bin_is_usable_for_retrieval(mask)
        && !has_quality(mask, BinQualityFlag::partial_overlap);
}

// ============================================================================
// 数据载体结构体
// ============================================================================

/**
 * @brief 站点信息
 *
 * 描述一个 LiDAR 设备部署站点的地理与标识信息。
 */
struct SiteInfo {
    std::string name;            ///< YLJ5 部署站点可读名称。
    double latitude_deg = 0.0;   ///< 纬度（°，WGS84）
    double longitude_deg = 0.0;  ///< 经度（°，WGS84）
    double altitude_m = 0.0;     ///< 海拔高度（m）
    std::string site_id;         ///< 站点唯一标识符。
};

/** @brief 一条由望远镜和偏振分光共同确定的物理接收路径。 */
struct LidarChannel {
    std::string channel_id;           ///< 稳定通道标识，如 near_parallel_532nm。
    std::string telescope;            ///< 望远镜路径：near（近场）或 far（远场）。
    std::string polarization;         ///< 偏振分量：parallel（平行）或 perpendicular（垂直）。
    double wavelength_nm = 0.0;       ///< 当前通道中心波长（nm）。
    double telescope_aperture_mm = 0.0; ///< 当前接收望远镜口径（mm）。
    double relative_gain = 1.0;       ///< 相对远场平行通道的仿真增益。
    double background_counts = 0.0;   ///< 单脉冲等效背景计数。
    std::vector<double> raw_counts;   ///< 与 profile 距离轴对齐的通道原始计数。
    std::vector<double> overlap;      ///< 与 profile 距离轴对齐的几何重叠因子。
    std::vector<BinQualityMask> device_bin_quality; ///< 固件/适配器给出的逐距离门质量位，可为空。
};

/**
 * @brief 单条 LiDAR 扫描射线（ray）的完整原始数据
 *
 * 包含一条 YLJ5 射线的角度、距离轴、原始光子计数、四物理通道、气象场、
 * 分子参考场以及仅用于仿真验证的真值场。
 */
struct LidarProfile {
    std::string site_id;          ///< 所属站点标识
    std::string timestamp;        ///< 时间戳（ISO8601 分钟精度，如 "2026-05-30T08:00"）
    std::string scan_id;          ///< 扫描批次标识
    std::string scan_mode;        ///< 扫描模式："stare"（定点凝视）或 "ppi"（平面位置显示）
    std::string source_kind;      ///< 数据来源类型，如 ylj5_synthetic_stare / ylj5_synthetic_ppi。
    double azimuth_deg = 0.0;     ///< 方位角（°，正北顺时针）
    double elevation_deg = 0.0;   ///< 仰角（°，水平为 0，天顶为 90）
    std::vector<double> ranges_m; ///< 各距离 bin 的距离（m）
    std::vector<double> raw_counts;///< 各距离 bin 的原始光子计数
    double laser_energy_mj = 0.0; ///< 单脉冲激光能量（mJ）
    double background_counts = 0.0;///< 背景光子计数（基底噪声估计）
    std::vector<double> overlap;   ///< 几何重叠因子随距离的变化（0~1）
    double relative_humidity = 0.0;///< 相对湿度（0~1）
    double temperature_c = 0.0;   ///< 温度（℃）
    double wind_speed_ms = 0.0;   ///< 风速（m/s）
    double wind_dir_deg = 0.0;    ///< 风向（°）
    std::vector<double> molecular_backscatter;  ///< 分子（Rayleigh）后向散射系数（km^-1 sr^-1）
    std::vector<double> molecular_extinction;   ///< 分子（Rayleigh）消光系数（km^-1）
    // ---- 以下真值字段仅在仿真/评估中使用，真实数据不存在 ----
    std::vector<double> true_backscatter;  ///< 分子+气溶胶总后向散射真值（用于评估）
    std::vector<double> true_extinction;   ///< 分子+气溶胶总消光真值（用于评估）
    std::vector<double> true_pm25;         ///< PM2.5 浓度真值（µg/m³，用于评估）
    std::vector<double> true_pm10;         ///< PM10 浓度真值（µg/m³，用于评估）
    std::vector<int> true_hotspot_mask;    ///< 热点像元掩膜真值（0/1，用于评估）
    // raw_counts 是近远场平行通道拼接后的反演主通道。
    std::vector<LidarChannel> channels; ///< 近/远场与平行/垂直偏振组成的物理通道列表。
    std::vector<double> depolarization_ratio; ///< 与距离轴对齐的体退偏比。
    std::vector<BinQualityMask> device_bin_quality; ///< 拼接主通道的上游逐距离门质量位，可为空。
};

/**
 * @brief 同址地面观测
 *
 * 与 LiDAR 共址（或临近）的地面站观测到的一组 PM 与气象量。
 */
struct GroundMeasurement {
    std::string site_id;              ///< 所属站点标识
    std::string timestamp;            ///< 时间戳（ISO8601 分钟精度）
    double pm25_ugm3 = 0.0;           ///< 地面 PM2.5 浓度（µg/m³）
    double pm10_ugm3 = 0.0;           ///< 地面 PM10 浓度（µg/m³）
    double relative_humidity = 0.0;   ///< 相对湿度（0~1）
    double temperature_c = 0.0;       ///< 温度（℃）
    double wind_speed_ms = 0.0;       ///< 风速（m/s）
    double wind_dir_deg = 0.0;        ///< 风向（°）
};

/** @brief 供惰性设备仿真使用、不执行反演和文件输出的轻量战役结果。 */
struct SyntheticCampaign {
    SiteInfo site;                                  ///< 当前战役的站点信息。
    std::vector<LidarProfile> profiles;             ///< 正演生成的原始扫描射线。
    std::vector<GroundMeasurement> ground_measurements; ///< 可选的合成地面观测。
};

/**
 * @brief L1 预处理中间结果。
 *
 * 对应商用 LiDAR 数据产品里的 background-corrected signal、range-corrected signal、
 * SNR/CNR 和 QC flag。公开设备软件通常不会只输出最终 PM，而会保留这些中间量用于复核。
 */
struct PreprocessResult {
    std::vector<double> l1_signal;              ///< 背景扣除后的 L1 信号
    std::vector<double> attenuated_backscatter; ///< 能量/overlap/range 校正后的衰减后向散射代理量
    std::vector<double> snr;                    ///< 距离门信噪比
    std::vector<BinQualityMask> bin_quality;     ///< 与距离轴对齐的逐距离门质量位。
    std::vector<std::string> qc_flags;          ///< 质控标志
};

/** @brief Fernald/Klett 光学反演输出，所有数组均与输入距离轴对齐。 */
struct OpticalRetrievalResult {
    std::vector<double> total_extinction;      ///< 分子+气溶胶总消光（环境湿态，km^-1）。
    std::vector<double> aerosol_backscatter;   ///< 气溶胶后向散射（km^-1 sr^-1）。
    std::vector<double> aerosol_extinction;    ///< 气溶胶消光（环境湿态，km^-1）。
};

/**
 * @brief 一条 LiDAR 射线经过全流程处理后的结果
 *
 * 在原始 LidarProfile 基础上，叠加了 L1 信号、衰减后向散射、信噪比、
 * 反演得到的消光与 PM 浓度、ENU 三维坐标、质控标志等下游产物。
 */
struct ProcessedProfile {
    LidarProfile profile;             ///< 原始剖面（透传保存）
    std::vector<double> l1_signal;    ///< L1 级信号（背景扣除+能量归一化+重叠校正后）
    std::vector<double> attenuated_backscatter; ///< 衰减后向散射（距离平方校正后）
    std::vector<double> snr;          ///< 各 bin 的信噪比（SNR）
    std::vector<double> extinction;   ///< 反演得到的分子+气溶胶总消光（环境湿态，km^-1）
    std::vector<double> aerosol_backscatter; ///< 反演得到的气溶胶后向散射（km^-1 sr^-1）
    std::vector<double> aerosol_extinction; ///< 反演得到的气溶胶消光（环境湿态，km^-1）
    std::vector<double> dry_extinction; ///< 湿度修正后的干态气溶胶消光（km^-1）
    std::vector<double> pm25;         ///< 估算的 PM2.5 浓度（µg/m³）
    std::vector<double> pm10;         ///< 估算的 PM10 浓度（µg/m³）
    std::vector<std::vector<double>> enu_points_m; ///< 各 bin 在 ENU 坐标系下的 [东,北,上]（m）
    std::vector<BinQualityMask> bin_quality; ///< 与产品数组对齐的逐距离门质量位。
    std::vector<std::string> qc_flags; ///< 质控标志列表（如 low-laser-energy 等）
    double latency_ms = 0.0;          ///< 处理该射线所耗费的时间（ms，用于性能统计）
};

/**
 * @brief 检测到的一个污染热点
 *
 * 由热点检测算法（在 PPI 网格上做连通域分析）产出的一个连通区域。
 */
struct Hotspot {
    std::string timestamp;          ///< 热点所在时间步的时间戳
    std::string scan_id;            ///< 热点所属的扫描批次
    std::string hotspot_id;         ///< 热点唯一编号
    std::vector<double> centroid_enu_m; ///< 热点质心在 ENU 坐标下的 [东,北,上]（m）
    double peak_pm25_ugm3 = 0.0;    ///< 热点区域的 PM2.5 峰值（µg/m³）
    double mean_pm25_ugm3 = 0.0;    ///< 热点区域的 PM2.5 平均值（µg/m³）
    double estimated_area_m2 = 0.0; ///< 热点区域的估计面积（m²）
    int cell_count = 0;             ///< 热点所包含的网格像元数
    std::string severity;           ///< 严重程度分级："medium" / "high" / "critical"
};

} // namespace lidar_demo
