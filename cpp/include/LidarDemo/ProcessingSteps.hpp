/**
 * @file ProcessingSteps.hpp
 * @brief Public single-profile processing step classes.
 */
#pragma once

#include "LidarDemo/PipelineConfig.hpp"

#include <utility>
#include <vector>

namespace lidar_demo {

/**
 * @brief L0 -> L1 预处理步骤。
 *
 * 职责：背景扣除、激光能量归一、overlap 修正、range^2 修正、SNR 和 QC flag。
 */
class BackgroundPreprocessStep {
public:
    PreprocessResult process(const LidarProfile& profile) const;
};

/**
 * @brief Fernald/Klett 弹性 LiDAR 反演步骤。
 *
 * 职责：由 attenuated backscatter 估计消光和气溶胶后向散射。
 */
class FernaldInversionStep {
public:
    explicit FernaldInversionStep(RetrievalConfig config);

    std::pair<std::vector<double>, std::vector<double>> process(
        const LidarProfile& profile,
        const std::vector<double>& attenuated_backscatter) const;

private:
    RetrievalConfig config_;
};

/**
 * @brief 湿度修正步骤。
 *
 * 职责：把湿消光换算到干参考状态，降低 RH 对 PM 估计的虚假放大。
 */
class HumidityCorrectionStep {
public:
    explicit HumidityCorrectionStep(HumidityConfig config);

    std::vector<double> process(
        const std::vector<double>& extinction,
        double relative_humidity) const;

private:
    HumidityConfig config_;
};

/**
 * @brief 极坐标到 ENU 坐标投影步骤。
 */
class CoordinateProjectionStep {
public:
    std::vector<std::vector<double>> process(const LidarProfile& profile) const;
};

/**
 * @brief PPI/体扫热点检测步骤。
 *
 * 职责：按 timestamp 内的扫描射线构建距离-方位网格，执行阈值、相对异常和连通域分析。
 */
class HotspotDetectionStep {
public:
    explicit HotspotDetectionStep(HotspotConfig config);

    std::vector<Hotspot> process(const std::vector<ProcessedProfile>& ppi_profiles) const;

private:
    HotspotConfig config_;
};

/**
 * @brief 单条 profile 的类化处理链。
 *
 * 这个类把商用设备软件常见的 L0 -> L1 -> L2 几个阶段拆开，便于调试、面试讲解和后续替换算法。
 */
class SingleProfileProcessingChain {
public:
    SingleProfileProcessingChain(RetrievalConfig retrieval, HumidityConfig humidity);

    ProcessedProfile process(const LidarProfile& profile, bool disable_humidity = false) const;

private:
    BackgroundPreprocessStep preprocess_;
    FernaldInversionStep inversion_;
    HumidityCorrectionStep humidity_;
    CoordinateProjectionStep projection_;
};

} // namespace lidar_demo
