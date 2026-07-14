/**
 * @file ProcessingSteps.hpp
 * @brief YLJ5 客户端使用的实时处理步骤接口。
 */
#pragma once

#include "LidarDemo/PipelineConfig.hpp"

#include <utility>
#include <vector>

namespace lidar_demo {

/** @brief 近远场拼接主通道的背景、能量、overlap、距离平方和 SNR 预处理。 */
class BackgroundPreprocessStep {
public:
    PreprocessResult process(const LidarProfile& profile) const;
};

/** @brief 使用远端参考和固定激光雷达比执行 Fernald/Klett 反演。 */
class FernaldInversionStep {
public:
    explicit FernaldInversionStep(RetrievalConfig config);

    std::pair<std::vector<double>, std::vector<double>> process(
        const LidarProfile& profile,
        const std::vector<double>& attenuated_backscatter,
        const std::vector<BinQualityMask>& bin_quality = {}) const;

private:
    RetrievalConfig config_; ///< 当前弹性反演参数。
};

/** @brief 把环境湿消光修正到干参考状态。 */
class HumidityCorrectionStep {
public:
    explicit HumidityCorrectionStep(HumidityConfig config);

    std::vector<double> process(
        const std::vector<double>& extinction,
        double relative_humidity) const;

private:
    HumidityConfig config_; ///< 当前吸湿增长修正参数。
};

/** @brief 把距离、方位和仰角投影到本地 ENU 坐标。 */
class CoordinateProjectionStep {
public:
    std::vector<std::vector<double>> process(const LidarProfile& profile) const;
};

} // namespace lidar_demo
