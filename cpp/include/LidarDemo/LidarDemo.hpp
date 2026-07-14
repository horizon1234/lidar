/**
 * @file LidarDemo.hpp
 * @brief YLJ5 正演、协议和实时处理共享的最小公共 API。
 */
#pragma once

#include "LidarDemo/Json.hpp"
#include "LidarDemo/ProcessingSteps.hpp"

#include <vector>

namespace lidar_demo {

/** @brief 对同周期已标定 PPI 廓线执行分层热点检测。 */
std::vector<Hotspot> detect_hotspots_from_processed(
    const std::vector<ProcessedProfile>& ppi_profiles,
    const HotspotConfig& hotspot_config);

/** @brief 只正演一个或少量 YLJ5 周期，不执行文件输出和离线评测。 */
SyntheticCampaign generate_synthetic_campaign(const PipelineConfig& config);

} // namespace lidar_demo
