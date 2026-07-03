/**
 * @file frame_processor.cpp
 * @brief 实时帧处理器实现。
 */
#include "lidar_client/frame_processor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>

namespace lidar_client {

FrameProcessor::FrameProcessor(const ProcessorConfig& config)
    : config_(config) {
}

void FrameProcessor::handle_frame(const lidar_protocol::Frame& frame) {
    switch (frame.type) {
    case lidar_protocol::FrameType::lidar_raw: {
        // 解析为 LidarProfile 并处理
        lidar_core::LidarProfile profile = lidar_protocol::json_to_profile(frame.payload);

        // parse_frame 会把 timestamp 从 payload 中剔除，用帧的时间戳补回
        profile.timestamp = frame.timestamp;

        // 如果时间戳变化了，先完成上一个时间步
        if (!current_timestamp_.empty() && frame.timestamp != current_timestamp_) {
            finalize_step();
        }

        current_timestamp_ = frame.timestamp;

        // 执行 L1 预处理 + Fernald 反演 + 湿度校正
        lidar_core::ProcessedProfile processed = lidar_core::process_single_profile(
            profile, config_.retrieval, config_.humidity);

        current_processed_.push_back(std::move(processed));
        ++current_raw_count_;
        ++total_processed_;
        break;
    }

    case lidar_protocol::FrameType::ground_obs: {
        lidar_core::GroundMeasurement gm = lidar_protocol::json_to_ground(frame.payload);
        // 同样用帧的时间戳补回（parse_frame 剔除了 timestamp）
        if (gm.timestamp.empty()) {
            gm.timestamp = frame.timestamp;
        }
        current_ground_.push_back(std::move(gm));
        break;
    }

    case lidar_protocol::FrameType::status: {
        // 更新站点信息
        if (frame.payload.contains("site_id")) {
            site_info_.site_id = frame.payload.at("site_id").string_value();
        }
        if (frame.payload.contains("site_name")) {
            site_info_.name = frame.payload.at("site_name").string_value();
        }
        std::cerr << "[processor] Status: " << lidar_core::dump_json(frame.payload, 0) << "\n";
        break;
    }

    case lidar_protocol::FrameType::heartbeat: {
        // heartbeat 标志着一个时间步的结束
        if (!current_timestamp_.empty()) {
            finalize_step();
        }
        break;
    }

    default:
        // 忽略其他帧类型
        break;
    }
}

void FrameProcessor::finalize_step() {
    if (current_timestamp_.empty()) {
        return;
    }

    StepResult result;
    result.timestamp = current_timestamp_;
    result.processed_profiles = std::move(current_processed_);
    result.ground_measurements = std::move(current_ground_);
    result.raw_count = current_raw_count_;

    // 对该时间步的 PPI 射线做热点检测（detect_hotspots 内部按仰角分层，
    // 多仰角体积扫描也能正确分组，不会跨层粘连）
    std::vector<lidar_core::ProcessedProfile> ppi_profiles;
    for (const auto& p : result.processed_profiles) {
        if (p.profile.scan_mode == "ppi") {
            ppi_profiles.push_back(p);
        }
    }

    if (!ppi_profiles.empty()) {
        result.hotspots = lidar_core::detect_hotspots_from_processed(
            ppi_profiles, config_.hotspot);
    }

    // 统计本步 PPI 的仰角层数（体积扫描时 >1）
    std::set<int> elevation_keys;
    for (const auto& p : ppi_profiles) {
        elevation_keys.insert(static_cast<int>(std::round(p.profile.elevation_deg * 100.0)));
    }

    // 输出处理摘要（含射线数、PPI 射线数、仰角层数、热点数）
    std::cerr << "[processor] Step " << current_timestamp_
              << " complete: " << result.raw_count << " rays, "
              << ppi_profiles.size() << " PPI ("
              << elevation_keys.size() << " elevations), "
              << result.hotspots.size() << " hotspots detected.\n";

    // 触发回调
    if (on_step_complete_) {
        on_step_complete_(result);
    }

    // 重置缓冲
    current_timestamp_.clear();
    current_processed_.clear();
    current_ground_.clear();
    current_raw_count_ = 0;
    ++total_steps_completed_;
}

} // namespace lidar_client
