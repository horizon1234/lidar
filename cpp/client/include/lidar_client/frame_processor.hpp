/**
 * @file frame_processor.hpp
 * @brief 实时帧处理器：接收原始帧 → L1 预处理 → Fernald 反演 → 热点检测。
 *
 * FrameProcessor 维护一个时间步缓冲区：当收到某时间步的所有 PPI 射线后，
 * 自动触发热点检测并输出结果。
 */
#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "lidar_core/lidar_core.hpp"
#include "lidar_protocol/frame.hpp"

namespace lidar_client {

/**
 * @brief 单步处理结果（一个时间步处理完毕后的输出）。
 */
struct StepResult {
    std::string timestamp;                             ///< 时间戳
    std::vector<lidar_core::ProcessedProfile> processed_profiles; ///< 该步所有已处理射线
    std::vector<lidar_core::GroundMeasurement> ground_measurements; ///< 该步的地面观测
    std::vector<lidar_core::Hotspot> hotspots;         ///< 检测到的热点
    int raw_count = 0;                                 ///< 接收到的原始射线数
};

/**
 * @brief 帧处理器配置。
 */
struct ProcessorConfig {
    lidar_core::RetrievalConfig retrieval;
    lidar_core::HumidityConfig humidity;
    lidar_core::HotspotConfig hotspot;
    ///< 预期的 PPI 方位数（用于判断一个时间步是否完整），0 表示自动检测
    int expected_ppi_azimuths = 12;
};

/**
 * @brief 实时帧处理器。
 *
 * 工作模式：
 *   1. 调用 handle_frame(frame) 逐帧处理
 *   2. 当一个时间步的帧处理完毕后，调用注册的 on_step_complete 回调
 *   3. 调用 finalize_step() 可强制处理当前缓冲区
 */
class FrameProcessor {
public:
    using StepCompleteCallback = std::function<void(const StepResult&)>;

    explicit FrameProcessor(const ProcessorConfig& config);

    /**
     * @brief 设置时间步完成回调。
     */
    void set_step_complete_callback(StepCompleteCallback callback) {
        on_step_complete_ = std::move(callback);
    }

    /**
     * @brief 处理一帧。
     *
     * 根据帧类型分发：
     *   - lidar_raw: 解析为 LidarProfile → process_single_profile → 缓存到当前时间步
     *   - ground_obs: 解析为 GroundMeasurement → 缓存到当前时间步
     *   - heartbeat: 触发当前时间步的完成（heartbeat 标志着一个时间步的结束）
     *   - status: 记录站点信息
     *
     * @param frame 收到的帧
     */
    void handle_frame(const lidar_protocol::Frame& frame);

    /**
     * @brief 强制完成当前缓冲的时间步。
     */
    void finalize_step();

    /**
     * @brief 获取已处理的总射线数。
     */
    int total_processed() const { return total_processed_; }

    /**
     * @brief 获取已处理的时间步数。
     */
    int total_steps_completed() const { return total_steps_completed_; }

    /**
     * @brief 获取站点信息（从 status 帧获取）。
     */
    const lidar_core::SiteInfo& site_info() const { return site_info_; }

private:
    ProcessorConfig config_;
    StepCompleteCallback on_step_complete_;

    // 当前时间步缓冲
    std::string current_timestamp_;
    std::vector<lidar_core::ProcessedProfile> current_processed_;
    std::vector<lidar_core::GroundMeasurement> current_ground_;
    int current_raw_count_ = 0;

    lidar_core::SiteInfo site_info_;
    int total_processed_ = 0;
    int total_steps_completed_ = 0;
};

} // namespace lidar_client
