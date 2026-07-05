/**
 * @file alarm_state_machine.hpp
 * @brief 告警状态机：管理热点事件的生命周期。
 *
 * 状态转移图：
 *
 *   candidate → confirmed → active → mitigating → resolved
 *       ↓           ↓           ↓          ↓
 *    dismissed  dismissed   dismissed  dismissed
 *
 * - candidate:  初次检测到，等待确认（避免误报）
 * - confirmed:  连续 N 步检测到，确认为真实热点
 * - active:     已确认的热点，正在持续监测
 * - mitigating: 已触发处置联动（喷雾/雾炮），监测浓度下降
 * - resolved:   浓度回落到阈值以下，事件关闭
 * - dismissed:  被操作员手动忽略或自动过期
 *
 * 每个状态变更都会记录到事件日志中，形成完整的证据链。
 */
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"

namespace lidar_client {

/**
 * @brief 告警状态。
 */
enum class AlarmState {
    candidate,   ///< 初次检测到，待确认
    confirmed,   ///< 连续检测确认
    active,      ///< 活跃热点
    mitigating,  ///< 处置中
    resolved,    ///< 已消退
    dismissed,   ///< 已忽略/过期
};

/**
 * @brief 状态变更记录（不可变日志条目）。
 */
struct StateChange {
    std::string timestamp;       ///< 发生时间
    AlarmState from_state;       ///< 原状态
    AlarmState to_state;         ///< 新状态
    std::string reason;          ///< 变更原因
    double peak_pm25_ugm3;       ///< 当时的峰值 PM2.5
};

/**
 * @brief 告警状态机配置。
 */
struct AlarmConfig {
    int confirm_consecutive_steps = 2;   ///< 连续 N 步检测到才确认（candidate → confirmed）
    int resolve_consecutive_steps = 3;   ///< 连续 N 步未检测到才关闭（active → resolved）
    int auto_dismiss_after_steps = 20;   ///< candidate 状态超过 N 步未确认则自动忽略
    double severity_threshold_ugm3 = 50.0; ///< 严重告警阈值
};

/**
 * @brief 单个热点告警事件的完整生命周期管理。
 *
 * 每个 HotspotEvent 对应一个物理热点区域，跨多个时间步持续跟踪。
 */
class HotspotEvent {
public:
    /**
     * @brief 构造一个新事件。
     * @param event_id 唯一事件 ID
     * @param hotspot 初始热点检测结果
     */
    HotspotEvent(const std::string& event_id, const lidar_core::Hotspot& hotspot);

    /**
     * @brief 获取当前状态。
     */
    AlarmState state() const { return state_; }

    /**
     * @brief 获取事件 ID。
     */
    const std::string& event_id() const { return event_id_; }

    /**
     * @brief 获取热点 ID（最后一次检测到的）。
     */
    const std::string& hotspot_id() const { return last_hotspot_id_; }

    /**
     * @brief 获取首次检测时间。
     */
    const std::string& first_seen() const { return first_seen_; }

    /**
     * @brief 获取最后检测时间。
     */
    const std::string& last_seen() const { return last_seen_; }

    /**
     * @brief 获取历史峰值 PM2.5。
     */
    double peak_pm25_ugm3() const { return peak_pm25_; }

    /**
     * @brief 获取最新 PM2.5。
     */
    double current_pm25_ugm3() const { return current_pm25_; }

    /**
     * @brief 获取质心 ENU 坐标（最后一次检测）。
     */
    const std::vector<double>& centroid_enu_m() const { return centroid_; }

    /**
     * @brief 获取状态变更历史。
     */
    const std::vector<StateChange>& history() const { return history_; }

    /**
     * @brief 获取连续检测计数。
     */
    int consecutive_detections() const { return consecutive_detections_; }

    /**
     * @brief 获取连续未检测计数。
     */
    int consecutive_misses() const { return consecutive_misses_; }

    /**
     * @brief 更新事件（检测到热点）。
     * @param hotspot 本步骤的热点检测结果
     * @param config 状态机配置
     * @return 状态是否发生了变更
     */
    bool update_detected(const lidar_core::Hotspot& hotspot, const AlarmConfig& config);

    /**
     * @brief 更新事件（本步骤未检测到）。
     * @param timestamp 当前时间戳
     * @param config 状态机配置
     * @return 状态是否发生了变更
     */
    bool update_missed(const std::string& timestamp, const AlarmConfig& config);

    /**
     * @brief 手动确认/忽略事件。
     */
    void confirm();
    void dismiss(const std::string& reason = "manual");

    /**
     * @brief 标记为正在处置。
     */
    void mark_mitigating();

    /**
     * @brief 判断事件是否处于终态（resolved/dismissed）。
     */
    bool is_terminal() const;

    /**
     * @brief 序列化为 JSON。
     */
    lidar_core::Json to_json() const;

private:
    std::string event_id_;
    std::string last_hotspot_id_;
    std::string first_seen_;
    std::string last_seen_;
    AlarmState state_ = AlarmState::candidate;

    double peak_pm25_ = 0.0;
    double current_pm25_ = 0.0;
    std::vector<double> centroid_;

    int consecutive_detections_ = 0;
    int consecutive_misses_ = 0;
    int steps_since_creation_ = 0;

    std::vector<StateChange> history_;

    void transition_to(AlarmState new_state, const std::string& timestamp,
                       const std::string& reason, double pm25);
};

/**
 * @brief 状态枚举转字符串。
 */
std::string alarm_state_to_string(AlarmState state);

} // namespace lidar_client
