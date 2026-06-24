/**
 * @file hotspot_tracker.hpp
 * @brief 热点事件追踪器：跨时间步关联热点检测，管理告警事件。
 *
 * 工作原理：
 * 1. 每个时间步接收一组新检测到的热点
 * 2. 将新热点与现有活跃事件做空间匹配（ENU 距离 < 阈值）
 * 3. 匹配成功 → 更新事件；未匹配 → 创建新事件
 * 4. 未被本步骤任何新热点匹配到的活跃事件 → 标记 missed
 */
#pragma once

#include <string>
#include <vector>
#include <memory>

#include "lidar_core/lidar_core.hpp"
#include "lidar_client/alarm_state_machine.hpp"

namespace lidar_client {

/**
 * @brief 追踪器配置。
 */
struct TrackerConfig {
    AlarmConfig alarm;                    ///< 告警状态机配置
    double match_distance_m = 300.0;      ///< 空间匹配阈值（米）
    bool enable_tracking = true;          ///< 是否启用跨步骤追踪
};

/**
 * @brief 步骤处理结果摘要。
 */
struct StepSummary {
    std::string timestamp;
    int total_events = 0;            ///< 所有事件总数
    int active_events = 0;           ///< 活跃事件数
    int new_events = 0;              ///< 本步骤新建事件数
    int state_changed_events = 0;    ///< 本步骤发生状态变更的事件数
    int resolved_events = 0;         ///< 本步骤关闭的事件数
    double max_pm25_ugm3 = 0.0;      ///< 本步骤最大 PM2.5
};

/**
 * @brief 热点事件追踪器。
 *
 * 负责将每一步检测到的热点列表与历史事件关联，
 * 驱动告警状态机运转。
 */
class HotspotTracker {
public:
    using EventPtr = std::shared_ptr<HotspotEvent>;

    /**
     * @brief 构造追踪器。
     * @param config 配置
     */
    explicit HotspotTracker(const TrackerConfig& config = {});

    /**
     * @brief 处理一个时间步的热点检测结果。
     * @param timestamp 时间戳
     * @param hotspots 本步骤检测到的热点列表
     * @return 步骤摘要
     */
    StepSummary process_step(const std::string& timestamp,
                             const std::vector<lidar_core::Hotspot>& hotspots);

    /**
     * @brief 获取所有事件（包括已关闭的）。
     */
    const std::vector<EventPtr>& events() const { return events_; }

    /**
     * @brief 获取活跃事件（非终态）。
     */
    std::vector<EventPtr> active_events() const;

    /**
     * @brief 根据事件 ID 获取事件。
     */
    EventPtr find_event(const std::string& event_id) const;

    /**
     * @brief 确认事件。
     */
    bool confirm_event(const std::string& event_id);

    /**
     * @brief 忽略事件。
     */
    bool dismiss_event(const std::string& event_id, const std::string& reason = "manual");

    /**
     * @brief 标记事件为正在处置。
     */
    bool mark_event_mitigating(const std::string& event_id);

    /**
     * @brief 获取已处理的时间步数。
     */
    int total_steps() const { return total_steps_; }

    /**
     * @brief 获取处理历史摘要。
     */
    const std::vector<StepSummary>& step_history() const { return step_history_; }

    /**
     * @brief 序列化所有事件为 JSON。
     */
    lidar_core::Json events_to_json() const;

    /**
     * @brief 序列化步骤历史摘要为 JSON。
     */
    lidar_core::Json step_history_to_json() const;

private:
    TrackerConfig config_;
    std::vector<EventPtr> events_;
    std::vector<StepSummary> step_history_;
    int total_steps_ = 0;
    int next_event_id_ = 1;

    /**
     * @brief 计算两个 ENU 坐标之间的距离。
     */
    static double enu_distance(const std::vector<double>& a,
                               const std::vector<double>& b);

    /**
     * @brief 为新热点生成唯一事件 ID。
     */
    std::string generate_event_id();
};

} // namespace lidar_client
