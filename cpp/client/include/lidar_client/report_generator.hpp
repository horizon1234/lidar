/**
 * @file report_generator.hpp
 * @brief 报表生成器：从累积的处理结果、告警事件和处置任务生成汇总报表。
 *
 * 支持生成：
 * - 周期性摘要报表（每个时间步或累积）
 * - 事件明细报表
 * - 处置效果报表
 * - 导出为 JSON 或文本格式
 */
#pragma once

#include <string>
#include <vector>

#include "lidar_core/lidar_core.hpp"
#include "lidar_client/hotspot_tracker.hpp"
#include "lidar_client/disposal_linkage.hpp"

namespace lidar_client {

/**
 * @brief 单步处理结果摘要（用于报表输入）。
 */
struct StepResultSummary {
    std::string timestamp;
    int raw_count = 0;          ///< 原始剖面数
    int processed_count = 0;    ///< 处理后剖面数
    int hotspot_count = 0;      ///< 检测到的热点数
    double max_pm25_ugm3 = 0.0; ///< 最大 PM2.5
    double avg_pm25_ugm3 = 0.0; ///< 平均 PM2.5
};

/**
 * @brief 报表配置。
 */
struct ReportConfig {
    std::string site_id = "demo_site";  ///< 站点 ID
    std::string site_name = "Demo Site"; ///< 站点名称
    bool include_event_details = true;   ///< 是否包含事件明细
    bool include_disposal_summary = true; ///< 是否包含处置摘要
};

/**
 * @brief 报表生成器。
 */
class ReportGenerator {
public:
    /**
     * @brief 构造报表生成器。
     * @param config 配置
     */
    explicit ReportGenerator(const ReportConfig& config = {});

    /**
     * @brief 添加一个步骤的结果摘要。
     */
    void add_step_result(const StepResultSummary& result);

    /**
     * @brief 生成完整的 JSON 报表。
     * @param tracker 热点追踪器（提供事件数据）
     * @param linkage 处置联动（提供任务数据）
     * @return JSON 报表
     */
    lidar_core::Json generate_json_report(const HotspotTracker& tracker,
                                          const DisposalLinkage& linkage) const;

    /**
     * @brief 生成文本格式的摘要报表。
     * @param tracker 热点追踪器
     * @param linkage 处置联动
     * @return 文本报表
     */
    std::string generate_text_report(const HotspotTracker& tracker,
                                     const DisposalLinkage& linkage) const;

    /**
     * @brief 获取已收集的步骤结果。
     */
    const std::vector<StepResultSummary>& step_results() const {
        return step_results_;
    }

    /**
     * @brief 生成单个时间步的简报 JSON。
     */
    static lidar_core::Json generate_step_brief(
        const StepResultSummary& step,
        const StepSummary& tracker_summary);

private:
    ReportConfig config_;
    std::vector<StepResultSummary> step_results_;

    /**
     * @brief 计算统计摘要。
     */
    struct AggregateStats {
        int total_steps = 0;
        int total_hotspots_detected = 0;
        int total_events = 0;
        int active_events = 0;
        int resolved_events = 0;
        double max_pm25_all_steps = 0.0;
        double avg_pm25_all_steps = 0.0;
        double total_duration_minutes = 0.0;
    };

    AggregateStats compute_stats(const HotspotTracker& tracker) const;
};

} // namespace lidar_client
