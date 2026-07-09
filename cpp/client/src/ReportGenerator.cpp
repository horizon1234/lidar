/**
 * @file report_generator.cpp
 * @brief 报表生成器实现。
 */
#include "lidar_client/ReportGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <sstream>
#include <iomanip>

namespace lidar_client {

namespace {

bool parse_report_timestamp(const std::string& text, std::time_t& output) {
    std::tm stamp{};
    std::istringstream input(text);
    input >> std::get_time(&stamp, "%Y-%m-%dT%H:%M");
    if (input.fail()) {
        return false;
    }
    stamp.tm_isdst = -1;
    output = std::mktime(&stamp);
    return output != static_cast<std::time_t>(-1);
}

double estimate_duration_minutes_from_timestamps(
    const std::vector<StepResultSummary>& steps) {
    if (steps.size() < 2) {
        return 0.0;
    }

    std::time_t first{};
    std::time_t last{};
    if (!parse_report_timestamp(steps.front().timestamp, first)
        || !parse_report_timestamp(steps.back().timestamp, last)) {
        return 0.0;
    }

    double span_minutes = std::difftime(last, first) / 60.0;
    if (span_minutes < 0.0) {
        return 0.0;
    }

    // 首尾时间戳描述的是各扫描周期的开始时刻；补上最后一个周期的平均跨度。
    double average_step_minutes = span_minutes / static_cast<double>(steps.size() - 1);
    return span_minutes + average_step_minutes;
}

} // namespace

// =========================================================================
// ReportGenerator
// =========================================================================

ReportGenerator::ReportGenerator(const ReportConfig& config)
    : config_(config)
{
}

void ReportGenerator::add_step_result(const StepResultSummary& result) {
    step_results_.push_back(result);
}

ReportGenerator::AggregateStats ReportGenerator::compute_stats(
    const HotspotTracker& tracker) const {

    AggregateStats stats;
    stats.total_steps = static_cast<int>(step_results_.size());

    double pm25_sum = 0.0;
    int pm25_count = 0;
    for (const auto& step : step_results_) {
        stats.total_hotspots_detected += step.hotspot_count;
        stats.max_pm25_all_steps = std::max(stats.max_pm25_all_steps,
                                             step.max_pm25_ugm3);
        if (step.avg_pm25_ugm3 > 0) {
            pm25_sum += step.avg_pm25_ugm3;
            ++pm25_count;
        }
    }
    if (pm25_count > 0) {
        stats.avg_pm25_all_steps = pm25_sum / pm25_count;
    }

    // 时间跨度估算：根据设备帧时间戳推导，避免客户端硬编码扫描周期。
    stats.total_duration_minutes = estimate_duration_minutes_from_timestamps(step_results_);

    // 事件统计
    stats.total_events = static_cast<int>(tracker.events().size());
    for (const auto& evt : tracker.events()) {
        if (!evt->is_terminal()) {
            ++stats.active_events;
        } else if (evt->state() == AlarmState::resolved) {
            ++stats.resolved_events;
        }
    }

    return stats;
}

lidar_core::Json ReportGenerator::generate_json_report(
    const HotspotTracker& tracker,
    const DisposalLinkage& linkage) const {

    using Json = lidar_core::Json;
    Json report{Json::Object{}};

    // 报表头
    Json header{Json::Object{}};
    header["site_id"] = Json(config_.site_id);
    header["site_name"] = Json(config_.site_name);
    header["report_type"] = Json("periodic_summary");
    {
        // 当前时间作为生成时间
        std::time_t now = std::time(nullptr);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ",
                      std::gmtime(&now));
        header["generated_at"] = Json(std::string(buf));
    }
    if (!step_results_.empty()) {
        header["period_start"] = Json(step_results_.front().timestamp);
        header["period_end"] = Json(step_results_.back().timestamp);
    }
    report["header"] = std::move(header);

    // 统计摘要
    auto stats = compute_stats(tracker);
    {
        Json summary{Json::Object{}};
        summary["total_steps"] = Json(static_cast<double>(stats.total_steps));
        summary["total_hotspots_detected"] = Json(static_cast<double>(stats.total_hotspots_detected));
        summary["total_events"] = Json(static_cast<double>(stats.total_events));
        summary["active_events"] = Json(static_cast<double>(stats.active_events));
        summary["resolved_events"] = Json(static_cast<double>(stats.resolved_events));
        summary["max_pm25_ugm3"] = Json(stats.max_pm25_all_steps);
        summary["avg_pm25_ugm3"] = Json(stats.avg_pm25_all_steps);
        summary["estimated_duration_minutes"] = Json(stats.total_duration_minutes);
        report["summary"] = std::move(summary);
    }

    // 步骤明细
    {
        Json::Array steps_arr;
        steps_arr.reserve(step_results_.size());
        const auto& tracker_history = tracker.step_history();
        for (size_t i = 0; i < step_results_.size(); ++i) {
            Json s{Json::Object{}};
            const auto& sr = step_results_[i];
            s["timestamp"] = Json(sr.timestamp);
            s["raw_count"] = Json(static_cast<double>(sr.raw_count));
            s["expected_raw_count"] = Json(static_cast<double>(sr.expected_raw_count));
            s["missing_raw_count"] = Json(static_cast<double>(sr.missing_raw_count));
            s["duplicate_raw_count"] = Json(static_cast<double>(sr.duplicate_raw_count));
            s["processed_count"] = Json(static_cast<double>(sr.processed_count));
            s["hotspot_count"] = Json(static_cast<double>(sr.hotspot_count));
            s["max_pm25_ugm3"] = Json(sr.max_pm25_ugm3);
            s["avg_pm25_ugm3"] = Json(sr.avg_pm25_ugm3);

            // 追踪器摘要（如果索引匹配）
            if (i < tracker_history.size()) {
                const auto& th = tracker_history[i];
                s["active_events"] = Json(static_cast<double>(th.active_events));
                s["new_events"] = Json(static_cast<double>(th.new_events));
            }
            steps_arr.emplace_back(std::move(s));
        }
        report["steps"] = Json(std::move(steps_arr));
    }

    // 事件明细
    if (config_.include_event_details) {
        report["events"] = tracker.events_to_json();
        report["step_history"] = tracker.step_history_to_json();
    }

    // 处置摘要
    if (config_.include_disposal_summary) {
        report["disposal"] = linkage.to_json();
    }

    return report;
}

std::string ReportGenerator::generate_text_report(
    const HotspotTracker& tracker,
    const DisposalLinkage& linkage) const {

    auto stats = compute_stats(tracker);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);

    oss << "==============================================================\n";
    oss << "         LiDAR PM Monitoring - Periodic Report\n";
    oss << "==============================================================\n";
    oss << "Site:         " << config_.site_name
        << " (" << config_.site_id << ")\n";

    if (!step_results_.empty()) {
        oss << "Period:       " << step_results_.front().timestamp
            << " to " << step_results_.back().timestamp << "\n";
    }
    oss << "--------------------------------------------------------------\n\n";

    // Summary
    oss << "SUMMARY\n";
    oss << "  Total time steps:         " << stats.total_steps << "\n";
    oss << "  Est. duration (minutes):  " << stats.total_duration_minutes << "\n";
    oss << "  Total hotspots detected:  " << stats.total_hotspots_detected << "\n";
    oss << "  Peak PM2.5 (ug/m3):       " << stats.max_pm25_all_steps << "\n";
    oss << "  Avg PM2.5 (ug/m3):        " << stats.avg_pm25_all_steps << "\n\n";

    // Events
    oss << "ALARM EVENTS\n";
    oss << "  Total events:             " << stats.total_events << "\n";
    oss << "  Active events:            " << stats.active_events << "\n";
    oss << "  Resolved events:          " << stats.resolved_events << "\n\n";

    // Active events detail
    auto active = tracker.active_events();
    if (!active.empty()) {
        oss << "  ACTIVE EVENT DETAILS:\n";
        for (const auto& evt : active) {
            oss << "    [" << alarm_state_to_string(evt->state()) << "] "
                << evt->event_id()
                << "  PM2.5=" << evt->current_pm25_ugm3()
                << "  peak=" << evt->peak_pm25_ugm3()
                << "  since " << evt->first_seen() << "\n";
            if (!evt->centroid_enu_m().empty()) {
                oss << "      centroid: ";
                for (size_t i = 0; i < evt->centroid_enu_m().size(); ++i) {
                    if (i > 0) oss << ", ";
                    oss << evt->centroid_enu_m()[i];
                }
                oss << " (ENU meters)\n";
            }
        }
        oss << "\n";
    }

    // Disposal
    if (config_.include_disposal_summary) {
        const auto& tasks = linkage.tasks();
        int active_tasks = 0;
        int completed_tasks = 0;
        for (const auto& t : tasks) {
            if (t.is_active) ++active_tasks;
            else ++completed_tasks;
        }

        oss << "DISPOSAL LINKAGE\n";
        oss << "  Total devices:            " << linkage.devices().size() << "\n";
        oss << "  Active devices:           ";
        {
            int active_devs = 0;
            for (const auto& dev : linkage.devices()) {
                if (dev.state == DeviceState::active) ++active_devs;
            }
            oss << active_devs << "\n";
        }
        oss << "  Total tasks:              " << tasks.size() << "\n";
        oss << "  Active tasks:             " << active_tasks << "\n";
        oss << "  Completed tasks:          " << completed_tasks << "\n\n";

        if (active_tasks > 0) {
            oss << "  ACTIVE TASKS:\n";
            for (const auto& t : tasks) {
                if (!t.is_active) continue;
                oss << "    " << t.task_id
                    << "  event=" << t.event_id
                    << "  device=" << t.device_id
                    << "  started=" << t.start_timestamp
                    << "  PM2.5_start=" << t.peak_pm25_at_start << "\n";
            }
            oss << "\n";
        }

        // Completed task effectiveness
        if (completed_tasks > 0) {
            oss << "  COMPLETED TASK EFFECTIVENESS:\n";
            for (const auto& t : tasks) {
                if (t.is_active) continue;
                if (t.peak_pm25_at_start <= 0) continue;
                double reduction = (t.peak_pm25_at_start - t.peak_pm25_at_end) /
                                   t.peak_pm25_at_start * 100.0;
                oss << "    " << t.task_id
                    << "  " << t.start_timestamp << " -> " << t.end_timestamp
                    << "  PM2.5: " << t.peak_pm25_at_start << " -> "
                    << t.peak_pm25_at_end
                    << "  reduction=" << reduction << "%\n";
            }
            oss << "\n";
        }
    }

    // Per-step timeline
    oss << "PER-STEP TIMELINE\n";
    const auto& tracker_history = tracker.step_history();
    oss << "  Step | Timestamp                 | Raw     | Miss | Dup | HS | ActEvt | NewEvt | MaxPM2.5\n";
    oss << "  -----|---------------------------|---------|------|-----|----|--------|--------|---------\n";
    for (size_t i = 0; i < step_results_.size(); ++i) {
        const auto& sr = step_results_[i];
        oss << "  " << std::setw(4) << (i + 1) << " | "
            << std::left << std::setw(25) << sr.timestamp << " | "
            << std::right << std::setw(3) << sr.raw_count << "/"
            << std::left << std::setw(3) << sr.expected_raw_count << " | "
            << std::right << std::setw(4) << sr.missing_raw_count << " | "
            << std::setw(3) << sr.duplicate_raw_count << " | "
            << std::right << std::setw(2) << sr.hotspot_count << " | ";
        if (i < tracker_history.size()) {
            oss << std::setw(6) << tracker_history[i].active_events << " | "
                << std::setw(6) << tracker_history[i].new_events << " | ";
        } else {
            oss << std::setw(6) << "-" << " | "
                << std::setw(6) << "-" << " | ";
        }
        oss << std::setw(7) << sr.max_pm25_ugm3 << "\n";
    }

    oss << "\n==============================================================\n";
    return oss.str();
}

lidar_core::Json ReportGenerator::generate_step_brief(
    const StepResultSummary& step,
    const StepSummary& tracker_summary) {

    using Json = lidar_core::Json;
    Json brief{Json::Object{}};
    brief["timestamp"] = Json(step.timestamp);
    brief["raw_count"] = Json(static_cast<double>(step.raw_count));
    brief["expected_raw_count"] = Json(static_cast<double>(step.expected_raw_count));
    brief["missing_raw_count"] = Json(static_cast<double>(step.missing_raw_count));
    brief["duplicate_raw_count"] = Json(static_cast<double>(step.duplicate_raw_count));
    brief["processed_count"] = Json(static_cast<double>(step.processed_count));
    brief["hotspot_count"] = Json(static_cast<double>(step.hotspot_count));
    brief["max_pm25_ugm3"] = Json(step.max_pm25_ugm3);
    brief["avg_pm25_ugm3"] = Json(step.avg_pm25_ugm3);
    brief["active_events"] = Json(static_cast<double>(tracker_summary.active_events));
    brief["new_events"] = Json(static_cast<double>(tracker_summary.new_events));
    brief["state_changed_events"] = Json(static_cast<double>(tracker_summary.state_changed_events));
    brief["resolved_events"] = Json(static_cast<double>(tracker_summary.resolved_events));
    return brief;
}

} // namespace lidar_client
