/**
 * @file hotspot_tracker.cpp
 * @brief 热点事件追踪器实现。
 */
#include "lidar_client/HotspotTracker.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace lidar_client {

// =========================================================================
// 辅助函数
// =========================================================================

double HotspotTracker::enu_distance(const std::vector<double>& a,
                                     const std::vector<double>& b) {
    if (a.empty() || b.empty()) return 1e18;
    double dx = 0, dy = 0, dz = 0;
    if (a.size() >= 1 && b.size() >= 1) dx = a[0] - b[0];
    if (a.size() >= 2 && b.size() >= 2) dy = a[1] - b[1];
    if (a.size() >= 3 && b.size() >= 3) dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string HotspotTracker::generate_event_id() {
    std::ostringstream oss;
    oss << "EVT-" << std::setfill('0') << std::setw(4) << next_event_id_++;
    return oss.str();
}

// =========================================================================
// HotspotTracker
// =========================================================================

HotspotTracker::HotspotTracker(const TrackerConfig& config)
    : config_(config)
{
}

StepSummary HotspotTracker::process_step(
    const std::string& timestamp,
    const std::vector<lidar_core::Hotspot>& hotspots)
{
    ++total_steps_;
    StepSummary summary;
    summary.timestamp = timestamp;

    // 收集活跃事件
    std::vector<EventPtr> active;
    for (auto& evt : events_) {
        if (!evt->is_terminal()) {
            active.push_back(evt);
        }
    }

    // 如果禁用了追踪，每一步都创建新事件
    if (!config_.enable_tracking) {
        for (const auto& hs : hotspots) {
            auto evt = std::make_shared<HotspotEvent>(generate_event_id(), hs);
            events_.push_back(evt);
            summary.new_events++;
            summary.max_pm25_ugm3 = std::max(summary.max_pm25_ugm3,
                                              hs.peak_pm25_ugm3);
        }
    } else {
        // 追踪模式：用贪心最近邻匹配
        std::vector<bool> matched_event(active.size(), false);
        std::vector<bool> matched_hotspot(hotspots.size(), false);

        // 对每个新热点，找最近的未匹配活跃事件
        for (size_t hi = 0; hi < hotspots.size(); ++hi) {
            double best_dist = config_.match_distance_m;
            int best_evt = -1;
            for (size_t ei = 0; ei < active.size(); ++ei) {
                if (matched_event[ei]) continue;
                if (active[ei]->is_terminal()) continue;
                double d = enu_distance(active[ei]->centroid_enu_m(),
                                        hotspots[hi].centroid_enu_m);
                if (d < best_dist) {
                    best_dist = d;
                    best_evt = static_cast<int>(ei);
                }
            }

            if (best_evt >= 0) {
                // 匹配成功
                bool changed = active[best_evt]->update_detected(
                    hotspots[hi], config_.alarm);
                matched_event[best_evt] = true;
                matched_hotspot[hi] = true;
                if (changed) summary.state_changed_events++;
                summary.max_pm25_ugm3 = std::max(
                    summary.max_pm25_ugm3, hotspots[hi].peak_pm25_ugm3);
            }
        }

        // 未匹配的热点 → 新建事件
        for (size_t hi = 0; hi < hotspots.size(); ++hi) {
            if (matched_hotspot[hi]) continue;
            auto evt = std::make_shared<HotspotEvent>(generate_event_id(),
                                                       hotspots[hi]);
            events_.push_back(evt);
            summary.new_events++;
            summary.max_pm25_ugm3 = std::max(summary.max_pm25_ugm3,
                                              hotspots[hi].peak_pm25_ugm3);
        }

        // 未匹配的活跃事件 → missed
        for (size_t ei = 0; ei < active.size(); ++ei) {
            if (matched_event[ei]) continue;
            if (active[ei]->is_terminal()) continue;
            bool changed = active[ei]->update_missed(timestamp, config_.alarm);
            if (changed) {
                summary.state_changed_events++;
                if (active[ei]->is_terminal()) {
                    summary.resolved_events++;
                }
            }
        }
    }

    // 统计
    summary.total_events = static_cast<int>(events_.size());
    for (const auto& evt : events_) {
        if (!evt->is_terminal()) {
            summary.active_events++;
        }
    }

    step_history_.push_back(std::move(summary));
    return step_history_.back();
}

std::vector<HotspotTracker::EventPtr> HotspotTracker::active_events() const {
    std::vector<EventPtr> result;
    for (const auto& evt : events_) {
        if (!evt->is_terminal()) {
            result.push_back(evt);
        }
    }
    return result;
}

HotspotTracker::EventPtr HotspotTracker::find_event(
    const std::string& event_id) const {
    for (const auto& evt : events_) {
        if (evt->event_id() == event_id) {
            return evt;
        }
    }
    return nullptr;
}

bool HotspotTracker::confirm_event(const std::string& event_id) {
    auto evt = find_event(event_id);
    if (!evt || evt->is_terminal()) return false;
    evt->confirm();
    return true;
}

bool HotspotTracker::dismiss_event(const std::string& event_id,
                                    const std::string& reason) {
    auto evt = find_event(event_id);
    if (!evt) return false;
    evt->dismiss(reason);
    return true;
}

bool HotspotTracker::mark_event_mitigating(const std::string& event_id) {
    auto evt = find_event(event_id);
    if (!evt || evt->is_terminal()) return false;
    evt->mark_mitigating();
    return true;
}

lidar_core::Json HotspotTracker::events_to_json() const {
    using Json = lidar_core::Json;
    Json::Array arr;
    arr.reserve(events_.size());
    for (const auto& evt : events_) {
        arr.emplace_back(evt->to_json());
    }
    return Json(std::move(arr));
}

lidar_core::Json HotspotTracker::step_history_to_json() const {
    using Json = lidar_core::Json;
    Json::Array arr;
    arr.reserve(step_history_.size());
    for (const auto& s : step_history_) {
        Json obj{Json::Object{}};
        obj["timestamp"] = Json(s.timestamp);
        obj["total_events"] = Json(static_cast<double>(s.total_events));
        obj["active_events"] = Json(static_cast<double>(s.active_events));
        obj["new_events"] = Json(static_cast<double>(s.new_events));
        obj["state_changed_events"] = Json(static_cast<double>(s.state_changed_events));
        obj["resolved_events"] = Json(static_cast<double>(s.resolved_events));
        obj["max_pm25_ugm3"] = Json(s.max_pm25_ugm3);
        arr.emplace_back(std::move(obj));
    }
    return Json(std::move(arr));
}

} // namespace lidar_client
