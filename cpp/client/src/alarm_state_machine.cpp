/**
 * @file alarm_state_machine.cpp
 * @brief 告警状态机实现。
 */
#include "lidar_client/alarm_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace lidar_client {

// =========================================================================
// 辅助函数
// =========================================================================

std::string alarm_state_to_string(AlarmState state) {
    switch (state) {
        case AlarmState::candidate:  return "candidate";
        case AlarmState::confirmed:  return "confirmed";
        case AlarmState::active:     return "active";
        case AlarmState::mitigating: return "mitigating";
        case AlarmState::resolved:   return "resolved";
        case AlarmState::dismissed:  return "dismissed";
    }
    return "unknown";
}

// =========================================================================
// HotspotEvent
// =========================================================================

HotspotEvent::HotspotEvent(const std::string& event_id,
                           const lidar_core::Hotspot& hotspot)
    : event_id_(event_id)
    , last_hotspot_id_(hotspot.hotspot_id)
    , first_seen_(hotspot.timestamp)
    , last_seen_(hotspot.timestamp)
    , state_(AlarmState::candidate)
    , peak_pm25_(hotspot.peak_pm25_ugm3)
    , current_pm25_(hotspot.peak_pm25_ugm3)
    , centroid_(hotspot.centroid_enu_m)
    , consecutive_detections_(1)
    , consecutive_misses_(0)
    , steps_since_creation_(0)
{
    history_.push_back(StateChange{
        hotspot.timestamp,
        AlarmState::candidate, // from (conceptually "none")
        AlarmState::candidate, // to
        "initial detection",
        hotspot.peak_pm25_ugm3
    });
}

void HotspotEvent::transition_to(AlarmState new_state,
                                  const std::string& timestamp,
                                  const std::string& reason,
                                  double pm25) {
    if (new_state == state_) return;

    history_.push_back(StateChange{
        timestamp,
        state_,
        new_state,
        reason,
        pm25
    });
    state_ = new_state;
}

bool HotspotEvent::update_detected(const lidar_core::Hotspot& hotspot,
                                    const AlarmConfig& config) {
    ++steps_since_creation_;
    last_seen_ = hotspot.timestamp;
    last_hotspot_id_ = hotspot.hotspot_id;
    current_pm25_ = hotspot.peak_pm25_ugm3;
    peak_pm25_ = std::max(peak_pm25_, hotspot.peak_pm25_ugm3);
    if (!hotspot.centroid_enu_m.empty()) {
        centroid_ = hotspot.centroid_enu_m;
    }

    consecutive_detections_++;
    consecutive_misses_ = 0;

    bool changed = false;

    switch (state_) {
        case AlarmState::candidate:
            if (consecutive_detections_ >= config.confirm_consecutive_steps) {
                transition_to(AlarmState::confirmed, hotspot.timestamp,
                              "confirmed after " +
                                  std::to_string(consecutive_detections_) +
                                  " consecutive detections",
                              hotspot.peak_pm25_ugm3);
                // 如果确认后严重度已经达标，直接进入 active
                if (hotspot.peak_pm25_ugm3 >= config.severity_threshold_ugm3) {
                    transition_to(AlarmState::active, hotspot.timestamp,
                                  "severity threshold reached",
                                  hotspot.peak_pm25_ugm3);
                }
                changed = true;
            }
            break;

        case AlarmState::confirmed:
            if (hotspot.peak_pm25_ugm3 >= config.severity_threshold_ugm3) {
                transition_to(AlarmState::active, hotspot.timestamp,
                              "severity threshold reached",
                              hotspot.peak_pm25_ugm3);
                changed = true;
            }
            break;

        case AlarmState::active:
            // 仍然活跃，无需状态变更
            break;

        case AlarmState::mitigating:
            // 如果浓度重新超过阈值，回到 active
            if (hotspot.peak_pm25_ugm3 >= config.severity_threshold_ugm3) {
                transition_to(AlarmState::active, hotspot.timestamp,
                              "concentration rebounded",
                              hotspot.peak_pm25_ugm3);
                changed = true;
            }
            break;

        case AlarmState::resolved:
            // 已关闭的事件重新检测到 → 重新激活
            transition_to(AlarmState::active, hotspot.timestamp,
                          "re-detected after resolution",
                          hotspot.peak_pm25_ugm3);
            consecutive_detections_ = 1;
            changed = true;
            break;

        case AlarmState::dismissed:
            // 忽略的事件不自动恢复
            break;
    }

    return changed;
}

bool HotspotEvent::update_missed(const std::string& timestamp,
                                  const AlarmConfig& config) {
    ++steps_since_creation_;
    consecutive_detections_ = 0;
    consecutive_misses_++;

    bool changed = false;

    switch (state_) {
        case AlarmState::candidate:
            // candidate 长时间未确认 → 自动忽略
            if (steps_since_creation_ >= config.auto_dismiss_after_steps) {
                transition_to(AlarmState::dismissed, timestamp,
                              "auto-dismissed: not confirmed within " +
                                  std::to_string(config.auto_dismiss_after_steps) +
                                  " steps",
                              current_pm25_);
                changed = true;
            }
            break;

        case AlarmState::confirmed:
        case AlarmState::active:
        case AlarmState::mitigating:
            // 连续 N 步未检测到 → 已解决
            if (consecutive_misses_ >= config.resolve_consecutive_steps) {
                transition_to(AlarmState::resolved, timestamp,
                              "concentration below detection for " +
                                  std::to_string(consecutive_misses_) +
                                  " consecutive steps",
                              current_pm25_);
                changed = true;
            }
            break;

        case AlarmState::resolved:
        case AlarmState::dismissed:
            // 终态，不做处理
            break;
    }

    return changed;
}

void HotspotEvent::confirm() {
    if (state_ == AlarmState::candidate) {
        transition_to(AlarmState::confirmed, last_seen_,
                      "manually confirmed", current_pm25_);
    }
}

void HotspotEvent::dismiss(const std::string& reason) {
    if (!is_terminal()) {
        transition_to(AlarmState::dismissed, last_seen_, reason, current_pm25_);
    }
}

void HotspotEvent::mark_mitigating() {
    if (state_ == AlarmState::active || state_ == AlarmState::confirmed) {
        transition_to(AlarmState::mitigating, last_seen_,
                      "disposal action triggered", current_pm25_);
    }
}

bool HotspotEvent::is_terminal() const {
    return state_ == AlarmState::resolved || state_ == AlarmState::dismissed;
}

lidar_core::Json HotspotEvent::to_json() const {
    using Json = lidar_core::Json;

    Json obj{Json::object_type{}};
    obj["event_id"] = Json(event_id_);
    obj["hotspot_id"] = Json(last_hotspot_id_);
    obj["state"] = Json(alarm_state_to_string(state_));
    obj["first_seen"] = Json(first_seen_);
    obj["last_seen"] = Json(last_seen_);
    obj["peak_pm25_ugm3"] = Json(peak_pm25_);
    obj["current_pm25_ugm3"] = Json(current_pm25_);
    obj["consecutive_detections"] = Json(static_cast<double>(consecutive_detections_));
    obj["consecutive_misses"] = Json(static_cast<double>(consecutive_misses_));

    if (!centroid_.empty()) {
        Json::array_type arr;
        arr.reserve(centroid_.size());
        for (double v : centroid_) {
            arr.emplace_back(v);
        }
        obj["centroid_enu_m"] = Json(std::move(arr));
    }

    // 状态变更历史
    Json::array_type hist_arr;
    hist_arr.reserve(history_.size());
    for (const auto& h : history_) {
        Json h_obj{Json::object_type{}};
        h_obj["timestamp"] = Json(h.timestamp);
        h_obj["from"] = Json(alarm_state_to_string(h.from_state));
        h_obj["to"] = Json(alarm_state_to_string(h.to_state));
        h_obj["reason"] = Json(h.reason);
        h_obj["peak_pm25_ugm3"] = Json(h.peak_pm25_ugm3);
        hist_arr.emplace_back(std::move(h_obj));
    }
    obj["history"] = Json(std::move(hist_arr));

    return obj;
}

} // namespace lidar_client
