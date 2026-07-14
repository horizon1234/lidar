#include "lidar_client/ScanCycleMonitor.hpp"

namespace lidar_client {

namespace {

std::string get_string(const lidar_core::Json& json, const char* key) {
    return json.contains(key) && json.at(key).is_string()
        ? json.at(key).string_value()
        : "";
}

int get_int(const lidar_core::Json& json, const char* key, int fallback = 0) {
    return json.contains(key) && json.at(key).is_number()
        ? json.at(key).int_value()
        : fallback;
}

} // namespace

void ScanCycleMonitor::observe_frame(const lidar_protocol::Frame& frame) {
    if (frame.type != lidar_protocol::FrameType::lidar_raw || !frame.payload.is_object()) {
        return;
    }

    std::string cycle_id = get_string(frame.payload, "scan_cycle_id");
    if (cycle_id.empty()) {
        return;
    }

    CycleState& state = cycles_by_id_[cycle_id];
    if (state.scan_cycle_id.empty()) {
        state.scan_cycle_id = cycle_id;
        state.timestamp = frame.timestamp;
    }

    int expected = get_int(frame.payload, "rays_in_cycle");
    if (expected > 0) {
        state.expected_rays = expected;
    }

    int ray_index = get_int(frame.payload, "ray_index", -1);
    if (ray_index >= 0 && !state.received_indices.insert(ray_index).second) {
        ++state.duplicate_rays;
    }
}

ScanCycleSummary ScanCycleMonitor::take_summary_for_timestamp(const std::string& timestamp) {
    ScanCycleSummary summary;
    for (auto iterator = cycles_by_id_.begin(); iterator != cycles_by_id_.end(); ++iterator) {
        if (iterator->second.timestamp != timestamp) {
            continue;
        }
        const CycleState& state = iterator->second;

        summary.scan_cycle_id = iterator->first;
        summary.timestamp = state.timestamp;
        summary.expected_rays = state.expected_rays;
        summary.received_rays = static_cast<int>(state.received_indices.size());
        summary.duplicate_rays = state.duplicate_rays;
        for (int index = 0; index < state.expected_rays; ++index) {
            if (state.received_indices.find(index) == state.received_indices.end()) {
                summary.missing_ray_indices.push_back(index);
            }
        }
        summary.complete = summary.expected_rays > 0
            && summary.received_rays == summary.expected_rays
            && summary.duplicate_rays == 0
            && summary.missing_ray_indices.empty();
        cycles_by_id_.erase(iterator);
        return summary;
    }
    return summary;
}

void ScanCycleMonitor::reset() {
    cycles_by_id_.clear();
}

} // namespace lidar_client
