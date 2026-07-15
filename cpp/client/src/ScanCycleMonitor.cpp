/**
 * @file ScanCycleMonitor.cpp
 * @brief 扫描周期原始射线接收完整性监控实现。
 */
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
    // 状态、心跳等帧不含射线索引，也不参与原始数据接收完整性统计。
    if (frame.type != lidar_protocol::FrameType::lidar_raw || !frame.payload.is_object()) {
        return;
    }

    std::string cycle_id = get_string(frame.payload, "scan_cycle_id");
    // 没有稳定周期 ID 就无法跨帧归组；旧协议帧保持兼容，但不生成完整性结论。
    if (cycle_id.empty()) {
        return;
    }

    CycleState& state = cycles_by_id_[cycle_id];
    if (state.scan_cycle_id.empty()) {
        // 首帧建立周期状态，并固定用于关联 StepResult 的时间戳。
        state.scan_cycle_id = cycle_id;
        state.timestamp = frame.timestamp;
    }

    int expected = get_int(frame.payload, "rays_in_cycle");
    if (expected > 0) {
        state.expected_rays = expected;
    }

    int ray_index = get_int(frame.payload, "ray_index", -1);
    // set 同时完成去重和索引覆盖记录；乱序到达不应被误判成缺帧。
    if (ray_index >= 0 && !state.received_indices.insert(ray_index).second) {
        ++state.duplicate_rays;
    }
}

ScanCycleSummary ScanCycleMonitor::take_summary_for_timestamp(const std::string& timestamp) {
    ScanCycleSummary summary;
    // FrameProcessor 的周期结果当前只暴露时间戳，因此这里用时间戳关联传输侧周期状态。
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
        // 协议约定合法索引范围为 [0, rays_in_cycle)，逐项构造可诊断的缺失列表。
        for (int index = 0; index < state.expected_rays; ++index) {
            if (state.received_indices.find(index) == state.received_indices.end()) {
                summary.missing_ray_indices.push_back(index);
            }
        }
        summary.complete = summary.expected_rays > 0
            && summary.received_rays == summary.expected_rays
            && summary.duplicate_rays == 0
            && summary.missing_ray_indices.empty();
        // 摘要是一次性消费结果，避免长时间运行时保留已封口周期的大量索引。
        cycles_by_id_.erase(iterator);
        return summary;
    }
    return summary;
}

void ScanCycleMonitor::reset() {
    cycles_by_id_.clear();
}

} // namespace lidar_client
