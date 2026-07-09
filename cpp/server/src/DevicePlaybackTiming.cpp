#include "lidar_server/DevicePlaybackTiming.hpp"

#include <algorithm>
#include <cmath>

namespace lidar_server {

namespace {

bool payload_scan_mode_is(const lidar_protocol::Frame& frame, const char* mode) {
    return frame.payload.is_object()
        && frame.payload.contains("scan_mode")
        && frame.payload.at("scan_mode").is_string()
        && frame.payload.at("scan_mode").string_value() == mode;
}

} // namespace

double acquisition_seconds_for_frame(
    const SimDeviceConfig& config,
    const lidar_protocol::Frame& frame) {
    if (frame.type != lidar_protocol::FrameType::lidar_raw) {
        return 0.0;
    }

    if (payload_scan_mode_is(frame, "stare")) {
        return std::max(config.stare_dwell_s, 0.0);
    }

    return std::max(config.ppi_line_dwell_s, 0.0)
        + std::max(config.ppi_step_overhead_s, 0.0);
}

int playback_delay_ms_for_frame(
    const SimDeviceConfig& config,
    const lidar_protocol::Frame& frame) {
    if (config.inter_frame_delay_ms > 0) {
        return config.inter_frame_delay_ms;
    }

    double acquisition_s = acquisition_seconds_for_frame(config, frame);
    if (acquisition_s <= 0.0 || config.playback_time_scale <= 0.0) {
        return 0;
    }

    return std::max(1, static_cast<int>(std::round(
        acquisition_s * 1000.0 / config.playback_time_scale)));
}

} // namespace lidar_server
