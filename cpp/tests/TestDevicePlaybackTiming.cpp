#include "lidar_server/DevicePlaybackTiming.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

lidar_protocol::Frame raw_frame_with_scan_mode(const std::string& mode) {
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::lidar_raw,
        "2026-05-30T08:00",
        lidar_core::Json::Object{{"scan_mode", mode}});
}

} // namespace

int main() {
    try {
        lidar_server::SimDeviceConfig config;
        config.ppi_line_dwell_s = 5.0;
        config.ppi_step_overhead_s = 0.25;
        config.playback_time_scale = 100.0;
        config.inter_frame_delay_ms = 0;

        require(lidar_server::acquisition_seconds_for_frame(
                    config, raw_frame_with_scan_mode("ppi")) == 5.25,
                "PPI acquisition seconds should include dwell and movement overhead");
        require(lidar_server::playback_delay_ms_for_frame(
                    config, raw_frame_with_scan_mode("ppi")) == 53,
                "PPI playback delay should include movement overhead and playback scale");
        require(lidar_server::playback_delay_ms_for_frame(
                    config, raw_frame_with_scan_mode("stare")) == 300,
                "Stare playback delay should use 30 seconds acquisition");

        auto status = lidar_protocol::make_frame(
            lidar_protocol::FrameType::status,
            "2026-05-30T08:00",
            lidar_core::Json::Object{});
        require(lidar_server::playback_delay_ms_for_frame(config, status) == 0,
                "Non-raw frames should not add acquisition delay");

        config.inter_frame_delay_ms = 17;
        require(lidar_server::playback_delay_ms_for_frame(
                    config, raw_frame_with_scan_mode("ppi")) == 17,
                "Fixed inter-frame override should win");

        std::cout << "Device playback timing assertions passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
