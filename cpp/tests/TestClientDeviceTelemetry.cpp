#include "lidar_client/DeviceStatusModel.hpp"
#include "lidar_client/ScanCycleMonitor.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

lidar_protocol::Frame raw_frame(int ray_index) {
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::lidar_raw,
        "2026-05-30T08:00",
        lidar_core::Json::Object{
            {"scan_cycle_id", "cycle-001"},
            {"ray_index", ray_index},
            {"rays_in_cycle", 3},
        });
}

} // namespace

int main() {
    try {
        lidar_client::DeviceStatusModel status;
        auto status_frame = lidar_protocol::make_frame(
            lidar_protocol::FrameType::status,
            "2026-05-30T08:00",
            lidar_core::Json::Object{
                {"site_id", "site-001"},
                {"device_model", "SIM-FIELD-PM-LIDAR"},
                {"vendor_profile", "raymetrics_pmeye_like"},
                {"ppi_azimuth_step_deg", 2.5},
                {"ppi_line_dwell_s", 5.0},
                {"ppi_scan_cycle_s", 365.0},
                {"pulse_repetition_hz", 20.0},
                {"integrated_pulses_per_line", 100},
                {"playback_time_scale", 100.0},
            });

        require(status.update_from_frame(status_frame), "Status frame should update model");
        require(status.snapshot().valid, "Status snapshot should be valid");
        require(status.snapshot().integrated_pulses_per_line == 100,
                "Integrated pulses should parse");
        require(status.snapshot().ppi_line_dwell_s == 5.0,
                "Line dwell should parse");

        auto telemetry_frame = lidar_protocol::make_frame(
            lidar_protocol::FrameType::status,
            "2026-05-30T08:00",
            lidar_core::Json::Object{
                {"site_id", "site-001"},
                {"window_transmission", 0.92},
            });
        require(status.update_from_frame(telemetry_frame),
                "Telemetry status should update model");
        require(status.snapshot().device_model == "SIM-FIELD-PM-LIDAR",
                "Telemetry status must not erase device capabilities");
        require(status.snapshot().pulse_repetition_hz == 20.0,
                "Telemetry status must preserve PRF");

        lidar_client::ScanCycleMonitor monitor;
        monitor.observe_frame(raw_frame(0));
        monitor.observe_frame(raw_frame(2));
        monitor.observe_frame(raw_frame(2));
        auto summary = monitor.summary_for_timestamp("2026-05-30T08:00");
        require(summary.expected_rays == 3, "Expected rays should parse");
        require(summary.received_rays == 2, "Unique received rays should count");
        require(summary.duplicate_rays == 1, "Duplicate rays should count");
        require(summary.missing_ray_indices.size() == 1
                    && summary.missing_ray_indices.front() == 1,
                "Missing ray index should be reported");
        require(!summary.complete, "Incomplete cycle should not pass");

        std::cout << "Client device telemetry assertions passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
