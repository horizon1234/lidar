#pragma once

#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"
#include "lidar_protocol/Frame.hpp"

namespace lidar_client {

struct DeviceStatusSnapshot {
    std::string site_id;
    std::string site_name;
    std::string device_model;
    std::string vendor_profile;
    std::string protocol_version;
    std::vector<double> ppi_elevations_deg;
    double ppi_azimuth_start_deg = 0.0;
    double ppi_azimuth_stop_deg = 360.0;
    double ppi_azimuth_step_deg = 0.0;
    double ppi_line_dwell_s = 0.0;
    double ppi_scan_cycle_s = 0.0;
    double playback_time_scale = 1.0;
    double pulse_repetition_hz = 0.0;
    int integrated_pulses_per_line = 0;
    int total_steps = -1;
    bool valid = false;
};

class DeviceStatusModel {
public:
    bool update_from_frame(const lidar_protocol::Frame& frame);

    const DeviceStatusSnapshot& snapshot() const { return snapshot_; }

    lidar_core::Json to_json() const;

    std::string brief() const;

private:
    DeviceStatusSnapshot snapshot_;
};

} // namespace lidar_client
