#include "lidar_client/DeviceStatusModel.hpp"

#include <sstream>

namespace lidar_client {

namespace {

std::string get_string(const lidar_core::Json& json, const char* key) {
    return json.contains(key) && json.at(key).is_string()
        ? json.at(key).string_value()
        : "";
}

double get_number(const lidar_core::Json& json, const char* key, double fallback = 0.0) {
    return json.contains(key) && json.at(key).is_number()
        ? json.at(key).number_value()
        : fallback;
}

int get_int(const lidar_core::Json& json, const char* key, int fallback = 0) {
    return json.contains(key) && json.at(key).is_number()
        ? json.at(key).int_value()
        : fallback;
}

std::vector<double> get_number_array(const lidar_core::Json& json, const char* key) {
    std::vector<double> values;
    if (!json.contains(key) || !json.at(key).is_array()) {
        return values;
    }
    for (const auto& item : json.at(key).array_items()) {
        if (item.is_number()) {
            values.push_back(item.number_value());
        }
    }
    return values;
}

lidar_core::Json number_array_to_json(const std::vector<double>& values) {
    lidar_core::Json::Array array;
    array.reserve(values.size());
    for (double value : values) {
        array.emplace_back(value);
    }
    return lidar_core::Json(std::move(array));
}

} // namespace

bool DeviceStatusModel::update_from_frame(const lidar_protocol::Frame& frame) {
    if (frame.type != lidar_protocol::FrameType::status || !frame.payload.is_object()) {
        return false;
    }

    if (frame.payload.contains("site_id")) {
        snapshot_.site_id = get_string(frame.payload, "site_id");
    }
    if (frame.payload.contains("site_name")) {
        snapshot_.site_name = get_string(frame.payload, "site_name");
    }
    if (frame.payload.contains("device_model")) {
        snapshot_.device_model = get_string(frame.payload, "device_model");
    }
    if (frame.payload.contains("vendor_profile")) {
        snapshot_.vendor_profile = get_string(frame.payload, "vendor_profile");
    }
    if (frame.payload.contains("protocol_version")) {
        snapshot_.protocol_version = get_string(frame.payload, "protocol_version");
    }
    if (frame.payload.contains("ppi_elevations_deg")) {
        snapshot_.ppi_elevations_deg = get_number_array(frame.payload, "ppi_elevations_deg");
    }
    if (frame.payload.contains("ppi_azimuth_start_deg")) {
        snapshot_.ppi_azimuth_start_deg = get_number(frame.payload, "ppi_azimuth_start_deg");
    }
    if (frame.payload.contains("ppi_azimuth_stop_deg")) {
        snapshot_.ppi_azimuth_stop_deg = get_number(frame.payload, "ppi_azimuth_stop_deg", 360.0);
    }
    if (frame.payload.contains("ppi_azimuth_step_deg")) {
        snapshot_.ppi_azimuth_step_deg = get_number(frame.payload, "ppi_azimuth_step_deg");
    }
    if (frame.payload.contains("ppi_line_dwell_s")) {
        snapshot_.ppi_line_dwell_s = get_number(frame.payload, "ppi_line_dwell_s");
    }
    if (frame.payload.contains("ppi_step_overhead_s")) {
        snapshot_.ppi_step_overhead_s = get_number(frame.payload, "ppi_step_overhead_s");
    }
    if (frame.payload.contains("stare_dwell_s")) {
        snapshot_.stare_dwell_s = get_number(frame.payload, "stare_dwell_s");
    }
    if (frame.payload.contains("ppi_scan_cycle_s")) {
        snapshot_.ppi_scan_cycle_s = get_number(frame.payload, "ppi_scan_cycle_s");
    }
    if (frame.payload.contains("full_scan_cycle_s")) {
        snapshot_.full_scan_cycle_s = get_number(frame.payload, "full_scan_cycle_s");
    }
    if (frame.payload.contains("playback_time_scale")) {
        snapshot_.playback_time_scale = get_number(frame.payload, "playback_time_scale", 1.0);
    }
    if (frame.payload.contains("pulse_repetition_hz")) {
        snapshot_.pulse_repetition_hz = get_number(frame.payload, "pulse_repetition_hz");
    }
    if (frame.payload.contains("integrated_pulses_per_line")) {
        snapshot_.integrated_pulses_per_line = get_int(frame.payload, "integrated_pulses_per_line");
    }
    if (frame.payload.contains("total_steps")) {
        snapshot_.total_steps = get_int(frame.payload, "total_steps", -1);
    }
    snapshot_.valid = true;
    return true;
}

lidar_core::Json DeviceStatusModel::to_json() const {
    return lidar_core::Json::Object{
        {"valid", snapshot_.valid},
        {"site_id", snapshot_.site_id},
        {"site_name", snapshot_.site_name},
        {"device_model", snapshot_.device_model},
        {"vendor_profile", snapshot_.vendor_profile},
        {"protocol_version", snapshot_.protocol_version},
        {"ppi_elevations_deg", number_array_to_json(snapshot_.ppi_elevations_deg)},
        {"ppi_azimuth_start_deg", snapshot_.ppi_azimuth_start_deg},
        {"ppi_azimuth_stop_deg", snapshot_.ppi_azimuth_stop_deg},
        {"ppi_azimuth_step_deg", snapshot_.ppi_azimuth_step_deg},
        {"ppi_line_dwell_s", snapshot_.ppi_line_dwell_s},
        {"ppi_step_overhead_s", snapshot_.ppi_step_overhead_s},
        {"stare_dwell_s", snapshot_.stare_dwell_s},
        {"ppi_scan_cycle_s", snapshot_.ppi_scan_cycle_s},
        {"full_scan_cycle_s", snapshot_.full_scan_cycle_s},
        {"playback_time_scale", snapshot_.playback_time_scale},
        {"pulse_repetition_hz", snapshot_.pulse_repetition_hz},
        {"integrated_pulses_per_line", snapshot_.integrated_pulses_per_line},
        {"total_steps", snapshot_.total_steps},
    };
}

std::string DeviceStatusModel::brief() const {
    if (!snapshot_.valid) {
        return "device status unavailable";
    }

    std::ostringstream output;
    output << snapshot_.device_model
           << " vendor=" << snapshot_.vendor_profile
           << " PRF=" << snapshot_.pulse_repetition_hz << "Hz"
           << " stare=" << snapshot_.stare_dwell_s << "s"
           << " dwell=" << snapshot_.ppi_line_dwell_s << "s"
           << " move=" << snapshot_.ppi_step_overhead_s << "s"
           << " sector=" << snapshot_.ppi_azimuth_start_deg
           << "-" << snapshot_.ppi_azimuth_stop_deg
           << " step=" << snapshot_.ppi_azimuth_step_deg
           << " cycle=" << snapshot_.full_scan_cycle_s << "s"
           << " playback=x" << snapshot_.playback_time_scale;
    return output.str();
}

} // namespace lidar_client
