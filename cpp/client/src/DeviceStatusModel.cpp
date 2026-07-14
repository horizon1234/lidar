#include "lidar_client/DeviceStatusModel.hpp"

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

std::vector<std::string> get_string_array(const lidar_core::Json& json, const char* key) {
    std::vector<std::string> values;
    if (!json.contains(key) || !json.at(key).is_array()) {
        return values;
    }
    for (const auto& item : json.at(key).array_items()) {
        if (item.is_string()) {
            values.push_back(item.string_value());
        }
    }
    return values;
}

bool get_bool(const lidar_core::Json& json, const char* key, bool fallback = false) {
    return json.contains(key) && json.at(key).is_bool()
        ? json.at(key).bool_value()
        : fallback;
}

} // namespace

bool DeviceStatusModel::update_from_frame(const lidar_protocol::Frame& frame) {
    if ((frame.type != lidar_protocol::FrameType::status
         && frame.type != lidar_protocol::FrameType::telemetry)
        || !frame.payload.is_object()) {
        return false;
    }

    if (frame.payload.contains("site_id")) {
        snapshot_.site_id = get_string(frame.payload, "site_id");
    }
    if (frame.payload.contains("site_name")) {
        snapshot_.site_name = get_string(frame.payload, "site_name");
    }
    if (frame.payload.contains("manufacturer")) {
        snapshot_.manufacturer = get_string(frame.payload, "manufacturer");
    }
    if (frame.payload.contains("product_name")) {
        snapshot_.product_name = get_string(frame.payload, "product_name");
    }
    if (frame.payload.contains("device_model")) {
        snapshot_.device_model = get_string(frame.payload, "device_model");
    }
    if (frame.payload.contains("regulatory_model")) {
        snapshot_.regulatory_model = get_string(frame.payload, "regulatory_model");
    }
    if (frame.payload.contains("device_state")) {
        snapshot_.device_state = get_string(frame.payload, "device_state");
    }
    if (frame.payload.contains("vendor_profile")) {
        snapshot_.vendor_profile = get_string(frame.payload, "vendor_profile");
    }
    if (frame.payload.contains("protocol_version")) {
        snapshot_.protocol_version = get_string(frame.payload, "protocol_version");
    }
    if (frame.payload.contains("calibration_status")) {
        snapshot_.calibration_status = get_string(frame.payload, "calibration_status");
    }
    if (frame.payload.contains("scan_program_mode")) {
        snapshot_.scan_program_mode = get_string(frame.payload, "scan_program_mode");
    }
    if (frame.payload.contains("active_azimuth_scan_pattern")) {
        snapshot_.active_scan_pattern = get_string(frame.payload, "active_azimuth_scan_pattern");
    }
    if (frame.payload.contains("azimuth_scan_pattern")) {
        snapshot_.active_scan_pattern = get_string(frame.payload, "azimuth_scan_pattern");
    }
    if (frame.payload.contains("elevation_cycle_policy")) {
        snapshot_.elevation_cycle_policy = get_string(frame.payload, "elevation_cycle_policy");
    }
    if (frame.payload.contains("specification_basis")) {
        snapshot_.specification_basis = get_string(frame.payload, "specification_basis");
    }
    if (frame.payload.contains("ingress_protection")) {
        snapshot_.ingress_protection = get_string(frame.payload, "ingress_protection");
    }
    if (frame.payload.contains("receiver_channels")) {
        snapshot_.receiver_channels = get_string_array(frame.payload, "receiver_channels");
    }
    if (frame.payload.contains("ppi_elevations_deg")) {
        snapshot_.ppi_elevations_deg = get_number_array(frame.payload, "ppi_elevations_deg");
    }
    if (frame.payload.contains("scheduled_elevations_deg")) {
        snapshot_.scheduled_elevations_deg = get_number_array(frame.payload, "scheduled_elevations_deg");
    }
    if (frame.payload.contains("active_ppi_elevation_deg")) {
        snapshot_.active_ppi_elevation_deg = get_number(frame.payload, "active_ppi_elevation_deg");
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
    if (frame.payload.contains("wavelength_nm")) {
        snapshot_.wavelength_nm = get_number(frame.payload, "wavelength_nm");
    }
    if (frame.payload.contains("range_resolution_m")) {
        snapshot_.range_resolution_m = get_number(frame.payload, "range_resolution_m");
    }
    if (frame.payload.contains("maximum_range_m")) {
        snapshot_.maximum_range_m = get_number(frame.payload, "maximum_range_m");
    }
    if (frame.payload.contains("near_telescope_aperture_mm")) {
        snapshot_.near_telescope_aperture_mm = get_number(frame.payload, "near_telescope_aperture_mm");
    }
    if (frame.payload.contains("far_telescope_aperture_mm")) {
        snapshot_.far_telescope_aperture_mm = get_number(frame.payload, "far_telescope_aperture_mm");
    }
    if (frame.payload.contains("electronics_temperature_c")) {
        snapshot_.electronics_temperature_c = get_number(frame.payload, "electronics_temperature_c");
    }
    if (frame.payload.contains("relative_humidity")) {
        snapshot_.relative_humidity = get_number(frame.payload, "relative_humidity");
    }
    if (frame.payload.contains("gimbal_azimuth_deg")) {
        snapshot_.gimbal_azimuth_deg = get_number(frame.payload, "gimbal_azimuth_deg");
    }
    if (frame.payload.contains("gimbal_elevation_deg")) {
        snapshot_.gimbal_elevation_deg = get_number(frame.payload, "gimbal_elevation_deg");
    }
    if (frame.payload.contains("elevation_schedule_index")) {
        snapshot_.elevation_schedule_index = get_int(frame.payload, "elevation_schedule_index");
    }
    if (frame.payload.contains("integrated_pulses_per_line")) {
        snapshot_.integrated_pulses_per_line = get_int(frame.payload, "integrated_pulses_per_line");
    }
    if (frame.payload.contains("total_steps")) {
        snapshot_.total_steps = get_int(frame.payload, "total_steps", -1);
    }
    if (frame.payload.contains("vendor_wire_protocol_known")) {
        snapshot_.vendor_wire_protocol_known = get_bool(frame.payload, "vendor_wire_protocol_known");
    }
    if (frame.payload.contains("polarization_channel")) {
        snapshot_.polarization_channel = get_bool(frame.payload, "polarization_channel");
    }
    if (frame.payload.contains("camera_online")) {
        snapshot_.camera_online = get_bool(frame.payload, "camera_online");
    }
    snapshot_.valid = true;
    return true;
}

} // namespace lidar_client
