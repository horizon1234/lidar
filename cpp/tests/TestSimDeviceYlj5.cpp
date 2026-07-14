#include "lidar_protocol/Frame.hpp"
#include "lidar_server/SimDevice.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool accepted(const lidar_protocol::Frame& frame) {
    return frame.type == lidar_protocol::FrameType::command_result
        && frame.payload.contains("accepted")
        && frame.payload.at("accepted").is_bool()
        && frame.payload.at("accepted").bool_value();
}

int count_frames(
    const std::vector<lidar_protocol::Frame>& frames,
    lidar_protocol::FrameType type) {
    int count = 0;
    for (const auto& frame : frames) {
        if (frame.type == type) {
            ++count;
        }
    }
    return count;
}

int count_raw_scan_pattern(
    const std::vector<lidar_protocol::Frame>& frames,
    const std::string& pattern) {
    int count = 0;
    for (const auto& frame : frames) {
        if (frame.type == lidar_protocol::FrameType::lidar_raw
            && frame.payload.contains("device_scan_pattern")
            && frame.payload.at("device_scan_pattern").is_string()
            && frame.payload.at("device_scan_pattern").string_value() == pattern) {
            ++count;
        }
    }
    return count;
}

const lidar_protocol::Frame* first_raw_scan_pattern(
    const std::vector<lidar_protocol::Frame>& frames,
    const std::string& pattern) {
    for (const auto& frame : frames) {
        if (frame.type == lidar_protocol::FrameType::lidar_raw
            && frame.payload.contains("device_scan_pattern")
            && frame.payload.at("device_scan_pattern").is_string()
            && frame.payload.at("device_scan_pattern").string_value() == pattern) {
            return &frame;
        }
    }
    return nullptr;
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--full-smoke") {
            lidar_server::SimDeviceConfig config;
            config.campaign_cycles = 2;
            config.stream.emit_camera_frames = false;
            config.stream.emit_product_frames = false;
            lidar_server::SimDevice device(config);
            int raw_count = 0;
            int horizontal_count = 0;
            int conical_count = 0;
            int vertical_count = 0;
            bool shape_valid = true;
            auto started = std::chrono::steady_clock::now();
            bool completed = true;
            for (int cycle_index = 0; cycle_index < 2; ++cycle_index) {
                completed = completed && device.stream_scan_cycle(
                    cycle_index,
                    [&](lidar_protocol::Frame&& frame) {
                        if (frame.type == lidar_protocol::FrameType::lidar_raw) {
                            ++raw_count;
                            shape_valid = shape_valid
                                && frame.payload.at("ranges_m").array_items().size() == 5334
                                && frame.payload.at("channels").array_items().size() == 4;
                            const std::string pattern =
                                frame.payload.at("device_scan_pattern").string_value();
                            horizontal_count += pattern == "horizontal_scan" ? 1 : 0;
                            conical_count += pattern == "conical_scan" ? 1 : 0;
                            vertical_count += pattern == "vertical_observation" ? 1 : 0;
                        }
                        return true;
                    });
            }
            double elapsed_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            require(completed && shape_valid && raw_count == 362
                        && horizontal_count == 180 && conical_count == 180
                        && vertical_count == 2,
                    "Full two-cycle public-spec scan schedule smoke test failed");
            std::cout << "Full YLJ5 two-cycle schedule: " << raw_count
                      << " raw frames, 5334 bins, elapsed=" << elapsed_s << "s\n";
            return 0;
        }

        lidar_server::SimDeviceConfig public_config;
        require(public_config.public_spec_violations().empty(),
                "Default YLJ5 public specification should validate");
        require(public_config.hardware.range_bin_count() == 5334,
                "20 km at 3.75 m resolution should produce 5334 bins");
        require(public_config.scan.horizontal_ray_count() == 180,
                "Default horizontal scan should contain 180 rays");
        require(public_config.scan.elevations_deg.size() == 2
                    && public_config.scan.elevation_for_cycle(0) == 0.0
                    && public_config.scan.elevation_for_cycle(1) == 5.0
                    && public_config.scan.elevation_for_cycle(2) == 0.0,
                "Default schedule should alternate 0 and 5 degree azimuth scans");
        require(public_config.scan.scan_pattern_for_cycle(0) == "horizontal_scan"
                    && public_config.scan.scan_pattern_for_cycle(1) == "conical_scan",
                "Default schedule should alternate horizontal and conical patterns");
        require(public_config.scan.integrated_pulses_per_ray(
                    public_config.scene.pulse_repetition_hz) == 10000,
                "Default ray should integrate 10000 pulses");
        require(public_config.scan.horizontal_scan_cycle_seconds() <= 900.0,
                "Default horizontal scan should finish within 15 minutes");

        lidar_server::SimDevice public_device(public_config);
        auto status = public_device.status_frame(0);
        require(status.payload.at("device_model").string_value() == "YLJ5",
                "Status should expose the YLJ5 model");
        require(status.payload.at("regulatory_model").string_value()
                    == "AGHJ-I-LIDAR(MPL)",
                "Status should expose the regulatory model");
        require(!status.payload.at("vendor_wire_protocol_known").bool_value(),
                "Unknown vendor wire protocol must be explicit");
        require(status.payload.at("receiver_channels").array_items().size() == 4,
                "Status should advertise four receiver channels");
        require(status.payload.at("supported_scan_patterns").array_items().size() == 3,
                "Status should advertise horizontal, vertical, and conical modes");
        require(status.payload.at("elevation_schedule").array_items().size() == 2,
                "Status should expose the two-cycle elevation schedule");
        require(status.payload.at("scheduled_elevations_deg").array_items().size() == 2
                    && status.payload.at("ppi_elevations_deg").array_items().size() == 1,
                "Status should separate the full schedule from the active PPI elevation");
        require(status.payload.at("active_azimuth_scan_pattern").string_value()
                    == "horizontal_scan",
                "Cycle zero status should identify the horizontal scan");
        require(status.payload.at("supports_usb").bool_value()
                    && status.payload.at("supports_serial").bool_value(),
                "Status should expose public USB and serial capabilities");
        require(status.payload.at("maximum_vehicle_speed_kmh").number_value() == 120.0,
                "Status should expose the public vehicle speed boundary");

        auto invalid_scan = public_device.handle_command(lidar_core::Json::Object{
            {"command", "set_scan"},
            {"parameters", lidar_core::Json::Object{{"azimuth_step_deg", 10.0}}},
        });
        require(!accepted(invalid_scan),
                "A public-spec device should reject a scan with fewer than 180 rays");
        require(!invalid_scan.payload.at("violations").array_items().empty(),
                "Rejected scan should explain its public-spec violations");

        lidar_server::SimDeviceConfig test_config;
        test_config.enforce_public_spec = false;
        test_config.hardware.maximum_range_m = 75.0;
        test_config.scan.azimuth_step_deg = 120.0;
        test_config.scan.line_dwell_s = 0.01;
        test_config.scan.vertical_stare_dwell_s = 0.02;
        test_config.scan.scan_speed_deg_s = 30.0;
        test_config.scan.scan_overhead_s = 0.0;
        test_config.scene.pulse_repetition_hz = 1000.0;
        test_config.scene.system_constant = 1000000.0;
        test_config.stream.emit_ground_observation = true;
        test_config.stream.emit_truth_fields = false;
        test_config.campaign_cycles = 2;

        lidar_server::SimDevice device(test_config);
        auto frames = device.produce_scan_cycle(0);
        require(count_frames(frames, lidar_protocol::FrameType::lidar_raw) == 4,
                "Reduced cycle should contain one stare and three horizontal rays");
        require(count_frames(frames, lidar_protocol::FrameType::camera) == 1,
                "Cycle should include synchronized camera metadata");
        require(count_frames(frames, lidar_protocol::FrameType::lidar_product) == 1,
                "Cycle should include a quicklook product");
        require(count_frames(frames, lidar_protocol::FrameType::ground_obs) == 1,
                "Configured cycle should include a ground observation");
        require(count_raw_scan_pattern(frames, "vertical_observation") == 1
                    && count_raw_scan_pattern(frames, "horizontal_scan") == 3
                    && count_raw_scan_pattern(frames, "conical_scan") == 0,
                "Cycle zero should contain one vertical observation and one horizontal sweep");

        const lidar_protocol::Frame* raw = nullptr;
        for (const auto& frame : frames) {
            if (frame.type == lidar_protocol::FrameType::lidar_raw) {
                raw = &frame;
                break;
            }
        }
        require(raw != nullptr, "A raw frame should exist");
        require(raw->payload.at("device_scan_pattern").string_value()
                    == "vertical_observation",
                "The first raw frame should be the vertical observation");
        require(raw->payload.at("ranges_m").array_items().size() == 20,
                "Reduced test range should contain 20 bins");
        require(raw->payload.at("channels").array_items().size() == 4,
                "Raw frame should serialize all four physical channels");
        require(raw->payload.contains("simulated_receiver_response")
                    && raw->payload.at("simulated_receiver_response").is_object(),
                "Synthetic raw frames should expose the assumed detector response model");
        require(!raw->payload.contains("true_backscatter"),
                "Truth fields should be omitted from the live protocol by default");

        auto parsed_frame = lidar_protocol::parse_frame(raw->to_json_line());
        auto parsed_profile = lidar_protocol::json_to_profile(parsed_frame.payload);
        require(parsed_profile.channels.size() == 4,
                "Four receiver channels should survive JSONL roundtrip");
        require(parsed_profile.channels.front().channel_id == "near_parallel_532nm",
                "Near parallel channel identifier should be stable");
        require(parsed_profile.channels.front().raw_counts.size() == 20,
                "Channel sample count should survive JSONL roundtrip");
        require(parsed_profile.depolarization_ratio.size() == 20,
                "Depolarization ratio should survive JSONL roundtrip");

        const auto* horizontal = first_raw_scan_pattern(frames, "horizontal_scan");
        require(horizontal != nullptr
                    && horizontal->payload.at("elevation_deg").number_value() == 0.0,
                "Cycle zero azimuth rays should use zero-degree elevation");

        auto pause = device.handle_command(lidar_core::Json::Object{{"command", "pause"}});
        require(accepted(pause) && !device.is_streaming(),
                "Pause command should stop stream production");
        require(device.produce_scan_cycle(1).empty(),
                "Paused device should not generate a cycle");

        auto start = device.handle_command(lidar_core::Json::Object{{"command", "start"}});
        require(accepted(start) && device.is_streaming(),
                "Start command should resume stream production");
        auto configure = device.handle_command(lidar_core::Json::Object{
            {"command", "set_scan"},
            {"parameters", lidar_core::Json::Object{{"azimuth_step_deg", 90.0}}},
        });
        require(accepted(configure), "Reduced diagnostic scan should be configurable");
        auto reconfigured_frames = device.produce_scan_cycle(1);
        require(count_frames(reconfigured_frames, lidar_protocol::FrameType::lidar_raw) == 5,
                "Reconfigured cycle should contain one stare and four conical rays");
        require(count_raw_scan_pattern(reconfigured_frames, "vertical_observation") == 1
                    && count_raw_scan_pattern(reconfigured_frames, "conical_scan") == 4,
                "Cycle one should switch from horizontal to conical scanning");
        const auto* conical = first_raw_scan_pattern(reconfigured_frames, "conical_scan");
        require(conical != nullptr
                    && conical->payload.at("elevation_deg").number_value() == 5.0
                    && conical->payload.at("elevation_schedule_index").number_value() == 1.0,
                "Cycle one conical rays should use the scheduled five-degree elevation");

        auto unsupported = device.handle_command(
            lidar_core::Json::Object{{"command", "erase_calibration"}});
        require(!accepted(unsupported), "Unsupported commands should be rejected");

        require(lidar_protocol::string_to_frame_type("telemetry")
                    == lidar_protocol::FrameType::telemetry,
                "Telemetry frame type should roundtrip");
        require(lidar_protocol::string_to_frame_type("command_result")
                    == lidar_protocol::FrameType::command_result,
                "Command result frame type should roundtrip");

        std::cout << "YLJ5 emulator assertions passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
