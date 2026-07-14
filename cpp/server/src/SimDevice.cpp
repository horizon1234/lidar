/**
 * @file SimDevice.cpp
 * @brief 按公开规格惰性生成数据的 YLJ5 / AGHJ-I-LIDAR(MPL) 仿真设备实现。
 */
#include "lidar_server/SimDevice.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace lidar_server {

namespace {

std::tm safe_localtime(std::time_t value) {
    std::tm result{};
    localtime_r(&value, &result);
    return result;
}

std::string format_timestamp(std::chrono::system_clock::time_point value) {
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(value);
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(value - seconds).count();
    std::time_t time = std::chrono::system_clock::to_time_t(seconds);
    std::tm stamp = safe_localtime(time);
    std::ostringstream output;
    output << std::put_time(&stamp, "%Y-%m-%dT%H:%M:%S")
           << '.' << std::setw(3) << std::setfill('0') << millis;
    return output.str();
}

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string get_string(const lidar_core::Json& object, const char* key) {
    return object.contains(key) && object.at(key).is_string()
        ? object.at(key).string_value()
        : "";
}

bool get_bool(const lidar_core::Json& object, const char* key, bool fallback) {
    return object.contains(key) && object.at(key).is_bool()
        ? object.at(key).bool_value()
        : fallback;
}

lidar_core::Json doubles_to_json(const std::vector<double>& values) {
    lidar_core::Json::Array output;
    output.reserve(values.size());
    for (double value : values) {
        output.emplace_back(value);
    }
    return lidar_core::Json(std::move(output));
}

lidar_core::Json strings_to_json(const std::vector<std::string>& values) {
    lidar_core::Json::Array output;
    output.reserve(values.size());
    for (const auto& value : values) {
        output.emplace_back(value);
    }
    return lidar_core::Json(std::move(output));
}

lidar_core::Json scan_schedule_to_json(const Ylj5ScanProgram& scan) {
    lidar_core::Json::Array output;
    output.reserve(scan.elevations_deg.size());
    for (std::size_t index = 0; index < scan.elevations_deg.size(); ++index) {
        const double elevation = scan.elevations_deg[index];
        output.emplace_back(lidar_core::Json::Object{
            {"cycle_offset", static_cast<int>(index)},
            {"scan_pattern", scan.scan_pattern_for_elevation(elevation)},
            {"elevation_deg", elevation},
        });
    }
    return lidar_core::Json(std::move(output));
}

std::string scan_pattern_for_profile(
    const Ylj5ScanProgram& scan,
    const lidar_core::LidarProfile& profile) {
    return profile.scan_mode == "stare"
        ? "vertical_observation"
        : scan.scan_pattern_for_elevation(profile.elevation_deg);
}

const lidar_core::Json& command_parameters(const lidar_core::Json& command) {
    if (command.contains("parameters") && command.at("parameters").is_object()) {
        return command.at("parameters");
    }
    return command;
}

bool assign_number_if_present(
    const lidar_core::Json& object,
    const char* primary_key,
    const char* compatibility_key,
    double& output) {
    if (object.contains(primary_key) && object.at(primary_key).is_number()) {
        output = object.at(primary_key).number_value();
        return true;
    }
    if (compatibility_key != nullptr
        && object.contains(compatibility_key)
        && object.at(compatibility_key).is_number()) {
        output = object.at(compatibility_key).number_value();
        return true;
    }
    return false;
}

} // namespace

std::string device_run_state_to_string(DeviceRunState state) {
    switch (state) {
    case DeviceRunState::booting: return "booting";
    case DeviceRunState::ready: return "ready";
    case DeviceRunState::scanning: return "scanning";
    case DeviceRunState::paused: return "paused";
    case DeviceRunState::stopped: return "stopped";
    case DeviceRunState::fault: return "fault";
    }
    return "fault";
}

SimDevice::SimDevice(const SimDeviceConfig& config)
    : config_(config),
      campaign_start_time_(std::chrono::system_clock::now()) {
    init();
}

void SimDevice::init() {
    std::lock_guard<std::mutex> lock(mutex_);
    run_state_ = DeviceRunState::booting;
    fault_reason_.clear();

    // 初始化阶段只校验公开规格，不生成任何回波；这样服务启动不会预占整场战役内存。
    const auto violations = config_.public_spec_violations();
    if (!violations.empty()) {
        std::ostringstream message;
        for (std::size_t index = 0; index < violations.size(); ++index) {
            if (index != 0) {
                message << "; ";
            }
            message << violations[index];
        }
        fault_reason_ = message.str();
        initialized_ = false;
        run_state_ = DeviceRunState::fault;
        return;
    }

    campaign_start_time_ = std::chrono::system_clock::now();
    initialized_ = true;
    run_state_ = config_.stream.auto_start
        ? DeviceRunState::scanning
        : DeviceRunState::ready;
}

lidar_core::PipelineConfig SimDevice::pipeline_config_for_cycle(
    const SimDeviceConfig& config,
    int step_index) const {
    // 设备层区分“有公开证据的硬件边界”和“等待实机标定的正演参数”；核心正演 API
    // 仍使用扁平 SimulationConfig，因此只在这一处完成映射，避免其他模块重复解释规格。
    lidar_core::PipelineConfig pipeline;
    pipeline.site.name = config.site.site_name;
    pipeline.site.site_id = config.site.site_id;
    pipeline.site.latitude_deg = config.site.latitude_deg;
    pipeline.site.longitude_deg = config.site.longitude_deg;
    pipeline.site.altitude_m = config.site.altitude_m;

    auto& simulation = pipeline.simulation;
    simulation.application_mode = config.scene.application_mode;
    simulation.seed = config.scene.seed;
    simulation.time_steps = 1;
    simulation.minutes_per_step = config.minutes_per_cycle;
    simulation.start_step_index = step_index;
    simulation.phase_time_steps = config.campaign_cycles;
    simulation.range_bin_count = config.hardware.range_bin_count();
    simulation.range_bin_m = config.hardware.range_resolution_m;
    // YLJ5 设备层把仰角队列解释为“逐周期轮换”，每周期只正演一层方位扫描。
    // 这样第 0 周期为水平扫描、第 1 周期为锥形扫描，同时保持单周期内存规模稳定。
    simulation.ppi_elevations_deg = {config.scan.elevation_for_cycle(step_index)};
    simulation.ppi_azimuth_start_deg = config.scan.azimuth_start_deg;
    simulation.ppi_azimuth_stop_deg = config.scan.azimuth_stop_deg;
    simulation.ppi_azimuth_step_deg = config.scan.azimuth_step_deg;
    simulation.ppi_line_dwell_s = config.scan.line_dwell_s;
    simulation.ppi_step_overhead_s = config.scan.movement_seconds_per_ray();
    simulation.ppi_scan_overhead_s = config.scan.scan_overhead_s;
    simulation.include_stare_profile = config.scan.include_vertical_stare;
    simulation.stare_dwell_s = config.scan.vertical_stare_dwell_s;
    simulation.pulse_repetition_hz = config.scene.pulse_repetition_hz;
    simulation.system_constant = config.scene.system_constant;
    simulation.lidar_ratio_sr = config.scene.aerosol_lidar_ratio_sr;
    simulation.wavelength_nm = config.hardware.wavelength_nm;
    simulation.pulse_energy_mj = config.scene.pulse_energy_mj;
    simulation.pulse_energy_jitter = config.scene.pulse_energy_jitter;
    simulation.background_counts_mean = config.scene.background_counts_mean;
    simulation.background_counts_jitter = config.scene.background_counts_jitter;
    simulation.detector_dark_counts = config.scene.detector_dark_counts;
    simulation.read_noise_counts = config.scene.read_noise_counts;
    simulation.adc_saturation_counts = config.scene.adc_saturation_counts;
    simulation.dead_time_loss = config.scene.dead_time_loss;
    simulation.afterpulsing_ratio = config.scene.afterpulsing_ratio;
    simulation.solar_background_scale = config.scene.solar_background_scale;
    simulation.vehicle_speed_ms = config.scene.vehicle_speed_ms;
    simulation.near_telescope_aperture_mm = config.hardware.near_telescope_aperture_mm;
    simulation.far_telescope_aperture_mm = config.hardware.far_telescope_aperture_mm;
    simulation.near_channel_gain = config.scene.near_channel_gain;
    simulation.near_full_overlap_m = config.scene.near_full_overlap_m;
    simulation.far_full_overlap_m = config.scene.far_full_overlap_m;
    simulation.far_min_overlap = config.scene.far_min_overlap;
    simulation.channel_stitch_range_m = config.scene.channel_stitch_range_m;

    return pipeline;
}

std::string SimDevice::timestamp_for_step(int step_index) const {
    int minutes = 1;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        minutes = config_.minutes_per_cycle;
    }
    return format_timestamp(
        campaign_start_time_ + std::chrono::minutes(static_cast<long long>(step_index) * minutes));
}

std::vector<lidar_protocol::Frame> SimDevice::produce_scan_cycle(int step_index) {
    // 兼容接口会收集整个周期；正式 TCP 服务使用 stream_scan_cycle 逐帧释放 JSON DOM。
    std::vector<lidar_protocol::Frame> frames;
    stream_scan_cycle(step_index, [&](lidar_protocol::Frame&& frame) {
        frames.push_back(std::move(frame));
        return true;
    });
    return frames;
}

bool SimDevice::stream_scan_cycle(
    int step_index,
    const std::function<bool(lidar_protocol::Frame&&)>& sink) {
    if (!sink) {
        return false;
    }
    // 先在锁内取得不可变配置快照，耗时的物理正演在锁外执行，避免阻塞控制命令。
    SimDeviceConfig config;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_
            || run_state_ != DeviceRunState::scanning
            || step_index < 0
            || step_index >= config_.campaign_cycles) {
            return false;
        }
        config = config_;
    }

    // 每次只生成一个周期。默认 5334 个距离门和四通道若预生成 180 个周期，
    // 会占用数十 GB 内存；单周期生成把实测峰值控制在约 155 MiB。
    lidar_core::SyntheticCampaign campaign;
    try {
        campaign = lidar_core::generate_synthetic_campaign(
            pipeline_config_for_cycle(config, step_index));
    } catch (const std::exception& error) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            fault_reason_ = error.what();
            run_state_ = DeviceRunState::fault;
        }
        sink(lidar_protocol::make_frame(
            lidar_protocol::FrameType::alarm,
            timestamp_for_step(step_index),
            lidar_core::Json::Object{
                {"alarm_code", "simulation_generation_failed"},
                {"severity", "critical"},
                {"message", error.what()},
            }));
        return false;
    }

    // 核心正演使用固定基准时间保证离线可复现；设备层对外发送时统一替换成当前战役时间。
    const std::string timestamp = timestamp_for_step(step_index);
    const std::string cycle_id = config.site.site_id + "_cycle_" + std::to_string(step_index);
    for (std::size_t index = 0; index < campaign.profiles.size(); ++index) {
        auto& profile = campaign.profiles[index];
        profile.timestamp = timestamp;
        profile.site_id = config.site.site_id;
        profile.scan_id = cycle_id + "_ray_" + std::to_string(index);
    }
    for (auto& ground : campaign.ground_measurements) {
        ground.timestamp = timestamp;
        ground.site_id = config.site.site_id;
    }
    if (!campaign.ground_measurements.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_ground_measurement_ = campaign.ground_measurements.front();
    }

    double elapsed_s = 0.0;
    const int ray_count = static_cast<int>(campaign.profiles.size());
    const auto cycle_start = campaign_start_time_
        + std::chrono::minutes(static_cast<long long>(step_index) * config.minutes_per_cycle);

    for (int ray_index = 0; ray_index < ray_count; ++ray_index) {
        const auto& profile = campaign.profiles[static_cast<std::size_t>(ray_index)];
        const bool stare = profile.scan_mode == "stare";
        const double dwell_s = stare
            ? config.scan.vertical_stare_dwell_s
            : config.scan.line_dwell_s;
        const double movement_s = stare
            ? 0.0
            : config.scan.movement_seconds_per_ray();
        const double acquisition_start_s = elapsed_s;
        const double acquisition_end_s = acquisition_start_s + std::max(dwell_s, 0.0);
        const double publish_offset_s = acquisition_end_s + std::max(movement_s, 0.0);

        // 实时帧默认不携带分子场和仿真真值；四物理通道与兼容主通道始终保留。
        lidar_core::Json payload = lidar_protocol::profile_to_json(
            profile,
            config.stream.emit_truth_fields);
        payload["sequence_id"] = next_sequence_id_++;
        payload["frame_id"] = profile.scan_id;
        payload["scan_cycle_id"] = cycle_id;
        payload["scan_cycle_timestamp"] = timestamp;
        payload["device_scan_pattern"] = scan_pattern_for_profile(config.scan, profile);
        payload["azimuth_scan_pattern"] = config.scan.scan_pattern_for_cycle(step_index);
        payload["scheduled_elevation_deg"] = config.scan.elevation_for_cycle(step_index);
        payload["elevation_schedule_index"] = static_cast<int>(
            config.scan.elevation_schedule_index(step_index));
        payload["elevation_schedule_length"] = static_cast<int>(
            config.scan.elevations_deg.size());
        payload["ray_index"] = ray_index;
        payload["rays_in_cycle"] = ray_count;
        payload["acquisition_start_offset_s"] = acquisition_start_s;
        payload["acquisition_end_offset_s"] = acquisition_end_s;
        payload["publish_offset_s"] = publish_offset_s;
        payload["acquisition_start_timestamp"] = format_timestamp(
            cycle_start + std::chrono::milliseconds(
                static_cast<long long>(std::llround(acquisition_start_s * 1000.0))));
        payload["acquisition_end_timestamp"] = format_timestamp(
            cycle_start + std::chrono::milliseconds(
                static_cast<long long>(std::llround(acquisition_end_s * 1000.0))));
        payload["publish_timestamp"] = format_timestamp(
            cycle_start + std::chrono::milliseconds(
                static_cast<long long>(std::llround(publish_offset_s * 1000.0))));
        payload["primary_channel_id"] = "stitched_parallel_532nm";
        payload["detector_mode"] = "simulated_photon_counting";
        payload["simulated_receiver_response"] = lidar_core::Json::Object{
            {"calibration_status", config.scene.calibration_status},
            {"dead_time_loss_per_count", config.scene.dead_time_loss},
            {"afterpulsing_ratio", config.scene.afterpulsing_ratio},
            {"saturation_counts", config.scene.adc_saturation_counts},
            {"near_full_overlap_m", config.scene.near_full_overlap_m},
            {"far_full_overlap_m", config.scene.far_full_overlap_m},
            {"stitch_range_m", config.scene.channel_stitch_range_m},
        };
        payload["range_resolution_m"] = config.hardware.range_resolution_m;
        payload["maximum_range_m"] = config.hardware.maximum_range_m;
        payload["wavelength_nm"] = config.hardware.wavelength_nm;
        payload["pulse_repetition_hz"] = config.scene.pulse_repetition_hz;
        payload["line_dwell_s"] = dwell_s;
        payload["motion_overhead_s"] = movement_s;
        payload["integrated_pulses"] = static_cast<int>(std::llround(
            config.scene.pulse_repetition_hz * dwell_s));
        payload["signal_unit"] = "mean_counts_per_pulse";
        payload["range_unit"] = "m";
        payload["azimuth_encoder_deg"] = profile.azimuth_deg
            + 0.015 * std::sin(static_cast<double>(step_index + ray_index));
        payload["elevation_encoder_deg"] = profile.elevation_deg
            + 0.01 * std::cos(static_cast<double>(step_index + ray_index));
        payload["channel_count"] = static_cast<int>(profile.channels.size());
        payload["data_provenance"] = "synthetic_public_spec_emulator";
        payload["calibration_status"] = config.scene.calibration_status;
        payload["vendor_wire_protocol_emulated"] = false;
        payload["qc_hint"] = lidar_core::Json::Object{
            {"near_range_overlap_limited", !profile.overlap.empty() && profile.overlap.front() < 0.35},
            {"background_counts", profile.background_counts},
            {"laser_energy_mj", profile.laser_energy_mj},
        };
        if (!sink(lidar_protocol::make_frame(
            lidar_protocol::FrameType::lidar_raw,
            timestamp,
            std::move(payload)))) {
            return false;
        }
        elapsed_s = publish_offset_s;
    }

    if (config.stream.emit_camera_frames) {
        if (!sink(camera_frame(step_index, timestamp, campaign.profiles))) {
            return false;
        }
    }
    if (config.stream.emit_product_frames) {
        if (!sink(product_frame(step_index, timestamp, campaign.profiles))) {
            return false;
        }
    }
    if (config.stream.emit_ground_observation && !campaign.ground_measurements.empty()) {
        if (!sink(lidar_protocol::make_frame(
            lidar_protocol::FrameType::ground_obs,
            timestamp,
            lidar_protocol::ground_to_json(campaign.ground_measurements.front())))) {
            return false;
        }
    }
    return true;
}

lidar_protocol::Frame SimDevice::camera_frame(
    int step_index,
    const std::string& timestamp,
    const std::vector<lidar_core::LidarProfile>& profiles) const {
    // 官网只证明存在同步相机能力，尚无图像编码和标定样本，因此只发送能力与指向元数据。
    SimDeviceConfig config = this->config();
    lidar_core::Json payload = lidar_core::Json::Object{
        {"sequence_id", next_sequence_id_++},
        {"camera_id", "synchronized_environment_camera"},
        {"scan_cycle_index", step_index},
        {"synchronized_to_lidar", true},
        {"backlight_compensation_supported", config.hardware.camera_backlight_compensation},
        {"night_ir_supported", config.hardware.camera_night_ir},
        {"image_available", false},
        {"image_unavailable_reason", "real camera sensor and vendor encoding are not available"},
        {"data_provenance", "capability_metadata_from_public_spec"},
    };
    if (!profiles.empty()) {
        payload["lidar_azimuth_deg"] = profiles.back().azimuth_deg;
        payload["lidar_elevation_deg"] = profiles.back().elevation_deg;
        payload["device_scan_pattern"] = scan_pattern_for_profile(config.scan, profiles.back());
    }
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::camera,
        timestamp,
        std::move(payload));
}

lidar_protocol::Frame SimDevice::product_frame(
    int step_index,
    const std::string& timestamp,
    const std::vector<lidar_core::LidarProfile>& profiles) const {
    // 快速产品只给出无需实机标定即可解释的统计摘要，不输出伪造的定量 PM 结果。
    double max_signal = 0.0;
    double depolarization_sum = 0.0;
    std::size_t depolarization_count = 0;
    for (const auto& profile : profiles) {
        if (!profile.raw_counts.empty()) {
            max_signal = std::max(
                max_signal,
                *std::max_element(profile.raw_counts.begin(), profile.raw_counts.end()));
        }
        depolarization_sum += std::accumulate(
            profile.depolarization_ratio.begin(),
            profile.depolarization_ratio.end(),
            0.0);
        depolarization_count += profile.depolarization_ratio.size();
    }
    SimDeviceConfig config = this->config();
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::lidar_product,
        timestamp,
        lidar_core::Json::Object{
            {"sequence_id", next_sequence_id_++},
            {"scan_cycle_index", step_index},
            {"azimuth_scan_pattern", config.scan.scan_pattern_for_cycle(step_index)},
            {"scheduled_elevation_deg", config.scan.elevation_for_cycle(step_index)},
            {"product_level", "emulator_quicklook"},
            {"ray_count", static_cast<int>(profiles.size())},
            {"range_bin_count", config.hardware.range_bin_count()},
            {"maximum_primary_signal", max_signal},
            {"mean_volume_depolarization_ratio", depolarization_count == 0
                ? 0.0
                : depolarization_sum / static_cast<double>(depolarization_count)},
            {"quantitative_pm_calibrated", false},
            {"calibration_status", config.scene.calibration_status},
            {"data_provenance", "derived_from_synthetic_receiver_channels"},
        });
}

int SimDevice::total_steps() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_ ? config_.campaign_cycles : 0;
}

lidar_core::SiteInfo SimDevice::site_info() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lidar_core::SiteInfo{
        config_.site.site_name,
        config_.site.latitude_deg,
        config_.site.longitude_deg,
        config_.site.altitude_m,
        config_.site.site_id,
    };
}

SimDeviceConfig SimDevice::config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

bool SimDevice::is_streaming() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_ && run_state_ == DeviceRunState::scanning;
}

DeviceRunState SimDevice::run_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return run_state_;
}

lidar_protocol::Frame SimDevice::status_frame(int step_index) const {
    SimDeviceConfig config;
    DeviceRunState state;
    std::string fault;
    bool initialized = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config = config_;
        state = run_state_;
        fault = fault_reason_;
        initialized = initialized_;
    }

    // 状态帧同时发布公开规格、仿真假设和未知项，客户端可据此区分证据等级。
    lidar_core::Json::Array channels;
    for (const char* id : {
             "near_parallel_532nm",
             "near_perpendicular_532nm",
             "far_parallel_532nm",
             "far_perpendicular_532nm"}) {
        channels.emplace_back(id);
    }
    lidar_core::Json payload = lidar_core::Json::Object{
        {"site_id", config.site.site_id},
        {"site_name", config.site.site_name},
        {"latitude_deg", config.site.latitude_deg},
        {"longitude_deg", config.site.longitude_deg},
        {"altitude_m", config.site.altitude_m},
        {"manufacturer", config.hardware.manufacturer},
        {"product_name", config.hardware.product_name},
        {"device_model", config.hardware.model},
        {"regulatory_model", config.hardware.regulatory_model},
        {"device_state", device_run_state_to_string(state)},
        {"initialized", initialized},
        {"fault_reason", fault},
        {"protocol_version", config.stream.emulator_protocol},
        {"vendor_wire_protocol_known", config.stream.vendor_wire_protocol_known},
        {"vendor_wire_protocol_emulated", false},
        {"specification_basis", config.hardware.specification_basis},
        {"specification_semantics", config.hardware.specification_semantics},
        {"calibration_status", config.scene.calibration_status},
        {"instrument_preset", "ylj5_aghj_i_lidar_mpl_public_spec"},
        {"application_mode", config.scene.application_mode},
        {"vendor_profile", "ylj5_jsonl_emulator"},
        {"wavelength_nm", config.hardware.wavelength_nm},
        {"wavelength_tolerance_nm", config.hardware.wavelength_tolerance_nm},
        {"range_bin_count", config.hardware.range_bin_count()},
        {"range_resolution_m", config.hardware.range_resolution_m},
        {"maximum_range_m", config.hardware.maximum_range_m},
        {"minimum_snr_db", config.hardware.minimum_snr_db},
        {"beam_divergence_mrad_max", config.hardware.beam_divergence_mrad_max},
        {"laser_power_instability_fraction_max",
            config.hardware.laser_power_instability_fraction_max},
        {"simulation_pulse_energy_jitter", config.scene.pulse_energy_jitter},
        {"telescope_count", config.hardware.telescope_count},
        {"near_telescope_aperture_mm", config.hardware.near_telescope_aperture_mm},
        {"far_telescope_aperture_mm", config.hardware.far_telescope_aperture_mm},
        {"polarization_channel", config.hardware.polarization_channel},
        {"receiver_channels", lidar_core::Json(std::move(channels))},
        {"scan_program_mode", config.scan.mode},
        {"elevation_cycle_policy", config.scan.elevation_cycle_policy},
        {"supported_scan_patterns", strings_to_json({
            "horizontal_scan",
            "vertical_observation",
            "conical_scan",
        })},
        {"elevation_schedule", scan_schedule_to_json(config.scan)},
        {"scheduled_elevations_deg", doubles_to_json(config.scan.elevations_deg)},
        {"ppi_elevations_deg", doubles_to_json({config.scan.elevation_for_cycle(step_index)})},
        {"active_ppi_elevation_deg", config.scan.elevation_for_cycle(step_index)},
        {"active_azimuth_scan_pattern", config.scan.scan_pattern_for_cycle(step_index)},
        {"elevation_schedule_index", static_cast<int>(
            config.scan.elevation_schedule_index(step_index))},
        {"elevation_schedule_source",
            "public_modes_plus_literature_based_emulator_assumption"},
        {"ppi_azimuth_start_deg", config.scan.azimuth_start_deg},
        {"ppi_azimuth_stop_deg", config.scan.azimuth_stop_deg},
        {"ppi_azimuth_step_deg", config.scan.azimuth_step_deg},
        {"ppi_line_dwell_s", config.scan.line_dwell_s},
        {"ppi_step_overhead_s", config.scan.movement_seconds_per_ray()},
        {"ppi_scan_overhead_s", config.scan.scan_overhead_s},
        {"stare_dwell_s", config.scan.vertical_stare_dwell_s},
        {"horizontal_ray_count", config.scan.horizontal_ray_count()},
        {"ppi_scan_cycle_s", config.scan.horizontal_scan_cycle_seconds()},
        {"full_scan_cycle_s", config.scan.full_scan_cycle_seconds()},
        {"elevation_schedule_total_s", config.scan.elevation_schedule_seconds()},
        {"playback_time_scale", config.stream.playback_time_scale},
        {"pulse_repetition_hz", config.scene.pulse_repetition_hz},
        {"integrated_pulses_per_line", config.scan.integrated_pulses_per_ray(
            config.scene.pulse_repetition_hz)},
        {"total_steps", config.campaign_cycles},
        {"minutes_per_step", config.minutes_per_cycle},
        {"ingress_protection", config.hardware.ingress_protection},
        {"enclosure_material", config.hardware.enclosure_material},
        {"autonomous_operation", config.hardware.autonomous_operation},
        {"supports_wired_network", config.hardware.supports_wired_network},
        {"supports_wireless_network", config.hardware.supports_wireless_network},
        {"supports_serial", config.hardware.supports_serial},
        {"supports_usb", config.hardware.supports_usb},
        {"supports_vehicle_operation", config.hardware.supports_vehicle_operation},
        {"maximum_vehicle_speed_kmh", config.hardware.maximum_vehicle_speed_kmh},
        {"maximum_mobile_record_spacing_m", config.hardware.maximum_mobile_record_spacing_m},
        {"synchronized_camera", config.hardware.synchronized_camera},
        {"camera_payload_mode", "capability_metadata_only"},
        {"emit_truth_fields", config.stream.emit_truth_fields},
        {"public_spec_enforced", config.enforce_public_spec},
        {"unverified_model_parameters", strings_to_json({
            "pulse_repetition_hz",
            "pulse_energy_mj",
            "system_constant",
            "receiver_gain_and_overlap_curves",
            "detector_noise_and_dead_time",
            "automatic_scan_mode_sequence_and_5deg_conical_elevation",
        })},
    };
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::status,
        timestamp_for_step(std::max(step_index, 0)),
        std::move(payload));
}

lidar_protocol::Frame SimDevice::telemetry_frame(int step_index) const {
    SimDeviceConfig config;
    DeviceRunState state;
    lidar_core::GroundMeasurement ground;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config = config_;
        state = run_state_;
        ground = last_ground_measurement_;
    }
    // 当前没有实机遥测寄存器和抓包，数值仅用于测试客户端增量状态更新，并在帧内明确
    // 标记 telemetry_provenance=synthetic_not_vendor_spec。
    const double phase = 2.0 * 3.14159265358979323846
        * static_cast<double>(step_index) / std::max(config.campaign_cycles, 1);
    const double ambient_temperature = ground.site_id.empty()
        ? 24.0 + 4.0 * std::sin(phase)
        : ground.temperature_c;
    const double relative_humidity = ground.site_id.empty()
        ? 0.52 + 0.12 * std::cos(phase)
        : ground.relative_humidity;
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::telemetry,
        timestamp_for_step(std::max(step_index, 0)),
        lidar_core::Json::Object{
            {"site_id", config.site.site_id},
            {"device_model", config.hardware.model},
            {"device_state", device_run_state_to_string(state)},
            {"electronics_temperature_c", ambient_temperature + 5.5},
            {"ambient_temperature_c", ambient_temperature},
            {"relative_humidity", relative_humidity},
            {"pressure_hpa", 1012.0 + 3.0 * std::sin(phase + 0.4)},
            {"estimated_power_w", 118.0 + 7.0 * std::sin(phase - 0.2)},
            {"gimbal_azimuth_deg", config.scan.azimuth_start_deg},
            {"gimbal_elevation_deg", config.scan.elevation_for_cycle(step_index)},
            {"azimuth_scan_pattern", config.scan.scan_pattern_for_cycle(step_index)},
            {"elevation_schedule_index", static_cast<int>(
                config.scan.elevation_schedule_index(step_index))},
            {"camera_online", config.hardware.synchronized_camera},
            {"laser_enabled", state == DeviceRunState::scanning},
            {"telemetry_provenance", "synthetic_not_vendor_spec"},
            {"calibration_status", config.scene.calibration_status},
        });
}

lidar_protocol::Frame SimDevice::handle_command(const lidar_core::Json& command) {
    std::string name = get_string(command, "command");
    if (name.empty()) {
        name = get_string(command, "action");
    }
    if (name.empty()) {
        name = get_string(command, "name");
    }
    name = lowercase(name);
    const std::string request_id = get_string(command, "request_id");

    bool accepted = false;
    std::string message;
    std::vector<std::string> violations;
    DeviceRunState resulting_state = DeviceRunState::fault;
    SimDeviceConfig resulting_config;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        resulting_state = run_state_;
        if (!initialized_ && name != "get_status" && name != "status") {
            message = "device initialization failed: " + fault_reason_;
        } else if (name == "start" || name == "resume") {
            run_state_ = DeviceRunState::scanning;
            resulting_state = run_state_;
            accepted = true;
            message = "scan streaming started";
        } else if (name == "pause") {
            run_state_ = DeviceRunState::paused;
            resulting_state = run_state_;
            accepted = true;
            message = "scan streaming paused";
        } else if (name == "stop") {
            run_state_ = DeviceRunState::stopped;
            resulting_state = run_state_;
            accepted = true;
            message = "scan streaming stopped";
        } else if (name == "get_status" || name == "status") {
            accepted = true;
            resulting_state = run_state_;
            message = "status returned";
        } else if (name == "set_scan" || name == "configure_scan") {
            // 命令先修改配置副本，只有全部公开规格校验通过后才原子替换当前配置；
            // 失败命令不会留下只更新了一半的扫描计划。
            SimDeviceConfig candidate = config_;
            const auto& parameters = command_parameters(command);
            if (parameters.contains("mode") && parameters.at("mode").is_string()) {
                candidate.scan.mode = parameters.at("mode").string_value();
            }
            if (parameters.contains("elevation_cycle_policy")
                && parameters.at("elevation_cycle_policy").is_string()) {
                candidate.scan.elevation_cycle_policy =
                    parameters.at("elevation_cycle_policy").string_value();
            }
            assign_number_if_present(parameters, "azimuth_start_deg", "ppi_azimuth_start_deg",
                candidate.scan.azimuth_start_deg);
            assign_number_if_present(parameters, "azimuth_stop_deg", "ppi_azimuth_stop_deg",
                candidate.scan.azimuth_stop_deg);
            assign_number_if_present(parameters, "azimuth_step_deg", "ppi_azimuth_step_deg",
                candidate.scan.azimuth_step_deg);
            assign_number_if_present(parameters, "line_dwell_s", "ppi_line_dwell_s",
                candidate.scan.line_dwell_s);
            assign_number_if_present(parameters, "scan_speed_deg_s", nullptr,
                candidate.scan.scan_speed_deg_s);
            assign_number_if_present(parameters, "pointing_settle_s", nullptr,
                candidate.scan.pointing_settle_s);
            assign_number_if_present(parameters, "scan_overhead_s", "ppi_scan_overhead_s",
                candidate.scan.scan_overhead_s);
            assign_number_if_present(parameters, "vertical_stare_dwell_s", "stare_dwell_s",
                candidate.scan.vertical_stare_dwell_s);
            candidate.scan.include_vertical_stare = get_bool(
                parameters,
                "include_vertical_stare",
                candidate.scan.include_vertical_stare);
            if (parameters.contains("elevations_deg")
                && parameters.at("elevations_deg").is_array()) {
                candidate.scan.elevations_deg.clear();
                for (const auto& value : parameters.at("elevations_deg").array_items()) {
                    if (value.is_number()) {
                        candidate.scan.elevations_deg.push_back(value.number_value());
                    }
                }
            } else if (parameters.contains("ppi_elevations_deg")
                && parameters.at("ppi_elevations_deg").is_array()) {
                candidate.scan.elevations_deg.clear();
                for (const auto& value : parameters.at("ppi_elevations_deg").array_items()) {
                    if (value.is_number()) {
                        candidate.scan.elevations_deg.push_back(value.number_value());
                    }
                }
            }
            violations = candidate.public_spec_violations();
            if (violations.empty()) {
                config_ = std::move(candidate);
                accepted = true;
                message = "scan configuration applied";
            } else {
                message = "scan configuration violates public device requirements";
            }
            resulting_state = run_state_;
        } else {
            message = name.empty() ? "missing command name" : "unsupported command: " + name;
        }
        resulting_config = config_;
    }

    lidar_core::Json payload = lidar_core::Json::Object{
        {"request_id", request_id},
        {"command", name},
        {"accepted", accepted},
        {"result", accepted ? "ok" : "error"},
        {"message", message},
        {"device_state", device_run_state_to_string(resulting_state)},
        {"violations", strings_to_json(violations)},
    };
    payload["status"] = lidar_core::Json::Object{
        {"device_model", resulting_config.hardware.model},
        {"regulatory_model", resulting_config.hardware.regulatory_model},
        {"vendor_wire_protocol_known", resulting_config.stream.vendor_wire_protocol_known},
        {"range_resolution_m", resulting_config.hardware.range_resolution_m},
        {"maximum_range_m", resulting_config.hardware.maximum_range_m},
        {"azimuth_start_deg", resulting_config.scan.azimuth_start_deg},
        {"azimuth_stop_deg", resulting_config.scan.azimuth_stop_deg},
        {"azimuth_step_deg", resulting_config.scan.azimuth_step_deg},
        {"elevation_cycle_policy", resulting_config.scan.elevation_cycle_policy},
        {"elevations_deg", doubles_to_json(resulting_config.scan.elevations_deg)},
        {"line_dwell_s", resulting_config.scan.line_dwell_s},
        {"scan_speed_deg_s", resulting_config.scan.scan_speed_deg_s},
        {"calibration_status", resulting_config.scene.calibration_status},
    };
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::command_result,
        format_timestamp(std::chrono::system_clock::now()),
        std::move(payload));
}

} // namespace lidar_server
