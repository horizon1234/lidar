/**
 * @file sim_device.cpp
 * @brief 仿真 LiDAR 设备实现：初始化时一次性生成全部仿真数据并缓存。
 */
#include "lidar_server/SimDevice.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <set>
#include <thread>

namespace lidar_server {

SimDevice::SimDevice(const SimDeviceConfig& config)
    : config_(config) {
    init();
}

void SimDevice::init() {
    // 构建 lidar_core 的 SimulationConfig
    sim_config_.seed = config_.seed;
    sim_config_.instrument_preset = config_.instrument_preset;
    sim_config_.application_mode = config_.application_mode;
    sim_config_.vendor_profile = config_.vendor_profile;
    sim_config_.time_steps = config_.time_steps;
    sim_config_.minutes_per_step = config_.minutes_per_step;
    sim_config_.range_bin_count = config_.range_bin_count;
    sim_config_.range_bin_m = config_.range_bin_m;
    sim_config_.ppi_elevations_deg = config_.ppi_elevations_deg;
    sim_config_.ppi_azimuth_start_deg = config_.ppi_azimuth_start_deg;
    sim_config_.ppi_azimuth_stop_deg = config_.ppi_azimuth_stop_deg;
    sim_config_.ppi_azimuth_step_deg = config_.ppi_azimuth_step_deg;
    sim_config_.ppi_line_dwell_s = config_.ppi_line_dwell_s;
    sim_config_.ppi_scan_overhead_s = config_.ppi_scan_overhead_s;
    sim_config_.pulse_repetition_hz = config_.pulse_repetition_hz;
    sim_config_.system_constant = config_.system_constant;
    sim_config_.lidar_ratio_sr = config_.lidar_ratio_sr;
    sim_config_.wavelength_nm = config_.wavelength_nm;
    sim_config_.pulse_energy_mj = config_.pulse_energy_mj;
    sim_config_.pulse_energy_jitter = config_.pulse_energy_jitter;
    sim_config_.background_counts_mean = config_.background_counts_mean;
    sim_config_.full_overlap_m = config_.full_overlap_m;

    // 构建最小 PipelineConfig 调用 run_end_to_end 生成原始仿真数据
    lidar_core::PipelineConfig pipeline_config;
    pipeline_config.source_mode = "simulation";
    pipeline_config.simulation = sim_config_;
    pipeline_config.site = site_info();
    pipeline_config.retrieval.aerosol_lidar_ratio_sr = config_.lidar_ratio_sr;
    pipeline_config.retrieval.reference_aerosol_backscatter = 0.0004;
    pipeline_config.humidity.dry_reference_rh = 0.45;
    pipeline_config.humidity.hygroscopicity = 1.1;
    pipeline_config.pm_calibration.surface_bin_count = 6;
    pipeline_config.hotspot.pm25_threshold_ugm3 = 50.0;
    pipeline_config.hotspot.min_cells = 3;
    // 用单元素列表减少灵敏度扫描开销
    pipeline_config.evaluation.sensitivity_lidar_ratios = {config_.lidar_ratio_sr};

    // 生成数据到临时目录
    std::filesystem::path temp_dir = std::filesystem::temp_directory_path() / "lidar_sim_device";
    std::filesystem::create_directories(temp_dir / "data" / "raw");

    std::cerr << "[sim_device] Generating simulation campaign ("
              << config_.time_steps << " steps, seed=" << config_.seed << ")...\n";

    try {
        lidar_core::run_end_to_end(pipeline_config, temp_dir);
    } catch (const std::exception& e) {
        std::cerr << "[sim_device] run_end_to_end failed: " << e.what() << "\n";
        initialized_ = false;
        return;
    }

    // 加载原始射线 JSON
    std::filesystem::path raw_file = temp_dir / "data" / "raw" / "simulated_demo_campaign.json";
    if (!std::filesystem::exists(raw_file)) {
        std::cerr << "[sim_device] Raw data file not found: " << raw_file.string() << "\n";
        initialized_ = false;
        return;
    }

    lidar_core::Json raw_data = lidar_core::read_json_file(raw_file);
    if (!raw_data.is_array()) {
        std::cerr << "[sim_device] Raw data is not a JSON array\n";
        initialized_ = false;
        return;
    }

    // 按时间戳分组所有 profile
    std::map<std::string, std::vector<lidar_core::LidarProfile>> grouped;

    for (const auto& item : raw_data.array_items()) {
        lidar_core::LidarProfile profile = lidar_protocol::json_to_profile(item);
        std::string ts = profile.timestamp; // 在 move 前保存
        grouped[ts].push_back(std::move(profile));
    }

    // 按时间戳排序填充缓存（键已经是排序的，因为 std::map）
    timestamps_.clear();
    profiles_by_step_.clear();
    for (auto& [ts, profiles] : grouped) {
        timestamps_.push_back(ts);
        profiles_by_step_.push_back(std::move(profiles));
    }

    // 尝试从 L2 结果中提取地面观测
    std::filesystem::path l2_file = temp_dir / "data" / "l2" / "demo_results.json";
    ground_measurements_.clear();
    ground_measurements_.resize(timestamps_.size());

    if (std::filesystem::exists(l2_file)) {
        lidar_core::Json l2_data = lidar_core::read_json_file(l2_file);
        if (l2_data.contains("source") && l2_data.at("source").contains("ground_measurements")) {
            const auto& gm_array = l2_data.at("source").at("ground_measurements");
            if (gm_array.is_array()) {
                std::map<std::string, lidar_core::GroundMeasurement> gm_by_ts;
                for (const auto& gm_json : gm_array.array_items()) {
                    auto gm = lidar_protocol::json_to_ground(gm_json);
                    gm_by_ts[gm.timestamp] = std::move(gm);
                }
                for (size_t i = 0; i < timestamps_.size(); ++i) {
                    auto it = gm_by_ts.find(timestamps_[i]);
                    if (it != gm_by_ts.end()) {
                        ground_measurements_[i] = it->second;
                    }
                }
            }
        }
    }

    // 清理临时文件
    std::error_code ec;
    std::filesystem::remove_all(temp_dir, ec);

    std::cerr << "[sim_device] Initialization complete: " << timestamps_.size()
              << " time steps, " << raw_data.array_items().size() << " total profiles cached.\n";
    initialized_ = true;
}

lidar_core::SiteInfo SimDevice::site_info() const {
    lidar_core::SiteInfo site;
    site.site_id = config_.site_id;
    site.name = config_.site_name;
    site.latitude_deg = config_.latitude_deg;
    site.longitude_deg = config_.longitude_deg;
    return site;
}

std::vector<lidar_protocol::Frame> SimDevice::produce_scan_cycle(int step_index) {
    //
    // 产出指定时间步的全部帧。返回的 frames 由 main.cpp 的主循环逐条发送给客户端。
    //
    // 每个时间步包含：
    //   - 1 条 stare 射线（scan_mode="stare", 仰角 90°，天顶凝视）
    //     ↑ 用于 PM2.5 反演的垂直锚定
    //   - 多仰角 PPI 体扫（scan_mode="ppi"，默认 6 个仰角 × 36 个方位）
    //     ↑ 用于热点检测和 L3 体素产品
    //   - 1 条地面观测帧（ground_obs）
    //     ↑ 共址地面站的 PM/气象数据
    //   共 1 + elevation_count * azimuth_count 条 lidar_raw + ground_obs 帧
    //
    // 数据来源说明：
    //   profiles_by_step_ 和 ground_measurements_ 都在 init() 中由
    //   lidar_core::run_end_to_end() 一次性正向仿真生成。
    //   该仿真链路为：
    //     大气场景（气溶胶廓线 + 气象场）
    //       → simulate_profile_fields()：按角度计算各距离 bin 的后向散射/消光真值
    //       → simulate_raw_counts()：代入 LiDAR 方程 P(r)=C·β(r)/r²·T(r)² 算出光子计数
    //       → 添加噪声（背景光 + 泊松散粒噪声）
    //       → 结果写入 JSON → 这里加载到内存缓存
    //   所有数据都是合成的（source_kind="synthetic_stare"/"synthetic_ppi"），非真实观测。
    //
    std::vector<lidar_protocol::Frame> frames;

    if (!initialized_ || step_index < 0 || step_index >= static_cast<int>(profiles_by_step_.size())) {
        return frames;
    }

    const std::string& timestamp = timestamps_[step_index];
    const auto& profiles = profiles_by_step_[step_index];
    int rays_in_cycle = static_cast<int>(profiles.size());
    std::string scan_cycle_id = config_.site_id + "_" + timestamp + "_cycle";

    // ② lidar_raw 帧：把该时间步的每条射线（stare + PPI）序列化为 JSON
    //    每条帧 payload 包含 22 个字段：
    //      - 角度信息：azimuth_deg、elevation_deg、scan_mode、scan_id
    //      - 距离轴：ranges_m[N]（默认 160 个距离 bin，间隔 37.5m，最远 6km）
    //      - 原始信号：raw_counts[30]（探测器光子计数，核心物理量）
    //      - 系统参数：laser_energy_mj、background_counts、overlap[30]
    //      - 气象场：relative_humidity、temperature_c、wind_speed_ms、wind_dir_deg
    //      - 分子场：molecular_backscatter[30]、molecular_extinction[30]（Rayleigh 散射背景）
    //      - 真值场（仅仿真有）：true_backscatter/extinction/pm25/pm10/hotspot_mask[30]
    int ray_index = 0;
    for (const auto& profile : profiles) {
        lidar_core::Json payload = lidar_protocol::profile_to_json(profile);
        payload["sequence_id"] = next_sequence_id_++;
        payload["frame_id"] = profile.scan_id;
        payload["scan_cycle_id"] = scan_cycle_id;
        payload["ray_index"] = ray_index++;
        payload["rays_in_cycle"] = rays_in_cycle;
        payload["channel_id"] = "elastic_" + std::to_string(static_cast<int>(std::round(config_.wavelength_nm))) + "nm";
        payload["detector_mode"] = "hybrid_photon_analog";
        payload["range_resolution_m"] = config_.range_bin_m;
        payload["wavelength_nm"] = config_.wavelength_nm;
        payload["full_overlap_m"] = config_.full_overlap_m;
        payload["pulse_repetition_hz"] = config_.pulse_repetition_hz;
        payload["line_dwell_s"] = config_.ppi_line_dwell_s;
        payload["integrated_pulses"] = static_cast<int>(std::round(config_.pulse_repetition_hz * config_.ppi_line_dwell_s));
        payload["signal_unit"] = "counts";
        payload["range_unit"] = "m";
        payload["azimuth_encoder_deg"] = profile.azimuth_deg + 0.015 * std::sin(static_cast<double>(step_index + ray_index));
        payload["elevation_encoder_deg"] = profile.elevation_deg + 0.01 * std::cos(static_cast<double>(step_index + ray_index));
        payload["integration_pulses"] = profile.scan_mode == "stare"
            ? static_cast<int>(std::round(config_.pulse_repetition_hz * 30.0))
            : static_cast<int>(std::round(config_.pulse_repetition_hz * config_.ppi_line_dwell_s));
        payload["accumulation_time_ms"] = profile.scan_mode == "stare"
            ? 30000
            : static_cast<int>(std::round(config_.ppi_line_dwell_s * 1000.0));
        payload["instrument_preset"] = config_.instrument_preset;
        payload["application_mode"] = config_.application_mode;
        payload["vendor_profile"] = config_.vendor_profile;
        payload["qc_hint"] = lidar_core::Json::Object{
            {"near_range_overlap_limited", !profile.overlap.empty() && profile.overlap.front() < 0.35},
            {"background_counts", profile.background_counts},
            {"laser_energy_mj", profile.laser_energy_mj},
        };
        frames.push_back(lidar_protocol::make_frame(
            lidar_protocol::FrameType::lidar_raw,
            timestamp,
            std::move(payload)
        ));
    }

    // ③ ground_obs 帧：共址地面站观测，用于 PM 浓度的地面校准与验证
    //    payload 包含：pm25_ugm3、pm10_ugm3、relative_humidity、temperature_c、
    //                  wind_speed_ms、wind_dir_deg（6 个标量）
    //    数据来源：仿真地面模型（init() 中从 L2 结果文件提取）
    if (step_index < static_cast<int>(ground_measurements_.size())
        && !ground_measurements_[step_index].timestamp.empty()) {
        lidar_core::Json ground_payload = lidar_protocol::ground_to_json(ground_measurements_[step_index]);
        frames.push_back(lidar_protocol::make_frame(
            lidar_protocol::FrameType::ground_obs,
            timestamp,
            std::move(ground_payload)
        ));
    }

    return frames;
}

lidar_protocol::Frame SimDevice::ground_frame(int step_index) const {
    if (!initialized_ || step_index < 0 || step_index >= static_cast<int>(ground_measurements_.size())) {
        return lidar_protocol::make_frame(
            lidar_protocol::FrameType::ground_obs,
            "",
            lidar_core::Json::Object{}
        );
    }

    const auto& gm = ground_measurements_[step_index];
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::ground_obs,
        gm.timestamp,
        lidar_protocol::ground_to_json(gm)
    );
}

lidar_protocol::Frame SimDevice::status_frame(int step_index) const {
    int safe_step = std::max(0, std::min(step_index, total_steps() - 1));
    std::string timestamp = timestamps_.empty() ? "" : timestamps_[safe_step];
    lidar_core::Json payload = lidar_core::Json::Object{
        {"site_id", config_.site_id},
        {"site_name", config_.site_name},
        {"instrument_preset", config_.instrument_preset},
        {"application_mode", config_.application_mode},
        {"vendor_profile", config_.vendor_profile},
        {"device_model", "SIM-FIELD-PM-LIDAR"},
        {"firmware_version", "sim-0.3.0"},
        {"protocol_version", "jsonl-lidar-0.2"},
        {"total_steps", total_steps()},
        {"minutes_per_step", config_.minutes_per_step},
        {"simulated_span_minutes", total_steps() * config_.minutes_per_step},
        {"time_axis_semantics", "one_step_per_complete_scan_cycle"},
        {"continuous_stream", true},
        {"cached_campaign_replay", true},
        {"scan_modes", lidar_core::Json::Array{lidar_core::Json("stare"), lidar_core::Json("ppi_volume")}},
        {"channels", lidar_core::Json::Array{lidar_core::Json("elastic_" + std::to_string(static_cast<int>(std::round(config_.wavelength_nm))) + "nm")}},
        {"range_bin_count", config_.range_bin_count},
        {"range_resolution_m", config_.range_bin_m},
        {"max_range_m", config_.range_bin_count * config_.range_bin_m},
        {"ppi_elevations_deg", lidar_core::Json::Array{}},
        {"ppi_azimuth_start_deg", config_.ppi_azimuth_start_deg},
        {"ppi_azimuth_stop_deg", config_.ppi_azimuth_stop_deg},
        {"ppi_azimuth_step_deg", config_.ppi_azimuth_step_deg},
        {"ppi_line_dwell_s", config_.ppi_line_dwell_s},
        {"ppi_scan_overhead_s", config_.ppi_scan_overhead_s},
        {"ppi_scan_cycle_s", sim_config_.ppi_scan_cycle_seconds()},
        {"playback_time_scale", config_.playback_time_scale},
        {"wavelength_nm", config_.wavelength_nm},
        {"pulse_repetition_hz", config_.pulse_repetition_hz},
        {"integrated_pulses_per_line", static_cast<int>(std::round(config_.pulse_repetition_hz * config_.ppi_line_dwell_s))},
        {"pulse_energy_mj_nominal", config_.pulse_energy_mj},
        {"full_overlap_m", config_.full_overlap_m},
        {"detector_mode", "hybrid_photon_analog"},
        {"supports_l3_volume", true},
        {"data_level", "L0_raw_plus_device_telemetry"},
    };
    auto& elevations = payload["ppi_elevations_deg"].array_items();
    for (double elevation : config_.ppi_elevations_deg) {
        elevations.emplace_back(elevation);
    }
    return lidar_protocol::make_frame(lidar_protocol::FrameType::status, timestamp, std::move(payload));
}

lidar_protocol::Frame SimDevice::telemetry_frame(int step_index) const {
    int safe_step = std::max(0, std::min(step_index, total_steps() - 1));
    std::string timestamp = timestamps_.empty() ? "" : timestamps_[safe_step];
    double phase = 2.0 * M_PI * static_cast<double>(safe_step) / std::max(total_steps(), 1);
    bool high_humidity = !ground_measurements_.empty()
        && safe_step < static_cast<int>(ground_measurements_.size())
        && ground_measurements_[safe_step].relative_humidity > 0.78;
    double window_transmission = std::max(0.72, 0.96 - 0.035 * safe_step - (high_humidity ? 0.05 : 0.0));
    double enclosure_temp_c = 31.0 + 4.0 * std::sin(phase + 0.4);
    double laser_head_temp_c = 34.0 + 3.2 * std::sin(phase + 0.9);
    double background = config_.background_counts_mean * (0.85 + 0.22 * std::sin(phase - 0.4));
    bool precipitation = high_humidity && std::sin(phase) > 0.55;

    lidar_core::Json payload = lidar_core::Json::Object{};
    payload["site_id"] = config_.site_id;
    payload["sequence_id"] = next_sequence_id_ + safe_step;
    payload["device_state"] = std::string(precipitation ? "weather_hold" : "running");
    payload["scan_scheduler_state"] = "executing_volume_scan";
    payload["gps_lock"] = true;
    payload["ntp_sync"] = true;
    payload["clock_offset_ms"] = 1.5 + 0.6 * std::sin(phase);
    payload["enclosure_temp_c"] = enclosure_temp_c;
    payload["laser_head_temp_c"] = laser_head_temp_c;
    payload["detector_temp_c"] = 22.5 + 0.8 * std::cos(phase);
    payload["relative_humidity_internal"] = 0.36 + 0.05 * std::sin(phase + 1.2);
    payload["window_transmission"] = window_transmission;
    payload["window_contamination_index"] = std::min(1.0, std::max(0.0, 1.0 - window_transmission));
    payload["rain_sensor_wet"] = precipitation;
    payload["sun_background_counts"] = std::max(0.0, background);
    payload["laser_energy_mj_nominal"] = config_.pulse_energy_mj;
    payload["laser_energy_jitter"] = config_.pulse_energy_jitter;
    int rays_per_scan = static_cast<int>(sim_config_.effective_ppi_elevations_deg().size()
        * sim_config_.effective_ppi_azimuths_deg().size());
    payload["laser_shots_total"] = static_cast<int>(std::round(
        static_cast<double>(safe_step * std::max(rays_per_scan, 1))
        * config_.pulse_repetition_hz * config_.ppi_line_dwell_s));
    payload["fan_state"] = std::string(enclosure_temp_c > 33.0 ? "high" : "normal");
    payload["door_state"] = "closed";
    payload["wiper_state"] = std::string(precipitation ? "active" : "idle");
    payload["diagnostic_flags"] = lidar_core::Json::Array{};
    auto& flags = payload["diagnostic_flags"].array_items();
    if (window_transmission < 0.85) {
        flags.emplace_back("window-transmission-low");
    }
    if (precipitation) {
        flags.emplace_back("precipitation-filter-active");
    }
    if (background > config_.background_counts_mean * 1.05) {
        flags.emplace_back("high-solar-background");
    }
    return lidar_protocol::make_frame(lidar_protocol::FrameType::status, timestamp, std::move(payload));
}

} // namespace lidar_server
