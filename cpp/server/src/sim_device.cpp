/**
 * @file sim_device.cpp
 * @brief 仿真 LiDAR 设备实现：初始化时一次性生成全部仿真数据并缓存。
 */
#include "lidar_server/sim_device.hpp"

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
    sim_config_.time_steps = config_.time_steps;
    sim_config_.minutes_per_step = config_.minutes_per_step;
    sim_config_.range_bin_count = config_.range_bin_count;
    sim_config_.range_bin_m = config_.range_bin_m;
    sim_config_.ppi_elevation_deg = config_.ppi_elevation_deg;
    sim_config_.ppi_azimuth_step_deg = config_.ppi_azimuth_step_deg;
    sim_config_.system_constant = config_.system_constant;
    sim_config_.lidar_ratio_sr = config_.lidar_ratio_sr;

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
    std::vector<lidar_protocol::Frame> frames;

    if (!initialized_ || step_index < 0 || step_index >= static_cast<int>(profiles_by_step_.size())) {
        return frames;
    }

    const std::string& timestamp = timestamps_[step_index];

    // 为该时间步的所有射线生成 lidar_raw 帧
    for (const auto& profile : profiles_by_step_[step_index]) {
        lidar_core::Json payload = lidar_protocol::profile_to_json(profile);
        frames.push_back(lidar_protocol::make_frame(
            lidar_protocol::FrameType::lidar_raw,
            timestamp,
            std::move(payload)
        ));
    }

    // 附带地面观测帧
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
            lidar_core::Json::object_type{}
        );
    }

    const auto& gm = ground_measurements_[step_index];
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::ground_obs,
        gm.timestamp,
        lidar_protocol::ground_to_json(gm)
    );
}

} // namespace lidar_server
