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
    sim_config_.ppi_elevations_deg = config_.ppi_elevations_deg;
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
    //
    // 产出指定时间步的全部帧。返回的 frames 由 main.cpp 的主循环逐条发送给客户端。
    //
    // 每个时间步包含：
    //   - 1 条 stare 射线（scan_mode="stare", 仰角 90°，天顶凝视）
    //     ↑ 用于 PM2.5 反演的垂直锚定
    //   - 12 条 PPI 射线（scan_mode="ppi", 仰角 8°，方位角 0/30/.../330°）
    //     ↑ 用于热点检测的水平扫描
    //   - 1 条地面观测帧（ground_obs）
    //     ↑ 共址地面站的 PM/气象数据
    //   共 14 条 lidar_raw + ground_obs 帧
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

    // ② lidar_raw 帧：把该时间步的每条射线（stare + PPI）序列化为 JSON
    //    每条帧 payload 包含 22 个字段：
    //      - 角度信息：azimuth_deg、elevation_deg、scan_mode、scan_id
    //      - 距离轴：ranges_m[30]（30 个距离 bin，间隔 50m，最远 1.5km）
    //      - 原始信号：raw_counts[30]（探测器光子计数，核心物理量）
    //      - 系统参数：laser_energy_mj、background_counts、overlap[30]
    //      - 气象场：relative_humidity、temperature_c、wind_speed_ms、wind_dir_deg
    //      - 分子场：molecular_backscatter[30]、molecular_extinction[30]（Rayleigh 散射背景）
    //      - 真值场（仅仿真有）：true_backscatter/extinction/pm25/pm10/hotspot_mask[30]
    for (const auto& profile : profiles_by_step_[step_index]) {
        lidar_core::Json payload = lidar_protocol::profile_to_json(profile);
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
