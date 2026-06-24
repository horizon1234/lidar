/**
 * @file sim_device.hpp
 * @brief 仿真 LiDAR 设备：按配置生成周期性扫描帧流。
 *
 * SimDevice 在初始化时一次性生成完整的仿真战役数据（通过调用
 * lidar_core::run_end_to_end 触发内部前向仿真），缓存所有原始射线，
 * 然后通过 produce_scan_cycle() 逐时间步产出帧。
 */
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "lidar_core/lidar_core.hpp"
#include "lidar_protocol/frame.hpp"

namespace lidar_server {

/**
 * @brief 仿真设备配置。
 */
struct SimDeviceConfig {
    std::string site_id = "sim-site-01";
    std::string site_name = "仿真站点";
    double latitude_deg = 39.9042;
    double longitude_deg = 116.4074;
    int seed = 7;
    int time_steps = 18;
    int minutes_per_step = 20;
    int range_bin_count = 30;
    double range_bin_m = 50.0;
    double ppi_elevation_deg = 8.0;
    double ppi_azimuth_step_deg = 30.0;
    double system_constant = 260000000.0;
    double lidar_ratio_sr = 45.0;
    ///< 每帧之间的间隔（毫秒），0 表示不延迟
    int inter_frame_delay_ms = 50;
};

/**
 * @brief 仿真 LiDAR 设备。
 *
 * 工作模式：
 *   1. 构造时调用 init() 一次性生成全部仿真数据并缓存
 *   2. produce_scan_cycle(step) 按时间步索引返回该步的所有帧
 *   3. 客户端按步索引循环调用，实现周期性数据推送
 */
class SimDevice {
public:
    explicit SimDevice(const SimDeviceConfig& config);

    /**
     * @brief 初始化：生成并缓存全部仿真数据。
     * 在构造函数中自动调用，也可手动重新调用。
     */
    void init();

    /**
     * @brief 执行一次仿真"扫描周期"，产出该时间步的所有帧。
     *
     * @param step_index 时间步索引（0 ~ total_steps()-1）
     * @return 该步产出的所有帧（stare + PPI 各方位）
     */
    std::vector<lidar_protocol::Frame> produce_scan_cycle(int step_index);

    /**
     * @brief 获取总时间步数。
     */
    int total_steps() const { return static_cast<int>(profiles_by_step_.size()); }

    /**
     * @brief 获取每步间隔（分钟）。
     */
    int minutes_per_step() const { return config_.minutes_per_step; }

    /**
     * @brief 获取站点信息。
     */
    lidar_core::SiteInfo site_info() const;

    /**
     * @brief 获取帧间延迟（毫秒）。
     */
    int inter_frame_delay_ms() const { return config_.inter_frame_delay_ms; }

    /**
     * @brief 获取指定时间步的地面观测帧。
     */
    lidar_protocol::Frame ground_frame(int step_index) const;

private:
    SimDeviceConfig config_;
    lidar_core::SimulationConfig sim_config_;
    ///< 按时间步分组的原始射线
    std::vector<std::vector<lidar_core::LidarProfile>> profiles_by_step_;
    ///< 按时间步分组的地面观测
    std::vector<lidar_core::GroundMeasurement> ground_measurements_;
    ///< 所有时间戳列表
    std::vector<std::string> timestamps_;
    bool initialized_ = false;
};

} // namespace lidar_server
