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

#include "lidar_core/LidarCore.hpp"
#include "lidar_protocol/Frame.hpp"

namespace lidar_server {

/**
 * @brief 仿真设备配置。
 *
 * 所有字段最终会透传给 lidar_core::SimulationConfig，控制整个前向仿真
 * （场景生成 → 射线投射 → 正演信号）的物理与几何参数。下列默认值
 * 对应一个位于北京的假想站点，时间跨度约 6 小时（18 步 × 20 分钟）。
 */
struct SimDeviceConfig {
    // ---- 站点标识 ----
    std::string site_id = "sim-site-01";   ///< 站点唯一标识符，用于协议帧中的 site_id 字段
    std::string site_name = "Field PM LiDAR Demo Site"; ///< 站点可读名称，推送给客户端作为展示用
    double latitude_deg = 39.9042;         ///< 站点纬度（°），北京天安门附近，用于地理坐标计算
    double longitude_deg = 116.4074;       ///< 站点经度（°），同上
    std::string instrument_preset = "field_scanning_pm_lidar"; ///< 工地/城市污染扫描 LiDAR 预设
    std::string application_mode = "construction_site";        ///< construction_site / urban_grid / mobile_mapping
    std::string vendor_profile = "raymetrics_pmeye_like";      ///< 公开格式映射 profile

    // ---- 仿真可复现性 ----
    int seed = 7;                          ///< 随机数种子，相同种子产生完全相同的仿真数据（可复现性）

    // ---- 时间维度 ----
    int time_steps = 18;                   ///< 仿真总时间步数；每步代表一次完整的扫描周期
    int minutes_per_step = 20;             ///< 相邻时间步之间的真实时间间隔（分钟），决定时间序列分辨率

    // ---- 距离分辨率（Range Bin）----
    int range_bin_count = 160;             ///< 每条射线的距离 bin 数量，即一条射线被分成多少段
    double range_bin_m = 37.5;             ///< 相邻距离 bin 的间距（米）；总探测距离 = count × m = 6000m
    //   ↑ 用 37.5 m 作为工程折中：比真实 3.75~15 m 粗，但适合实时 demo 和 3D 显示

    // ---- PPI 扫描几何 ----
    // PPI（Plan Position Indicator）= 雷达绕垂直轴旋转，在固定仰角下扫描一圈；
    // 多个仰角组合即体积扫描(volume scan)，可重建三维污染场。
    // 默认按商用 PM 扫描 LiDAR 的扇区模式：低仰角、180° 扇区、2.5° 方位步进。
    std::vector<double> ppi_elevations_deg = {5.0}; ///< PPI/扇区扫描的仰角序列（°）
    double ppi_azimuth_start_deg = 0.0;    ///< 扇区起始方位角（°）
    double ppi_azimuth_stop_deg = 180.0;   ///< 扇区结束方位角（°），闭区间
    double ppi_azimuth_step_deg = 2.5;     ///< PPI 扫描的方位角步进（°），0..180 每 2.5°约 73 条视线
    double ppi_line_dwell_s = 5.0;         ///< 每条视线积分/驻留时间（s），20 Hz 下约 100 shots
    double ppi_scan_overhead_s = 0.0;      ///< 转台回扫、状态上报、调度余量（s）
    double pulse_repetition_hz = 20.0;     ///< 激光脉冲重复频率（Hz）

    // ---- 正演物理常数 ----
    // 这些参数直接进入 LiDAR 正演方程：P(r) = C × β(r) / r² × T(r)²
    double system_constant = 15000000.0;   ///< LiDAR 系统常数 C，综合发射能量、光学效率、接收口径等因素
    //   ↑ 决定回波信号的绝对量级；值越大，同一大气条件下接收到的信号越强
    double lidar_ratio_sr = 45.0;          ///< 气溶胶激光雷达比（单位 sr），即消光系数与后向散射系数之比
    //   ↑ 典型值 40~60 sr（城市污染气溶胶 ~45 sr）；用于反演时区分气溶胶与分子贡献
    double wavelength_nm = 355.0;          ///< PM 扫描 LiDAR 常用 UV 弹性通道
    double pulse_energy_mj = 8.0;          ///< 单脉冲能量均值（mJ）
    double pulse_energy_jitter = 0.04;     ///< 脉冲能量相对抖动
    double background_counts_mean = 120.0; ///< 白天户外背景计数均值
    double full_overlap_m = 200.0;         ///< 完整 overlap 距离（m）

    // ---- 推送节奏 ----
    double playback_time_scale = 100.0;    ///< 播放加速倍率；1.0 表示按真实 5s dwell 推送，100.0 表示 100 倍加速
    int inter_frame_delay_ms = 0;          ///< 兼容旧入口的固定帧间隔覆盖；0 表示按 dwell/playback_time_scale 自动计算
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
     * @brief 获取设备配置。
     */
    const SimDeviceConfig& config() const { return config_; }

    /**
     * @brief 获取指定时间步的地面观测帧。
     */
    lidar_protocol::Frame ground_frame(int step_index) const;

    /**
     * @brief 获取设备能力与运行状态帧。
     */
    lidar_protocol::Frame status_frame(int step_index) const;

    /**
     * @brief 获取设备健康、天气和链路遥测帧。
     */
    lidar_protocol::Frame telemetry_frame(int step_index) const;

private:
    SimDeviceConfig config_;
    lidar_core::SimulationConfig sim_config_;
    ///< 按时间步分组的原始射线
    std::vector<std::vector<lidar_core::LidarProfile>> profiles_by_step_;
    ///< 按时间步分组的地面观测
    std::vector<lidar_core::GroundMeasurement> ground_measurements_;
    ///< 所有时间戳列表
    std::vector<std::string> timestamps_;
    int next_sequence_id_ = 1;
    bool initialized_ = false;
};

} // namespace lidar_server
