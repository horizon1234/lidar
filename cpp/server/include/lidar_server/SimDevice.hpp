/**
 * @file sim_device.hpp
 * @brief 仿真 LiDAR 设备：按配置生成周期性扫描帧流。
 *
 * SimDevice 在初始化时一次性生成约 24 小时的仿真战役数据（通过调用
 * lidar_core::run_end_to_end 触发内部前向仿真），缓存所有原始射线。
 * 服务端随后按采集节奏循环调用 produce_scan_cycle()，模拟设备 24/7 连续上报。
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
 * 对应一个位于北京的假想站点，时间跨度约 24 小时（72 步 × 20 分钟）。
 */
struct SimDeviceConfig {
    // ---- 站点标识 ----
    std::string site_id = "sim-site-01";   ///< 站点唯一标识符，用于协议帧中的 site_id 字段
    std::string site_name = "Field PM LiDAR Demo Site"; ///< 站点可读名称，推送给客户端作为展示用
    double latitude_deg = 39.9042;         ///< 站点纬度（°），北京天安门附近，用于地理坐标计算
    double longitude_deg = 116.4074;       ///< 站点经度（°），同上
    std::string instrument_preset = "field_scanning_pm_lidar"; ///< 工地/城市污染扫描 LiDAR 预设
    // application_mode 可选值：
    // - construction_site：固定工地/厂区扬尘场景，近地面施工源更强，适合模拟围挡、料场、道路扬尘。
    // - urban_grid：城市网格化固定站场景，交通/区域输送背景更强，适合模拟多站点城市污染巡查。
    // - mobile_mapping：走航车载场景，平台按 vehicle_speed_ms 前进，并叠加近源移动扬尘羽流。
    std::string application_mode = "construction_site";
    std::string vendor_profile = "raymetrics_pmeye_like";      ///< 公开格式映射 profile

    // ---- 仿真可复现性 ----
    int seed = 7;                          ///< 随机数种子，相同种子产生完全相同的仿真数据（可复现性）

    // ---- 时间维度 ----
    // 真实设备会 24/7 连续发射与上报；这里的时间维度只定义预生成缓存中“不重复环境状态”的时间轴。
    // 每个 step 代表一次完整采集周期（stare + PPI 扇区扫描），不是单个激光脉冲，也不是单个 TCP 帧。
    // 默认 72 × 20min 约等于 24h；20min 来自 30s stare + 3 层 PPI × 72 方位 × (5s dwell + 0.25s 转动稳定) + 15s 周期余量。
    int time_steps = 72;                   ///< 预生成扫描周期数量；服务端播完后循环，模拟全天连续运行
    int minutes_per_step = 20;             ///< 相邻扫描周期时间戳间隔；应接近一次完整体扫的真实采集耗时

    // ---- 距离分辨率（Range Bin）----
    int range_bin_count = 160;             ///< 每条射线的距离 bin 数量，即一条射线被分成多少段
    double range_bin_m = 37.5;             ///< 相邻距离 bin 的间距（米）；总探测距离 = count × m = 6000m
    //   ↑ 用 37.5 m 作为工程折中：比真实 3.75~15 m 粗，但适合实时 demo 和 3D 显示

    // ---- PPI 扫描几何 ----
    // PPI（Plan Position Indicator）= 雷达绕垂直轴旋转，在固定仰角下扫描一圈；
    // 多个仰角组合即体积扫描(volume scan)，可重建三维污染场。
    // 商用设备通常支持可编程扫描：PM/扬尘设备常做水平 360° 或目标扇区扫描；
    // 边界层/风场扫描多普勒设备常见 1°、5°、15°、35°、75° PPI，再穿插 RHI 或垂直凝视。
    // 默认采用工地/园区 PM 监测的紧凑三层：1° 捕捉近地面源，5° 覆盖低空输送，15° 提供烟羽抬升约束。
    std::vector<double> ppi_elevations_deg = {1.0, 5.0, 15.0}; ///< PPI/扇区扫描的仰角序列（°）
    double ppi_azimuth_start_deg = 0.0;    ///< 扇区起始方位角（°）
    double ppi_azimuth_stop_deg = 360.0;   ///< 扇区结束方位角（°）；360 表示完整一圈，内部不会重复 0°
    double ppi_azimuth_step_deg = 5.0;     ///< PPI 扫描的方位角步进（°），0..360 每 5°得到 72 条视线
    double ppi_line_dwell_s = 5.0;         ///< 每条视线积分/驻留时间（s），20 Hz 下约积分 100 个激光脉冲
    double ppi_step_overhead_s = 0.25;     ///< 相邻方位/仰角切换、转台稳定、编码器确认的单视线平均额外耗时（s）
    double ppi_scan_overhead_s = 15.0;     ///< 一轮体扫末尾回扫、状态上报、调度余量（s）
    // 这里的 PRF 是激光发射脉冲频率，不是应用层完整 profile 的上报频率。
    // Raymetrics PMeye-like UV PM 雷达可用 20 Hz 高能量低 PRF；Halo/StreamLine 等多普勒雷达常见 10~15 kHz 低能量高 PRF。
    double pulse_repetition_hz = 20.0;     ///< 激光脉冲重复频率（Hz）；每条 profile 通常由 dwell 内多次脉冲平均/积分得到

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
    double playback_time_scale = 100.0;    ///< 播放加速倍率；1.0 表示按真实采集耗时推送，100.0 表示 100 倍加速演示
    int inter_frame_delay_ms = 0;          ///< 兼容旧入口的固定帧间隔覆盖；0 表示按 dwell/playback_time_scale 自动计算
};

/**
 * @brief 仿真 LiDAR 设备。
 *
 * 工作模式：
 *   1. 构造时调用 init() 一次性生成全部仿真数据并缓存
 *   2. produce_scan_cycle(step) 按时间步索引返回该步的所有帧
 *   3. 客户端按步索引循环调用，实现全天连续数据推送；超过缓存末尾后从第 0 步重播
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
