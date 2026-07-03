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

#include "lidar_core/lidar_core.hpp"
#include "lidar_protocol/frame.hpp"

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
    std::string site_name = "仿真站点";     ///< 站点可读名称，推送给客户端作为展示用
    double latitude_deg = 39.9042;         ///< 站点纬度（°），北京天安门附近，用于地理坐标计算
    double longitude_deg = 116.4074;       ///< 站点经度（°），同上

    // ---- 仿真可复现性 ----
    int seed = 7;                          ///< 随机数种子，相同种子产生完全相同的仿真数据（可复现性）

    // ---- 时间维度 ----
    int time_steps = 18;                   ///< 仿真总时间步数；每步代表一次完整的扫描周期
    int minutes_per_step = 20;             ///< 相邻时间步之间的真实时间间隔（分钟），决定时间序列分辨率

    // ---- 距离分辨率（Range Bin）----
    int range_bin_count = 60;              ///< 每条射线的距离 bin 数量，即一条射线被分成多少段
    double range_bin_m = 100.0;            ///< 相邻距离 bin 的间距（米）；总探测距离 = count × m = 6000m
    //   ↑ 探测距离 = range_bin_count × range_bin_m = 60 × 100 = 6.0 km（商用尺度）

    // ---- PPI 扫描几何 ----
    // PPI（Plan Position Indicator）= 雷达绕垂直轴旋转，在固定仰角下扫描一圈；
    // 多个仰角组合即体积扫描(volume scan)，可重建三维污染场。
    // 默认 6 层 {5,15,30,45,60,75}°：接近商用气溶胶扫描雷达的常规体积扫描方案
    // （如 Raymetrics 等），配套量程 6 km、边界层标高 1200 m、烟羽高度 280/380 m。
    std::vector<double> ppi_elevations_deg = {5.0, 15.0, 30.0, 45.0, 60.0, 75.0}; ///< PPI 体积扫描的仰角序列（°）
    double ppi_azimuth_step_deg = 30.0;    ///< PPI 扫描的方位角步进（°），每 30°一条射线 → 一圈 12 条
    //   ↑ 每层方位射线数 = 360 / ppi_azimuth_step_deg = 360 / 30 = 12 条

    // ---- 正演物理常数 ----
    // 这些参数直接进入 LiDAR 正演方程：P(r) = C × β(r) / r² × T(r)²
    double system_constant = 260000000.0;  ///< LiDAR 系统常数 C，综合发射能量、光学效率、接收口径等因素
    //   ↑ 决定回波信号的绝对量级；值越大，同一大气条件下接收到的信号越强
    double lidar_ratio_sr = 45.0;          ///< 气溶胶激光雷达比（单位 sr），即消光系数与后向散射系数之比
    //   ↑ 典型值 40~60 sr（城市污染气溶胶 ~45 sr）；用于反演时区分气溶胶与分子贡献

    // ---- 推送节奏 ----
    int inter_frame_delay_ms = 50;         ///< 每帧之间的发送间隔（毫秒），0 表示立即发送不延迟
    //   ↑ 模拟真实雷达的扫描节奏，避免一次性灌满客户端缓冲区
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
