#pragma once

#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"
#include "lidar_protocol/Frame.hpp"

namespace lidar_client {

/**
 * @brief 客户端持有的设备能力与最新遥测快照。
 *
 * `status` 帧更新设备静态能力，`telemetry` 帧只增量更新运行量；未出现在新帧中的字段
 * 会保留旧值，避免高频遥测把低频能力信息清空。
 */
struct DeviceStatusSnapshot {
    std::string site_id;             ///< 设备部署站点唯一标识。
    std::string site_name;           ///< 站点可读名称。
    std::string manufacturer;        ///< 设备制造商名称。
    std::string product_name;        ///< 产品名称。
    std::string device_model;        ///< 官网产品型号，如 YLJ5。
    std::string regulatory_model;    ///< 采购或监管型号，如 AGHJ-I-LIDAR(MPL)。
    std::string device_state;        ///< 当前运行状态：ready/scanning/paused/stopped/fault。
    std::string vendor_profile;      ///< 当前仿真数据映射名称，不代表厂商私有格式。
    std::string protocol_version;    ///< 当前项目 JSONL 仿真协议版本。
    std::string calibration_status;  ///< 光电参数标定状态。
    std::string scan_program_mode;   ///< 服务端当前扫描调度程序名称。
    std::string active_scan_pattern; ///< 当前周期水平扫描或锥形扫描模式。
    std::string elevation_cycle_policy; ///< 仰角调度策略，如 round_robin。
    std::string specification_basis; ///< 公开规格证据索引。
    std::string ingress_protection;  ///< 整机防护等级，如 IP66。
    std::vector<std::string> receiver_channels; ///< 设备声明的物理接收通道 ID。
    std::vector<double> ppi_elevations_deg;     ///< 当前水平扫描使用的仰角列表（度）。
    std::vector<double> scheduled_elevations_deg; ///< 完整逐周期仰角调度队列（度）。
    double active_ppi_elevation_deg = 0.0;      ///< 当前周期实际方位扫描仰角（度）。
    double ppi_azimuth_start_deg = 0.0;         ///< 水平扫描起始方位角（度）。
    double ppi_azimuth_stop_deg = 360.0;        ///< 水平扫描终止方位角（度）。
    double ppi_azimuth_step_deg = 0.0;          ///< 相邻水平射线方位角步进（度）。
    double ppi_line_dwell_s = 0.0;              ///< 每条水平射线的积分驻留时间（s）。
    double ppi_step_overhead_s = 0.0;           ///< 相邻射线间的云台运动和稳定耗时（s）。
    double stare_dwell_s = 0.0;                 ///< 垂直凝视积分时间（s）。
    double ppi_scan_cycle_s = 0.0;              ///< 水平扫描周期总时长（s）。
    double full_scan_cycle_s = 0.0;             ///< 包含垂直凝视的完整周期时长（s）。
    double playback_time_scale = 1.0;           ///< 服务端播放等待倍率。
    double pulse_repetition_hz = 0.0;           ///< 仿真假设的激光脉冲重复频率（Hz）。
    double wavelength_nm = 0.0;                 ///< 工作波长（nm）。
    double range_resolution_m = 0.0;            ///< 距离分辨率（m）。
    double maximum_range_m = 0.0;               ///< 最大距离轴范围（m）。
    double near_telescope_aperture_mm = 0.0;    ///< 近场望远镜口径（mm）。
    double far_telescope_aperture_mm = 0.0;     ///< 远场望远镜口径（mm）。
    double electronics_temperature_c = 0.0;     ///< 最新合成电子学温度（摄氏度）。
    double relative_humidity = 0.0;             ///< 最新合成环境相对湿度（0~1）。
    double gimbal_azimuth_deg = 0.0;            ///< 最新云台方位角遥测（度）。
    double gimbal_elevation_deg = 0.0;          ///< 最新云台俯仰角遥测（度）。
    int elevation_schedule_index = 0;           ///< 当前周期在仰角队列中的索引。
    int integrated_pulses_per_line = 0;         ///< 每条水平射线的积分脉冲数。
    int total_steps = -1;                       ///< 一轮仿真战役的周期总数；-1 表示未知。
    bool vendor_wire_protocol_known = false;    ///< 是否已掌握厂家私有线协议。
    bool polarization_channel = false;          ///< 是否具备偏振通道。
    bool camera_online = false;                 ///< 同步相机能力是否在线。
    bool valid = false;                         ///< 是否至少成功吸收过一条状态或遥测帧。
};

/** @brief 将低频状态帧和高频遥测帧合并成稳定客户端快照。 */
class DeviceStatusModel {
public:
    /** @brief 增量吸收 status 或 telemetry 帧；其他帧返回 false。 */
    bool update_from_frame(const lidar_protocol::Frame& frame);

    /** @brief 返回当前只读快照。 */
    const DeviceStatusSnapshot& snapshot() const { return snapshot_; }

private:
    DeviceStatusSnapshot snapshot_; ///< 按字段增量更新的最新设备状态快照。
};

} // namespace lidar_client
