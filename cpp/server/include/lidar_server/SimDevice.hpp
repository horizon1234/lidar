/**
 * @file SimDevice.hpp
 * @brief 按公开规格惰性生成数据的 YLJ5 / AGHJ-I-LIDAR(MPL) 仿真设备。
 */
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"
#include "lidar_protocol/Frame.hpp"
#include "lidar_server/Ylj5DeviceSpec.hpp"

namespace lidar_server {

enum class DeviceRunState {
    booting,  ///< 正在初始化和校验配置。
    ready,    ///< 初始化完成，等待启动命令。
    scanning, ///< 正在生成或发送扫描周期。
    paused,   ///< 暂停发送，可由 start/resume 恢复。
    stopped,  ///< 已停止发送，可由 start 重新启动。
    fault,    ///< 初始化或周期生成失败，需要检查 fault_reason。
};

/** @brief 将设备运行状态转换为稳定的协议字符串。 */
std::string device_run_state_to_string(DeviceRunState state);

/**
 * @brief YLJ5 公开规格仿真设备。
 *
 * 设备按需生成单个扫描周期，不预生成整日战役。默认距离轴包含 5334 个距离门，
 * 若缓存 180 个周期会占用数十 GB 内存；逐周期、逐帧接口把峰值限制在单周期规模。
 */
class SimDevice {
public:
    /** @brief 构造设备并立即执行公开规格校验。 */
    explicit SimDevice(const SimDeviceConfig& config = {});

    /** @brief 重置设备状态、校验配置，并根据 auto_start 进入就绪或扫描状态。 */
    void init();

    /**
     * @brief 生成一个周期并把全部帧收集到内存。
     * @note 主要用于测试和嵌入式调用；正式服务器应使用 stream_scan_cycle 降低内存峰值。
     */
    std::vector<lidar_protocol::Frame> produce_scan_cycle(int step_index);

    /**
     * @brief 生成一个周期，并在每帧完成后立即交给回调处理。
     * @param step_index 周期索引，范围为 [0, total_steps)。
     * @param sink 帧消费回调；返回 false 会中止当前周期。
     * @return 整个周期是否完整生成并被回调接受。
     */
    bool stream_scan_cycle(
        int step_index,
        const std::function<bool(lidar_protocol::Frame&&)>& sink);

    /** @brief 返回当前战役的周期总数；初始化失败时返回 0。 */
    int total_steps() const;

    /** @brief 返回当前站点信息副本。 */
    lidar_core::SiteInfo site_info() const;
    /** @brief 在线程锁保护下返回完整设备配置副本。 */
    SimDeviceConfig config() const;

    /** @brief 判断设备是否已初始化且处于 scanning 状态。 */
    bool is_streaming() const;
    /** @brief 返回当前设备运行状态。 */
    DeviceRunState run_state() const;

    /** @brief 执行仿真控制命令并返回结构化 command_result 帧。 */
    lidar_protocol::Frame handle_command(const lidar_core::Json& command);

    /** @brief 构造设备能力、公开规格和校准状态帧。 */
    lidar_protocol::Frame status_frame(int step_index) const;
    /** @brief 构造明确标为合成值的设备遥测帧。 */
    lidar_protocol::Frame telemetry_frame(int step_index) const;

private:
    /** @brief 把设备层嵌套配置映射成单周期核心正演配置。 */
    lidar_core::PipelineConfig pipeline_config_for_cycle(
        const SimDeviceConfig& config,
        int step_index) const;
    /** @brief 根据战役起点和周期索引生成统一时间戳。 */
    std::string timestamp_for_step(int step_index) const;
    /** @brief 构造同步相机能力元数据；当前不伪造实机图像。 */
    lidar_protocol::Frame camera_frame(
        int step_index,
        const std::string& timestamp,
        const std::vector<lidar_core::LidarProfile>& profiles) const;
    /** @brief 构造基于本周期合成回波的未标定快速产品摘要。 */
    lidar_protocol::Frame product_frame(
        int step_index,
        const std::string& timestamp,
        const std::vector<lidar_core::LidarProfile>& profiles) const;

    mutable std::mutex mutex_; ///< 保护配置、运行状态、故障信息和最近地面观测。
    SimDeviceConfig config_;  ///< 当前生效配置；命令修改时先复制校验，再整体替换。
    DeviceRunState run_state_ = DeviceRunState::booting; ///< 当前运行状态。
    std::string fault_reason_; ///< 最近一次初始化或生成失败原因；正常状态为空。
    std::chrono::system_clock::time_point campaign_start_time_; ///< 当前战役的实时时间起点。
    lidar_core::GroundMeasurement last_ground_measurement_; ///< 最近周期生成的合成地面观测。
    mutable std::atomic<int> next_sequence_id_{1}; ///< 跨线程递增的协议帧序号。
    bool initialized_ = false; ///< 公开规格校验是否通过且设备是否完成初始化。
};

} // namespace lidar_server
