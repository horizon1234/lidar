/**
 * @file ScanCycleMonitor.hpp
 * @brief 扫描周期原始射线接收完整性监控。
 */
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"
#include "lidar_protocol/Frame.hpp"

namespace lidar_client {

/** @brief 一个扫描周期在传输层的射线接收质量摘要。 */
struct ScanCycleSummary {
    std::string scan_cycle_id; ///< 设备提供的扫描周期唯一标识。
    std::string timestamp; ///< 当前扫描周期的统一时间戳。
    int expected_rays = 0; ///< 设备声明的本周期应有射线数。
    int received_rays = 0; ///< 按射线索引去重后的实收数量。
    int duplicate_rays = 0; ///< 本周期收到的重复射线数量。
    std::vector<int> missing_ray_indices; ///< 周期封口时仍未收到的射线索引。
    bool complete = false; ///< 是否数量完整且没有重复或缺失射线。
};

/**
 * @brief 按设备周期统计原始射线是否收齐以及是否重复。
 *
 * `observe_frame()` 在协议帧成功解析后、科学算法执行前调用，因此本类回答的是“设备声明
 * 的射线是否都到达客户端”，而不是“每条射线是否通过科学处理”。它只消费带有
 * `scan_cycle_id`、`ray_index` 和 `rays_in_cycle` 的 `lidar_raw` 帧，并允许射线乱序到达。
 *
 * 本类不自行观察 `heartbeat` 或决定周期何时结束。`FrameProcessor` 封口并生成结果后，
 * 调用方以结果时间戳取走摘要；取走操作同时释放该周期状态。对象没有内部锁，应由一个
 * 工作线程独占，或由调用方提供同步。
 */
class ScanCycleMonitor {
public:
    /**
     * @brief 记录一条协议帧携带的周期 ID、预期射线数和射线索引。
     *
     * 非 `lidar_raw` 帧以及缺少 `scan_cycle_id` 的帧会被忽略。重复索引会单独累计，
     * 到达顺序不会影响统计结果。
     */
    void observe_frame(const lidar_protocol::Frame& frame);

    /**
     * @brief 返回指定时间戳的周期质量摘要，并释放该周期的缓存状态。
     * @return 找不到对应周期时返回各计数均为零、`complete=false` 的空摘要。
     */
    ScanCycleSummary take_summary_for_timestamp(const std::string& timestamp);

    /** @brief 连接切换时清空未完成周期，防止跨连接累计重复帧。 */
    void reset();

private:
    struct CycleState {
        std::string scan_cycle_id; ///< 尚未封口周期的设备标识。
        std::string timestamp; ///< 用于将传输统计与 StepResult 关联的周期时间戳。
        int expected_rays = 0; ///< 最近一帧声明的周期射线总数。
        int duplicate_rays = 0; ///< 已发现的重复射线累计数。
        std::set<int> received_indices; ///< 已接收且去重后的射线索引集合。
    };

    std::map<std::string, CycleState> cycles_by_id_; ///< 尚未封口的扫描周期及已收射线索引。
};

} // namespace lidar_client
