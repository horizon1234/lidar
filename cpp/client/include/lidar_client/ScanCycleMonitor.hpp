#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"
#include "lidar_protocol/Frame.hpp"

namespace lidar_client {

struct ScanCycleSummary {
    std::string scan_cycle_id; ///< 设备提供的扫描周期唯一标识。
    std::string timestamp; ///< 当前扫描周期的统一时间戳。
    int expected_rays = 0; ///< 设备声明的本周期应有射线数。
    int received_rays = 0; ///< 按射线索引去重后的实收数量。
    int duplicate_rays = 0; ///< 本周期收到的重复射线数量。
    std::vector<int> missing_ray_indices; ///< 周期封口时仍未收到的射线索引。
    bool complete = false; ///< 是否数量完整且没有重复或缺失射线。
};

class ScanCycleMonitor {
public:
    void observe_frame(const lidar_protocol::Frame& frame);

    /** @brief 返回指定时间戳的周期质量摘要，并释放该周期的缓存状态。 */
    ScanCycleSummary take_summary_for_timestamp(const std::string& timestamp);

    /** @brief 连接切换时清空未完成周期，防止跨连接累计重复帧。 */
    void reset();

private:
    struct CycleState {
        std::string scan_cycle_id; ///< 尚未封口周期的设备标识。
        std::string timestamp; ///< 用于与处理结果关联的周期时间戳。
        int expected_rays = 0; ///< 最近一帧声明的周期射线总数。
        int duplicate_rays = 0; ///< 已发现的重复射线累计数。
        std::set<int> received_indices; ///< 已接收且去重后的射线索引集合。
    };

    std::map<std::string, CycleState> cycles_by_id_; ///< 尚未封口的扫描周期及已收射线索引。
};

} // namespace lidar_client
