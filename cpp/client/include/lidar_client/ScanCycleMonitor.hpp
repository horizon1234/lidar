#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"
#include "lidar_protocol/Frame.hpp"

namespace lidar_client {

struct ScanCycleSummary {
    std::string scan_cycle_id;
    std::string timestamp;
    int expected_rays = 0;
    int received_rays = 0;
    int duplicate_rays = 0;
    std::vector<int> missing_ray_indices;
    bool complete = false;
};

class ScanCycleMonitor {
public:
    void observe_frame(const lidar_protocol::Frame& frame);

    ScanCycleSummary summary_for_timestamp(const std::string& timestamp) const;

    lidar_core::Json summary_to_json(const ScanCycleSummary& summary) const;

private:
    struct CycleState {
        std::string scan_cycle_id;
        std::string timestamp;
        int expected_rays = 0;
        int duplicate_rays = 0;
        std::set<int> received_indices;
    };

    std::map<std::string, CycleState> cycles_by_id_;
};

} // namespace lidar_client
