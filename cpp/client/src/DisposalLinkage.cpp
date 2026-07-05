/**
 * @file disposal_linkage.cpp
 * @brief 处置联动管理器实现。
 */
#include "lidar_client/DisposalLinkage.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace lidar_client {

// =========================================================================
// 辅助函数
// =========================================================================

std::string device_type_to_string(DeviceType type) {
    switch (type) {
        case DeviceType::water_cannon:    return "water_cannon";
        case DeviceType::mist_fogger:     return "mist_fogger";
        case DeviceType::sprinkler:       return "sprinkler";
        case DeviceType::dust_suppressor: return "dust_suppressor";
    }
    return "unknown";
}

std::string device_state_to_string(DeviceState state) {
    switch (state) {
        case DeviceState::idle:       return "idle";
        case DeviceState::active:     return "active";
        case DeviceState::faulted:    return "faulted";
        case DeviceState::maintenance:return "maintenance";
    }
    return "unknown";
}

// =========================================================================
// DisposalLinkage
// =========================================================================

DisposalLinkage::DisposalLinkage(const LinkageConfig& config)
    : config_(config)
{
}

void DisposalLinkage::register_device(const DisposalDevice& device) {
    devices_.push_back(device);
}

void DisposalLinkage::register_default_devices(const std::vector<double>& center_enu,
                                                double radius_m,
                                                int count) {
    double cx = (center_enu.size() >= 1) ? center_enu[0] : 0.0;
    double cy = (center_enu.size() >= 2) ? center_enu[1] : 0.0;
    double cz = (center_enu.size() >= 3) ? center_enu[2] : 0.0;

    DeviceType types[] = {
        DeviceType::water_cannon,
        DeviceType::mist_fogger,
        DeviceType::sprinkler,
        DeviceType::dust_suppressor
    };

    for (int i = 0; i < count; ++i) {
        double angle = 2.0 * M_PI * i / count;
        DisposalDevice dev;
        dev.device_id = "DEV-" + std::to_string(i + 1);
        std::ostringstream name;
        name << "Device " << (i + 1) << " (" << device_type_to_string(types[i % 4]) << ")";
        dev.name = name.str();
        dev.type = types[i % 4];
        dev.position_enu_m = {cx + radius_m * std::cos(angle),
                              cy + radius_m * std::sin(angle),
                              cz};
        dev.effective_range_m = 500.0 + i * 100.0;
        dev.flow_rate_lpm = 20.0 + i * 10.0;
        dev.state = DeviceState::idle;
        devices_.push_back(dev);
    }
}

double DisposalLinkage::enu_distance(const std::vector<double>& a,
                                      const std::vector<double>& b) {
    if (a.empty() || b.empty()) return 1e18;
    double dx = (a.size() >= 1 && b.size() >= 1) ? a[0] - b[0] : 0;
    double dy = (a.size() >= 2 && b.size() >= 2) ? a[1] - b[1] : 0;
    double dz = (a.size() >= 3 && b.size() >= 3) ? a[2] - b[2] : 0;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string DisposalLinkage::find_best_device(
    const std::vector<double>& target_enu) const {
    double best_dist = 1e18;
    std::string best_id;
    for (const auto& dev : devices_) {
        if (dev.state != DeviceState::idle) continue;
        double d = enu_distance(dev.position_enu_m, target_enu);
        if (d < dev.effective_range_m && d < best_dist) {
            best_dist = d;
            best_id = dev.device_id;
        }
    }
    return best_id;
}

std::string DisposalLinkage::generate_task_id() {
    std::ostringstream oss;
    oss << "TASK-" << std::setfill('0') << std::setw(4) << next_task_id_++;
    return oss.str();
}

int DisposalLinkage::on_event_activated(const std::string& event_id,
                                         const std::string& timestamp,
                                         const std::vector<double>& centroid_enu,
                                         double peak_pm25) {
    if (!config_.auto_trigger) {
        return 0;
    }
    if (peak_pm25 < config_.trigger_threshold_ugm3) {
        return 0;
    }

    std::string dev_id = find_best_device(centroid_enu);
    if (dev_id.empty()) {
        return 0;
    }

    std::string task_id = start_task(event_id, dev_id, timestamp, peak_pm25,
                                     "auto-triggered: PM2.5=" +
                                         std::to_string(peak_pm25) +
                                         " >= " +
                                         std::to_string(config_.trigger_threshold_ugm3));
    return task_id.empty() ? 0 : 1;
}

int DisposalLinkage::on_event_resolved(const std::string& event_id,
                                        const std::string& timestamp,
                                        double final_pm25) {
    if (!config_.stop_on_resolve) return 0;

    int stopped = 0;
    for (auto& task : tasks_) {
        if (task.event_id == event_id && task.is_active) {
            task.is_active = false;
            task.end_timestamp = timestamp;
            task.peak_pm25_at_end = final_pm25;
            task.reason = "event resolved, auto-stopped";

            // 释放设备
            for (auto& dev : devices_) {
                if (dev.device_id == task.device_id && dev.state == DeviceState::active) {
                    dev.state = DeviceState::idle;
                }
            }
            ++stopped;
        }
    }
    return stopped;
}

std::string DisposalLinkage::start_task(const std::string& event_id,
                                         const std::string& device_id,
                                         const std::string& timestamp,
                                         double peak_pm25,
                                         const std::string& reason) {
    // 检查设备是否存在且空闲
    DisposalDevice* dev_ptr = nullptr;
    for (auto& dev : devices_) {
        if (dev.device_id == device_id) {
            dev_ptr = &dev;
            break;
        }
    }
    if (!dev_ptr) return "";
    if (dev_ptr->state != DeviceState::idle) return "";

    DisposalTask task;
    task.task_id = generate_task_id();
    task.event_id = event_id;
    task.device_id = device_id;
    task.start_timestamp = timestamp;
    task.peak_pm25_at_start = peak_pm25;
    task.is_active = true;
    task.reason = reason;

    tasks_.push_back(std::move(task));
    dev_ptr->state = DeviceState::active;

    return tasks_.back().task_id;
}

bool DisposalLinkage::stop_task(const std::string& task_id,
                                 const std::string& timestamp,
                                 double final_pm25,
                                 const std::string& reason) {
    for (auto& task : tasks_) {
        if (task.task_id == task_id && task.is_active) {
            task.is_active = false;
            task.end_timestamp = timestamp;
            task.peak_pm25_at_end = final_pm25;
            task.reason = reason;

            // 释放设备
            for (auto& dev : devices_) {
                if (dev.device_id == task.device_id && dev.state == DeviceState::active) {
                    dev.state = DeviceState::idle;
                }
            }
            return true;
        }
    }
    return false;
}

std::vector<DisposalTask> DisposalLinkage::active_tasks() const {
    std::vector<DisposalTask> result;
    for (const auto& task : tasks_) {
        if (task.is_active) {
            result.push_back(task);
        }
    }
    return result;
}

std::vector<std::string> DisposalLinkage::recommend_devices(
    const std::vector<double>& centroid_enu,
    double peak_pm25) const {

    struct DeviceScore {
        std::string id;
        double distance;
    };
    std::vector<DeviceScore> scores;

    for (const auto& dev : devices_) {
        double d = enu_distance(dev.position_enu_m, centroid_enu);
        if (d < dev.effective_range_m) {
            scores.push_back({dev.device_id, d});
        }
    }
    std::sort(scores.begin(), scores.end(),
              [](const DeviceScore& a, const DeviceScore& b) {
                  return a.distance < b.distance;
              });

    std::vector<std::string> result;
    result.reserve(scores.size());
    for (const auto& s : scores) {
        result.push_back(s.id);
    }
    return result;
}

lidar_core::Json DisposalLinkage::to_json() const {
    using Json = lidar_core::Json;
    Json obj{Json::Object{}};

    // 设备列表
    {
        Json::Array dev_arr;
        dev_arr.reserve(devices_.size());
        for (const auto& dev : devices_) {
            Json d{Json::Object{}};
            d["device_id"] = Json(dev.device_id);
            d["name"] = Json(dev.name);
            d["type"] = Json(device_type_to_string(dev.type));
            d["state"] = Json(device_state_to_string(dev.state));
            d["effective_range_m"] = Json(dev.effective_range_m);
            d["flow_rate_lpm"] = Json(dev.flow_rate_lpm);

            Json::Array pos;
            pos.reserve(dev.position_enu_m.size());
            for (double v : dev.position_enu_m) {
                pos.emplace_back(v);
            }
            d["position_enu_m"] = Json(std::move(pos));
            dev_arr.emplace_back(std::move(d));
        }
        obj["devices"] = Json(std::move(dev_arr));
    }

    // 任务列表
    obj["tasks"] = tasks_to_json();

    // 统计
    Json stats{Json::Object{}};
    stats["total_devices"] = Json(static_cast<double>(devices_.size()));
    int active_count = 0;
    for (const auto& dev : devices_) {
        if (dev.state == DeviceState::active) ++active_count;
    }
    stats["active_devices"] = Json(static_cast<double>(active_count));
    stats["total_tasks"] = Json(static_cast<double>(tasks_.size()));
    int active_tasks = 0;
    for (const auto& t : tasks_) {
        if (t.is_active) ++active_tasks;
    }
    stats["active_tasks"] = Json(static_cast<double>(active_tasks));
    obj["statistics"] = std::move(stats);

    return obj;
}

lidar_core::Json DisposalLinkage::tasks_to_json() const {
    using Json = lidar_core::Json;
    Json::Array arr;
    arr.reserve(tasks_.size());
    for (const auto& task : tasks_) {
        Json t{Json::Object{}};
        t["task_id"] = Json(task.task_id);
        t["event_id"] = Json(task.event_id);
        t["device_id"] = Json(task.device_id);
        t["start_timestamp"] = Json(task.start_timestamp);
        t["end_timestamp"] = Json(task.end_timestamp);
        t["peak_pm25_at_start"] = Json(task.peak_pm25_at_start);
        t["peak_pm25_at_end"] = Json(task.peak_pm25_at_end);
        t["is_active"] = Json(task.is_active);
        t["reason"] = Json(task.reason);

        // 计算消减效果
        double reduction = 0.0;
        if (task.peak_pm25_at_start > 0) {
            reduction = (task.peak_pm25_at_start - task.peak_pm25_at_end) /
                        task.peak_pm25_at_start * 100.0;
        }
        t["pm25_reduction_pct"] = Json(reduction);
        arr.emplace_back(std::move(t));
    }
    return Json(std::move(arr));
}

} // namespace lidar_client
