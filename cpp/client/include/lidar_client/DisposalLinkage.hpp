/**
 * @file disposal_linkage.hpp
 * @brief 处置联动模块：根据告警状态自动触发/停止处置设备（喷雾炮/雾炮）。
 *
 * 工作流程：
 * 1. 当告警进入 active 状态时，根据热点位置查找最近的可用处置设备
 * 2. 如果找到设备且在射程内，创建处置任务
 * 3. 当告警进入 resolved/dismissed 状态时，停止对应处置任务
 * 4. 记录完整的处置链日志
 */
#pragma once

#include <string>
#include <vector>
#include <memory>

#include "lidar_core/LidarCore.hpp"

namespace lidar_client {

/**
 * @brief 处置设备类型。
 */
enum class DeviceType {
    water_cannon,   ///< 喷雾炮
    mist_fogger,    ///< 雾炮
    sprinkler,      ///< 喷淋
    dust_suppressor ///< 抑尘剂
};

/**
 * @brief 设备状态。
 */
enum class DeviceState {
    idle,       ///< 空闲
    active,     ///< 运行中
    faulted,    ///< 故障
    maintenance ///< 维护中
};

/**
 * @brief 处置设备定义。
 */
struct DisposalDevice {
    std::string device_id;       ///< 设备唯一 ID
    std::string name;            ///< 显示名称
    DeviceType type;             ///< 设备类型
    std::vector<double> position_enu_m; ///< 设备安装位置 (E, N, U)
    double effective_range_m = 500.0;   ///< 有效射程（米）
    double flow_rate_lpm = 30.0;        ///< 流量（升/分钟）
    DeviceState state = DeviceState::idle; ///< 当前状态
};

/**
 * @brief 处置任务（一个事件可能对应多个设备的处置任务）。
 */
struct DisposalTask {
    std::string task_id;              ///< 任务唯一 ID
    std::string event_id;             ///< 关联的告警事件 ID
    std::string device_id;            ///< 执行设备 ID
    std::string start_timestamp;      ///< 开始时间
    std::string end_timestamp;        ///< 结束时间（空表示进行中）
    double peak_pm25_at_start = 0.0;  ///< 启动时的 PM2.5
    double peak_pm25_at_end = 0.0;    ///< 结束时的 PM2.5
    bool is_active = true;            ///< 是否进行中
    std::string reason;               ///< 启动/停止原因
};

/**
 * @brief 联动配置。
 */
struct LinkageConfig {
    bool auto_trigger = true;             ///< 自动触发（false=仅记录建议）
    double trigger_threshold_ugm3 = 50.0; ///< 自动触发阈值
    bool stop_on_resolve = true;          ///< 事件关闭后自动停止
};

/**
 * @brief 处置联动管理器。
 */
class DisposalLinkage {
public:
    /**
     * @brief 构造联动管理器。
     * @param config 配置
     */
    explicit DisposalLinkage(const LinkageConfig& config = {});

    /**
     * @brief 注册一个处置设备。
     */
    void register_device(const DisposalDevice& device);

    /**
     * @brief 注册默认设备（根据站点中心生成环形布局）。
     * @param center_enu 站点中心 ENU 坐标
     * @param radius_m 环形半径
     * @param count 设备数量
     */
    void register_default_devices(const std::vector<double>& center_enu,
                                  double radius_m = 800.0,
                                  int count = 4);

    /**
     * @brief 获取所有设备。
     */
    const std::vector<DisposalDevice>& devices() const { return devices_; }

    /**
     * @brief 获取所有任务。
     */
    const std::vector<DisposalTask>& tasks() const { return tasks_; }

    /**
     * @brief 获取进行中的任务。
     */
    std::vector<DisposalTask> active_tasks() const;

    /**
     * @brief 当告警事件激活时触发联动。
     * @param event_id 事件 ID
     * @param timestamp 时间戳
     * @param centroid_enu 热点质心 ENU
     * @param peak_pm25 峰值 PM2.5
     * @return 创建的处置任务数量
     */
    int on_event_activated(const std::string& event_id,
                           const std::string& timestamp,
                           const std::vector<double>& centroid_enu,
                           double peak_pm25);

    /**
     * @brief 当告警事件关闭时停止联动。
     * @param event_id 事件 ID
     * @param timestamp 时间戳
     * @param final_pm25 最终 PM2.5
     * @return 停止的任务数量
     */
    int on_event_resolved(const std::string& event_id,
                          const std::string& timestamp,
                          double final_pm25);

    /**
     * @brief 手动启动处置任务。
     */
    std::string start_task(const std::string& event_id,
                           const std::string& device_id,
                           const std::string& timestamp,
                           double peak_pm25,
                           const std::string& reason = "manual");

    /**
     * @brief 手动停止处置任务。
     */
    bool stop_task(const std::string& task_id,
                   const std::string& timestamp,
                   double final_pm25,
                   const std::string& reason = "manual");

    /**
     * @brief 获取设备的最佳处置建议。
     * @param centroid_enu 热点位置
     * @param peak_pm25 PM2.5
     * @return 建议的设备 ID 列表（按距离排序）
     */
    std::vector<std::string> recommend_devices(
        const std::vector<double>& centroid_enu,
        double peak_pm25) const;

    /**
     * @brief 序列化为 JSON。
     */
    lidar_core::Json to_json() const;

    /**
     * @brief 序列化任务列表为 JSON。
     */
    lidar_core::Json tasks_to_json() const;

private:
    LinkageConfig config_;
    std::vector<DisposalDevice> devices_;
    std::vector<DisposalTask> tasks_;
    int next_task_id_ = 1;

    /**
     * @brief 查找最近的空闲且在射程内的设备。
     */
    std::string find_best_device(const std::vector<double>& target_enu) const;

    /**
     * @brief 计算两点距离。
     */
    static double enu_distance(const std::vector<double>& a,
                               const std::vector<double>& b);

    /**
     * @brief 生成任务 ID。
     */
    std::string generate_task_id();
};

/**
 * @brief 设备类型转字符串。
 */
std::string device_type_to_string(DeviceType type);

/**
 * @brief 设备状态转字符串。
 */
std::string device_state_to_string(DeviceState state);

} // namespace lidar_client
