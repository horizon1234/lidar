/**
 * @file FrameProcessor.hpp
 * @brief YLJ5 实时数据处理器：设备 L0 帧到质量受控 L2 产品。
 */
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"
#include "lidar_protocol/Frame.hpp"

namespace lidar_client {

/** @brief 站点 PM 标定系数；只有 valid=true 时才允许输出定量 PM。 */
struct PmCalibrationModel {
    bool valid = false;                    ///< 是否已加载经过实测验证的站点标定模型。
    std::string calibration_id;            ///< 标定文件或标定批次的唯一标识。
    std::string valid_from;                ///< 标定生效时间，用于结果追溯。
    double pm25_intercept_ugm3 = 0.0;      ///< PM2.5 线性模型截距（微克/立方米）。
    double pm25_slope_ugm3_per_km = 0.0;  ///< 干消光到 PM2.5 的斜率。
    double pm10_intercept_ugm3 = 0.0;      ///< PM10 线性模型截距（微克/立方米）。
    double pm10_slope_ugm3_per_km = 0.0;  ///< 干消光到 PM10 的斜率。
};

/** @brief 实时设备处理参数。 */
struct ProcessorConfig {
    lidar_core::RetrievalConfig retrieval; ///< Fernald/Klett 反演参数。
    lidar_core::HumidityConfig humidity;   ///< 吸湿增长修正参数。
    lidar_core::HotspotConfig hotspot;     ///< 已标定 PM 热点检测参数。
    PmCalibrationModel pm_calibration;     ///< 当前站点定量 PM 标定模型。
    double minimum_channel_snr = 3.0;      ///< 通道参与拼接所需的最低信噪比。
    double glue_start_range_m = 75.0;      ///< 近远场开始交叉拼接的距离（米）。
    double glue_stop_range_m = 300.0;      ///< 近远场结束交叉拼接的距离（米）。
    double minimum_overlap = 0.02;         ///< overlap 除法保护下限。
    int expected_ppi_azimuths = 180;       ///< 公开规格下单层扫描的预期方位射线数。
};

/** @brief 一个扫描周期完成后的不可变数据快照。 */
struct StepResult {
    std::string timestamp;                              ///< 扫描周期时间戳。
    std::string scan_pattern;                           ///< 水平、锥形或其他设备扫描模式。
    std::vector<lidar_core::ProcessedProfile> processed_profiles; ///< 周期内质量受控廓线。
    std::vector<lidar_core::GroundMeasurement> ground_measurements; ///< 同周期地面观测。
    std::vector<lidar_core::Hotspot> hotspots;          ///< 仅在 PM 已标定时生成的热点。
    std::vector<std::string> qc_flags;                  ///< 周期汇总质量标志。
    int raw_count = 0;                                  ///< 收到的原始射线数。
    int rejected_count = 0;                             ///< 因结构或数值问题拒绝的射线数。
    int ppi_count = 0;                                  ///< 可用于平面/锥形显示的射线数。
    int vertical_count = 0;                             ///< 垂直观测射线数。
    int elevation_layer_count = 0;                      ///< 周期内有效仰角层数。
    double mean_processing_latency_ms = 0.0;            ///< 单射线平均处理耗时（毫秒）。
    bool pm_calibrated = false;                         ///< 本周期 PM 是否来自有效站点标定。
    std::string calibration_id;                         ///< 当前使用的 PM 标定批次。
};

/**
 * @brief 无 UI、无线程状态的设备处理器。
 *
 * 该对象必须只在客户端工作线程内使用。网络线程把帧顺序交给 handle_frame，heartbeat
 * 或时间戳变化会封口当前周期并通过回调移动交付 StepResult。
 */
class FrameProcessor {
public:
    using StepCompleteCallback = std::function<void(StepResult)>;

    explicit FrameProcessor(ProcessorConfig config = {});

    /** @brief 设置周期完成回调；回调在调用 handle_frame 的工作线程执行。 */
    void set_step_complete_callback(StepCompleteCallback callback);

    /** @brief 顺序处理一条协议帧。 */
    void handle_frame(const lidar_protocol::Frame& frame);

    /** @brief 强制封口当前周期。 */
    void finalize_step();

    /** @brief 原子替换后续射线使用的 PM 标定模型。 */
    void set_pm_calibration(const PmCalibrationModel& calibration);

    /** @brief 返回已成功处理的射线总数。 */
    int total_processed() const { return total_processed_; }

    /** @brief 返回被质量门控拒绝的射线总数。 */
    int total_rejected() const { return total_rejected_; }

    /** @brief 返回已完成扫描周期数。 */
    int total_steps_completed() const { return total_steps_completed_; }

    /** @brief 返回最近状态帧提供的站点信息。 */
    const lidar_core::SiteInfo& site_info() const { return site_info_; }

private:
    /** @brief 执行一条真实设备射线的完整 L0-L2 处理链。 */
    lidar_core::ProcessedProfile process_device_profile(
        const lidar_protocol::Frame& frame,
        lidar_core::LidarProfile profile,
        std::vector<std::string>& qc_flags) const;

    ProcessorConfig config_;               ///< 当前处理与标定配置。
    StepCompleteCallback on_step_complete_; ///< 周期完成后的工作线程回调。
    std::string current_timestamp_;         ///< 当前正在聚合的周期时间戳。
    std::string current_scan_pattern_;      ///< 当前周期的设备级扫描模式。
    std::vector<lidar_core::ProcessedProfile> current_processed_; ///< 当前周期已处理射线。
    std::vector<lidar_core::GroundMeasurement> current_ground_;   ///< 当前周期地面观测。
    std::vector<std::string> current_qc_flags_; ///< 当前周期去重前的质量标志。
    int current_raw_count_ = 0;             ///< 当前周期收到的原始射线数。
    int current_rejected_count_ = 0;        ///< 当前周期被拒绝的射线数。
    lidar_core::SiteInfo site_info_;        ///< 状态帧维护的站点和海拔信息。
    int total_processed_ = 0;               ///< 进程启动后成功处理的射线累计数。
    int total_rejected_ = 0;                ///< 进程启动后拒绝的射线累计数。
    int total_steps_completed_ = 0;         ///< 进程启动后完成的周期累计数。
};

} // namespace lidar_client
