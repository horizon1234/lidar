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

/**
 * @brief 站点 PM 标定系数；只有它和接收机标定均 valid 时才允许输出定量 PM。
 *
 * 线性关系为 PM = max(0, intercept + slope * dry_extinction)，其中干消光单位 km^-1。
 * 系数必须来自目标站点与参考颗粒物仪器共址数据，不能跨站点直接复用。
 */
struct PmCalibrationModel {
    bool valid = false;                    ///< 是否已加载经过实测验证的站点标定模型。
    std::string calibration_id;            ///< 标定文件或标定批次的唯一标识。
    std::string valid_from;                ///< 标定生效时间，用于结果追溯。
    double pm25_intercept_ugm3 = 0.0;      ///< PM2.5 线性模型截距（微克/立方米）。
    double pm25_slope_ugm3_per_km = 0.0;  ///< 干消光到 PM2.5 的斜率。
    double pm10_intercept_ugm3 = 0.0;      ///< PM10 线性模型截距（微克/立方米）。
    double pm10_slope_ugm3_per_km = 0.0;  ///< 干消光到 PM10 的斜率。
};

/**
 * @brief 接收机非线性、近场有效区和通道校正参数。
 *
 * 这些参数必须与设备序列号、固件版本、探测模式和 signal_unit 绑定。默认值仅用于当前
 * 公开规格仿真器，所以 valid 默认为 false。实机加载标定后，帧中的探测模式/计数单位若
 * 与本模型冲突，处理器会拒绝整条射线，而不是尝试隐式换算。
 */
struct ReceiverCalibrationModel {
    bool valid = false;                    ///< 是否来自目标设备的实测标定。
    std::string calibration_id = "ylj5-public-spec-assumption-v1";
    std::string detector_mode = "photon_counting"; ///< photon_counting 或 analog。
    std::string signal_unit = "mean_counts_per_pulse"; ///< 适配器归一化后的计数单位。
    double range_zero_offset_m = 0.0;      ///< 从协议距离轴中扣除的触发/电缆延迟偏移。
    double minimum_valid_range_m = 3.75;  ///< 触发瞬态或几何原因导致的最小有效距离。
    double minimum_retrieval_overlap = 0.10; ///< 低于该 overlap 时禁止反演。
    double minimum_quantitative_overlap = 0.90; ///< 低于该值时禁止 PM/热点等定量产品。
    double saturation_counts = 2000000.0; ///< 当前计数单位下的硬饱和上限。
    double saturation_guard_fraction = 0.98; ///< 接近饱和上限时提前标记无效。
    double dead_time_loss_per_count = 0.0000025; ///< m=n/(1+a*n) 中的 a；反校正 n=m/(1-a*m)。
    double maximum_dead_time_occupancy = 0.80; ///< 超过该占用率时不再尝试反校正。
    std::vector<double> afterpulse_kernel = {0.02}; ///< 前序距离门泄漏卷积核，必须由暗帧标定替换。
};

/** @brief 实时设备处理参数；同时控制 L1、L2、定量门控和热点检测。 */
struct ProcessorConfig {
    lidar_core::RetrievalConfig retrieval; ///< Fernald/Klett 反演参数。
    lidar_core::HumidityConfig humidity;   ///< 吸湿增长修正参数。
    lidar_core::HotspotConfig hotspot;     ///< 已标定 PM 热点检测参数。
    PmCalibrationModel pm_calibration;     ///< 当前站点定量 PM 标定模型。
    ReceiverCalibrationModel receiver_calibration; ///< 接收机与近场标定。
    double minimum_channel_snr = 3.0;      ///< 通道参与拼接所需的最低信噪比。
    double glue_start_range_m = 75.0;      ///< 近远场 smoothstep 权重开始由 near 向 far 过渡。
    double glue_stop_range_m = 300.0;      ///< 权重过渡结束；超过该距离默认使用 far。
    double background_tail_fraction = 0.05; ///< 无独立背景窗时使用的远端尾段比例。
    double background_minimum_range_m = 15000.0; ///< 默认只在远端背景候选区拟合基线。
    int minimum_background_bins = 16;      ///< 稳健背景拟合所需的最少距离门。
    int maximum_background_bins = 256;     ///< 背景拟合的最大尾段长度。
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
    bool receiver_calibrated = false;                   ///< 接收机校正是否来自实测标定。
    std::string receiver_calibration_id;                ///< 当前接收机标定批次。
    int valid_bin_count = 0;                            ///< 可用于光学反演的距离门总数。
    int masked_bin_count = 0;                           ///< 被逐 bin QC 排除的距离门总数。
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

    /** @brief 先封口当前周期，再替换后续射线使用的 PM 标定模型。 */
    void set_pm_calibration(const PmCalibrationModel& calibration);

    /** @brief 先封口当前周期，再替换后续射线使用的接收机标定模型。 */
    void set_receiver_calibration(const ReceiverCalibrationModel& calibration);

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
};

} // namespace lidar_client
