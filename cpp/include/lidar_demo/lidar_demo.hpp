/**
 * @file lidar_demo.hpp
 * @brief 大气激光雷达颗粒物监测系统 —— 公共 API 头文件
 *
 * 本头文件定义了整个 C++ 管线的对外接口，包含：
 *   - 轻量级 JSON 值模型（Json 类）及其读写函数；
 *   - 数据载体结构体（站点、LiDAR 剖面、地面观测、处理结果、热点等）；
 *   - 各阶段的配置结构体（仿真、反演、湿度校正、标定、热点检测、评估）；
 *   - 顶层管线的入口函数 run_end_to_end，以及仪表盘渲染、摘要构建、
 *     公开数据抓取等工具函数。
 *
 * 使用者只需包含本文件即可调用全部功能，所有内部实现细节都封装在
 * lidar_demo.cpp 的匿名命名空间中。
 */
#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace lidar_demo {

/**
 * @brief 轻量级 JSON 值模型
 *
 * 使用 std::variant 承载 null / bool / number(double) / string / array / object
 * 六种 JSON 值类型。提供类型查询、取值访问与键值查找等便捷接口，
 * 不依赖任何第三方 JSON 库。与配套的 parse_json / dump_json / read_json_file /
 * write_json_file 一起构成本项目自带的 JSON 引擎。
 */
struct Json {
    using array_type = std::vector<Json>;                  ///< JSON 数组类型别名
    using object_type = std::map<std::string, Json>;       ///< JSON 对象（键有序映射）类型别名
    using value_type = std::variant<
        std::nullptr_t, bool, double, std::string,
        array_type, object_type>;                          ///< 底层的标签联合体类型

    value_type value;   ///< 实际存储的 JSON 值

    /** @name 构造函数 */
    ///@{
    Json();                              ///< 构造一个 null 值（默认）
    Json(std::nullptr_t);                ///< 构造一个 null 值
    Json(bool input);                    ///< 构造一个布尔值
    Json(int input);                     ///< 构造一个数值（由 int 提升为 double）
    Json(double input);                  ///< 构造一个数值
    Json(const char* input);             ///< 构造一个字符串
    Json(const std::string& input);      ///< 构造一个字符串（拷贝）
    Json(std::string&& input);           ///< 构造一个字符串（移动）
    Json(const array_type& input);       ///< 构造一个数组（拷贝）
    Json(array_type&& input);            ///< 构造一个数组（移动）
    Json(const object_type& input);      ///< 构造一个对象（拷贝）
    Json(object_type&& input);           ///< 构造一个对象（移动）
    ///@}

    /** @name 类型查询 */
    ///@{
    bool is_null() const;                ///< 是否为 null
    bool is_bool() const;                ///< 是否为布尔值
    bool is_number() const;              ///< 是否为数值
    bool is_string() const;              ///< 是否为字符串
    bool is_array() const;               ///< 是否为数组
    bool is_object() const;              ///< 是否为对象
    ///@}

    /** @name 取值访问 */
    ///@{
    bool bool_value() const;                               ///< 取布尔值（类型不符时抛异常）
    double number_value() const;                           ///< 取数值（double）
    int int_value() const;                                 ///< 取数值并四舍五入为 int
    const std::string& string_value() const;               ///< 取字符串引用
    const array_type& array_items() const;                 ///< 取数组（只读引用）
    array_type& array_items();                             ///< 取数组（可写引用）
    const object_type& object_items() const;               ///< 取对象（只读引用）
    object_type& object_items();                           ///< 取对象（可写引用）
    ///@}

    /** @name 键值访问 */
    ///@{
    bool contains(const std::string& key) const;          ///< 判断对象中是否包含指定键
    const Json& at(const std::string& key) const;         ///< 按 key 取子值（不存在时抛异常）
    Json& operator[](const std::string& key);             ///< 按 key 取子值（不存在时自动插入空对象）
    const Json& operator[](const std::string& key) const; ///< 按 key 取子值（只读，等价于 at）
    ///@}
};

/**
 * @brief 将一段文本解析为 Json 值
 * @param text 符合 JSON 语法的字符串
 * @return 解析得到的 Json 值
 * @throws std::runtime_error 当文本不符合 JSON 语法时抛出
 */
Json parse_json(const std::string& text);

/**
 * @brief 将 Json 值序列化为字符串
 * @param value  要序列化的 Json 值
 * @param indent 缩进空格数；传 0 时输出紧凑（无空白）格式
 * @return 序列化后的字符串
 */
std::string dump_json(const Json& value, int indent = 2);

/**
 * @brief 从文件读取并解析 JSON
 * @param path JSON 文件路径
 * @return 解析得到的 Json 值
 * @throws std::runtime_error 当文件无法打开或解析失败时抛出
 */
Json read_json_file(const std::filesystem::path& path);

/**
 * @brief 将 Json 值写入文件（会自动创建父目录）
 * @param path  目标文件路径
 * @param value 要写入的 Json 值
 */
void write_json_file(const std::filesystem::path& path, const Json& value);

// ============================================================================
// 数据载体结构体
// ============================================================================

/**
 * @brief 站点信息
 *
 * 描述一个 LiDAR 设备部署站点的地理与标识信息。
 */
struct SiteInfo {
    std::string name;            ///< 站点可读名称（如 "北京演示场"）
    double latitude_deg = 0.0;   ///< 纬度（°，WGS84）
    double longitude_deg = 0.0;  ///< 经度（°，WGS84）
    double altitude_m = 0.0;     ///< 海拔高度（m）
    std::string site_id;         ///< 站点唯一标识符（可与 Cloudnet 对齐）
};

/**
 * @brief Cloudnet 混合数据源配置
 *
 * 当使用真实 Cloudnet 云高仪 NetCDF 数据 + 合成 PPI 时所需的下载与裁剪参数。
 */
struct CloudnetSourceConfig {
    std::string site_id;            ///< Cloudnet 站点标识（如 "bucharest"）
    std::string site_name;          ///< 站点可读名称
    std::string date;               ///< 数据日期，格式 "YYYY-MM-DD"
    bool verify_ssl = true;         ///< 下载时是否校验 SSL 证书
    std::string local_file;         ///< 本地已存在的 .nc 文件路径（可空，则触发下载）
    std::string download_url;       ///< .nc 文件的下载 URL
    int time_steps = 18;            ///< 需要采样的时间步数量
    int range_bin_count = 30;       ///< 每条扫描射线的目标距离分辨率
    double min_range_m = 75.0;      ///< 保留的最小距离（m），滤除近端噪声
    double max_range_m = 3200.0;    ///< 保留的最大距离（m），滤除远端噪声
    double pseudo_signal_scale = 600000.0; ///< 由 beta 反推伪 raw_counts 时的缩放系数
};

/**
 * @brief 数据源配置
 *
 * 决定数据从仿真生成还是从真实文件加载，以及相关根目录。
 */
struct SourceConfig {
    std::string mode = "simulation";  ///< 数据源模式："simulation" 或 "cloudnet_hybrid"
    std::string root = ".";           ///< 数据根目录（缓存、产物等的基准路径）
    CloudnetSourceConfig cloudnet;    ///< cloudnet_hybrid 模式下的详细参数
};

/**
 * @brief 单条 LiDAR 扫描射线（ray）的完整原始数据
 *
 * 对应 Python 端的 LidarProfile 数据类。包含一条射线的全部测量量（角度、
 * 距离轴、原始光子计数、系统参数、气象场、分子场）以及仅用于评估的真值场。
 */
struct LidarProfile {
    std::string site_id;          ///< 所属站点标识
    std::string timestamp;        ///< 时间戳（ISO8601 分钟精度，如 "2026-05-30T08:00"）
    std::string scan_id;          ///< 扫描批次标识
    std::string scan_mode;        ///< 扫描模式："stare"（定点凝视）或 "ppi"（平面位置显示）
    std::string source_kind;      ///< 数据来源类型（如 synthetic_stare / synthetic_ppi / cloudnet_real）
    double azimuth_deg = 0.0;     ///< 方位角（°，正北顺时针）
    double elevation_deg = 0.0;   ///< 仰角（°，水平为 0，天顶为 90）
    std::vector<double> ranges_m; ///< 各距离 bin 的距离（m）
    std::vector<double> raw_counts;///< 各距离 bin 的原始光子计数
    double laser_energy_mj = 0.0; ///< 单脉冲激光能量（mJ）
    double background_counts = 0.0;///< 背景光子计数（基底噪声估计）
    std::vector<double> overlap;   ///< 几何重叠因子随距离的变化（0~1）
    double relative_humidity = 0.0;///< 相对湿度（0~1）
    double temperature_c = 0.0;   ///< 温度（℃）
    double wind_speed_ms = 0.0;   ///< 风速（m/s）
    double wind_dir_deg = 0.0;    ///< 风向（°）
    std::vector<double> molecular_backscatter;  ///< 分子（Rayleigh）后向散射系数（1/(m·sr)）
    std::vector<double> molecular_extinction;   ///< 分子（Rayleigh）消光系数（1/m）
    // ---- 以下真值字段仅在仿真/评估中使用，真实数据不存在 ----
    std::vector<double> true_backscatter;  ///< 气溶胶后向散射真值（用于评估）
    std::vector<double> true_extinction;   ///< 气溶胶消光真值（用于评估）
    std::vector<double> true_pm25;         ///< PM2.5 浓度真值（µg/m³，用于评估）
    std::vector<double> true_pm10;         ///< PM10 浓度真值（µg/m³，用于评估）
    std::vector<int> true_hotspot_mask;    ///< 热点像元掩膜真值（0/1，用于评估）
};

/**
 * @brief 同址地面观测
 *
 * 与 LiDAR 共址（或临近）的地面站观测到的一组 PM 与气象量。
 */
struct GroundMeasurement {
    std::string site_id;              ///< 所属站点标识
    std::string timestamp;            ///< 时间戳（ISO8601 分钟精度）
    double pm25_ugm3 = 0.0;           ///< 地面 PM2.5 浓度（µg/m³）
    double pm10_ugm3 = 0.0;           ///< 地面 PM10 浓度（µg/m³）
    double relative_humidity = 0.0;   ///< 相对湿度（0~1）
    double temperature_c = 0.0;       ///< 温度（℃）
    double wind_speed_ms = 0.0;       ///< 风速（m/s）
    double wind_dir_deg = 0.0;        ///< 风向（°）
};

/**
 * @brief L1 预处理中间结果。
 *
 * 对应商用 LiDAR 数据产品里的 background-corrected signal、range-corrected signal、
 * SNR/CNR 和 QC flag。公开设备软件通常不会只输出最终 PM，而会保留这些中间量用于复核。
 */
struct PreprocessResult {
    std::vector<double> l1_signal;              ///< 背景扣除后的 L1 信号
    std::vector<double> attenuated_backscatter; ///< 能量/overlap/range 校正后的衰减后向散射代理量
    std::vector<double> snr;                    ///< 距离门信噪比
    std::vector<std::string> qc_flags;          ///< 质控标志
};

/**
 * @brief 一条 LiDAR 射线经过全流程处理后的结果
 *
 * 在原始 LidarProfile 基础上，叠加了 L1 信号、衰减后向散射、信噪比、
 * 反演得到的消光与 PM 浓度、ENU 三维坐标、质控标志等下游产物。
 */
struct ProcessedProfile {
    LidarProfile profile;             ///< 原始剖面（透传保存）
    std::vector<double> l1_signal;    ///< L1 级信号（背景扣除+能量归一化+重叠校正后）
    std::vector<double> attenuated_backscatter; ///< 衰减后向散射（距离平方校正后）
    std::vector<double> snr;          ///< 各 bin 的信噪比（SNR）
    std::vector<double> extinction;   ///< 反演得到的（含湿）消光系数（1/m）
    std::vector<double> dry_extinction; ///< 干状态消光系数（湿度校正后，1/m）
    std::vector<double> pm25;         ///< 估算的 PM2.5 浓度（µg/m³）
    std::vector<double> pm10;         ///< 估算的 PM10 浓度（µg/m³）
    std::vector<std::vector<double>> enu_points_m; ///< 各 bin 在 ENU 坐标系下的 [东,北,上]（m）
    std::vector<std::string> qc_flags; ///< 质控标志列表（如 low-laser-energy 等）
    double latency_ms = 0.0;          ///< 处理该射线所耗费的时间（ms，用于性能统计）
};

/**
 * @brief 检测到的一个污染热点
 *
 * 由热点检测算法（在 PPI 网格上做连通域分析）产出的一个连通区域。
 */
struct Hotspot {
    std::string timestamp;          ///< 热点所在时间步的时间戳
    std::string scan_id;            ///< 热点所属的扫描批次
    std::string hotspot_id;         ///< 热点唯一编号
    std::vector<double> centroid_enu_m; ///< 热点质心在 ENU 坐标下的 [东,北,上]（m）
    double peak_pm25_ugm3 = 0.0;    ///< 热点区域的 PM2.5 峰值（µg/m³）
    double mean_pm25_ugm3 = 0.0;    ///< 热点区域的 PM2.5 平均值（µg/m³）
    double estimated_area_m2 = 0.0; ///< 热点区域的估计面积（m²）
    int cell_count = 0;             ///< 热点所包含的网格像元数
    std::string severity;           ///< 严重程度分级："medium" / "high" / "critical"
};

// ============================================================================
// 各阶段配置
// ============================================================================

/**
 * @brief 仿真配置
 *
 * 控制合成 LiDAR 场景生成的全部参数（时间步、距离分辨率、PPI 扫描几何、
 * 系统常数、激光雷达比等）。
 *
 * PPI 多仰角：真实扫描雷达一个体积扫描（volume scan）会顺序扫描多个仰角，
 * 每个仰角各做一圈方位扫描，从而得到三维体数据。`ppi_elevations_deg`
 * 为空时退化为旧行为（仅一个仰角），保证向后兼容。
 */
struct SimulationConfig {
    std::string instrument_preset = "demo_lidar";              ///< 设备预设：demo_lidar / field_scanning_pm_lidar / ceilometer_profile / mobile_mapping_lidar
    std::string application_mode = "legacy_demo";              ///< 应用场景：legacy_demo / construction_site / urban_grid / mobile_mapping
    std::string vendor_profile = "generic_jsonl";              ///< 公开格式映射：generic_jsonl / vaisala_cl61_like / raymetrics_pmeye_like / halo_hpl_like
    int seed = 7;                          ///< 随机数种子（决定可复现性）
    int time_steps = 18;                   ///< 仿真的时间步数量
    int minutes_per_step = 20;            ///< 相邻时间步的间隔（分钟）
    int range_bin_count = 60;             ///< 每条射线的距离 bin 数量
    double range_bin_m = 100.0;           ///< 相邻距离 bin 的间距（m）
    /// PPI 体积扫描的仰角序列（°，高于水平面）。
    /// 默认 {5,15,30,45,60,75} 为接近商用气溶胶扫描雷达的 6 层体积扫描方案
    /// （如 Raymetrics / 国产扫描激光雷达的常规配置）。配套参数已按商用尺度标定：
    /// 量程 6 km（60 bin × 100 m）、边界层标高 1200 m、烟羽高度 280/380 m。
    /// 高仰角 75° 在 6 km 处达 5.8 km，仍在对流层下部，信号有意义；
    /// 烟羽 1（280 m/2200 m）所需仰角约 7°，由 15° 层捕捉；最低 5° 层覆盖近场低空源。
    /// 为空时退化为单个低仰角（兼容旧配置）。
    std::vector<double> ppi_elevations_deg = {5.0, 15.0, 30.0, 45.0, 60.0, 75.0};
    double ppi_azimuth_step_deg = 30.0;   ///< PPI 扫描的方位角步进（°）
    double system_constant = 260000000.0; ///< LiDAR 系统常数 C（正演发射方程用）
    double lidar_ratio_sr = 45.0;         ///< 气溶胶激光雷达比（sr，消光后向散射比）
    double wavelength_nm = 532.0;         ///< 激光波长（nm），PM 扫描雷达常用 355nm，云高仪常用 905/910nm
    double pulse_energy_mj = 1.0;         ///< 单脉冲能量均值（mJ），用于仿真每条射线的能量抖动
    double pulse_energy_jitter = 0.03;    ///< 脉冲能量相对抖动（1 sigma）
    double background_counts_mean = 10.5; ///< 背景/暗计数均值（计数）
    double background_counts_jitter = 0.5;///< 背景计数随机扰动（1 sigma）
    double full_overlap_m = 500.0;        ///< 进入完整 overlap 的距离（m）
    double min_overlap = 0.22;            ///< 近场最小 overlap
    double detector_dark_counts = 0.0;    ///< 探测器暗计数等效均值
    double read_noise_counts = 0.0;       ///< 读出噪声（计数）
    double adc_saturation_counts = 50000000.0; ///< 模拟/ADC 饱和上限
    double dead_time_loss = 0.000000018;  ///< photon-counting 死时间损失近似系数
    double solar_background_scale = 1.0;  ///< 太阳背景强度缩放，移动/白天工地可调高
    double vehicle_speed_ms = 0.0;        ///< 走航模式下平台速度（m/s），固定站为 0

    /**
     * @brief 取有效仰角列表（兼容旧配置）。
     *
     * 若 `ppi_elevations_deg` 为空，则返回单个低仰角占位值 {8.0}，
     * 使得旧的单仰角配置仍能运行（只是不再多仰角扫描）。
     */
    std::vector<double> effective_ppi_elevations_deg() const {
        return ppi_elevations_deg.empty()
            ? std::vector<double>{8.0}
            : ppi_elevations_deg;
    }
};

/**
 * @brief 反演配置
 *
 * Fernald/Klett 反演所用的参数。
 */
struct RetrievalConfig {
    double aerosol_lidar_ratio_sr = 45.0;        ///< 气溶胶激光雷达比（sr）
    double reference_aerosol_backscatter = 0.0004; ///< 参考点处的气溶胶后向散射系数
};

/**
 * @brief 湿度校正配置
 *
 * 控制吸湿增长因子 f(RH) 的两个参数，用于把"湿"消光换算为"干"消光。
 */
struct HumidityConfig {
    double dry_reference_rh = 0.45;  ///< 干燥参考点的相对湿度（0~1）
    double hygroscopicity = 1.1;     ///< 吸湿性参数 κ（越大表示颗粒越易吸湿增长）
};

/**
 * @brief PM 标定配置
 *
 * 控制 PM2.5/PM10 回归标定的训练集划分与特征构造。
 */
struct PmCalibrationConfig {
    double train_ratio = 0.6;   ///< 训练集比例（其余按 val_ratio 划分验证/测试）
    double val_ratio = 0.2;     ///< 验证集比例
    int surface_bin_count = 6;  ///< 自地面起算用于特征提取的近端 bin 数
};

/**
 * @brief 热点检测配置
 *
 * 控制热点判定阈值与最小连通域规模。
 */
struct HotspotConfig {
    double pm25_threshold_ugm3 = 50.0;             ///< PM2.5 绝对阈值（µg/m³）
    double scan_relative_pm25_threshold_ugm3 = 0.18; ///< PM2.5 相对阈值（相对中位数的增量）
    double scan_relative_dry_ext_threshold = 0.02;  ///< 干消光相对阈值
    int min_cells = 3;                             ///< 热点连通域所需的最小像元数
};

/**
 * @brief 评估配置
 */
struct EvaluationConfig {
    std::vector<double> sensitivity_lidar_ratios; ///< 做灵敏度分析时要尝试的激光雷达比列表
};

/**
 * @brief L0 -> L1 预处理步骤。
 *
 * 职责：背景扣除、激光能量归一、overlap 修正、range^2 修正、SNR 和 QC flag。
 */
class BackgroundPreprocessStep {
public:
    PreprocessResult process(const LidarProfile& profile) const;
};

/**
 * @brief Fernald/Klett 弹性 LiDAR 反演步骤。
 *
 * 职责：由 attenuated backscatter 估计消光和气溶胶后向散射。
 */
class FernaldInversionStep {
public:
    explicit FernaldInversionStep(RetrievalConfig config);

    std::pair<std::vector<double>, std::vector<double>> process(
        const LidarProfile& profile,
        const std::vector<double>& attenuated_backscatter) const;

private:
    RetrievalConfig config_;
};

/**
 * @brief 湿度修正步骤。
 *
 * 职责：把湿消光换算到干参考状态，降低 RH 对 PM 估计的虚假放大。
 */
class HumidityCorrectionStep {
public:
    explicit HumidityCorrectionStep(HumidityConfig config);

    std::vector<double> process(
        const std::vector<double>& extinction,
        double relative_humidity) const;

private:
    HumidityConfig config_;
};

/**
 * @brief 极坐标到 ENU 坐标投影步骤。
 */
class CoordinateProjectionStep {
public:
    std::vector<std::vector<double>> process(const LidarProfile& profile) const;
};

/**
 * @brief PPI/体扫热点检测步骤。
 *
 * 职责：按 timestamp 内的扫描射线构建距离-方位网格，执行阈值、相对异常和连通域分析。
 */
class HotspotDetectionStep {
public:
    explicit HotspotDetectionStep(HotspotConfig config);

    std::vector<Hotspot> process(const std::vector<ProcessedProfile>& ppi_profiles) const;

private:
    HotspotConfig config_;
};

/**
 * @brief 单条 profile 的类化处理链。
 *
 * 这个类把商用设备软件常见的 L0 -> L1 -> L2 几个阶段拆开，便于调试、面试讲解和后续替换算法。
 */
class SingleProfileProcessingChain {
public:
    SingleProfileProcessingChain(RetrievalConfig retrieval, HumidityConfig humidity);

    ProcessedProfile process(const LidarProfile& profile, bool disable_humidity = false) const;

private:
    BackgroundPreprocessStep preprocess_;
    FernaldInversionStep inversion_;
    HumidityCorrectionStep humidity_;
    CoordinateProjectionStep projection_;
};

/**
 * @brief 管线总配置
 *
 * 汇总上述所有配置段，以及顶层的数据源模式。通常由 parse_pipeline_config
 * 从一个 JSON 配置文件解析得到。
 */
struct PipelineConfig {
    std::string source_mode = "simulation"; ///< 数据源模式（取自 source.mode，便于快速判断）
    SourceConfig source;     ///< 数据源配置
    SiteInfo site;           ///< 站点信息
    SimulationConfig simulation;     ///< 仿真配置
    RetrievalConfig retrieval;       ///< 反演配置
    HumidityConfig humidity;         ///< 湿度校正配置
    PmCalibrationConfig pm_calibration; ///< PM 标定配置
    HotspotConfig hotspot;           ///< 热点检测配置
    EvaluationConfig evaluation;     ///< 评估配置
};

// ============================================================================
// 单步处理 API（供实时客户端逐帧处理使用）
// ============================================================================

/**
 * @brief 对单条原始 LiDAR 廓线执行 L1 预处理 + Fernald 反演 + 湿度校正。
 *
 * 这是客户端实时处理的核心入口。给定一条 LidarProfile 和反演/湿度配置，
 * 返回包含 L1 信号、衰减后向散射、SNR、消光、干消光的 ProcessedProfile。
 * 注意：PM 浓度字段（pm25/pm10）在此步不填，需要地面标定后才有意义。
 *
 * @param profile     原始 LiDAR 廓线
 * @param retrieval   反演配置（激光比、参考后向散射）
 * @param humidity    湿度校正配置
 * @return            预处理 + 反演后的 ProcessedProfile（pm25/pm10 为空）
 */
ProcessedProfile process_single_profile(
    const LidarProfile& profile,
    const RetrievalConfig& retrieval,
    const HumidityConfig& humidity
);

/**
 * @brief 对一组 PPI 处理后廓线执行热点检测。
 *
 * 对同一时间步的所有 PPI 廓线（通常 12 条，覆盖 0~360°）构建距离-方位网格，
 * 做阈值 + 连通域分析，返回检测到的热点列表。
 *
 * @param ppi_profiles  同一时间步的所有 PPI ProcessedProfile
 * @param hotspot_cfg   热点检测配置（阈值、最小连通域）
 * @return              检测到的热点列表
 */
std::vector<Hotspot> detect_hotspots_from_processed(
    const std::vector<ProcessedProfile>& ppi_profiles,
    const HotspotConfig& hotspot_cfg
);

/**
 * @brief 从 L2 demo 结果 JSON 中提取地面观测列表。
 *
 * @param results  run_end_to_end 返回的结果 Json
 * @return         地面观测列表
 */
std::vector<GroundMeasurement> extract_ground_measurements(const Json& results);

/**
 * @brief 将一条 ProcessedProfile 序列化为 JSON。
 * @param value 处理后的廓线
 * @return      JSON 对象（L2 格式）
 */
Json to_json_processed(const ProcessedProfile& value);

/**
 * @brief 将一个 Hotspot 序列化为 JSON。
 * @param value 热点
 * @return      JSON 对象
 */
Json to_json_hotspot(const Hotspot& value);

// ============================================================================
// 顶层 API
// ============================================================================

/**
 * @brief 从 Json 值解析出 PipelineConfig
 * @param value 已加载的配置 Json 对象
 * @return 填充好的 PipelineConfig
 */
PipelineConfig parse_pipeline_config(const Json& value);

/**
 * @brief 端到端运行整个 LiDAR 颗粒物监测管线
 *
 * 完整执行：数据加载 → 预处理(L0→L1) → Fernald 反演 → 湿度校正 →
 * PM 标定与应用 → 热点检测 → 质量评估 → 消融/灵敏度/失效案例测试 →
 * demo 汇总 payload。可选择把 raw/L1/L2 JSON 产物写出。
 *
 * @param config      已解析的管线配置
 * @param output_root 可选的产物输出根目录；为空则不写文件
 * @return 包含全部结果（metrics、sensitivity、ablation、failure_cases、demo 等）的 Json
 */
Json run_end_to_end(const PipelineConfig& config, const std::optional<std::filesystem::path>& output_root = std::nullopt);

/**
 * @brief 根据管线结果渲染一个自包含的 HTML 仪表盘页面
 * @param data run_end_to_end 返回的结果 Json
 * @return 完整的 HTML 字符串
 */
std::string render_dashboard(const Json& data);

/**
 * @brief 从完整结果中提取一份"摘要"payload（指标 + 最新热点 + 告警数等）
 * @param results run_end_to_end 返回的结果 Json
 * @return 摘要 Json
 */
Json build_summary_payload(const Json& results);

/**
 * @brief 抓取公开的地面 PM 与气象数据（Open-Meteo）
 *
 * 调用 Open-Meteo 空气质量 API（PM2.5、PM10）与历史归档 API（温度、湿度、风），
 * 抓取指定经纬度、日期范围内的地面观测，并缓存为 JSON + CSV。
 *
 * @param latitude_deg  纬度
 * @param longitude_deg 经度
 * @param start_date    起始日期（"YYYY-MM-DD"）
 * @param end_date      结束日期（"YYYY-MM-DD"）
 * @param timezone      时区名（如 "Asia/Shanghai"）
 * @param output_dir    缓存输出目录
 * @param prefix        输出文件名前缀（不含扩展名）
 * @return 包含抓取到的地面记录的 Json
 */
Json fetch_public_ground_data(
    double latitude_deg,
    double longitude_deg,
    const std::string& start_date,
    const std::string& end_date,
    const std::string& timezone,
    const std::filesystem::path& output_dir,
    const std::string& prefix
);

/**
 * @brief 抓取一份公开的 Cloudnet 云高仪样本（.nc + 对齐的地面数据）
 *
 * 根据 config 中 cloudnet 配置，下载指定的 .nc 文件以及匹配日期的
 * Open-Meteo 地面数据，统一存放到 output_root 下。
 *
 * @param config     管线配置（读取其中的 source.cloudnet 与 site）
 * @param output_root 产物输出根目录
 * @return 包含下载元信息（路径、记录数等）的 Json
 */
Json fetch_cloudnet_public_sample(const PipelineConfig& config, const std::filesystem::path& output_root);

} // namespace lidar_demo
