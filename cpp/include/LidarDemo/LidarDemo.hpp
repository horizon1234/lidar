/**
 * @file LidarDemo.hpp
 * @brief Aggregate public API for the atmospheric LiDAR particulate monitoring demo.
 */
#pragma once

#include "LidarDemo/Json.hpp"
#include "LidarDemo/ProcessingSteps.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lidar_demo {

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
