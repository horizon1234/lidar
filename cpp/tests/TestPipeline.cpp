/**
 * @file TestPipeline.cpp
 * @brief 端到端流水线的 CTest 集成断言。
 *
 * 该可执行文件注册为 CTest 用例，用于在构建阶段对 lidar_demo 的端到端
 * 处理结果做"防回归"检查：
 * - 读取 JSON 流水线配置（默认 configs/DefaultPipeline.json，也可由 argv[1] 覆盖）；
 * - 执行 run_end_to_end，得到完整结果 JSON；
 * - 逐条断言 metrics / curtain / ppi / alerts 的关键字段是否落在期望区间；
 * - 任一断言失败将抛出 std::runtime_error，返回码 1，CTest 据此标记失败。
 */

#include "LidarDemo/LidarDemo.hpp"

#include <cmath>
#include <iostream>

namespace {

/**
 * @brief 断言辅助：条件不成立时抛出带消息的异常。
 *
 * 由于本测试用例作为单元测试使用，引入该宏式辅助可让断言保持一行表达，
 * 同时附带可读的失败消息，便于在 CTest 输出中定位失败原因。
 *
 * @param condition 待检查的布尔条件。
 * @param message   条件不成立时作为异常 what() 的消息字符串。
 */
void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

/**
 * @brief 测试主入口：执行流水线并逐项断言关键指标。
 *
 * @param argc 参数个数，可用于传入自定义配置路径。
 * @param argv argv[1]（可选）为流水线配置路径，否则使用 configs/DefaultPipeline.json。
 * @return int 0 表示全部断言通过；1 表示异常/断言失败。
 */
int main(int argc, char** argv) {
    try {
        // === 配置加载：argv[1] 优先，否则用默认配置 ===
        std::filesystem::path config_path = argc > 1 ? argv[1] : std::filesystem::path("configs/DefaultPipeline.json");
        lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(lidar_demo::read_json_file(config_path));
        lidar_demo::Json results = lidar_demo::run_end_to_end(config);

        const auto& metrics = results.at("metrics");

        // --- 必备字段存在性 --- 字段缺失即视为流水线产物损坏
        require(metrics.contains("pm25"), "Missing pm25 metrics");
        require(metrics.contains("pm10"), "Missing pm10 metrics");
        require(metrics.contains("hotspot"), "Missing hotspot metrics");
        require(metrics.at("runtime").at("throughput_profiles_per_s").number_value() > 0.0, "Invalid throughput");

        // --- 回归指标阈值：避免精度退化 --- RMSE 与 R² 必须落在期望区间
        require(metrics.at("pm25").at("rmse").number_value() < 20.0, "PM2.5 RMSE too high");
        require(metrics.at("pm10").at("rmse").number_value() < 25.0, "PM10 RMSE too high");
        require(metrics.at("pm25").at("r2").number_value() > 0.7, "PM2.5 R2 too low");

        // --- 热点（hotspot）质量：F1 与告警数量必须达标 ---
        require(metrics.at("hotspot").at("f1").number_value() > 0.55, "Hotspot F1 too low");
        require(static_cast<int>(results.at("alerts").array_items().size()) >= 1, "No hotspot alerts produced");

        // --- 形状一致性：curtain 时间轴长度须等于数据集时间戳数 ---
        const auto& curtain = results.at("curtain");
        require(curtain.at("times").array_items().size() == static_cast<std::size_t>(results.at("dataset_summary").at("timestamp_count").int_value()), "Curtain time count mismatch");
        require(curtain.at("pm25").array_items().size() == curtain.at("times").array_items().size(), "Curtain pm25 shape mismatch");
        require(results.at("ppi").at("cells").array_items().size() > 100, "Insufficient PPI cells"); // PPI 网格必须足够细

        std::cout << "All C++ pipeline assertions passed." << std::endl;
        return 0;
    } catch (const std::exception& error) {
        // 异常被吞并并以错误信息打印到 stderr，返回 1 触发 CTest 失败标记
        std::cerr << error.what() << std::endl;
        return 1;
    }
}