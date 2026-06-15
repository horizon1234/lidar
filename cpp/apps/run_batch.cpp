/**
 * @file run_batch.cpp
 * @brief 批处理 Worker 命令行入口。
 *
 * 该可执行文件是 lidar_demo 库在批量处理场景下的命令行封装：
 * - 读取 JSON 流水线配置（默认 configs/default_pipeline.json）；
 * - 调用 lidar_demo::run_end_to_end 执行端到端处理；
 * - 将 metrics（指标）写入 L2 层 worker_summary.json；
 * - 同时将 metrics 以缩进 JSON 形式打印到标准输出，便于调度系统采集。
 *
 * 作为批处理 Worker，返回码遵循 Unix 惯例：0 表示成功，1 表示异常。
 */

#include "lidar_demo/lidar_demo.hpp"

#include <iostream>
#include <optional>

/**
 * @brief 批处理主入口：解析参数并执行一次完整流水线。
 *
 * 支持以下命令行参数（可任意顺序）：
 *   --config <path>    流水线 JSON 配置文件路径，默认 configs/default_pipeline.json
 *   --output <dir>     输出根目录，默认当前目录（"."），L2 结果会写入 <dir>/data/l2/
 *
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组（argv[0] 为程序名）。
 * @return int 进程返回码：0 表示成功，1 表示出错。
 */
int main(int argc, char** argv) {
    try {
        // === 默认参数：Worker 在未被显式指定时使用的兜底值 ===
        std::filesystem::path config_path = "configs/default_pipeline.json"; ///< 流水线配置文件路径
        std::filesystem::path output_root = "."; ///< 输出根目录

        // === 命令行参数解析：解析 --config / --output，侦测 --config 后必须紧跟一个值 ===
        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--config" && index + 1 < argc) {
                config_path = argv[++index]; // 取出下一个 token 作为配置文件路径
            } else if (argument == "--output" && index + 1 < argc) {
                output_root = argv[++index]; // 取出下一个 token 作为输出根目录
            }
        }

        // === 流水线执行：读配置 -> 解析 -> 端到端运行 ===
        lidar_demo::Json config_json = lidar_demo::read_json_file(config_path); // 加载 JSON 配置文件
        lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(config_json); // 解析为强类型流水线配置
        lidar_demo::Json results = lidar_demo::run_end_to_end(config, output_root); // 执行完整流水线，得到结果 JSON

        // === 结果落盘 + 标准输出：worker_summary.json 给下游，stdout 给调度器 ===
        lidar_demo::write_json_file(output_root / "data" / "l2" / "worker_summary.json", results.at("metrics")); // 仅持久化 metrics 子树
        std::cout << lidar_demo::dump_json(results.at("metrics"), 2) << std::endl; // 缩进 2 空格的 JSON
        return 0;
    } catch (const std::exception& error) {
        // 统一异常处理：错误信息写入 stderr，返回码 1 表示失败
        std::cerr << error.what() << std::endl;
        return 1;
    }
}