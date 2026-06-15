/**
 * @file fetch_cloudnet_public_sample.cpp
 * @brief Cloudnet 公开样品抓取 CLI：基于混合流水线配置拉取云雷达样本数据。
 *
 * 该工具是 lidar_demo 面向 Cloudnet 公开数据集的命令行封装：
 * - 读取混合流水线配置（默认 configs/cloudnet_hybrid_pipeline.json）；
 * - 调用 lidar_demo::fetch_cloudnet_public_sample 抓取样品；
 * - 把 manifest 以缩进 JSON 形式打印到标准输出，便于下游流水线引用。
 */

#include "lidar_demo/lidar_demo.hpp"

#include <iostream>

/**
 * @brief Cloudnet 样品抓取主入口。
 *
 * 支持以下命令行参数（顺序无关，均为“键 + 值”形式）：
 *   --config <path>        混合流水线 JSON 配置文件，默认 configs/cloudnet_hybrid_pipeline.json
 *   --output-root <dir>    输出根目录，默认当前目录（"."）
 *
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组。
 * @return int 进程返回码：0 表示成功，1 表示异常。
 */
int main(int argc, char** argv) {
    try {
        // === 默认参数：使用 Cloudnet 混合流水线配置 ===
        std::filesystem::path config_path = "configs/cloudnet_hybrid_pipeline.json"; ///< 混合流水线配置文件路径
        std::filesystem::path output_root = "."; ///< 输出根目录

        // === 命令行参数解析：--config 与 --output-root ===
        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--config" && index + 1 < argc) {
                config_path = argv[++index];
            } else if (argument == "--output-root" && index + 1 < argc) {
                output_root = argv[++index];
            }
        }

        // === 样品抓取：读配置 -> 解析 -> 抓取 Cloudnet 公开样品 ===
        lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(lidar_demo::read_json_file(config_path));
        lidar_demo::Json manifest = lidar_demo::fetch_cloudnet_public_sample(config, output_root);
        std::cout << lidar_demo::dump_json(manifest, 2) << std::endl;
        return 0;
    } catch (const std::exception& error) {
        // 统一异常处理：错误信息写入 stderr，返回码 1
        std::cerr << error.what() << std::endl;
        return 1;
    }
}