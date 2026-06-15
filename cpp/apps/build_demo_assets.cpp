/**
 * @file build_demo_assets.cpp
 * @brief 演示资产生成 CLI：端到端运行流水线并写出可视化 Dashboard HTML。
 *
 * 该工具用于在构建阶段一次性生成演示产物：
 * - 读取 JSON 流水线配置（默认 configs/default_pipeline.json）；
 * - 调用 lidar_demo::run_end_to_end 得到结果 JSON；
 * - 用 lidar_demo::render_dashboard 把结果渲染成自包含的 HTML 看板；
 * - 默认输出到 web/demo_dashboard.html，可由 --dashboard 覆盖；
 * - 自动创建目标文件的父目录，保证可写入任意嵌套路径。
 */

#include "lidar_demo/lidar_demo.hpp"

#include <fstream>
#include <iostream>

/**
 * @brief 演示资产主入口：执行流水线并写出 Dashboard HTML。
 *
 * 支持以下命令行参数（顺序无关）：
 *   --config <path>        流水线 JSON 配置文件，默认 configs/default_pipeline.json
 *   --output-root <dir>    结果文件的输出根目录，默认当前目录（"."）
 *   --dashboard <path>     生成 HTML 的目标路径，默认 <output-root>/web/demo_dashboard.html
 *
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组（argv[0] 为程序名）。
 * @return int 进程返回码：0 表示成功，1 表示异常。
 */
int main(int argc, char** argv) {
    try {
        // === 默认参数：未显式指定时使用的兜底值 ===
        std::filesystem::path config_path = "configs/default_pipeline.json"; ///< 流水线配置文件路径
        std::filesystem::path output_root = "."; ///< 结果输出根目录
        std::optional<std::filesystem::path> dashboard_path; ///< 可选的 HTML 输出路径（缺省时由 output_root 推导）

        // === 命令行参数解析：三个开关均为“键 + 值”形式 ===
        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--config" && index + 1 < argc) {
                config_path = argv[++index];
            } else if (argument == "--output-root" && index + 1 < argc) {
                output_root = argv[++index];
            } else if (argument == "--dashboard" && index + 1 < argc) {
                dashboard_path = std::filesystem::path(argv[++index]);
            }
        }

        // === 流水线执行：读配置 -> 解析 -> 端到端运行 ===
        lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(lidar_demo::read_json_file(config_path));
        lidar_demo::Json results = lidar_demo::run_end_to_end(config, output_root);

        // === Dashboard 落盘：确定最终路径并创建父目录 ===
        std::filesystem::path final_dashboard = dashboard_path.value_or(output_root / "web" / "demo_dashboard.html");
        std::filesystem::create_directories(final_dashboard.parent_path()); // 确保父目录存在，避免 ofstream 打开失败
        std::ofstream handle(final_dashboard, std::ios::binary); // 以二进制模式写出，避免行尾被平台改写
        handle << lidar_demo::render_dashboard(results); // 将结果 JSON 渲染为自包含 HTML 写入
        std::cout << "Generated " << final_dashboard.string() << std::endl;
        return 0;
    } catch (const std::exception& error) {
        // 统一异常处理：错误信息写入 stderr，返回码 1
        std::cerr << error.what() << std::endl;
        return 1;
    }
}