/**
 * @file RunFullDemo.cpp
 * @brief 全链路主入口: 一键串联运行教学示例与完整 pipeline 的总编排器。
 *
 * 该可执行文件作为顶层 orchestrator, 把第 19 章的所有教学示例 + 真实
 * 多时间步 pipeline 拼成一条端到端演示:
 *
 * - **Stage 1** 一维射线 §19.1–§19.15: 调用 `lidar_example_1d_ray`
 * - **Stage 2** 二维 PPI  §19.17–§19.27: 调用 `lidar_example_2d_ppi`
 * - **Stage 3** 二维 RHI  §19.28–§19.37: 调用 `lidar_example_3d_rhi`
 * - **Stage 4** 完整 pipeline: 直接调用库函数 `lidar_demo::run_end_to_end()`
 *
 * 设计选择说明:
 *   - Stage 1–3 通过 std::system() 调用独立的可执行文件,
 *     这样每个示例自己的"逐步打印"都能完整输出, 不会被合并进主进程。
 *   - Stage 4 直接链接 lidar_demo 库并调用 run_end_to_end(),
 *     完成 stare + PPI 混合场景的真正多时间步数据链
 *     (L0 仿真 → L1 预处理 → L2 反演 → 指标 → dashboard)。
 *
 * 用法:
 * @code
 *   build/lidar_run_full_demo [--config configs/DefaultPipeline.json] \
 *                             [--output-root .]
 * @endcode
 *
 * 输出: 各 stage 的 JSON + summary 表格, 列出所有产物文件与大小。
 */

#include "ExampleCommon.hpp"
#include "LidarDemo/LidarDemo.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

using namespace example_common;

namespace {

#ifdef _WIN32
constexpr char PATH_SEP = '\\';   ///< Windows 路径分隔符
#else
constexpr char PATH_SEP = '/';    ///< Unix 路径分隔符
#endif

/**
 * @brief 在与主程序同目录查找兄弟可执行文件, 退化到 PATH。
 *
 * 由于 cmake 的多配置生成器 (Visual Studio、Ninja Multi-Config) 会把可执行
 * 放到 Release/Debug 子目录, 这里依次尝试以下路径, 命中即返回:
 *   1. 当前工作目录
 *   2. ./build/
 *   3. ./build/Release/
 *   4. ./build/Debug/
 *   5. 都找不到则原样返回 exe_name, 让 shell 退化到 PATH 解析
 *
 * @param exe_name 可执行文件基名 (不含路径)
 * @return 完整可执行路径字符串
 */
std::string exe_in_build_dir(const std::string& exe_name) {
    // 与主程序同目录找兄弟 exe (cmake build/Release 同目录放)
    // 先查 build/, 然后查 build/Release/ 再退化到 PATH
    namespace fs = std::filesystem;

    auto try_paths = std::vector<fs::path>{
        fs::current_path() / exe_name,
        fs::current_path() / "build" / exe_name,
        fs::current_path() / "build" / "Release" / exe_name,
        fs::current_path() / "build" / "Debug" / exe_name,
    };
    for (const auto& p : try_paths) {
        if (fs::exists(p)) return p.string();
    }
    return exe_name;  // fallback to PATH
}

/**
 * @brief 通过 std::system() 运行子进程并捕获返回码。
 *
 * 失败时仅打印 WARN 而不抛异常——即使某个教学 stage 失败 (例如缺少示例数据),
 * 主流程仍然继续往后跑 Stage 4 pipeline, 保证最大化可见的诊断信息。
 *
 * @param exe_name 可执行文件名 (会被 exe_in_build_dir() 解析路径)
 * @return 进程返回码 (0 = 成功)
 */
int run_subprocess(const std::string& exe_name) {
    std::string path = exe_in_build_dir(exe_name);
    int rc = std::system(path.c_str());
    if (rc != 0) {
        std::cerr << "[WARN] " << exe_name << " exited with code " << rc << "\n";
    }
    return rc;
}

}  // namespace

/**
 * @brief 全链路 demo 的入口点: 解析参数并依次运行 4 个 stage。
 *
 * 命令行参数:
 *   - `--config <path>`        pipeline 配置 JSON (默认 `configs/DefaultPipeline.json`)
 *   - `--output-root <path>`   产物根目录 (默认当前目录 `.`)
 *
 * 运行成功返回 0; 任意 stage 抛异常时返回 1。
 *
 * @param argc 参数个数
 * @param argv 参数数组
 * @return 进程退出码 (0 / 1)
 */
int main(int argc, char** argv) {
    enable_ansi_color();

    // ---- 命令行解析 ----
    std::filesystem::path config_path = "configs/DefaultPipeline.json";
    std::filesystem::path output_root = ".";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (a == "--output-root" && i + 1 < argc) output_root = argv[++i];
    }

    // stage 标题分隔条 (用 lambda 避免全局函数)
    auto stage_header = [](const std::string& title) {
        std::cout << "\n========================================================================\n";
        std::cout << " " << title << "\n";
        std::cout << "========================================================================\n";
    };

    // ---- Stage 1: 一维射线 §19.1–§19.15 ----
    stage_header("Stage 1: 1D ray example (sections 19.1-19.15)");
    run_subprocess("lidar_example_1d_ray");

    // ---- Stage 2: 二维 PPI §19.17–§19.27 ----
    stage_header("Stage 2: 2D PPI example (sections 19.17-19.27)");
    run_subprocess("lidar_example_2d_ppi");

    // ---- Stage 3: 二维 RHI §19.28–§19.37 ----
    stage_header("Stage 3: 2D RHI example (sections 19.28-19.37)");
    run_subprocess("lidar_example_3d_rhi");

    // ---- Stage 4: 完整 pipeline (lidar_demo::run_end_to_end) ----
    stage_header("Stage 4: full pipeline (lidar_demo::run_end_to_end)");
    try {
        // 读取 pipeline 配置 (站点、时间步、传感器型号等)
        lidar_demo::Json config_json = lidar_demo::read_json_file(config_path);
        lidar_demo::PipelineConfig cfg = lidar_demo::parse_pipeline_config(config_json);
        std::cout << "config: " << config_path.string() << "\n";
        std::cout << "site:   " << cfg.site.name << " ("
                  << cfg.site.latitude_deg << ", " << cfg.site.longitude_deg << ")\n";
        std::cout << "time_steps=" << cfg.simulation.time_steps
                  << " range_bins=" << cfg.simulation.range_bin_count << "\n";
        // 计时: 评估端到端 latency
        auto t0 = std::chrono::steady_clock::now();
        lidar_demo::Json results = lidar_demo::run_end_to_end(cfg, output_root);
        auto t1 = std::chrono::steady_clock::now();
        double sec = std::chrono::duration<double>(t1 - t0).count();

        // 打印关键指标 (PM2.5 / PM10 拟合质量、热点 F1、latency)
        const auto& m = results.at("metrics");
        printf("elapsed_s = %.3f\n", sec);
        printf("PM2.5    R2 = %s\n", lidar_demo::dump_json(m.at("pm25").at("r2"), 0).c_str());
        printf("PM2.5  RMSE = %s\n", lidar_demo::dump_json(m.at("pm25").at("rmse"), 0).c_str());
        printf("PM10     R2 = %s\n", lidar_demo::dump_json(m.at("pm10").at("r2"), 0).c_str());
        printf("Hotspot  F1 = %s\n", lidar_demo::dump_json(m.at("hotspot").at("f1"), 0).c_str());
        printf("latency_ms = %s\n", lidar_demo::dump_json(m.at("runtime").at("mean_latency_ms"), 0).c_str());
        std::cout << "L1 -> data/l1/demo_preprocessed.json\n";
        std::cout << "L2 -> data/l2/demo_results.json\n";
        std::cout << "dashboard -> web/demo_dashboard.html\n";
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] pipeline stage failed: " << e.what() << "\n";
        return 1;
    }

    // ---- summary: 完整 demo 完成汇总表 ----
    stage_header("summary: full demo complete");
    std::cout << "example outputs:\n";

    // 列出所有预期产物文件并打印大小, 缺失的标 [--]
    for (const char* name : {"data/examples/1d_ray_result.json",
                             "data/examples/2d_ppi_result.json",
                             "data/examples/3d_rhi_result.json",
                             "data/l2/demo_results.json",
                             "web/demo_dashboard.html"}) {
        std::filesystem::path p = output_root / name;
        const char* mark = std::filesystem::exists(p) ? "[OK]" : "[--]";
        std::cout << "  " << mark << " " << name;
        if (std::filesystem::exists(p)) {
            std::uintmax_t sz = std::filesystem::file_size(p);
            printf("  %7.1f KB\n", sz / 1024.0);
        } else {
            std::cout << "\n";
        }
    }
    return 0;
}
