// cpp/examples/run_full_demo.cpp
//
// 全链路主入口: 一键运行 4 个 stage
//   Stage 1: 一维射线 (lidar_example_1d_ray)
//   Stage 2: 二维 PPI (lidar_example_2d_ppi)
//   Stage 3: 二维 RHI (lidar_example_3d_rhi)
//   Stage 4: 完整 pipeline (lidar_demo::run_end_to_end)
//
// Stage 1-3 通过 system() 调用单独的可执行,
// 这样它们各自的中间输出 (每一步打印) 都能被走到。
// Stage 4 直接调用 lidar_demo 库的 run_end_to_end(),
// 完成多时间步 stare + PPI 混合场景的整条数据链。
//
// 用法:
//   build\lidar_run_full_demo [--config configs/default_pipeline.json] [--output-root .]

#include "example_common.hpp"
#include "lidar_demo/lidar_demo.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

using namespace example_common;

namespace {

#ifdef _WIN32
constexpr char PATH_SEP = '\\';
#else
constexpr char PATH_SEP = '/';
#endif

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

int run_subprocess(const std::string& exe_name) {
    std::string path = exe_in_build_dir(exe_name);
    int rc = std::system(path.c_str());
    if (rc != 0) {
        std::cerr << "[WARN] " << exe_name << " exited with code " << rc << "\n";
    }
    return rc;
}

}  // namespace

int main(int argc, char** argv) {
    enable_ansi_color();

    std::filesystem::path config_path = "configs/default_pipeline.json";
    std::filesystem::path output_root = ".";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (a == "--output-root" && i + 1 < argc) output_root = argv[++i];
    }

    auto stage_header = [](const std::string& title) {
        std::cout << "\n========================================================================\n";
        std::cout << " " << title << "\n";
        std::cout << "========================================================================\n";
    };

    stage_header("Stage 1: 1D ray example (sections 19.1-19.15)");
    run_subprocess("lidar_example_1d_ray");

    stage_header("Stage 2: 2D PPI example (sections 19.17-19.27)");
    run_subprocess("lidar_example_2d_ppi");

    stage_header("Stage 3: 2D RHI example (sections 19.28-19.37)");
    run_subprocess("lidar_example_3d_rhi");

    stage_header("Stage 4: full pipeline (lidar_demo::run_end_to_end)");
    try {
        lidar_demo::Json config_json = lidar_demo::read_json_file(config_path);
        lidar_demo::PipelineConfig cfg = lidar_demo::parse_pipeline_config(config_json);
        std::cout << "config: " << config_path.string() << "\n";
        std::cout << "site:   " << cfg.site.name << " ("
                  << cfg.site.latitude_deg << ", " << cfg.site.longitude_deg << ")\n";
        std::cout << "time_steps=" << cfg.simulation.time_steps
                  << " range_bins=" << cfg.simulation.range_bin_count << "\n";
        auto t0 = std::chrono::steady_clock::now();
        lidar_demo::Json results = lidar_demo::run_end_to_end(cfg, output_root);
        auto t1 = std::chrono::steady_clock::now();
        double sec = std::chrono::duration<double>(t1 - t0).count();

        const auto& m = results.at("metrics");
        printf("elapsed_s = %.3f\n", sec);
        printf("PM2.5    R2 = %s\n", m.at("pm25").at("r2").dump().c_str());
        printf("PM2.5  RMSE = %s\n", m.at("pm25").at("rmse").dump().c_str());
        printf("PM10     R2 = %s\n", m.at("pm10").at("r2").dump().c_str());
        printf("Hotspot  F1 = %s\n", m.at("hotspot").at("f1").dump().c_str());
        printf("latency_ms = %s\n", m.at("runtime").at("mean_latency_ms").dump().c_str());
        std::cout << "L1 -> data/l1/demo_preprocessed.json\n";
        std::cout << "L2 -> data/l2/demo_results.json\n";
        std::cout << "dashboard -> web/demo_dashboard.html\n";
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] pipeline stage failed: " << e.what() << "\n";
        return 1;
    }

    stage_header("summary: full demo complete");
    std::cout << "example outputs:\n";

    // 列产物
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
