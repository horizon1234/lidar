#include "lidar_demo/lidar_demo.hpp"

#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    try {
        std::filesystem::path config_path = "configs/default_pipeline.json";
        std::filesystem::path output_root = ".";
        std::optional<std::filesystem::path> dashboard_path;

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

        lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(lidar_demo::read_json_file(config_path));
        lidar_demo::Json results = lidar_demo::run_end_to_end(config, output_root);
        std::filesystem::path final_dashboard = dashboard_path.value_or(output_root / "web" / "demo_dashboard.html");
        std::filesystem::create_directories(final_dashboard.parent_path());
        std::ofstream handle(final_dashboard, std::ios::binary);
        handle << lidar_demo::render_dashboard(results);
        std::cout << "Generated " << final_dashboard.string() << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}