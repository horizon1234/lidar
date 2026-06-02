#include "lidar_demo/lidar_demo.hpp"

#include <iostream>
#include <optional>

int main(int argc, char** argv) {
    try {
        std::filesystem::path config_path = "configs/default_pipeline.json";
        std::filesystem::path output_root = ".";
        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--config" && index + 1 < argc) {
                config_path = argv[++index];
            } else if (argument == "--output" && index + 1 < argc) {
                output_root = argv[++index];
            }
        }

        lidar_demo::Json config_json = lidar_demo::read_json_file(config_path);
        lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(config_json);
        lidar_demo::Json results = lidar_demo::run_end_to_end(config, output_root);
        lidar_demo::write_json_file(output_root / "data" / "l2" / "worker_summary.json", results.at("metrics"));
        std::cout << lidar_demo::dump_json(results.at("metrics"), 2) << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}