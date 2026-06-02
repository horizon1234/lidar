#include "lidar_demo/lidar_demo.hpp"

#include <iostream>

int main(int argc, char** argv) {
    try {
        std::filesystem::path config_path = "configs/cloudnet_hybrid_pipeline.json";
        std::filesystem::path output_root = ".";

        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--config" && index + 1 < argc) {
                config_path = argv[++index];
            } else if (argument == "--output-root" && index + 1 < argc) {
                output_root = argv[++index];
            }
        }

        lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(lidar_demo::read_json_file(config_path));
        lidar_demo::Json manifest = lidar_demo::fetch_cloudnet_public_sample(config, output_root);
        std::cout << lidar_demo::dump_json(manifest, 2) << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}