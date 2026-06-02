#include "lidar_demo/lidar_demo.hpp"

#include <iostream>

int main(int argc, char** argv) {
    try {
        double latitude = 39.9042;
        double longitude = 116.4074;
        std::string start_date = "2026-05-30";
        std::string end_date = "2026-05-31";
        std::string timezone = "Asia/Shanghai";
        std::string prefix = "open_meteo_beijing";
        std::filesystem::path output_root = ".";

        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--latitude" && index + 1 < argc) {
                latitude = std::stod(argv[++index]);
            } else if (argument == "--longitude" && index + 1 < argc) {
                longitude = std::stod(argv[++index]);
            } else if (argument == "--start-date" && index + 1 < argc) {
                start_date = argv[++index];
            } else if (argument == "--end-date" && index + 1 < argc) {
                end_date = argv[++index];
            } else if (argument == "--timezone" && index + 1 < argc) {
                timezone = argv[++index];
            } else if (argument == "--prefix" && index + 1 < argc) {
                prefix = argv[++index];
            } else if (argument == "--output-root" && index + 1 < argc) {
                output_root = argv[++index];
            }
        }

        lidar_demo::Json manifest = lidar_demo::fetch_public_ground_data(
            latitude,
            longitude,
            start_date,
            end_date,
            timezone,
            output_root / "data" / "public",
            prefix
        );
        std::cout << lidar_demo::dump_json(manifest, 2) << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}