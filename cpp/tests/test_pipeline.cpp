#include "lidar_demo/lidar_demo.hpp"

#include <cmath>
#include <iostream>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::filesystem::path config_path = argc > 1 ? argv[1] : std::filesystem::path("configs/default_pipeline.json");
        lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(lidar_demo::read_json_file(config_path));
        lidar_demo::Json results = lidar_demo::run_end_to_end(config);

        const auto& metrics = results.at("metrics");
        require(metrics.contains("pm25"), "Missing pm25 metrics");
        require(metrics.contains("pm10"), "Missing pm10 metrics");
        require(metrics.contains("hotspot"), "Missing hotspot metrics");
        require(metrics.at("runtime").at("throughput_profiles_per_s").number_value() > 0.0, "Invalid throughput");

        require(metrics.at("pm25").at("rmse").number_value() < 20.0, "PM2.5 RMSE too high");
        require(metrics.at("pm10").at("rmse").number_value() < 25.0, "PM10 RMSE too high");
        require(metrics.at("pm25").at("r2").number_value() > 0.7, "PM2.5 R2 too low");

        require(metrics.at("hotspot").at("f1").number_value() > 0.55, "Hotspot F1 too low");
        require(static_cast<int>(results.at("alerts").array_items().size()) >= 1, "No hotspot alerts produced");

        const auto& curtain = results.at("curtain");
        require(curtain.at("times").array_items().size() == static_cast<std::size_t>(results.at("dataset_summary").at("timestamp_count").int_value()), "Curtain time count mismatch");
        require(curtain.at("pm25").array_items().size() == curtain.at("times").array_items().size(), "Curtain pm25 shape mismatch");
        require(results.at("ppi").at("cells").array_items().size() > 100, "Insufficient PPI cells");

        std::cout << "All C++ pipeline assertions passed." << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}