#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace lidar_demo {

struct Json {
    using array_type = std::vector<Json>;
    using object_type = std::map<std::string, Json>;
    using value_type = std::variant<std::nullptr_t, bool, double, std::string, array_type, object_type>;

    value_type value;

    Json();
    Json(std::nullptr_t);
    Json(bool input);
    Json(int input);
    Json(double input);
    Json(const char* input);
    Json(const std::string& input);
    Json(std::string&& input);
    Json(const array_type& input);
    Json(array_type&& input);
    Json(const object_type& input);
    Json(object_type&& input);

    bool is_null() const;
    bool is_bool() const;
    bool is_number() const;
    bool is_string() const;
    bool is_array() const;
    bool is_object() const;

    bool bool_value() const;
    double number_value() const;
    int int_value() const;
    const std::string& string_value() const;
    const array_type& array_items() const;
    array_type& array_items();
    const object_type& object_items() const;
    object_type& object_items();

    bool contains(const std::string& key) const;
    const Json& at(const std::string& key) const;
    Json& operator[](const std::string& key);
    const Json& operator[](const std::string& key) const;
};

Json parse_json(const std::string& text);
std::string dump_json(const Json& value, int indent = 2);
Json read_json_file(const std::filesystem::path& path);
void write_json_file(const std::filesystem::path& path, const Json& value);

struct SiteInfo {
    std::string name;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    std::string site_id;
};

struct CloudnetSourceConfig {
    std::string site_id;
    std::string site_name;
    std::string date;
    bool verify_ssl = true;
    std::string local_file;
    std::string download_url;
    int time_steps = 18;
    int range_bin_count = 30;
    double min_range_m = 75.0;
    double max_range_m = 3200.0;
    double pseudo_signal_scale = 600000.0;
};

struct SourceConfig {
    std::string mode = "simulation";
    std::string root = ".";
    CloudnetSourceConfig cloudnet;
};

struct LidarProfile {
    std::string site_id;
    std::string timestamp;
    std::string scan_id;
    std::string scan_mode;
    std::string source_kind;
    double azimuth_deg = 0.0;
    double elevation_deg = 0.0;
    std::vector<double> ranges_m;
    std::vector<double> raw_counts;
    double laser_energy_mj = 0.0;
    double background_counts = 0.0;
    std::vector<double> overlap;
    double relative_humidity = 0.0;
    double temperature_c = 0.0;
    double wind_speed_ms = 0.0;
    double wind_dir_deg = 0.0;
    std::vector<double> molecular_backscatter;
    std::vector<double> molecular_extinction;
    std::vector<double> true_backscatter;
    std::vector<double> true_extinction;
    std::vector<double> true_pm25;
    std::vector<double> true_pm10;
    std::vector<int> true_hotspot_mask;
};

struct GroundMeasurement {
    std::string site_id;
    std::string timestamp;
    double pm25_ugm3 = 0.0;
    double pm10_ugm3 = 0.0;
    double relative_humidity = 0.0;
    double temperature_c = 0.0;
    double wind_speed_ms = 0.0;
    double wind_dir_deg = 0.0;
};

struct ProcessedProfile {
    LidarProfile profile;
    std::vector<double> l1_signal;
    std::vector<double> attenuated_backscatter;
    std::vector<double> snr;
    std::vector<double> extinction;
    std::vector<double> dry_extinction;
    std::vector<double> pm25;
    std::vector<double> pm10;
    std::vector<std::vector<double>> enu_points_m;
    std::vector<std::string> qc_flags;
    double latency_ms = 0.0;
};

struct Hotspot {
    std::string timestamp;
    std::string scan_id;
    std::string hotspot_id;
    std::vector<double> centroid_enu_m;
    double peak_pm25_ugm3 = 0.0;
    double mean_pm25_ugm3 = 0.0;
    double estimated_area_m2 = 0.0;
    int cell_count = 0;
    std::string severity;
};

struct SimulationConfig {
    int seed = 7;
    int time_steps = 18;
    int minutes_per_step = 20;
    int range_bin_count = 30;
    double range_bin_m = 50.0;
    double ppi_elevation_deg = 8.0;
    double ppi_azimuth_step_deg = 30.0;
    double system_constant = 260000000.0;
    double lidar_ratio_sr = 45.0;
};

struct RetrievalConfig {
    double aerosol_lidar_ratio_sr = 45.0;
    double reference_aerosol_backscatter = 0.0004;
};

struct HumidityConfig {
    double dry_reference_rh = 0.45;
    double hygroscopicity = 1.1;
};

struct PmCalibrationConfig {
    double train_ratio = 0.6;
    double val_ratio = 0.2;
    int surface_bin_count = 6;
};

struct HotspotConfig {
    double pm25_threshold_ugm3 = 50.0;
    double scan_relative_pm25_threshold_ugm3 = 0.18;
    double scan_relative_dry_ext_threshold = 0.02;
    int min_cells = 3;
};

struct EvaluationConfig {
    std::vector<double> sensitivity_lidar_ratios;
};

struct PipelineConfig {
    std::string source_mode = "simulation";
    SourceConfig source;
    SiteInfo site;
    SimulationConfig simulation;
    RetrievalConfig retrieval;
    HumidityConfig humidity;
    PmCalibrationConfig pm_calibration;
    HotspotConfig hotspot;
    EvaluationConfig evaluation;
};

PipelineConfig parse_pipeline_config(const Json& value);
Json run_end_to_end(const PipelineConfig& config, const std::optional<std::filesystem::path>& output_root = std::nullopt);
std::string render_dashboard(const Json& data);
Json build_summary_payload(const Json& results);
Json fetch_public_ground_data(
    double latitude_deg,
    double longitude_deg,
    const std::string& start_date,
    const std::string& end_date,
    const std::string& timezone,
    const std::filesystem::path& output_dir,
    const std::string& prefix
);
Json fetch_cloudnet_public_sample(const PipelineConfig& config, const std::filesystem::path& output_root);

}