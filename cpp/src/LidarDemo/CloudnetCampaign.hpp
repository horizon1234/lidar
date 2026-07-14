// Internal include fragment: Cloudnet hybrid campaign loader.
#if defined(LIDAR_DEMO_HAS_NETCDF)

CampaignData load_cloudnet_hybrid_campaign(const PipelineConfig& config) {
    std::filesystem::path source_root = std::filesystem::absolute(config.source.root.empty() ? std::filesystem::path(".") : std::filesystem::path(config.source.root));
    std::filesystem::path local_file = source_root / config.source.cloudnet.local_file;
    std::string day_token = strip_hyphens(config.source.cloudnet.date);
    std::filesystem::path ground_json_path = source_root / "data" / "public" / "cloudnet" / ("open_meteo_" + day_token + "_ground_pm_meteo.json");
    if (!std::filesystem::exists(local_file) || !std::filesystem::exists(ground_json_path)) {
        fetch_cloudnet_public_sample(config, ".");
    }
    if (!std::filesystem::exists(local_file)) {
        throw std::runtime_error("Cloudnet .nc file not found for C++ path after download attempt: " + local_file.string());
    }
    if (!std::filesystem::exists(ground_json_path)) {
        throw std::runtime_error("Aligned Open-Meteo ground file not found for Cloudnet C++ path after fetch attempt: " + ground_json_path.string());
    }

    NetcdfFile dataset(netcdf_compatible_path(local_file));
    int ncid = dataset.id();
    int time_varid = variable_id(ncid, "time");
    int beta_varid = variable_id(ncid, "beta");
    std::string time_units = read_text_attribute(ncid, time_varid, "units");
    std::vector<double> time_values = read_numeric_variable(ncid, "time");
    std::vector<double> range_values_m = read_numeric_variable(ncid, "range");
    double zenith_angle_deg = read_optional_scalar_numeric_variable(ncid, "zenith_angle", 0.0);
    double elevation_deg = 90.0 - zenith_angle_deg;

    CampaignData data;
    data.site = SiteInfo{
        config.source.cloudnet.site_name,
        read_scalar_numeric_variable(ncid, "latitude"),
        read_scalar_numeric_variable(ncid, "longitude"),
        read_scalar_numeric_variable(ncid, "altitude"),
        config.source.cloudnet.site_id,
    };

    std::vector<GroundRecord> ground_records = read_ground_records(ground_json_path);

    std::vector<int> time_indices = select_even_indices(static_cast<int>(time_values.size()), config.source.cloudnet.time_steps);
    std::vector<int> range_indices = filter_range_indices(
        range_values_m,
        config.source.cloudnet.min_range_m,
        config.source.cloudnet.max_range_m,
        config.source.cloudnet.range_bin_count
    );
    std::vector<double> ranges_m;
    ranges_m.reserve(range_indices.size());
    for (int index : range_indices) {
        ranges_m.push_back(range_values_m[static_cast<std::size_t>(index)]);
    }
    std::vector<double> overlap = build_overlap(ranges_m, config.simulation.full_overlap_m, config.simulation.min_overlap);
    auto [molecular_extinction, molecular_backscatter] = standard_molecular_fields(ranges_m, data.site.altitude_m, elevation_deg);

    for (int time_index : time_indices) {
        std::string timestamp = cloudnet_time_to_iso_minute(time_values[static_cast<std::size_t>(time_index)], time_units);
        const GroundRecord& matched = nearest_ground_record(parse_timestamp(timestamp), ground_records);
        data.ground_measurements.push_back(GroundMeasurement{
            config.source.cloudnet.site_id,
            timestamp,
            matched.pm25,
            matched.pm10,
            matched.relative_humidity / 100.0,
            matched.temperature_c,
            matched.wind_speed_ms,
            matched.wind_dir_deg,
        });
    }

    std::map<std::string, GroundMeasurement> ground_by_timestamp;
    for (const auto& measurement : data.ground_measurements) {
        ground_by_timestamp[measurement.timestamp] = measurement;
    }

    for (std::size_t order = 0; order < time_indices.size(); ++order) {
        int time_index = time_indices[order];
        std::string timestamp = cloudnet_time_to_iso_minute(time_values[static_cast<std::size_t>(time_index)], time_units);
        const GroundMeasurement& ground = ground_by_timestamp.at(timestamp);
        std::vector<double> beta_values_full = read_beta_row(ncid, beta_varid, time_index, range_values_m.size());
        std::vector<double> beta_values;
        beta_values.reserve(range_indices.size());
        for (int index : range_indices) {
            beta_values.push_back(beta_values_full[static_cast<std::size_t>(index)]);
        }

        double background_counts = 5.0 + 0.15 * static_cast<double>(order % 5);
        double laser_energy_mj = 1.0 + 0.01 * std::sin(static_cast<double>(order));
        std::vector<double> raw_counts;
        std::vector<double> approx_extinction;
        std::vector<double> combined_backscatter;
        raw_counts.reserve(beta_values.size());
        approx_extinction.reserve(beta_values.size());
        combined_backscatter.reserve(beta_values.size());
        for (std::size_t index = 0; index < beta_values.size(); ++index) {
            double range_km = ranges_m[index] / 1000.0;
            raw_counts.push_back(background_counts + beta_values[index] * config.source.cloudnet.pseudo_signal_scale * overlap[index] / std::max(range_km * range_km, 1e-6));
            double aerosol_ext = beta_values[index] * config.retrieval.aerosol_lidar_ratio_sr;
            approx_extinction.push_back(molecular_extinction[index] + aerosol_ext);
            combined_backscatter.push_back(molecular_backscatter[index] + beta_values[index]);
        }

        data.profiles.push_back(LidarProfile{
            config.source.cloudnet.site_id,
            timestamp,
            timestamp + "_stare_real",
            "stare",
            "cloudnet_real_stare",
            0.0,
            elevation_deg,
            ranges_m,
            raw_counts,
            laser_energy_mj,
            background_counts,
            overlap,
            ground.relative_humidity,
            ground.temperature_c,
            ground.wind_speed_ms,
            ground.wind_dir_deg,
            molecular_backscatter,
            molecular_extinction,
            combined_backscatter,
            approx_extinction,
            std::vector<double>(ranges_m.size(), 0.0),
            std::vector<double>(ranges_m.size(), 0.0),
            std::vector<int>(ranges_m.size(), 0),
        });
    }

    std::mt19937 rng(config.simulation.seed + 101);
    for (std::size_t step_index = 0; step_index < data.ground_measurements.size(); ++step_index) {
        const GroundMeasurement& ground = data.ground_measurements[step_index];
        // 多仰角体积/扇区扫描：每个仰角按配置方位角扫描
        const std::vector<double> azimuths = config.simulation.effective_ppi_azimuths_deg();
        for (double elevation : config.simulation.effective_ppi_elevations_deg()) {
            for (double azimuth : azimuths) {
                SimulatedFields fields = simulate_profile_fields(
                    ranges_m,
                    azimuth,
                    elevation,
                    static_cast<int>(step_index),
                    static_cast<int>(data.ground_measurements.size()),
                    ground.relative_humidity,
                    config.simulation.lidar_ratio_sr,
                    ground.wind_speed_ms,
                    ground.wind_dir_deg,
                    config.simulation
                );
                double background_counts = std::max(2.0,
                    config.simulation.background_counts_mean
                    + sample_gaussian(rng, 0.0, config.simulation.background_counts_jitter));
                double laser_energy_mj = std::max(0.05,
                    config.simulation.pulse_energy_mj
                    * (1.0 + sample_gaussian(rng, 0.0, config.simulation.pulse_energy_jitter)));
                std::vector<double> raw_counts = simulate_raw_counts(
                    fields.true_backscatter,
                    fields.true_extinction,
                    overlap,
                    ranges_m,
                    laser_energy_mj,
                    background_counts,
                    config.simulation.ppi_line_dwell_s,
                    1.0,
                    config.simulation,
                    rng
                );
                auto pad3 = [](double deg) {
                    int v = static_cast<int>(std::round(deg));
                    std::string s = std::to_string(v);
                    while (s.size() < 3) s = "0" + s;
                    return s;
                };
                std::string scan_id = ground.timestamp + "_ppi_e" + pad3(elevation) + "_a" + pad3(azimuth);
                data.profiles.push_back(LidarProfile{
                    config.source.cloudnet.site_id,
                    ground.timestamp,
                    scan_id,
                    "ppi",
                    "synthetic_ppi_hybrid",
                    azimuth,
                    elevation,
                    ranges_m,
                    raw_counts,
                    laser_energy_mj,
                    background_counts,
                    overlap,
                    ground.relative_humidity,
                    ground.temperature_c,
                    ground.wind_speed_ms,
                    ground.wind_dir_deg,
                    fields.molecular_backscatter,
                    fields.molecular_extinction,
                    fields.true_backscatter,
                    fields.true_extinction,
                    fields.true_pm25,
                    fields.true_pm10,
                    fields.true_hotspot_mask,
                });
            }
        }
    }

    std::stable_sort(data.profiles.begin(), data.profiles.end(), [](const LidarProfile& left, const LidarProfile& right) {
        if (left.timestamp != right.timestamp) {
            return left.timestamp < right.timestamp;
        }
        if (left.scan_mode != right.scan_mode) {
            return left.scan_mode < right.scan_mode;
        }
        return left.azimuth_deg < right.azimuth_deg;
    });

    int ppi_count = 0;
    for (const auto& profile : data.profiles) {
        if (profile.scan_mode == "ppi") {
            ++ppi_count;
        }
    }
    data.source_metadata = Json::Object{
        {"mode", "cloudnet_hybrid"},
        {"site_id", data.site.site_id},
        {"site_name", data.site.name},
        {"cloudnet_file", local_file.string()},
        {"measurement_date", config.source.cloudnet.date},
        {"ground_provider", "open-meteo"},
        {"instrument_preset", config.simulation.instrument_preset},
        {"application_mode", config.simulation.application_mode},
        {"vendor_profile", config.simulation.vendor_profile},
        {"wavelength_nm", config.simulation.wavelength_nm},
        {"range_bin_m", config.simulation.range_bin_m},
        {"full_overlap_m", config.simulation.full_overlap_m},
        {"ppi_elevations_deg", [&]() {
            Json::Array values;
            for (double elevation : config.simulation.effective_ppi_elevations_deg()) {
                values.emplace_back(elevation);
            }
            return Json(std::move(values));
        }()},
        {"ppi_azimuth_start_deg", config.simulation.ppi_azimuth_start_deg},
        {"ppi_azimuth_stop_deg", config.simulation.ppi_azimuth_stop_deg},
        {"ppi_azimuth_step_deg", config.simulation.ppi_azimuth_step_deg},
        {"ppi_line_dwell_s", config.simulation.ppi_line_dwell_s},
        {"ppi_step_overhead_s", config.simulation.ppi_step_overhead_s},
        {"ppi_scan_overhead_s", config.simulation.ppi_scan_overhead_s},
        {"ppi_scan_cycle_s", config.simulation.ppi_scan_cycle_seconds()},
        {"pulse_repetition_hz", config.simulation.pulse_repetition_hz},
        {"integrated_pulses_per_line", static_cast<int>(std::round(config.simulation.pulse_repetition_hz * config.simulation.ppi_line_dwell_s))},
        {"real_stare_profile_count", static_cast<int>(data.ground_measurements.size())},
        {"synthetic_ppi_profile_count", ppi_count},
    };
    return data;
}

#endif
