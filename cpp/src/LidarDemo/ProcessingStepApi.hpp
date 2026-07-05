// Public API implementation fragment: realtime processing step wrappers.
// ============================================================================
// 单步处理 API —— 供实时客户端逐帧处理使用
// ============================================================================

PreprocessResult BackgroundPreprocessStep::process(const LidarProfile& profile) const {
    return preprocess_profile(profile);
}

FernaldInversionStep::FernaldInversionStep(RetrievalConfig config)
    : config_(std::move(config)) {
}

std::pair<std::vector<double>, std::vector<double>> FernaldInversionStep::process(
    const LidarProfile& profile,
    const std::vector<double>& attenuated_backscatter) const {
    return run_fernald_inversion(
        profile,
        attenuated_backscatter,
        config_.aerosol_lidar_ratio_sr,
        config_.reference_aerosol_backscatter
    );
}

HumidityCorrectionStep::HumidityCorrectionStep(HumidityConfig config)
    : config_(std::move(config)) {
}

std::vector<double> HumidityCorrectionStep::process(
    const std::vector<double>& extinction,
    double relative_humidity) const {
    return apply_humidity_correction(
        extinction,
        relative_humidity,
        config_.dry_reference_rh,
        config_.hygroscopicity
    );
}

std::vector<std::vector<double>> CoordinateProjectionStep::process(const LidarProfile& profile) const {
    return profile_bins_to_enu(profile);
}

HotspotDetectionStep::HotspotDetectionStep(HotspotConfig config)
    : config_(std::move(config)) {
}

std::vector<Hotspot> HotspotDetectionStep::process(const std::vector<ProcessedProfile>& ppi_profiles) const {
    return detect_hotspots_from_processed(ppi_profiles, config_);
}

SingleProfileProcessingChain::SingleProfileProcessingChain(RetrievalConfig retrieval, HumidityConfig humidity)
    : inversion_(std::move(retrieval)),
      humidity_(std::move(humidity)) {
}

ProcessedProfile SingleProfileProcessingChain::process(const LidarProfile& profile, bool disable_humidity) const {
    auto started_at = std::chrono::steady_clock::now();
    PreprocessResult preprocessed = preprocess_.process(profile);
    auto inversion = inversion_.process(profile, preprocessed.attenuated_backscatter);
    std::vector<double> dry_extinction = disable_humidity
        ? inversion.first
        : humidity_.process(inversion.first, profile.relative_humidity);

    std::vector<double> pm25;
    std::vector<double> pm10;
    pm25.reserve(dry_extinction.size());
    pm10.reserve(dry_extinction.size());
    for (double ext : dry_extinction) {
        double est_pm25 = std::max(0.0, ext) * 25.0;
        pm25.push_back(est_pm25);
        pm10.push_back(est_pm25 * 1.5);
    }

    auto ended_at = std::chrono::steady_clock::now();
    return ProcessedProfile{
        profile,
        preprocessed.l1_signal,
        preprocessed.attenuated_backscatter,
        preprocessed.snr,
        inversion.first,
        dry_extinction,
        std::move(pm25),
        std::move(pm10),
        projection_.process(profile),
        preprocessed.qc_flags,
        std::chrono::duration<double, std::milli>(ended_at - started_at).count(),
    };
}

ProcessedProfile process_single_profile(
    const LidarProfile& profile,
    const RetrievalConfig& retrieval,
    const HumidityConfig& humidity
) {
    return SingleProfileProcessingChain(retrieval, humidity).process(profile);
}

std::vector<Hotspot> detect_hotspots_from_processed(
    const std::vector<ProcessedProfile>& ppi_profiles,
    const HotspotConfig& hotspot_cfg
) {
    // detect_hotspots 接受 ProcessedProfile* 指针列表
    std::vector<ProcessedProfile*> ptrs;
    ptrs.reserve(ppi_profiles.size());
    for (const auto& p : ppi_profiles) {
        ptrs.push_back(const_cast<ProcessedProfile*>(&p));
    }

    DetectionResult detection = detect_hotspots(
        ptrs,
        hotspot_cfg.pm25_threshold_ugm3,
        hotspot_cfg.scan_relative_pm25_threshold_ugm3,
        hotspot_cfg.scan_relative_dry_ext_threshold,
        hotspot_cfg.min_cells
    );
    return detection.hotspots;
}

std::vector<GroundMeasurement> extract_ground_measurements(const Json& results) {
    std::vector<GroundMeasurement> output;
    if (results.contains("source") && results.at("source").contains("ground_measurements")) {
        const auto& arr = results.at("source").at("ground_measurements");
        if (arr.is_array()) {
            for (const auto& item : arr.array_items()) {
                GroundMeasurement gm;
                gm.site_id = item.contains("site_id") ? item.at("site_id").string_value() : "";
                gm.timestamp = item.contains("timestamp") ? item.at("timestamp").string_value() : "";
                gm.pm25_ugm3 = item.contains("pm25_ugm3") ? item.at("pm25_ugm3").number_value() : 0.0;
                gm.pm10_ugm3 = item.contains("pm10_ugm3") ? item.at("pm10_ugm3").number_value() : 0.0;
                gm.relative_humidity = item.contains("relative_humidity") ? item.at("relative_humidity").number_value() : 0.0;
                gm.temperature_c = item.contains("temperature_c") ? item.at("temperature_c").number_value() : 0.0;
                gm.wind_speed_ms = item.contains("wind_speed_ms") ? item.at("wind_speed_ms").number_value() : 0.0;
                gm.wind_dir_deg = item.contains("wind_dir_deg") ? item.at("wind_dir_deg").number_value() : 0.0;
                output.push_back(std::move(gm));
            }
        }
    }
    return output;
}

Json to_json_processed(const ProcessedProfile& value) {
    return to_json(value);
}

Json to_json_hotspot(const Hotspot& value) {
    return to_json(value);
}

