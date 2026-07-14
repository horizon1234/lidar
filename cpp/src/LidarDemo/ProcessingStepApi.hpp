// YLJ5 正演与实时处理公共 API 的实现包装。

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
        config_.reference_aerosol_backscatter);
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
        config_.hygroscopicity);
}

std::vector<std::vector<double>> CoordinateProjectionStep::process(
    const LidarProfile& profile) const {
    return profile_bins_to_enu(profile);
}

std::vector<Hotspot> detect_hotspots_from_processed(
    const std::vector<ProcessedProfile>& ppi_profiles,
    const HotspotConfig& hotspot_config) {
    std::vector<const ProcessedProfile*> pointers;
    pointers.reserve(ppi_profiles.size());
    for (const auto& profile : ppi_profiles) pointers.push_back(&profile);
    return detect_hotspots(
        pointers,
        hotspot_config.pm25_threshold_ugm3,
        hotspot_config.scan_relative_pm25_threshold_ugm3,
        hotspot_config.scan_relative_dry_ext_threshold,
        hotspot_config.min_cells);
}

SyntheticCampaign generate_synthetic_campaign(const PipelineConfig& config) {
    CampaignData campaign = simulate_campaign(config);
    return SyntheticCampaign{
        std::move(campaign.site),
        std::move(campaign.profiles),
        std::move(campaign.ground_measurements),
    };
}
