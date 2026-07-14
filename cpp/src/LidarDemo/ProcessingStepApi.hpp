// YLJ5 正演与实时处理公共 API 的实现包装。
// 这里保持类接口轻量，具体公式集中在 RealtimeProcessingAlgorithms.hpp 中便于整体审阅。

PreprocessResult BackgroundPreprocessStep::process(const LidarProfile& profile) const {
    // 单主通道兼容入口；实时四通道设备主要由 FrameProcessor 的 gluing 链处理。
    return preprocess_profile(profile);
}

FernaldInversionStep::FernaldInversionStep(RetrievalConfig config)
    : config_(std::move(config)) {
}

std::pair<std::vector<double>, std::vector<double>> FernaldInversionStep::process(
    const LidarProfile& profile,
    const std::vector<double>& attenuated_backscatter,
    const std::vector<BinQualityMask>& bin_quality) const {
    // config_ 在构造时按值保存，保证一条射线反演期间参数不会被外部修改。
    return run_fernald_inversion(
        profile,
        attenuated_backscatter,
        bin_quality,
        config_);
}

HumidityCorrectionStep::HumidityCorrectionStep(HumidityConfig config)
    : config_(std::move(config)) {
}

std::vector<double> HumidityCorrectionStep::process(
    const std::vector<double>& extinction,
    double relative_humidity) const {
    // 湿度对整条射线使用同一环境值；若未来有距离分辨湿度场，应扩展接口而非在此插值猜测。
    return apply_humidity_correction(
        extinction,
        relative_humidity,
        config_.dry_reference_rh,
        config_.hygroscopicity);
}

std::vector<std::vector<double>> CoordinateProjectionStep::process(
    const LidarProfile& profile) const {
    // 只做当前射线局部 ENU 几何投影，不修改科学产品或质量位。
    return profile_bins_to_enu(profile);
}

std::vector<Hotspot> detect_hotspots_from_processed(
    const std::vector<ProcessedProfile>& ppi_profiles,
    const HotspotConfig& hotspot_config) {
    // 内部算法只读廓线，转换为指针视图可避免复制每条 5334-bin ProcessedProfile。
    std::vector<const ProcessedProfile*> pointers;
    pointers.reserve(ppi_profiles.size());
    for (const auto& profile : ppi_profiles) pointers.push_back(&profile);
    // 绝对 PM、相对 PM、相对光学增强和最小连通面积四项门限统一从 HotspotConfig 注入。
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
