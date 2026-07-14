// Public API implementation fragment: pipeline configuration parser.
// ---- 配置解析 (JSON -> PipelineConfig) ----

/**
 * @brief 把流水线 JSON 配置反序列化为 PipelineConfig 结构（用户面对的核心入口之一）。
 *
 * 严格字段直接用 .at(...) 读取（缺失会抛错）；可选字段用 source.contains(x) ? ... : 默认值
 * 的三目模式兜底，方便用户写最小配置。本函数同时填充 config.source_mode 与
 * config.source.mode 两个等价字段，兼容老代码。
 *
 * 解析的子段：
 *  - source / source.cloudnet：数据来源方式与 Cloudnet 子项（含默认值）；
 *  - site：站点元信息（site_id 缺省时由 name slugify 自动生成）；
 *  - simulation：模拟参数（种子、时间步、PPI 仰角/方位角步长等）；
 *  - retrieval：Fernald 反演参数；
 *  - humidity：湿度修正参数；
 *  - pm_calibration：PM 模型训练比例与 bin 数；
 *  - hotspot：热点检测阈值；
 *  - evaluation：敏感性扫描使用的 lidar ratio 列表。
 *
 * @param value 已解析的根 JSON 对象。
 * @return PipelineConfig 完整配置，可直接用于后续 run_end_to_end。
 */
PipelineConfig parse_pipeline_config(const Json& value) {
    PipelineConfig config;
    // source_mode 顶层存放，便于外层快速判断走 simulation / cloudnet_hybrid 等分支
    if (value.contains("source") && value.at("source").is_object() && value.at("source").contains("mode")) {
        config.source_mode = value.at("source").at("mode").string_value();
    }
    config.source.mode = config.source_mode;  ///< 同步到 source 子结构以兼容内部代码
    if (value.contains("source") && value.at("source").is_object()) {
        const Json& source = value.at("source");
        // root 缺省为 "."（当前目录）
        config.source.root = source.contains("root") ? source.at("root").string_value() : ".";
        if (source.contains("cloudnet") && source.at("cloudnet").is_object()) {
            // Cloudnet 子段：每个字段都提供合理默认值，降低用户配置负担
            const Json& cloudnet = source.at("cloudnet");
            config.source.cloudnet.site_id = cloudnet.contains("site_id") ? cloudnet.at("site_id").string_value() : "";
            config.source.cloudnet.site_name = cloudnet.contains("site_name") ? cloudnet.at("site_name").string_value() : "";
            config.source.cloudnet.date = cloudnet.contains("date") ? cloudnet.at("date").string_value() : "";
            config.source.cloudnet.verify_ssl = cloudnet.contains("verify_ssl") ? cloudnet.at("verify_ssl").bool_value() : true;
            config.source.cloudnet.local_file = cloudnet.contains("local_file") ? cloudnet.at("local_file").string_value() : "";
            config.source.cloudnet.download_url = cloudnet.contains("download_url") ? cloudnet.at("download_url").string_value() : "";
            config.source.cloudnet.time_steps = cloudnet.contains("time_steps") ? cloudnet.at("time_steps").int_value() : 18;
            config.source.cloudnet.range_bin_count = cloudnet.contains("range_bin_count") ? cloudnet.at("range_bin_count").int_value() : 30;
            config.source.cloudnet.min_range_m = cloudnet.contains("min_range_m") ? cloudnet.at("min_range_m").number_value() : 75.0;
            config.source.cloudnet.max_range_m = cloudnet.contains("max_range_m") ? cloudnet.at("max_range_m").number_value() : 3200.0;
            config.source.cloudnet.pseudo_signal_scale = cloudnet.contains("pseudo_signal_scale") ? cloudnet.at("pseudo_signal_scale").number_value() : 600000.0;  ///< 把 Cloudnet 消光缩放到模拟器近似信号尺度的经验系数
        }
    }

    // 站点（必填字段）
    const Json& site = value.at("site");
    config.site.name = site.at("name").string_value();
    config.site.latitude_deg = site.at("latitude_deg").number_value();
    config.site.longitude_deg = site.at("longitude_deg").number_value();
    config.site.altitude_m = site.at("altitude_m").number_value();
    // site_id 缺省时用 name 的 slug 形式自动生成
    config.site.site_id = site.contains("site_id") ? site.at("site_id").string_value() : slugify(config.site.name);

    // 模拟参数（必填）
    const Json& simulation = value.at("simulation");
    config.simulation.instrument_preset = simulation.contains("instrument_preset") ? simulation.at("instrument_preset").string_value() : config.simulation.instrument_preset;
    config.simulation.application_mode = simulation.contains("application_mode") ? simulation.at("application_mode").string_value() : config.simulation.application_mode;
    config.simulation.vendor_profile = simulation.contains("vendor_profile") ? simulation.at("vendor_profile").string_value() : config.simulation.vendor_profile;
    config.simulation.seed = simulation.at("seed").int_value();
    config.simulation.time_steps = simulation.at("time_steps").int_value();
    config.simulation.minutes_per_step = simulation.at("minutes_per_step").int_value();
    config.simulation.start_step_index = simulation.contains("start_step_index") ? simulation.at("start_step_index").int_value() : config.simulation.start_step_index;
    config.simulation.phase_time_steps = simulation.contains("phase_time_steps") ? simulation.at("phase_time_steps").int_value() : config.simulation.phase_time_steps;
    config.simulation.range_bin_count = simulation.at("range_bin_count").int_value();
    config.simulation.range_bin_m = simulation.at("range_bin_m").number_value();
    // PPI 仰角：优先读取多仰角数组 ppi_elevations_deg；
    // 旧配置仅含单值 ppi_elevation_deg 时，退化为单元素数组以保持向后兼容。
    config.simulation.ppi_elevations_deg.clear();
    if (simulation.contains("ppi_elevations_deg") && simulation.at("ppi_elevations_deg").is_array()) {
        for (const auto& elv : simulation.at("ppi_elevations_deg").array_items()) {
            config.simulation.ppi_elevations_deg.push_back(elv.number_value());
        }
    } else if (simulation.contains("ppi_elevation_deg")) {
        config.simulation.ppi_elevations_deg.push_back(simulation.at("ppi_elevation_deg").number_value());
    }
    config.simulation.ppi_azimuth_start_deg = simulation.contains("ppi_azimuth_start_deg") ? simulation.at("ppi_azimuth_start_deg").number_value() : config.simulation.ppi_azimuth_start_deg;
    config.simulation.ppi_azimuth_stop_deg = simulation.contains("ppi_azimuth_stop_deg") ? simulation.at("ppi_azimuth_stop_deg").number_value() : config.simulation.ppi_azimuth_stop_deg;
    config.simulation.ppi_azimuth_step_deg = simulation.at("ppi_azimuth_step_deg").number_value();
    config.simulation.ppi_line_dwell_s = simulation.contains("ppi_line_dwell_s") ? simulation.at("ppi_line_dwell_s").number_value() : config.simulation.ppi_line_dwell_s;
    config.simulation.ppi_step_overhead_s = simulation.contains("ppi_step_overhead_s") ? simulation.at("ppi_step_overhead_s").number_value() : config.simulation.ppi_step_overhead_s;
    config.simulation.ppi_scan_overhead_s = simulation.contains("ppi_scan_overhead_s") ? simulation.at("ppi_scan_overhead_s").number_value() : config.simulation.ppi_scan_overhead_s;
    config.simulation.include_stare_profile = simulation.contains("include_stare_profile") ? simulation.at("include_stare_profile").bool_value() : config.simulation.include_stare_profile;
    config.simulation.stare_dwell_s = simulation.contains("stare_dwell_s") ? simulation.at("stare_dwell_s").number_value() : config.simulation.stare_dwell_s;
    config.simulation.pulse_repetition_hz = simulation.contains("pulse_repetition_hz") ? simulation.at("pulse_repetition_hz").number_value() : config.simulation.pulse_repetition_hz;
    config.simulation.system_constant = simulation.at("system_constant").number_value();
    config.simulation.lidar_ratio_sr = simulation.at("lidar_ratio_sr").number_value();
    config.simulation.wavelength_nm = simulation.contains("wavelength_nm") ? simulation.at("wavelength_nm").number_value() : config.simulation.wavelength_nm;
    config.simulation.pulse_energy_mj = simulation.contains("pulse_energy_mj") ? simulation.at("pulse_energy_mj").number_value() : config.simulation.pulse_energy_mj;
    config.simulation.pulse_energy_jitter = simulation.contains("pulse_energy_jitter") ? simulation.at("pulse_energy_jitter").number_value() : config.simulation.pulse_energy_jitter;
    config.simulation.background_counts_mean = simulation.contains("background_counts_mean") ? simulation.at("background_counts_mean").number_value() : config.simulation.background_counts_mean;
    config.simulation.background_counts_jitter = simulation.contains("background_counts_jitter") ? simulation.at("background_counts_jitter").number_value() : config.simulation.background_counts_jitter;
    config.simulation.full_overlap_m = simulation.contains("full_overlap_m") ? simulation.at("full_overlap_m").number_value() : config.simulation.full_overlap_m;
    config.simulation.min_overlap = simulation.contains("min_overlap") ? simulation.at("min_overlap").number_value() : config.simulation.min_overlap;
    config.simulation.detector_dark_counts = simulation.contains("detector_dark_counts") ? simulation.at("detector_dark_counts").number_value() : config.simulation.detector_dark_counts;
    config.simulation.read_noise_counts = simulation.contains("read_noise_counts") ? simulation.at("read_noise_counts").number_value() : config.simulation.read_noise_counts;
    config.simulation.adc_saturation_counts = simulation.contains("adc_saturation_counts") ? simulation.at("adc_saturation_counts").number_value() : config.simulation.adc_saturation_counts;
    config.simulation.dead_time_loss = simulation.contains("dead_time_loss") ? simulation.at("dead_time_loss").number_value() : config.simulation.dead_time_loss;
    config.simulation.afterpulsing_ratio = simulation.contains("afterpulsing_ratio") ? simulation.at("afterpulsing_ratio").number_value() : config.simulation.afterpulsing_ratio;
    config.simulation.solar_background_scale = simulation.contains("solar_background_scale") ? simulation.at("solar_background_scale").number_value() : config.simulation.solar_background_scale;
    config.simulation.vehicle_speed_ms = simulation.contains("vehicle_speed_ms") ? simulation.at("vehicle_speed_ms").number_value() : config.simulation.vehicle_speed_ms;
    config.simulation.truth_hotspot_ext_threshold = simulation.contains("truth_hotspot_ext_threshold") ? simulation.at("truth_hotspot_ext_threshold").number_value() : config.simulation.truth_hotspot_ext_threshold;
    config.simulation.enable_ylj5_receiver_channels = simulation.contains("enable_ylj5_receiver_channels") ? simulation.at("enable_ylj5_receiver_channels").bool_value() : config.simulation.enable_ylj5_receiver_channels;
    config.simulation.near_telescope_aperture_mm = simulation.contains("near_telescope_aperture_mm") ? simulation.at("near_telescope_aperture_mm").number_value() : config.simulation.near_telescope_aperture_mm;
    config.simulation.far_telescope_aperture_mm = simulation.contains("far_telescope_aperture_mm") ? simulation.at("far_telescope_aperture_mm").number_value() : config.simulation.far_telescope_aperture_mm;
    config.simulation.near_channel_gain = simulation.contains("near_channel_gain") ? simulation.at("near_channel_gain").number_value() : config.simulation.near_channel_gain;
    config.simulation.near_full_overlap_m = simulation.contains("near_full_overlap_m") ? simulation.at("near_full_overlap_m").number_value() : config.simulation.near_full_overlap_m;
    config.simulation.far_full_overlap_m = simulation.contains("far_full_overlap_m") ? simulation.at("far_full_overlap_m").number_value() : config.simulation.far_full_overlap_m;
    config.simulation.far_min_overlap = simulation.contains("far_min_overlap") ? simulation.at("far_min_overlap").number_value() : config.simulation.far_min_overlap;
    config.simulation.channel_stitch_range_m = simulation.contains("channel_stitch_range_m") ? simulation.at("channel_stitch_range_m").number_value() : config.simulation.channel_stitch_range_m;

    // 反演参数（必填）
    const Json& retrieval = value.at("retrieval");
    config.retrieval.aerosol_lidar_ratio_sr = retrieval.at("aerosol_lidar_ratio_sr").number_value();
    config.retrieval.reference_aerosol_backscatter = retrieval.at("reference_aerosol_backscatter").number_value();

    // 湿度修正参数（必填）
    const Json& humidity = value.at("humidity");
    config.humidity.dry_reference_rh = humidity.at("dry_reference_rh").number_value();
    config.humidity.hygroscopicity = humidity.at("hygroscopicity").number_value();

    // PM 校准切分参数（必填）
    const Json& calibration = value.at("pm_calibration");
    config.pm_calibration.train_ratio = calibration.at("train_ratio").number_value();
    config.pm_calibration.val_ratio = calibration.at("val_ratio").number_value();
    config.pm_calibration.surface_bin_count = calibration.at("surface_bin_count").int_value();

    // 热点检测阈值（必填）
    const Json& hotspot = value.at("hotspot");
    config.hotspot.pm25_threshold_ugm3 = hotspot.at("pm25_threshold_ugm3").number_value();
    config.hotspot.scan_relative_pm25_threshold_ugm3 = hotspot.at("scan_relative_pm25_threshold_ugm3").number_value();
    config.hotspot.scan_relative_dry_ext_threshold = hotspot.at("scan_relative_dry_ext_threshold").number_value();
    config.hotspot.min_cells = hotspot.at("min_cells").int_value();

    // 敏感性扫描对应的 lidar_ratio 列表（数组项数可为 0）
    const Json& evaluation = value.at("evaluation");
    for (const auto& item : evaluation.at("sensitivity_lidar_ratios").array_items()) {
        config.evaluation.sensitivity_lidar_ratios.push_back(item.number_value());
    }
    return config;
}
