// 内部实现片段：合成观测战役生成。
struct CampaignData {
    SiteInfo site;
    std::vector<LidarProfile> profiles;
    std::vector<GroundMeasurement> ground_measurements;
};

/**
 * @brief 生成一整次合成观测战役（CampaignData），含 stare 与 PPI 两类扫描。
 *
 * 设备层通常一次只请求一个周期：可选的 90 度垂直观测、一个固定仰角方位圈，
 * 以及可选的合成地面观测。各气象量沿周期相位变化并叠加可复现噪声。
 *
 * @param[in] config 流水线配置（含 simulation 段：seed、time_steps、ppi 步进、lidar_ratio 等）。
 * @return 当前 YLJ5 周期数据（站点 + 廓线 + 地面观测）。
 */
CampaignData simulate_campaign(const PipelineConfig& config) {
    CampaignData data;
    data.site = config.site;
    // 若未显式指定 site_id，则由站点名 slugify 生成稳定标识
    data.site.site_id = data.site.site_id.empty() ? slugify(data.site.name) : data.site.site_id;

    // 基准时间 2026-05-30 08:00（本地），作为循环起始
    std::tm base_tm{};
    base_tm.tm_year = 2026 - 1900;
    base_tm.tm_mon = 4;
    base_tm.tm_mday = 30;
    base_tm.tm_hour = 8;
    base_tm.tm_min = 0;
    base_tm.tm_isdst = -1;
    std::time_t base_time = std::mktime(&base_tm);

    // 距离 bin：均匀分布，斜距 = range_bin_m * (i+1)
    std::vector<double> ranges_m;
    for (int index = 0; index < config.simulation.range_bin_count; ++index) {
        ranges_m.push_back(config.simulation.range_bin_m * static_cast<double>(index + 1));
    }
    int phase_steps = config.simulation.phase_time_steps > 0
        ? config.simulation.phase_time_steps
        : config.simulation.time_steps;

    for (int local_step_index = 0; local_step_index < config.simulation.time_steps; ++local_step_index) {
        int step_index = config.simulation.start_step_index + local_step_index;
        std::uint32_t step_seed = static_cast<std::uint32_t>(config.simulation.seed)
            ^ (0x9e3779b9U + static_cast<std::uint32_t>(step_index) * 0x85ebca6bU);
        std::mt19937 rng(step_seed);
        std::string timestamp = format_timestamp(base_time + static_cast<long long>(step_index * config.simulation.minutes_per_step * 60));
        // 时间相位：0~2π 一个完整周期
        double time_phase = 2.0 * std::numbers::pi * static_cast<double>(step_index) / std::max(phase_steps, 1);
        // 湿度：约 0.48 + 日振幅 0.20，并裁剪到 [0.28, 0.90] 合理范围
        double relative_humidity = clamp(0.48 + 0.20 * std::sin(time_phase - 0.6) + sample_gaussian(rng, 0.0, 0.015), 0.28, 0.90);
        // 温度：日变化约 ±7°C，基线 27°C，相位偏移 -0.2 错峰模拟典型午后最高
        double temperature_c = 27.0 - 7.0 * std::sin(time_phase - 0.2) + sample_gaussian(rng, 0.0, 0.3);
        // 风速：基线 2.8 m/s，截断防止出现 0 或过大值
        double wind_speed_ms = clamp(2.8 + 1.2 * std::cos(time_phase + 0.3) + sample_gaussian(rng, 0.0, 0.2), 0.6, 6.5);
        // 风向：以 120° 为基线做日摆动，加 360° 取模防止负值
        double wind_dir_deg = std::fmod(120.0 + 45.0 * std::sin(time_phase) + sample_gaussian(rng, 0.0, 4.0) + 360.0, 360.0);
        // 背景计数与激光能量：基线 + 小幅随机抖动，反映仪器状态
        double solar_cycle = 0.65 + 0.35 * (1.0 + std::sin(time_phase - 0.4)) / 2.0;
        const double background_counts = std::max(2.0,
            config.simulation.background_counts_mean * solar_cycle
            + sample_gaussian(rng, 0.0, config.simulation.background_counts_jitter));
        const double laser_energy_mj = std::max(1e-6,
            config.simulation.pulse_energy_mj
            * (1.0 + sample_gaussian(rng, 0.0, config.simulation.pulse_energy_jitter)));

        // -- zenith stare 廓线：用于 PM 反演真值锚定 --
        SimulatedFields stare_fields = simulate_profile_fields(
            ranges_m,
            0.0,
            90.0,
            step_index,
            phase_steps,
            relative_humidity,
            config.simulation.lidar_ratio_sr,
            wind_speed_ms,
            wind_dir_deg,
            config.simulation);
        LidarProfile stare_profile{
            data.site.site_id,
            timestamp,
            timestamp + "_stare",
            "stare",
            "ylj5_synthetic_stare",
            0.0,
            90.0,
            ranges_m,
            {},
            laser_energy_mj,
            background_counts,
            {},
            relative_humidity,
            temperature_c,
            wind_speed_ms,
            wind_dir_deg,
            stare_fields.molecular_backscatter,
            stare_fields.molecular_extinction,
            stare_fields.true_backscatter,
            stare_fields.true_extinction,
            stare_fields.true_pm25,
            stare_fields.true_pm10,
            stare_fields.true_hotspot_mask,
            {},
            {},
        };
        auto stare_channels = simulate_ylj5_receiver_channels(
            stare_fields,
            ranges_m,
            laser_energy_mj,
            background_counts,
            config.simulation.stare_dwell_s,
            config.simulation,
            rng);
        stare_profile.raw_counts = std::move(stare_channels.merged_parallel_counts);
        stare_profile.overlap = std::move(stare_channels.merged_overlap);
        stare_profile.channels = std::move(stare_channels.channels);
        stare_profile.depolarization_ratio = std::move(stare_channels.depolarization_ratio);
        if (config.simulation.include_stare_profile) {
            data.profiles.push_back(std::move(stare_profile));
        }

        // -- PPI/扇区扫描：顺序遍历多个仰角 × 配置方位角，用于三维热点检测 --
        // 真实扫描雷达体积扫描(volume scan) = 每个仰角各扫一圈或一个扇区。
        // scan_id 同时编码仰角与方位角，保证全空间唯一且文件系统排序稳定。
        double hotspot_proxy = 0.0;
        const std::vector<double> azimuths = config.simulation.effective_ppi_azimuths_deg();
        for (double elevation : config.simulation.effective_ppi_elevations_deg()) {
            for (double azimuth : azimuths) {
                SimulatedFields ppi_fields = simulate_profile_fields(
                    ranges_m,
                    azimuth,
                    elevation,
                    step_index,
                    phase_steps,
                    relative_humidity,
                    config.simulation.lidar_ratio_sr,
                    wind_speed_ms,
                    wind_dir_deg,
                    config.simulation
                );
                double local_peak = 0.0;
                for (int index = 0;
                     index < std::min<int>(4, static_cast<int>(ppi_fields.true_extinction.size()));
                     ++index) {
                    double humid_factor = 1.0
                        + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;
                    local_peak = std::max(
                        local_peak,
                        ppi_fields.true_extinction[index] / humid_factor);
                }
                hotspot_proxy = std::max(hotspot_proxy, local_peak);
                // scan_id 形如 <ts>_ppi_e05_a080：前导 0 对齐，便于排序与分层解析
                auto pad3 = [](double deg) {
                    int v = static_cast<int>(std::round(deg));
                    std::string s = std::to_string(v);
                    while (s.size() < 3) s = "0" + s;
                    return s;
                };
                std::string scan_id = timestamp + "_ppi_e" + pad3(elevation) + "_a" + pad3(azimuth);
                LidarProfile profile{
                    data.site.site_id,
                    timestamp,
                    scan_id,
                    "ppi",
                    "ylj5_synthetic_ppi",
                    azimuth,
                    elevation,
                    ranges_m,
                    {},
                    laser_energy_mj,
                    background_counts,
                    {},
                    relative_humidity,
                    temperature_c,
                    wind_speed_ms,
                    wind_dir_deg,
                    ppi_fields.molecular_backscatter,
                    ppi_fields.molecular_extinction,
                    ppi_fields.true_backscatter,
                    ppi_fields.true_extinction,
                    ppi_fields.true_pm25,
                    ppi_fields.true_pm10,
                    ppi_fields.true_hotspot_mask,
                    {},
                    {},
                };
                auto channels = simulate_ylj5_receiver_channels(
                    ppi_fields,
                    ranges_m,
                    laser_energy_mj,
                    background_counts,
                    config.simulation.ppi_line_dwell_s,
                    config.simulation,
                    rng);
                profile.raw_counts = std::move(channels.merged_parallel_counts);
                profile.overlap = std::move(channels.merged_overlap);
                profile.channels = std::move(channels.channels);
                profile.depolarization_ratio = std::move(channels.depolarization_ratio);
                data.profiles.push_back(std::move(profile));
            }
        }

        // -- 由真值消光反推地面观测的 PM（用于校准）--
        // 取 stare 前 6 个 bin 的真值干消光均值作为近地面代表（除以湿度增长还原为"干"值）
        std::vector<double> near_surface_values;
        for (int index = 0; index < std::min<int>(4, static_cast<int>(stare_fields.true_extinction.size())); ++index) {
            double humid_factor = 1.0 + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;
            // 除以吸湿因子：还原为"干"消光，避免双倍计入湿度
            near_surface_values.push_back(stare_fields.true_extinction[index] / humid_factor);
        }
        double near_surface_dry = mean(near_surface_values);
        // 在所有 PPI 方位中找最近 4 bin（约 400 m 近地面层）的干消光峰值，作为该时刻的污染热点代理
        // 地面 PM 反推：经验线性模型，含近地面干消光、热点代理、湿度项，加降噪
        // 其中 0.24·RH·100 表示湿度对颗粒物计数的额外贡献（吸湿后颗粒物质量被高估）
        data.ground_measurements.push_back(GroundMeasurement{
            data.site.site_id,
            timestamp,
            std::max(24.0, 430.0 * near_surface_dry + 305.0 * hotspot_proxy + 0.24 * relative_humidity * 100.0 + sample_gaussian(rng, 0.0, 1.8)),
            std::max(34.0, 610.0 * near_surface_dry + 410.0 * hotspot_proxy + 0.31 * relative_humidity * 100.0 + sample_gaussian(rng, 0.0, 2.4)),
            relative_humidity,
            temperature_c,
            wind_speed_ms,
            wind_dir_deg,
        });
    }

    return data;
}
