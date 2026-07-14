// Internal include fragment: forward simulation physics primitives.
// ====================================================================================
// ---- 正向物理仿真：合成 LiDAR 场景 / Forward physics simulation -----------------------------------
// 本节按"大气真实状态 → 雷达观测"的正向过程合成训练数据：先给定分子/气溶胶/PM 的真值场，
// 再代入 LiDAR 方程加上 detector 噪声，得到可被反演算法处理的"伪 raw_counts"。
// ====================================================================================

/**
 * @brief 生成期望为 mean_value、标准差为 sigma 的高斯随机数。
 *
 * 用于给仿真各通道加入观测噪声（背景噪声、激光能量抖动、温湿度随机扰动等），
 * 保证训练数据真实反映仪器统计涨落。
 *
 * @param[in,out] rng 梅森旋转随机数引擎，保证可复现。
 * @param[in] mean_value 期望。
 * @param[in] sigma 标准差（>0）。
 * @return 单次高斯抽样结果。
 */
double sample_gaussian(std::mt19937& rng, double mean_value, double sigma) {
    std::normal_distribution<double> distribution(mean_value, sigma);
    return distribution(rng);
}

/**
 * @brief 近似 LiDAR 双轴重叠因子 overlap(r) 随距离变化曲线。
 *
 * 物理含义：双轴 LiDAR 发射轴与接收视轴不完全重合，近距离处部分回波落在视场外，
 * overlap 在很近距离处接近常数下限（约 0.22）并随距离按幂函数趋近于 1。
 * 这里用 (r/500)^0.82 拟合商用尺度双轴几何并夹取 [0.22, 1.0]，避免近场虚假高值。
 *
 * @param[in] ranges_m 各距离 bin 的斜距（米）。
 * @return 与 ranges_m 等长的 overlap 系数（无量纲 0~1）。
 */
std::vector<double> build_overlap(const std::vector<double>& ranges_m, double full_overlap_m, double min_overlap) {
    std::vector<double> overlap;
    overlap.reserve(ranges_m.size());
    double full_overlap = std::max(full_overlap_m, 25.0);
    double lower = clamp(min_overlap, 0.02, 0.5);
    for (double distance_m : ranges_m) {
        // 幂律拟合：r/500 的 0.82 次方，模拟商用尺度双轴系统从近距离逐步进入全视场的过程
        double ratio = std::pow(distance_m / full_overlap, 0.82);
        // 下限 0.22 防止除零导致近场发散，上限 1.0 表示远处完全进入视场
        overlap.push_back(clamp(ratio, lower, 1.0));
    }
    return overlap;
}

/**
 * @brief 在给定扫描几何与时间步下，仿真生成单条 LiDAR 廓线对应的真值场。
 *
 * 物理模型构成：
 * 1. 分子（Rayleigh）背景：消光系数 0.012·exp(-h/8000)（标准大气高度衰减，标高 8 km），
 *    后向散射取消光的 1/8；
 * 2. 边界层气溶胶：近地面增强项，标高 1200 m，含时间日变化（sin 相位，模拟昼夜 PBL 演化）；
 * 3. 高架抬升层（lofted）：高度约 1000 m 处的高斯层，常对应区域输送；
 * 4. 两个烟羽（plume）：固定水平/方位角/高度上的三维高斯团（约 850/1200 m 高，
 *    高架工业排放+区域输送），是测试热点检测的关键目标。整体几何按商用扫描雷达
 *    尺度标定（量程 6 km）。
 *
 * 湿度修正：相对湿度 > 0.55 时按比例放大消光（吸湿增长）。
 * PM 反推：由干气溶胶消光经线性经验关系换算到 PM2.5/PM10，反映典型质量消光效率。
 *
 * @param[in] ranges_m 距离 bin 序列。
 * @param[in] azimuth_deg 方位角（度），用于定位烟羽横向位置（PPI 模式）。
 * @param[in] elevation_deg 仰角（度），决定每个 bin 对应的高度与水平距离。
 * @param[in] step_index 当前时间步索引，用于日变化相位。
 * @param[in] total_steps 时间步总数，归一化相位。
 * @param[in] relative_humidity 相对湿度（0~1），用于吸湿增长修正。
 * @param[in] lidar_ratio_sr 气溶胶激光比（sr），将消光换算为后向散射（典型 40~80 sr）。
 * @return 该廓线的真值场（分子/总后向散射、总消光、PM、热点掩膜）。
 */
SimulatedFields simulate_profile_fields(
    const std::vector<double>& ranges_m,
    double azimuth_deg,
    double elevation_deg,
    int step_index,
    int total_steps,
    double relative_humidity,
    double lidar_ratio_sr,
    double wind_speed_ms,
    double wind_dir_deg,
    const SimulationConfig& simulation
) {
    SimulatedFields output;
    // 日变化相位 2π·step/total：让边界层高度与气溶胶浓度随昼夜循环变化
    double time_phase = 2.0 * std::numbers::pi * static_cast<double>(step_index) / std::max(total_steps, 1);
    // 吸湿增长因子：相对湿度越高，气溶胶散射截面越大（仅在 RH>0.55 后启动）
    double humidity_growth = 1.0 + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;

    for (double range_m : ranges_m) {
        double elevation_rad = elevation_deg * std::numbers::pi / 180.0;
        double altitude_m = range_m * std::sin(elevation_rad);
        double horizontal_m = range_m * std::cos(elevation_rad);
        // 标准 Rayleigh 分子消光：海平面 0.012，标高 8 km，随高度指数衰减
        double molecular_ext = 0.012 * std::exp(-altitude_m / 8000.0);
        // 分子后向散射约为消光的 1/8（即标准分子激光比约 8 sr）
        double molecular_beta = molecular_ext / 8.0;

        double mobile_offset_m = simulation.application_mode == "mobile_mapping"
            ? simulation.vehicle_speed_ms * 60.0 * static_cast<double>(step_index)
            : 0.0;
        double wind_rad = wind_dir_deg * std::numbers::pi / 180.0;
        double elapsed_s = static_cast<double>(step_index * std::max(simulation.minutes_per_step, 1) * 60);
        double advect_x_m = wind_speed_ms * elapsed_s * std::sin(wind_rad);
        double advect_y_m = wind_speed_ms * elapsed_s * std::cos(wind_rad);
        double ray_x_m = horizontal_m * std::sin(azimuth_deg * std::numbers::pi / 180.0) - mobile_offset_m;
        double ray_y_m = horizontal_m * std::cos(azimuth_deg * std::numbers::pi / 180.0);

        double boundary_layer = 0.0;
        double lofted_layer = 0.0;
        double plume_1 = 0.0;
        double plume_2 = 0.0;
        const double traffic_factor = simulation.application_mode == "urban_grid" ? 1.25 : 1.0;
        const double construction_factor = simulation.application_mode == "construction_site" ? 1.35 : 1.0;
        boundary_layer = traffic_factor
            * (0.014 + 0.015 * (1.0 + std::sin(time_phase)) / 2.0)
            * std::exp(-altitude_m / 1200.0);
        lofted_layer = 0.018
            * gaussian(altitude_m, 1000.0 + 200.0 * std::sin(time_phase), 280.0);
        const double source1_x = 1450.0 + 0.65 * advect_x_m;
        const double source1_y = 900.0 + 0.65 * advect_y_m;
        const double source1_range_sigma = 300.0
            + 18.0 * std::sqrt(std::max(elapsed_s / 60.0, 0.0));
        const double source1_dist = std::hypot(ray_x_m - source1_x, ray_y_m - source1_y);
        const double emission_pulse = 0.65
            + 0.55 * gaussian(std::sin(time_phase + 0.7), 0.8, 0.45);
        plume_1 = construction_factor * 0.072 * emission_pulse
            * gaussian(source1_dist, 0.0, source1_range_sigma)
            * gaussian(altitude_m, 180.0 + 80.0 * wind_speed_ms, 140.0);
        const double source2_x = -900.0 + 0.45 * advect_x_m;
        const double source2_y = 3150.0 + 0.45 * advect_y_m;
        const double source2_dist = std::hypot(ray_x_m - source2_x, ray_y_m - source2_y);
        plume_2 = 0.042 * traffic_factor
            * gaussian(source2_dist, 0.0, 760.0)
            * gaussian(altitude_m, 650.0 + 180.0 * std::sin(time_phase), 260.0);
        // 走航车辆近源扬尘，低空、近距离、随车坐标移动。
        double mobile_plume = simulation.application_mode == "mobile_mapping"
            ? 0.025 * gaussian(ray_x_m, 350.0, 280.0) * gaussian(ray_y_m, 0.0, 260.0) * gaussian(altitude_m, 70.0, 90.0)
            : 0.0;

        // 干气溶胶消光（不含吸湿），随后再叠加湿度增长
        double aerosol_dry_ext = boundary_layer + lofted_layer + plume_1 + plume_2 + mobile_plume;
        // 实际（湿）气溶胶消光：以 humidity_growth 放大
        double aerosol_ext = aerosol_dry_ext * humidity_growth;
        // 后向散射：按给定激光比由消光换算（β = σ / S）
        double aerosol_beta = aerosol_ext / lidar_ratio_sr;

        output.molecular_extinction.push_back(molecular_ext);
        output.molecular_backscatter.push_back(molecular_beta);
        output.true_backscatter.push_back(molecular_beta + aerosol_beta);
        output.true_extinction.push_back(molecular_ext + aerosol_ext);
        // 干气溶胶消光到 PM2.5 的经验线性关系：系数 640 为典型质量消光效率倒数（约 1/0.00156 m²/g）
        // plume_1 单独加权表示局部源对细颗粒的额外贡献，常数 15 为再悬浮/本底
        output.true_pm25.push_back(640.0 * aerosol_dry_ext + 210.0 * (plume_1 + mobile_plume) + 15.0);
        // PM10 略高于 PM2.5，额外含两个烟羽（粗粒贡献）和本底 22
        output.true_pm10.push_back(920.0 * aerosol_dry_ext + 260.0 * (plume_1 + plume_2 + 1.6 * mobile_plume) + 22.0);
        // 仿真真值仅用于验证，不进入默认实时协议。
        output.true_hotspot_mask.push_back(
            (plume_1 + plume_2 + mobile_plume > simulation.truth_hotspot_ext_threshold) ? 1 : 0);
    }
    return output;
}

/**
 * @brief 调用 LiDAR 方程将真值后向散射/消光转换为带噪声的 raw_counts。
 *
 * 实现的离散化 LiDAR 方程：
 * \f$ P(r) = C \cdot E \cdot O(r) \cdot \beta(r) \cdot \exp(-2\tau(0,r)) / r^2 + P_\text{bg} \f$
 * - C 为系统常数，E 激光能量，O(r) overlap，β 总后向散射；
 * - 指数项为双程大气透过率，τ 为累积光学厚度（沿路径积分的消光）；
 * - 1/r² 为光束几何发散衰减。
 * 同时按探测器噪声模型加入背景+信号相对扰动 5%~7%，并设置下限防止出现负计数。
 *
 * @param[in] true_backscatter 总后向散射廓线（分子+气溶胶）。
 * @param[in] true_extinction  总消光廓线，用于积分光学厚度。
 * @param[in] overlap          双轴重叠因子。
 * @param[in] ranges_m         距离 bin 序列（米）。
 * @param[in] laser_energy_mj  当前脉冲激光能量（mJ）。
 * @param[in] background_counts 单脉冲等效探测器背景计数。
 * @param[in] integration_time_s 该 profile 的脉冲积分时间。
 * @param[in] channel_gain 接收通道相对增益。
 * @param[in,out] rng          随机数引擎。
 * @return 含噪声的 raw_counts 序列。
 */
std::vector<double> simulate_raw_counts(
    const std::vector<double>& true_backscatter,
    const std::vector<double>& true_extinction,
    const std::vector<double>& overlap,
    const std::vector<double>& ranges_m,
    double laser_energy_mj,
    double background_counts,
    double integration_time_s,
    double channel_gain,
    const SimulationConfig& simulation,
    std::mt19937& rng
) {
    std::vector<double> raw_counts;
    raw_counts.reserve(ranges_m.size());
    double optical_depth = 0.0;
    double step_km = ranges_m.size() > 1 ? (ranges_m[1] - ranges_m[0]) / 1000.0 : 0.05;
    double integrated_pulses = std::max(
        1.0,
        std::round(std::max(simulation.pulse_repetition_hz, 0.0)
            * std::max(integration_time_s, 0.0)));
    double effective_gain = std::max(channel_gain, 1e-6);
    // afterpulsing：上一 bin 泄漏到当前 bin 的延迟伪计数（APD/SPAD 捕获载流子释放）
    double prev_true_counts = 0.0;

    for (std::size_t index = 0; index < ranges_m.size(); ++index) {
        // 累积光学厚度，对应 LiDAR 方程双程透过率 exp(-2τ)
        optical_depth += true_extinction[index] * step_km;
        double range_km = ranges_m[index] / 1000.0;
        // 离散化 LiDAR 方程：C·E·O·β·exp(-2τ)/r²
        double signal = simulation.system_constant * effective_gain * laser_energy_mj
            * overlap[index] * true_backscatter[index]
            * std::exp(-2.0 * optical_depth) / std::max(range_km * range_km, 1e-6);
        // 白天户外太阳背景会随仰角/距离弱变化，这里把它并入距离相关背景项。
        double range_background = background_counts + simulation.detector_dark_counts
            + simulation.solar_background_scale * 0.018 * background_counts * range_km;

        // afterpulsing 叠加：上一 bin 真实计数按比例泄漏到当前 bin，模拟 APD 捕获载流子延迟释放。
        // 这会产生"强信号后拖尾"，是真实 MPL 近场数据最常见的伪影。
        double afterpulse = simulation.afterpulsing_ratio * prev_true_counts;
        double expected_per_pulse = std::max(signal + range_background + afterpulse, 0.0);
        double dead_time_corrected_per_pulse = expected_per_pulse
            / (1.0 + simulation.dead_time_loss * expected_per_pulse);
        double integrated_expected = dead_time_corrected_per_pulse * integrated_pulses;

        double sampled = 0.0;
        if (integrated_expected < 900000.0) {
            std::poisson_distribution<int> poisson(integrated_expected);
            sampled = static_cast<double>(poisson(rng));
        } else {
            sampled = integrated_expected
                + sample_gaussian(rng, 0.0, std::sqrt(integrated_expected));
        }
        sampled /= integrated_pulses;
        sampled += sample_gaussian(
            rng,
            0.0,
            simulation.read_noise_counts / std::sqrt(integrated_pulses));

        // 硬饱和作为安全阀（仅极端情况触发）
        double saturated = std::min(sampled, simulation.adc_saturation_counts);
        raw_counts.push_back(std::max(saturated, range_background + 0.1));

        // 记录当前 bin 的真实计数（不含 afterpulsing），供下一 bin 泄漏使用
        prev_true_counts = std::max(signal + range_background, 0.0);
    }
    return raw_counts;
}

struct SimulatedReceiverChannels {
    std::vector<LidarChannel> channels; ///< 近/远场与平行/垂直偏振组成的四条物理通道。
    std::vector<double> merged_parallel_counts; ///< 供实时反演使用的近远场拼接平行主通道。
    std::vector<double> merged_overlap; ///< 与拼接主通道对应的等效重叠因子。
    std::vector<double> depolarization_ratio; ///< 由粒径组成和热点状态构造的体退偏比真值。
};

/**
 * @brief 按 YLJ5 双望远镜和偏振能力生成四条接收通道。
 *
 * 近远场增益、重叠曲线和拼接距离尚无实机标定证据，因此只属于正演假设；函数同时
 * 同时生成拼接平行主通道，作为客户端 Fernald 反演的输入。
 */
SimulatedReceiverChannels simulate_ylj5_receiver_channels(
    const SimulatedFields& fields,
    const std::vector<double>& ranges_m,
    double laser_energy_mj,
    double background_counts,
    double integration_time_s,
    const SimulationConfig& simulation,
    std::mt19937& rng
) {
    SimulatedReceiverChannels result;
    result.depolarization_ratio.reserve(ranges_m.size());

    std::vector<double> parallel_backscatter;
    std::vector<double> perpendicular_backscatter;
    parallel_backscatter.reserve(ranges_m.size());
    perpendicular_backscatter.reserve(ranges_m.size());

    for (std::size_t index = 0; index < ranges_m.size(); ++index) {
        double pm25 = index < fields.true_pm25.size() ? fields.true_pm25[index] : 0.0;
        double pm10 = index < fields.true_pm10.size() ? fields.true_pm10[index] : pm25;
        double coarse_fraction = clamp((pm10 - pm25) / std::max(pm10, 1.0), 0.0, 1.0);
        bool hotspot = index < fields.true_hotspot_mask.size()
            && fields.true_hotspot_mask[index] != 0;
        double depolarization = clamp(
            0.04 + 0.30 * coarse_fraction + (hotspot ? 0.08 : 0.0),
            0.03,
            0.45);
        result.depolarization_ratio.push_back(depolarization);
        double total_beta = fields.true_backscatter[index];
        double parallel = total_beta / (1.0 + depolarization);
        parallel_backscatter.push_back(parallel);
        perpendicular_backscatter.push_back(total_beta - parallel);
    }

    std::vector<double> near_overlap;
    near_overlap.reserve(ranges_m.size());
    double near_full = std::max(simulation.near_full_overlap_m, 0.1);
    for (double range_m : ranges_m) {
        near_overlap.push_back(clamp(range_m / near_full, 0.05, 1.0));
    }
    std::vector<double> far_overlap = build_overlap(
        ranges_m,
        simulation.far_full_overlap_m,
        simulation.far_min_overlap);

    auto make_channel = [&](
        const std::string& id,
        const std::string& telescope,
        const std::string& polarization,
        double aperture_mm,
        double gain,
        const std::vector<double>& backscatter,
        const std::vector<double>& channel_overlap) {
        LidarChannel channel;
        channel.channel_id = id;
        channel.telescope = telescope;
        channel.polarization = polarization;
        channel.wavelength_nm = simulation.wavelength_nm;
        channel.telescope_aperture_mm = aperture_mm;
        channel.relative_gain = gain;
        channel.background_counts = background_counts;
        channel.overlap = channel_overlap;
        channel.raw_counts = simulate_raw_counts(
            backscatter,
            fields.true_extinction,
            channel_overlap,
            ranges_m,
            laser_energy_mj,
            background_counts,
            integration_time_s,
            gain,
            simulation,
            rng);
        return channel;
    };

    double near_gain = std::max(simulation.near_channel_gain, 1e-6);
    result.channels.push_back(make_channel(
        "near_parallel_532nm", "near", "parallel",
        simulation.near_telescope_aperture_mm, near_gain,
        parallel_backscatter, near_overlap));
    result.channels.push_back(make_channel(
        "near_perpendicular_532nm", "near", "perpendicular",
        simulation.near_telescope_aperture_mm, near_gain,
        perpendicular_backscatter, near_overlap));
    result.channels.push_back(make_channel(
        "far_parallel_532nm", "far", "parallel",
        simulation.far_telescope_aperture_mm, 1.0,
        parallel_backscatter, far_overlap));
    result.channels.push_back(make_channel(
        "far_perpendicular_532nm", "far", "perpendicular",
        simulation.far_telescope_aperture_mm, 1.0,
        perpendicular_backscatter, far_overlap));

    const auto& near_parallel = result.channels[0].raw_counts;
    const auto& far_parallel = result.channels[2].raw_counts;
    result.merged_parallel_counts.reserve(ranges_m.size());
    result.merged_overlap.reserve(ranges_m.size());
    double stitch = std::max(simulation.channel_stitch_range_m, near_full);
    for (std::size_t index = 0; index < ranges_m.size(); ++index) {
        double blend = clamp((ranges_m[index] - 0.5 * stitch) / stitch, 0.0, 1.0);
        blend = blend * blend * (3.0 - 2.0 * blend);
        double near_normalized = background_counts
            + (near_parallel[index] - background_counts) / near_gain;
        result.merged_parallel_counts.push_back(
            (1.0 - blend) * near_normalized + blend * far_parallel[index]);
        result.merged_overlap.push_back(
            (1.0 - blend) * near_overlap[index] + blend * far_overlap[index]);
    }
    return result;
}

/**
 * @brief 一次完整观测战役的输入聚合体：站点、原始 LiDAR 廓线、地面观测、来源元数据。
 */
