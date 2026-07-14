// YLJ5 客户端实时使用的预处理、反演、湿度修正和 ENU 算法。
// 本文件被包含到 LidarDemo.cpp 的匿名命名空间中，函数只作为公共 Step API 的内部实现。

/**
 * @brief 单主通道的兼容预处理实现。
 *
 * 真实 YLJ5 四通道数据主要走 FrameProcessor.cpp 中更完整的接收机校正与 gluing。本函数
 * 服务于缺少物理通道的兼容调用：执行固定背景扣除、能量归一、overlap 和 r^2 校正，
 * 并使用同一套逐 bin 质量位语义。它没有死时间和 afterpulse 标定，因此不能替代四通道链。
 */
PreprocessResult preprocess_profile(const LidarProfile& profile) {
    PreprocessResult output;
    output.l1_signal.reserve(profile.raw_counts.size());
    output.attenuated_backscatter.reserve(profile.raw_counts.size());
    output.snr.reserve(profile.raw_counts.size());
    output.bin_quality.resize(profile.raw_counts.size(), 0);
    // 不可用 bin 写 NaN 而不是 0：0 代表可观测但无气溶胶，NaN 才代表没有可信测量。
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    const bool energy_valid = std::isfinite(profile.laser_energy_mj)
        && profile.laser_energy_mj > 1e-6;
    for (std::size_t index = 0; index < profile.raw_counts.size(); ++index) {
        auto& quality = output.bin_quality[index];
        // 固件/适配器质量位与客户端派生质量位做按位并集，任何上游原因都不会丢失。
        if (profile.device_bin_quality.size() == profile.raw_counts.size()) {
            quality |= profile.device_bin_quality[index];
        }
        if (!std::isfinite(profile.raw_counts[index])
            || index >= profile.overlap.size()
            || !std::isfinite(profile.overlap[index])) {
            quality |= quality_mask(BinQualityFlag::non_finite_input);
        }
        if (!energy_valid) {
            quality |= quality_mask(BinQualityFlag::invalid_laser_energy);
        }
        if (index < profile.overlap.size() && profile.overlap[index] < 0.15) {
            // 兼容路径使用固定 0.15 反演下限；实机应由 ReceiverCalibrationModel 提供。
            quality |= quality_mask(BinQualityFlag::overlap_unusable);
        } else if (index < profile.overlap.size() && profile.overlap[index] < 0.90) {
            quality |= quality_mask(BinQualityFlag::partial_overlap);
        }
        // 净信号 P1(r)=max(Praw(r)-B,0)，负统计涨落按零净信号处理。
        const double signal = std::max(
            profile.raw_counts[index] - profile.background_counts, 0.0);
        output.l1_signal.push_back(signal);
        // 单次/已归一计数的 Poisson 近似 SNR；四通道链还会乘 sqrt(integrated_pulses)。
        const double snr = signal / std::max(
            std::sqrt(profile.raw_counts[index] + profile.background_counts), 1.0);
        output.snr.push_back(snr);
        if (snr < 3.0) {
            quality |= quality_mask(BinQualityFlag::low_snr);
        }
        if (!bin_is_usable_for_retrieval(quality)) {
            output.attenuated_backscatter.push_back(invalid);
            continue;
        }
        // P2=P1/E，Pcorr=P2/O，RCS=Pcorr*r^2；r 用 km 以匹配后续 km^-1 单位链。
        const double energy_normalized = signal / profile.laser_energy_mj;
        const double overlap_corrected = energy_normalized / profile.overlap[index];
        const double range_km = profile.ranges_m[index] / 1000.0;
        output.attenuated_backscatter.push_back(
            std::max(overlap_corrected * range_km * range_km, 1e-9));
    }
    // 廓线级 QC 是给操作员的摘要；逐 bin 位置仍以 bin_quality 为准。
    if (profile.overlap.size() >= 3
        && *std::min_element(profile.overlap.begin(), profile.overlap.begin() + 3) < 0.4) {
        output.qc_flags.emplace_back("near-range-partial-overlap");
    }
    if (profile.laser_energy_mj < 0.01) {
        output.qc_flags.emplace_back("low-laser-energy");
    }
    if (output.snr.size() >= 4
        && mean(std::vector<double>(output.snr.begin(), output.snr.begin() + 4)) < 3.0) {
        output.qc_flags.emplace_back("weak-near-range-snr");
    }
    return output;
}

/**
 * @brief 在最长连续有效区间上执行简化的 Fernald/Klett 弹性反演。
 *
 * @return pair.first 为总消光 alpha_total（km^-1），pair.second 为气溶胶后向散射
 * beta_aerosol（km^-1 sr^-1）。无效或未参与反演的位置为 NaN。
 *
 * 单波长弹性雷达只有一个观测方程，无法同时独立求解消光和后向散射，因此必须假设
 * 气溶胶激光雷达比 S_a=alpha_a/beta_a，并在远端参考区给定气溶胶后向散射边界。
 * 本实现采用由远向近的稳定积分方向；它是工程近似，不等于厂商私有反演算法。
 */
std::pair<std::vector<double>, std::vector<double>> run_fernald_inversion(
    const LidarProfile& profile,
    const std::vector<double>& attenuated_backscatter,
    const std::vector<BinQualityMask>& bin_quality,
    const RetrievalConfig& config) {
    // 所有物理量必须共享同一距离轴；错位时不能通过截短数组“凑合”计算。
    if (attenuated_backscatter.empty()
        || profile.ranges_m.size() != attenuated_backscatter.size()
        || profile.molecular_backscatter.size() != attenuated_backscatter.size()
        || profile.molecular_extinction.size() != attenuated_backscatter.size()) {
        throw std::runtime_error("Fernald input arrays are not range-aligned");
    }
    if (!bin_quality.empty() && bin_quality.size() != attenuated_backscatter.size()) {
        throw std::runtime_error("Fernald quality mask is not range-aligned");
    }

    // 一个反演 bin 必须同时具备有限信号、有限分子场和通过反演级 QC。
    const auto usable = [&](std::size_t index) {
        return std::isfinite(attenuated_backscatter[index])
            && std::isfinite(profile.molecular_backscatter[index])
            && std::isfinite(profile.molecular_extinction[index])
            && (bin_quality.empty() || bin_is_usable_for_retrieval(bin_quality[index]));
    };
    /*
     * Fernald 是沿路径积分，不能跨越饱和、盲区或缺测段直接连接两边。这里扫描所有连续
     * 有效段，只保留最长一段 [best_begin,best_end)，避免短暂噪声岛被误当成完整廓线。
     * 循环走到 size() 这个哨兵位置，确保末尾有效段也会被结算。
     */
    std::size_t best_begin = 0;
    std::size_t best_end = 0;
    std::size_t current_begin = 0;
    bool in_segment = false;
    for (std::size_t index = 0; index <= attenuated_backscatter.size(); ++index) {
        if (index < attenuated_backscatter.size() && usable(index)) {
            if (!in_segment) {
                current_begin = index;
                in_segment = true;
            }
            continue;
        }
        if (in_segment && index - current_begin > best_end - best_begin) {
            best_begin = current_begin;
            best_end = index;
        }
        in_segment = false;
    }

    // 先把整条输出初始化为 NaN，只覆盖真正参与反演的连续区间。
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> extinction(attenuated_backscatter.size(), invalid);
    std::vector<double> aerosol_backscatter(attenuated_backscatter.size(), invalid);
    const std::size_t minimum_valid_bins = static_cast<std::size_t>(
        std::max(config.minimum_valid_bins, 5));
    if (best_end - best_begin < minimum_valid_bins) {
        // 连续有效段太短时没有稳定参考窗，返回全 NaN，由调用方追加 retrieval QC。
        return {std::move(extinction), std::move(aerosol_backscatter)};
    }

    // 参考窗取有效段最远端的若干 bin，至少 5 个，以平均随机噪声和局部细结构。
    const std::size_t reference_count = std::min(
        best_end - best_begin,
        static_cast<std::size_t>(std::max(config.reference_window_bins, 5)));
    const std::size_t reference_index = best_end - reference_count;
    const std::vector<double> reference_signal(
        attenuated_backscatter.begin() + reference_index,
        attenuated_backscatter.begin() + best_end);
    const std::vector<double> reference_molecular(
        profile.molecular_backscatter.begin() + reference_index,
        profile.molecular_backscatter.begin() + best_end);
    /*
     * 将相对衰减后向散射缩放到参考边界：
     * scale = (mean(beta_m_ref)+beta_a_ref) / mean(signal_ref)。
     * reference_aerosol_backscatter 是反演必要假设，其敏感性应在实机验收中单独评估。
     */
    const double scale = (
        mean(reference_molecular) + config.reference_aerosol_backscatter)
        / std::max(mean(reference_signal), 1e-9);

    // optical_depth 保存从当前 bin 外侧到参考端累计的单程光学厚度 tau。
    double optical_depth = 0.0;
    for (int index = static_cast<int>(best_end) - 1;
         index >= static_cast<int>(best_begin); --index) {
        // 把相对观测量归一到参考后向散射尺度，并保留正下限防止数值下溢。
        const double scaled_signal = std::max(
            attenuated_backscatter[static_cast<std::size_t>(index)] * scale, 1e-9);
        // 观测包含双程透过率 exp(-2*tau)，由远向近时乘 exp(+2*tau)补回外侧衰减。
        const double total_backscatter = std::max(
            scaled_signal * std::exp(2.0 * optical_depth),
            profile.molecular_backscatter[static_cast<std::size_t>(index)]);
        // 总后向散射减去分子项得到气溶胶项；负值通常来自噪声，按 0 气溶胶约束。
        const double aerosol = std::max(
            total_backscatter
                - profile.molecular_backscatter[static_cast<std::size_t>(index)],
            0.0);
        const double molecular_extinction =
            profile.molecular_extinction[static_cast<std::size_t>(index)];
        // alpha_total = alpha_m + S_a * beta_a，并用工程上限抑制反向积分爆炸。
        const double total_extinction = std::clamp(
            molecular_extinction + config.aerosol_lidar_ratio_sr * aerosol,
            molecular_extinction,
            config.maximum_extinction_per_km);
        aerosol_backscatter[static_cast<std::size_t>(index)] = aerosol;
        extinction[static_cast<std::size_t>(index)] = total_extinction;
        if (index > static_cast<int>(best_begin)) {
            // 使用实际相邻距离差而非固定 bin 宽，兼容重采样或非均匀距离轴。
            const double step_km = (
                profile.ranges_m[static_cast<std::size_t>(index)]
                - profile.ranges_m[static_cast<std::size_t>(index - 1)]) / 1000.0;
            // 无量纲光学厚度增量 d_tau = alpha(km^-1) * dr(km)。
            optical_depth += total_extinction * step_km;
        }
    }
    return {std::move(extinction), std::move(aerosol_backscatter)};
}

/**
 * @brief 根据环境相对湿度计算简化的气溶胶吸湿增长因子 g(RH)。
 *
 * 该函数借鉴 kappa-Kohler 形式，用 RH/(1-RH) 描述接近饱和时的非线性增长。它不是
 * 严格粒径谱微物理模型；hygroscopicity 和 0.18 系数必须通过站点实验验证。
 */
double humidity_growth_factor(
    double relative_humidity,
    double dry_reference_rh,
    double hygroscopicity) {
    // 限制到 [5%,98%]，避免 0 或 100% 附近分母为零及不受控发散。
    const double humidity = std::clamp(relative_humidity, 0.05, 0.98);
    // 将环境 RH 与干参考 RH 都变换到 RH/(1-RH) 的增长坐标。
    const double dry_ratio = dry_reference_rh / std::max(1.0 - dry_reference_rh, 0.02);
    const double humid_ratio = humidity / std::max(1.0 - humidity, 0.02);
    // RH 低于干参考时不反向“增湿”，增长因子最小保持 1。
    return std::max(
        1.0,
        1.0 + hygroscopicity * std::max(humid_ratio - dry_ratio, 0.0) * 0.18);
}

/**
 * @brief 把环境湿态消光换算到 PM 标定使用的干参考状态。
 *
 * dry_extinction = ambient_extinction / g(RH)。NaN 输入通过 IEEE 浮点除法自然保持 NaN，
 * 因而逐 bin 无效区不会在湿度校正阶段重新变成有限数值。
 */
std::vector<double> apply_humidity_correction(
    const std::vector<double>& extinction,
    double relative_humidity,
    double dry_reference_rh,
    double hygroscopicity) {
    const double factor = humidity_growth_factor(
        relative_humidity, dry_reference_rh, hygroscopicity);
    std::vector<double> output;
    output.reserve(extinction.size());
    for (double value : extinction) output.push_back(value / factor);
    return output;
}

/**
 * @brief 把一条极坐标射线的所有距离门投影到本地 ENU 直角坐标。
 *
 * 方位角定义为正北 0 度、顺时针增加；仰角 0 度为水平、90 度为天顶。因此：
 * East=r*cos(el)*sin(az)，North=r*cos(el)*cos(az)，Up=r*sin(el)。站点经纬度平移和
 * 车载姿态补偿不在本函数内，未来应在 VendorProtocolAdapter/平台位姿层先完成。
 */
std::vector<std::vector<double>> profile_bins_to_enu(const LidarProfile& profile) {
    std::vector<std::vector<double>> output;
    output.reserve(profile.ranges_m.size());
    // 三角函数使用弧度，输入协议角度统一为度。
    const double azimuth = profile.azimuth_deg * std::numbers::pi / 180.0;
    const double elevation = profile.elevation_deg * std::numbers::pi / 180.0;
    for (double range_m : profile.ranges_m) {
        // 输出顺序固定为 [East, North, Up]，与热点质心和 GUI 坐标约定一致。
        output.push_back({
            range_m * std::cos(elevation) * std::sin(azimuth),
            range_m * std::cos(elevation) * std::cos(azimuth),
            range_m * std::sin(elevation),
        });
    }
    return output;
}
