// YLJ5 客户端实时使用的预处理、反演、湿度修正和 ENU 算法。

PreprocessResult preprocess_profile(const LidarProfile& profile) {
    PreprocessResult output;
    output.l1_signal.reserve(profile.raw_counts.size());
    output.attenuated_backscatter.reserve(profile.raw_counts.size());
    output.snr.reserve(profile.raw_counts.size());
    for (std::size_t index = 0; index < profile.raw_counts.size(); ++index) {
        const double signal = std::max(
            profile.raw_counts[index] - profile.background_counts, 0.0);
        const double energy_normalized = signal / std::max(profile.laser_energy_mj, 1e-6);
        const double overlap_corrected = energy_normalized
            / std::max(profile.overlap[index], 0.15);
        const double range_km = profile.ranges_m[index] / 1000.0;
        output.l1_signal.push_back(signal);
        output.attenuated_backscatter.push_back(
            std::max(overlap_corrected * range_km * range_km, 1e-9));
        output.snr.push_back(signal / std::max(
            std::sqrt(profile.raw_counts[index] + profile.background_counts), 1.0));
    }
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

std::pair<std::vector<double>, std::vector<double>> run_fernald_inversion(
    const LidarProfile& profile,
    const std::vector<double>& attenuated_backscatter,
    double aerosol_lidar_ratio_sr,
    double reference_aerosol_backscatter) {
    if (attenuated_backscatter.empty()
        || profile.ranges_m.size() != attenuated_backscatter.size()
        || profile.molecular_backscatter.size() != attenuated_backscatter.size()
        || profile.molecular_extinction.size() != attenuated_backscatter.size()) {
        throw std::runtime_error("Fernald input arrays are not range-aligned");
    }

    const int reference_index = std::max<int>(
        static_cast<int>(attenuated_backscatter.size()) - 5, 0);
    const std::vector<double> reference_signal(
        attenuated_backscatter.begin() + reference_index,
        attenuated_backscatter.end());
    const std::vector<double> reference_molecular(
        profile.molecular_backscatter.begin() + reference_index,
        profile.molecular_backscatter.end());
    const double scale = (
        mean(reference_molecular) + reference_aerosol_backscatter)
        / std::max(mean(reference_signal), 1e-9);

    std::vector<double> extinction(attenuated_backscatter.size(), 0.0);
    std::vector<double> aerosol_backscatter(attenuated_backscatter.size(), 0.0);
    const double step_km = profile.ranges_m.size() > 1
        ? (profile.ranges_m[1] - profile.ranges_m[0]) / 1000.0
        : 0.05;
    double optical_depth = 0.0;
    for (int index = static_cast<int>(attenuated_backscatter.size()) - 1;
         index >= 0; --index) {
        const double scaled_signal = std::max(
            attenuated_backscatter[static_cast<std::size_t>(index)] * scale, 1e-9);
        const double total_backscatter = std::max(
            scaled_signal * std::exp(2.0 * optical_depth),
            profile.molecular_backscatter[static_cast<std::size_t>(index)]);
        const double aerosol = std::max(
            total_backscatter
                - profile.molecular_backscatter[static_cast<std::size_t>(index)],
            0.0);
        const double molecular_extinction =
            profile.molecular_extinction[static_cast<std::size_t>(index)];
        const double total_extinction = std::clamp(
            molecular_extinction + aerosol_lidar_ratio_sr * aerosol,
            molecular_extinction,
            0.45);
        aerosol_backscatter[static_cast<std::size_t>(index)] = aerosol;
        extinction[static_cast<std::size_t>(index)] = total_extinction;
        optical_depth += total_extinction * step_km;
    }
    return {std::move(extinction), std::move(aerosol_backscatter)};
}

double humidity_growth_factor(
    double relative_humidity,
    double dry_reference_rh,
    double hygroscopicity) {
    const double humidity = std::clamp(relative_humidity, 0.05, 0.98);
    const double dry_ratio = dry_reference_rh / std::max(1.0 - dry_reference_rh, 0.02);
    const double humid_ratio = humidity / std::max(1.0 - humidity, 0.02);
    return std::max(
        1.0,
        1.0 + hygroscopicity * std::max(humid_ratio - dry_ratio, 0.0) * 0.18);
}

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

std::vector<std::vector<double>> profile_bins_to_enu(const LidarProfile& profile) {
    std::vector<std::vector<double>> output;
    output.reserve(profile.ranges_m.size());
    const double azimuth = profile.azimuth_deg * std::numbers::pi / 180.0;
    const double elevation = profile.elevation_deg * std::numbers::pi / 180.0;
    for (double range_m : profile.ranges_m) {
        output.push_back({
            range_m * std::cos(elevation) * std::sin(azimuth),
            range_m * std::cos(elevation) * std::cos(azimuth),
            range_m * std::sin(elevation),
        });
    }
    return output;
}
