/**
 * @file TestClientFrameProcessor.cpp
 * @brief 实时客户端四通道处理链与 PM 标定边界回归测试。
 */
#include "lidar_client/FrameProcessor.hpp"
#include "lidar_client/DisplaySnapshot.hpp"
#include "lidar_protocol/Frame.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/** @brief 断言失败时抛出可由 CTest 展示的异常。 */
void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

/** @brief 判断周期或廓线 QC 是否包含指定标志。 */
bool contains_flag(const std::vector<std::string>& flags, const std::string& expected) {
    return std::find(flags.begin(), flags.end(), expected) != flags.end();
}

/** @brief 用相对+绝对容差比较反演浮点量。 */
bool nearly_equal(double left, double right) {
    const double scale = std::max({1.0, std::abs(left), std::abs(right)});
    return std::abs(left - right) <= 1e-10 * scale;
}

/** @brief 构造近/远场、平行/垂直偏振四条物理接收路径。 */
lidar_core::LidarChannel make_channel(
    const std::string& id,
    const std::string& telescope,
    const std::string& polarization,
    const std::vector<double>& ranges,
    double signal_scale,
    double relative_gain) {
    lidar_core::LidarChannel channel;
    channel.channel_id = id;
    channel.telescope = telescope;
    channel.polarization = polarization;
    channel.wavelength_nm = 532.0;
    channel.telescope_aperture_mm = telescope == "near" ? 80.0 : 200.0;
    channel.relative_gain = relative_gain;
    channel.background_counts = 8.0;
    channel.raw_counts.reserve(ranges.size());
    channel.overlap.reserve(ranges.size());
    for (std::size_t index = 0; index < ranges.size(); ++index) {
        const double decay = std::exp(-static_cast<double>(index) / 22.0);
        channel.raw_counts.push_back(8.0 + signal_scale * decay);
        channel.overlap.push_back(telescope == "near"
            ? std::min(1.0, 0.35 + static_cast<double>(index) * 0.025)
            : std::min(1.0, 0.08 + static_cast<double>(index) * 0.018));
    }
    return channel;
}

/** @brief 构造不携带分子场的设备 L0 廓线，模拟实时链路默认载荷。 */
lidar_core::LidarProfile make_device_profile(const std::string& scan_id) {
    constexpr std::size_t BinCount = 64;
    lidar_core::LidarProfile profile;
    profile.site_id = "test-site";
    profile.scan_id = scan_id;
    profile.scan_mode = "ppi";
    profile.source_kind = "device_l0";
    profile.azimuth_deg = 30.0;
    profile.elevation_deg = 5.0;
    profile.laser_energy_mj = 1.1;
    profile.background_counts = 8.0;
    profile.relative_humidity = 0.58;
    profile.ranges_m.reserve(BinCount);
    profile.overlap.reserve(BinCount);
    profile.raw_counts.reserve(BinCount);
    profile.depolarization_ratio.reserve(BinCount);
    for (std::size_t index = 0; index < BinCount; ++index) {
        profile.ranges_m.push_back(3.75 * static_cast<double>(index + 1));
        profile.overlap.push_back(std::min(1.0, 0.2 + static_cast<double>(index) * 0.03));
        profile.raw_counts.push_back(8.0 + 1800.0 * std::exp(-static_cast<double>(index) / 22.0));
        profile.depolarization_ratio.push_back(0.08 + 0.001 * static_cast<double>(index));
    }
    profile.channels = {
        make_channel("near_parallel_532nm", "near", "parallel", profile.ranges_m, 1800.0, 1.8),
        make_channel("near_perpendicular_532nm", "near", "perpendicular", profile.ranges_m, 180.0, 0.18),
        make_channel("far_parallel_532nm", "far", "parallel", profile.ranges_m, 900.0, 1.0),
        make_channel("far_perpendicular_532nm", "far", "perpendicular", profile.ranges_m, 90.0, 0.1),
    };
    return profile;
}

/** @brief 把 L0 廓线封装为客户端实际接收的 JSONL 协议帧。 */
lidar_protocol::Frame make_raw_frame(const std::string& timestamp, const std::string& scan_id) {
    auto profile = make_device_profile(scan_id);
    auto payload = lidar_protocol::profile_to_json(profile, false);
    payload["integrated_pulses"] = 10000;
    payload["wavelength_nm"] = 532.0;
    payload["signal_unit"] = "mean_counts_per_pulse";
    payload["azimuth_scan_pattern"] = "conical";
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::lidar_raw,
        timestamp,
        std::move(payload));
}

lidar_protocol::Frame make_raw_frame(
    const std::string& timestamp,
    lidar_core::LidarProfile profile) {
    auto payload = lidar_protocol::profile_to_json(profile, false);
    payload["integrated_pulses"] = 10000;
    payload["wavelength_nm"] = 532.0;
    payload["signal_unit"] = "mean_counts_per_pulse";
    payload["azimuth_scan_pattern"] = "conical";
    return lidar_protocol::make_frame(
        lidar_protocol::FrameType::lidar_raw,
        timestamp,
        std::move(payload));
}

/** @brief 送入一条射线和周期心跳，返回处理器封口后的周期结果。 */
lidar_client::StepResult run_one_cycle(
    lidar_client::FrameProcessor& processor,
    const std::string& timestamp,
    const std::string& scan_id,
    std::optional<lidar_core::LidarProfile> profile = std::nullopt) {
    std::optional<lidar_client::StepResult> completed;
    processor.set_step_complete_callback([&completed](lidar_client::StepResult result) {
        completed = std::move(result);
    });
    processor.handle_frame(profile.has_value()
        ? make_raw_frame(timestamp, std::move(*profile))
        : make_raw_frame(timestamp, scan_id));
    processor.handle_frame(lidar_protocol::make_frame(
        lidar_protocol::FrameType::heartbeat,
        timestamp,
        lidar_core::Json::Object{}));
    require(completed.has_value(), "Heartbeat should finalize a client processing cycle");
    return std::move(*completed);
}

} // namespace

int main() {
    try {
        lidar_client::FrameProcessor uncalibrated;
        const auto optical_only = run_one_cycle(
            uncalibrated, "2026-07-14T10:00:00Z", "scan-optical");
        require(optical_only.processed_profiles.size() == 1,
                "Four-channel L0 frame should produce one optical profile");
        require(contains_flag(optical_only.qc_flags, "molecular-reference-standard-atmosphere"),
                "Missing molecular fields should use the declared standard atmosphere fallback");
        require(contains_flag(optical_only.qc_flags, "near-far-channels-glued"),
                "Near and far parallel receiver channels should be glued");
        require(contains_flag(optical_only.qc_flags, "pm-calibration-missing"),
                "Uncalibrated cycle should expose the PM calibration QC boundary");
        require(contains_flag(
                    optical_only.qc_flags,
                    "receiver-calibration-assumed-not-device-verified"),
                "Default receiver assumptions must not be presented as device calibration");
        require(!optical_only.pm_calibrated,
                "Uncalibrated cycle must not claim quantitative PM");
        require(optical_only.processed_profiles.front().pm25.empty()
                    && optical_only.processed_profiles.front().pm10.empty(),
                "PM arrays must remain empty before a site calibration is loaded");
        require(optical_only.processed_profiles.front().bin_quality.size() == 64,
                "Every processed profile should expose range-aligned bin quality");
        const auto& optical_profile = optical_only.processed_profiles.front();
        require(optical_profile.extinction.size() == 64
                    && optical_profile.aerosol_backscatter.size() == 64
                    && optical_profile.aerosol_extinction.size() == 64
                    && optical_profile.dry_extinction.size() == 64,
                "Total, aerosol and dry optical products must remain range-aligned");
        const auto expected_dry_extinction = lidar_core::HumidityCorrectionStep(
            lidar_core::HumidityConfig{}).process(
                optical_profile.aerosol_extinction,
                optical_profile.profile.relative_humidity);
        bool checked_finite_optical_bin = false;
        for (std::size_t index = 0; index < optical_profile.extinction.size(); ++index) {
            if (!std::isfinite(optical_profile.extinction[index])) continue;
            require(std::isfinite(optical_profile.profile.molecular_extinction[index])
                        && std::isfinite(optical_profile.aerosol_backscatter[index])
                        && std::isfinite(optical_profile.aerosol_extinction[index])
                        && std::isfinite(optical_profile.dry_extinction[index]),
                    "Every retrieved optical component must be finite in a valid bin");
            require(nearly_equal(
                        optical_profile.extinction[index],
                        optical_profile.profile.molecular_extinction[index]
                            + optical_profile.aerosol_extinction[index]),
                    "Total extinction must equal molecular plus aerosol extinction");
            require(nearly_equal(
                        optical_profile.dry_extinction[index],
                        expected_dry_extinction[index]),
                    "Humidity correction must use aerosol extinction only");
            checked_finite_optical_bin = true;
        }
        require(checked_finite_optical_bin,
                "Optical component assertions require at least one valid retrieval bin");
        require(lidar_core::has_quality(
                    optical_only.processed_profiles.front().bin_quality.front(),
                    lidar_core::BinQualityFlag::partial_overlap),
                "Partially overlapped near-range bins should remain explicitly marked");
        const auto optical_display = lidar_client::build_display_snapshot(optical_only);
        require(!optical_display.ppi_image.isNull()
                    && optical_display.ppi_ray_count == 1
                    && optical_display.ppi_title.contains(QStringLiteral("PM 未标定")),
                "Worker display builder should rasterize the optical-only PPI product");

        lidar_client::ProcessorConfig calibrated_config;
        calibrated_config.pm_calibration.valid = true;
        calibrated_config.pm_calibration.calibration_id = "test-site-calibration";
        calibrated_config.pm_calibration.valid_from = "2026-07-01";
        calibrated_config.pm_calibration.pm25_intercept_ugm3 = 2.0;
        calibrated_config.pm_calibration.pm25_slope_ugm3_per_km = 120.0;
        calibrated_config.pm_calibration.pm10_intercept_ugm3 = 3.0;
        calibrated_config.pm_calibration.pm10_slope_ugm3_per_km = 180.0;
        calibrated_config.receiver_calibration.valid = true;
        calibrated_config.receiver_calibration.calibration_id = "test-receiver-calibration";
        calibrated_config.receiver_calibration.minimum_valid_range_m = 15.0;
        calibrated_config.receiver_calibration.dead_time_loss_per_count = 0.0;
        calibrated_config.receiver_calibration.afterpulse_kernel.clear();
        lidar_client::FrameProcessor calibrated(calibrated_config);
        const auto quantitative = run_one_cycle(
            calibrated, "2026-07-14T10:10:00Z", "scan-calibrated");
        require(quantitative.pm_calibrated
                    && quantitative.calibration_id == "test-site-calibration",
                "Loaded calibration identity should propagate into the cycle product");
        require(quantitative.processed_profiles.front().pm25.size() == 64
                    && quantitative.processed_profiles.front().pm10.size() == 64,
                "Valid site calibration should produce range-aligned PM products");
        require(quantitative.receiver_calibrated
                    && quantitative.receiver_calibration_id == "test-receiver-calibration",
                "Receiver calibration identity should propagate into the cycle product");
        require(std::isnan(quantitative.processed_profiles.front().pm25.front()),
                "Bins below minimum range or quantitative overlap must remain masked in PM");
        require(std::any_of(
                    quantitative.processed_profiles.front().pm25.begin(),
                    quantitative.processed_profiles.front().pm25.end(),
                    [](double value) { return std::isfinite(value); }),
                "A calibrated profile should retain finite PM outside the masked near range");
        require(!contains_flag(quantitative.qc_flags, "pm-calibration-missing"),
                "Calibrated cycle should not retain the missing calibration flag");
        const auto pm_display = lidar_client::build_display_snapshot(quantitative);
        require(pm_display.ppi_field_label.contains(QStringLiteral("PM2.5")),
                "Calibrated display snapshot should select the PM2.5 product");
        require(pm_display.ppi_masked_bin_count > 0
                    && pm_display.ppi_valid_bin_count > 0,
                "Display snapshot should report both masked and valid PM bins");

        lidar_client::ProcessorConfig saturation_config = calibrated_config;
        saturation_config.receiver_calibration.saturation_counts = 1500.0;
        saturation_config.receiver_calibration.saturation_guard_fraction = 1.0;
        lidar_client::FrameProcessor saturation_processor(saturation_config);
        const auto saturated = run_one_cycle(
            saturation_processor, "2026-07-14T10:20:00Z", "scan-saturated");
        require(contains_flag(saturated.qc_flags, "saturated-bins-masked"),
                "Strong near-field returns should expose saturation QC");
        require(lidar_core::has_quality(
                    saturated.processed_profiles.front().bin_quality.front(),
                    lidar_core::BinQualityFlag::saturated),
                "Saturated bins should carry a machine-readable quality bit");
        require(std::isnan(saturated.processed_profiles.front().pm25.front()),
                "Saturated bins must not enter quantitative PM products");

        auto missing_energy_profile = make_device_profile("scan-missing-energy");
        missing_energy_profile.laser_energy_mj = 0.0;
        lidar_client::FrameProcessor missing_energy_processor(calibrated_config);
        const auto missing_energy = run_one_cycle(
            missing_energy_processor,
            "2026-07-14T10:30:00Z",
            "scan-missing-energy",
            std::move(missing_energy_profile));
        require(lidar_core::has_quality(
                    missing_energy.processed_profiles.front().bin_quality.back(),
                    lidar_core::BinQualityFlag::invalid_laser_energy),
                "Missing pulse energy should invalidate every range-corrected product bin");
        require(std::all_of(
                    missing_energy.processed_profiles.front().pm25.begin(),
                    missing_energy.processed_profiles.front().pm25.end(),
                    [](double value) { return std::isnan(value); }),
                "Missing pulse energy must never create numerically amplified PM values");

        std::cout << "Client frame processor assertions passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }
}
