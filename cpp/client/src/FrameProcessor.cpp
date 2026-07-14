/**
 * @file FrameProcessor.cpp
 * @brief YLJ5 实时设备处理链实现。
 */
#include "lidar_client/FrameProcessor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>

namespace lidar_client {

namespace {

double json_number(const lidar_core::Json& json, const char* key, double fallback) {
    return json.contains(key) && json.at(key).is_number()
        ? json.at(key).number_value()
        : fallback;
}

std::string json_string(const lidar_core::Json& json, const char* key) {
    return json.contains(key) && json.at(key).is_string()
        ? json.at(key).string_value()
        : "";
}

double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
    double result = values[middle];
    if (values.size() % 2 == 0) {
        auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
        result = (*lower + result) * 0.5;
    }
    return result;
}

double tail_background(const lidar_core::LidarChannel& channel) {
    if (channel.raw_counts.empty()) {
        return std::max(channel.background_counts, 0.0);
    }
    const std::size_t tail_count = std::clamp<std::size_t>(channel.raw_counts.size() / 20, 16, 256);
    const std::size_t begin = channel.raw_counts.size() > tail_count
        ? channel.raw_counts.size() - tail_count
        : 0;
    std::vector<double> tail;
    tail.reserve(channel.raw_counts.size() - begin);
    for (std::size_t index = begin; index < channel.raw_counts.size(); ++index) {
        if (std::isfinite(channel.raw_counts[index])) {
            tail.push_back(channel.raw_counts[index]);
        }
    }
    const double measured = median(std::move(tail));
    return measured > 0.0 ? measured : std::max(channel.background_counts, 0.0);
}

const lidar_core::LidarChannel* find_channel(
    const lidar_core::LidarProfile& profile,
    const std::string& telescope,
    const std::string& polarization) {
    for (const auto& channel : profile.channels) {
        if (channel.telescope == telescope && channel.polarization == polarization) {
            return &channel;
        }
    }
    return nullptr;
}

void validate_profile(const lidar_core::LidarProfile& profile) {
    const std::size_t count = profile.ranges_m.size();
    if (count < 8 || profile.raw_counts.size() != count || profile.overlap.size() != count) {
        throw std::runtime_error("invalid lidar profile shape");
    }
    double previous = -std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < count; ++index) {
        if (!std::isfinite(profile.ranges_m[index])
            || !std::isfinite(profile.raw_counts[index])
            || profile.ranges_m[index] <= previous) {
            throw std::runtime_error("invalid or non-monotonic lidar samples");
        }
        previous = profile.ranges_m[index];
    }
    if (!std::isfinite(profile.azimuth_deg) || !std::isfinite(profile.elevation_deg)
        || profile.azimuth_deg < 0.0 || profile.azimuth_deg >= 360.0
        || profile.elevation_deg < 0.0 || profile.elevation_deg > 180.0) {
        throw std::runtime_error("lidar pointing is outside public gimbal limits");
    }
    for (const auto& channel : profile.channels) {
        if (channel.raw_counts.size() != count || channel.overlap.size() != count) {
            throw std::runtime_error("receiver channel shape does not match range axis");
        }
    }
}

void fill_standard_molecular_reference(
    lidar_core::LidarProfile& profile,
    double site_altitude_m,
    double wavelength_nm,
    std::vector<std::string>& qc_flags) {
    const std::size_t count = profile.ranges_m.size();
    if (profile.molecular_backscatter.size() == count
        && profile.molecular_extinction.size() == count) {
        return;
    }

    profile.molecular_backscatter.assign(count, 0.0);
    profile.molecular_extinction.assign(count, 0.0);
    const double elevation_rad = profile.elevation_deg * std::acos(-1.0) / 180.0;
    const double rayleigh_scale = std::pow(532.0 / std::max(wavelength_nm, 1.0), 4.0);
    for (std::size_t index = 0; index < count; ++index) {
        const double height_m = std::max(
            0.0,
            site_altitude_m + profile.ranges_m[index] * std::sin(elevation_rad));
        const double molecular_extinction_km = 0.012 * rayleigh_scale * std::exp(-height_m / 8000.0);
        profile.molecular_extinction[index] = molecular_extinction_km;
        profile.molecular_backscatter[index] = molecular_extinction_km / 8.0;
    }
    qc_flags.emplace_back("molecular-reference-standard-atmosphere");
}

struct ChannelSignal {
    std::vector<double> corrected; ///< 背景、增益和 overlap 修正后的信号。
    std::vector<double> snr;       ///< 按积分脉冲数估计的信噪比。
};

ChannelSignal correct_channel(
    const lidar_core::LidarChannel& channel,
    int integrated_pulses,
    double minimum_overlap) {
    ChannelSignal output;
    output.corrected.resize(channel.raw_counts.size(), 0.0);
    output.snr.resize(channel.raw_counts.size(), 0.0);
    const double background = tail_background(channel);
    const double pulse_gain = std::sqrt(static_cast<double>(std::max(integrated_pulses, 1)));
    const double receiver_gain = std::max(channel.relative_gain, 1e-9);
    for (std::size_t index = 0; index < channel.raw_counts.size(); ++index) {
        const double raw = std::max(channel.raw_counts[index], 0.0);
        const double signal = std::max(raw - background, 0.0);
        const double overlap = std::max(channel.overlap[index], minimum_overlap);
        output.corrected[index] = signal / receiver_gain / overlap;
        output.snr[index] = signal * pulse_gain / std::max(std::sqrt(raw + background), 1e-9);
    }
    return output;
}

lidar_core::PreprocessResult preprocess_receiver_channels(
    const lidar_core::LidarProfile& profile,
    int integrated_pulses,
    const ProcessorConfig& config) {
    const auto* near_parallel = find_channel(profile, "near", "parallel");
    const auto* far_parallel = find_channel(profile, "far", "parallel");
    if (near_parallel == nullptr || far_parallel == nullptr) {
        lidar_core::BackgroundPreprocessStep fallback;
        auto output = fallback.process(profile);
        const double pulse_gain = std::sqrt(static_cast<double>(std::max(integrated_pulses, 1)));
        for (double& value : output.snr) {
            value *= pulse_gain;
        }
        output.qc_flags.emplace_back("receiver-channel-fusion-unavailable");
        return output;
    }

    ChannelSignal near = correct_channel(*near_parallel, integrated_pulses, config.minimum_overlap);
    ChannelSignal far = correct_channel(*far_parallel, integrated_pulses, config.minimum_overlap);

    std::vector<double> ratios;
    for (std::size_t index = 0; index < profile.ranges_m.size(); ++index) {
        const double range = profile.ranges_m[index];
        if (range >= config.glue_start_range_m && range <= config.glue_stop_range_m
            && near.snr[index] >= config.minimum_channel_snr
            && far.snr[index] >= config.minimum_channel_snr
            && far.corrected[index] > 1e-12) {
            ratios.push_back(near.corrected[index] / far.corrected[index]);
        }
    }
    const double far_scale = ratios.size() >= 5
        ? std::clamp(median(std::move(ratios)), 0.05, 20.0)
        : 1.0;

    lidar_core::PreprocessResult output;
    output.l1_signal.resize(profile.ranges_m.size(), 0.0);
    output.attenuated_backscatter.resize(profile.ranges_m.size(), 1e-9);
    output.snr.resize(profile.ranges_m.size(), 0.0);
    const double glue_span = std::max(config.glue_stop_range_m - config.glue_start_range_m, 1.0);
    const double energy = std::max(profile.laser_energy_mj, 1e-9);
    int low_snr_bins = 0;
    for (std::size_t index = 0; index < profile.ranges_m.size(); ++index) {
        double far_weight = std::clamp(
            (profile.ranges_m[index] - config.glue_start_range_m) / glue_span,
            0.0,
            1.0);
        if (far.snr[index] < config.minimum_channel_snr) {
            far_weight = 0.0;
        } else if (near.snr[index] < config.minimum_channel_snr) {
            far_weight = 1.0;
        }
        const double corrected = (1.0 - far_weight) * near.corrected[index]
            + far_weight * far.corrected[index] * far_scale;
        output.l1_signal[index] = corrected;
        const double range_km = profile.ranges_m[index] / 1000.0;
        output.attenuated_backscatter[index] = std::max(corrected / energy * range_km * range_km, 1e-9);
        output.snr[index] = (1.0 - far_weight) * near.snr[index] + far_weight * far.snr[index];
        low_snr_bins += output.snr[index] < config.minimum_channel_snr ? 1 : 0;
    }
    output.qc_flags.emplace_back("near-far-channels-glued");
    if (far_scale == 1.0) {
        output.qc_flags.emplace_back("channel-gluing-ratio-unverified");
    }
    if (low_snr_bins * 2 > static_cast<int>(profile.ranges_m.size())) {
        output.qc_flags.emplace_back("low-snr-majority-of-range");
    }
    return output;
}

void append_unique(std::vector<std::string>& output, const std::vector<std::string>& values) {
    for (const auto& value : values) {
        if (std::find(output.begin(), output.end(), value) == output.end()) {
            output.push_back(value);
        }
    }
}

} // namespace

FrameProcessor::FrameProcessor(ProcessorConfig config)
    : config_(std::move(config)) {
}

void FrameProcessor::set_step_complete_callback(StepCompleteCallback callback) {
    on_step_complete_ = std::move(callback);
}

void FrameProcessor::set_pm_calibration(const PmCalibrationModel& calibration) {
    config_.pm_calibration = calibration;
}

lidar_core::ProcessedProfile FrameProcessor::process_device_profile(
    const lidar_protocol::Frame& frame,
    lidar_core::LidarProfile profile,
    std::vector<std::string>& qc_flags) const {
    const auto started = std::chrono::steady_clock::now();
    if (profile.overlap.empty() && !profile.ranges_m.empty()) {
        profile.overlap.assign(profile.ranges_m.size(), 1.0);
        qc_flags.emplace_back("main-channel-overlap-assumed-unity");
    }
    validate_profile(profile);
    const int integrated_pulses = std::max(
        1,
        static_cast<int>(std::llround(json_number(frame.payload, "integrated_pulses", 1.0))));
    const double wavelength_nm = json_number(frame.payload, "wavelength_nm", 532.0);
    fill_standard_molecular_reference(
        profile,
        site_info_.altitude_m,
        wavelength_nm,
        qc_flags);

    lidar_core::PreprocessResult preprocessed = preprocess_receiver_channels(
        profile,
        integrated_pulses,
        config_);
    append_unique(qc_flags, preprocessed.qc_flags);

    lidar_core::FernaldInversionStep inversion(config_.retrieval);
    auto optical = inversion.process(profile, preprocessed.attenuated_backscatter);
    lidar_core::HumidityCorrectionStep humidity(config_.humidity);
    std::vector<double> dry_extinction = humidity.process(optical.first, profile.relative_humidity);

    std::vector<double> pm25;
    std::vector<double> pm10;
    if (config_.pm_calibration.valid) {
        pm25.reserve(dry_extinction.size());
        pm10.reserve(dry_extinction.size());
        for (double extinction_km : dry_extinction) {
            pm25.push_back(std::max(
                0.0,
                config_.pm_calibration.pm25_intercept_ugm3
                    + config_.pm_calibration.pm25_slope_ugm3_per_km * extinction_km));
            pm10.push_back(std::max(
                0.0,
                config_.pm_calibration.pm10_intercept_ugm3
                    + config_.pm_calibration.pm10_slope_ugm3_per_km * extinction_km));
        }
    } else {
        qc_flags.emplace_back("pm-calibration-missing");
    }

    lidar_core::CoordinateProjectionStep projection;
    auto enu_points = projection.process(profile);
    const auto ended = std::chrono::steady_clock::now();
    return lidar_core::ProcessedProfile{
        std::move(profile),
        std::move(preprocessed.l1_signal),
        std::move(preprocessed.attenuated_backscatter),
        std::move(preprocessed.snr),
        std::move(optical.first),
        std::move(dry_extinction),
        std::move(pm25),
        std::move(pm10),
        std::move(enu_points),
        qc_flags,
        std::chrono::duration<double, std::milli>(ended - started).count(),
    };
}

void FrameProcessor::handle_frame(const lidar_protocol::Frame& frame) {
    switch (frame.type) {
    case lidar_protocol::FrameType::lidar_raw: {
        if (!current_timestamp_.empty() && frame.timestamp != current_timestamp_) {
            finalize_step();
        }
        current_timestamp_ = frame.timestamp;
        const std::string scan_pattern = json_string(frame.payload, "azimuth_scan_pattern");
        if (!scan_pattern.empty()) {
            current_scan_pattern_ = scan_pattern;
        }
        ++current_raw_count_;
        try {
            lidar_core::LidarProfile profile = lidar_protocol::json_to_profile(frame.payload);
            profile.timestamp = frame.timestamp;
            std::vector<std::string> profile_qc;
            auto processed = process_device_profile(frame, std::move(profile), profile_qc);
            append_unique(current_qc_flags_, profile_qc);
            current_processed_.push_back(std::move(processed));
        } catch (const std::exception&) {
            ++current_rejected_count_;
            append_unique(current_qc_flags_, {"malformed-or-unprocessable-lidar-frame"});
        }
        break;
    }
    case lidar_protocol::FrameType::ground_obs: {
        auto ground = lidar_protocol::json_to_ground(frame.payload);
        if (ground.timestamp.empty()) {
            ground.timestamp = frame.timestamp;
        }
        current_ground_.push_back(std::move(ground));
        break;
    }
    case lidar_protocol::FrameType::status:
    case lidar_protocol::FrameType::telemetry:
        if (frame.payload.contains("site_id") && frame.payload.at("site_id").is_string()) {
            site_info_.site_id = frame.payload.at("site_id").string_value();
        }
        if (frame.payload.contains("site_name") && frame.payload.at("site_name").is_string()) {
            site_info_.name = frame.payload.at("site_name").string_value();
        }
        if (frame.payload.contains("altitude_m") && frame.payload.at("altitude_m").is_number()) {
            site_info_.altitude_m = frame.payload.at("altitude_m").number_value();
        }
        break;
    case lidar_protocol::FrameType::heartbeat:
        finalize_step();
        break;
    default:
        break;
    }
}

void FrameProcessor::finalize_step() {
    if (current_timestamp_.empty()) {
        return;
    }

    StepResult result;
    result.timestamp = current_timestamp_;
    result.scan_pattern = current_scan_pattern_;
    result.processed_profiles = std::move(current_processed_);
    result.ground_measurements = std::move(current_ground_);
    result.qc_flags = std::move(current_qc_flags_);
    result.raw_count = current_raw_count_;
    result.rejected_count = current_rejected_count_;
    result.pm_calibrated = config_.pm_calibration.valid;
    result.calibration_id = config_.pm_calibration.calibration_id;

    std::vector<lidar_core::ProcessedProfile> ppi_profiles;
    std::set<int> elevations;
    double latency_sum = 0.0;
    for (const auto& processed : result.processed_profiles) {
        latency_sum += processed.latency_ms;
        if (processed.profile.scan_mode == "ppi") {
            ppi_profiles.push_back(processed);
            elevations.insert(static_cast<int>(std::llround(processed.profile.elevation_deg * 100.0)));
            ++result.ppi_count;
        } else if (processed.profile.scan_mode == "stare") {
            ++result.vertical_count;
        }
    }
    result.elevation_layer_count = static_cast<int>(elevations.size());
    result.mean_processing_latency_ms = result.processed_profiles.empty()
        ? 0.0
        : latency_sum / static_cast<double>(result.processed_profiles.size());

    if (result.pm_calibrated && !ppi_profiles.empty()) {
        result.hotspots = lidar_core::detect_hotspots_from_processed(ppi_profiles, config_.hotspot);
    }

    current_timestamp_.clear();
    current_scan_pattern_.clear();
    current_processed_.clear();
    current_ground_.clear();
    current_qc_flags_.clear();
    current_raw_count_ = 0;
    current_rejected_count_ = 0;
    if (on_step_complete_) {
        on_step_complete_(std::move(result));
    }
}

} // namespace lidar_client
