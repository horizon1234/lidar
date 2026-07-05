// Internal include fragment: demo payload assembly.
// ---- 演示 payload 序列化辅助 ----

/**
 * @brief 把单个 FeatureSample 序列化为前端友好的 JSON。
 * @param sample 地面特征样本。
 * @return Json 含 timestamp、features、各类地面观测量。
 */
Json feature_sample_to_json(const FeatureSample& sample) {
    return Json::Object{
        {"timestamp", sample.timestamp},
        {"features", json_array_from_double_vector(sample.features)},
        {"surface_dry_ext", sample.surface_dry_ext},
        {"surface_attenuated", sample.surface_attenuated},
        {"hotspot_proxy", sample.hotspot_proxy},
        {"site_id", sample.site_id},
        {"pm25_true", sample.pm25_true},
        {"pm10_true", sample.pm10_true},
        {"relative_humidity", sample.relative_humidity},
        {"wind_speed_ms", sample.wind_speed_ms},
    };
}

/**
 * @brief 把站点偏置映射序列化为以 site_id 为键的 JSON 对象。
 * @param station_offsets 站点偏置映射。
 * @return Json 每个站点含 pm25_offset / pm10_offset / sample_count。
 */
Json station_offsets_to_json(const std::map<std::string, StationOffset>& station_offsets) {
    Json::Object output;
    for (const auto& [site_id, offset] : station_offsets) {
        output[site_id] = Json::Object{
            {"pm25_offset", offset.pm25_offset},
            {"pm10_offset", offset.pm10_offset},
            {"sample_count", offset.sample_count},
        };
    }
    return Json(std::move(output));
}

Json build_device_product_schema(const Json& source_metadata) {
    std::string vendor_profile = source_metadata.contains("vendor_profile")
        ? source_metadata.at("vendor_profile").string_value()
        : "generic_jsonl";

    Json::Array l0_fields{
        Json("type"), Json("timestamp"), Json("sequence_id"), Json("frame_id"),
        Json("scan_cycle_id"), Json("scan_mode"), Json("azimuth_deg"),
        Json("elevation_deg"), Json("ranges_m"), Json("raw_counts"),
        Json("laser_energy_mj"), Json("background_counts"), Json("overlap"),
        Json("channel_id"), Json("detector_mode"), Json("integration_pulses"),
        Json("accumulation_time_ms")
    };
    Json::Array telemetry_fields{
        Json("device_state"), Json("scan_scheduler_state"), Json("gps_lock"),
        Json("ntp_sync"), Json("clock_offset_ms"), Json("enclosure_temp_c"),
        Json("laser_head_temp_c"), Json("detector_temp_c"),
        Json("window_transmission"), Json("window_contamination_index"),
        Json("rain_sensor_wet"), Json("sun_background_counts"),
        Json("fan_state"), Json("door_state"), Json("wiper_state"),
        Json("diagnostic_flags")
    };
    Json::Array l3_fields{
        Json("x_m"), Json("y_m"), Json("z_m"), Json("pm25"),
        Json("confidence"), Json("source_ray_count")
    };

    Json profile_specific = Json::Object{};
    if (vendor_profile == "vaisala_cl61_like") {
        profile_specific = Json::Object{
            {"public_format_basis", "Vaisala CL61-like NetCDF public product schema"},
            {"dimensions", Json::Array{Json("time"), Json("range"), Json("layer")}},
            {"variables", Json::Array{
                Json("p_pol"), Json("x_pol"), Json("beta_att"),
                Json("linear_depol_ratio"), Json("cloud_base_heights"),
                Json("vertical_visibility"), Json("window_condition"),
                Json("laser_power_percent"), Json("background_radiance"),
                Json("internal_temperature"), Json("internal_humidity"),
                Json("laser_temperature")
            }},
            {"note", "Real CL61 low-level message frames are vendor-specific; this project maps public NetCDF-style products."}
        };
    } else if (vendor_profile == "raymetrics_pmeye_like") {
        profile_specific = Json::Object{
            {"public_format_basis", "Raymetrics PMeye-like scanning aerosol LiDAR product"},
            {"channels", Json::Array{Json("elastic_355nm"), Json("depolarization_355nm_optional")}},
            {"variables", Json::Array{
                Json("raw_signal_analog"), Json("raw_signal_photon_counting"),
                Json("overlap_corrected_signal"), Json("attenuated_backscatter"),
                Json("extinction"), Json("pm25"), Json("pm10"),
                Json("depolarization_ratio_optional"), Json("scan_azimuth"),
                Json("scan_elevation"), Json("source_location_3d")
            }},
            {"note", "Public brochures expose hardware/product behavior, not private TCP frame layouts."}
        };
    } else if (vendor_profile == "halo_hpl_like") {
        profile_specific = Json::Object{
            {"public_format_basis", "HALO StreamLine HPL/ASCII-like Doppler LiDAR replay product"},
            {"variables", Json::Array{
                Json("range"), Json("azimuth"), Json("elevation"),
                Json("radial_velocity_optional"), Json("snr_or_cnr"),
                Json("beta_or_intensity"), Json("scan_schedule")
            }},
            {"note", "HALO atmospheric Doppler products differ from PM elastic LiDAR; this is useful for replay/scheduler realism."}
        };
    } else {
        profile_specific = Json::Object{
            {"public_format_basis", "Project generic JSON line protocol"},
            {"note", "Use this when no vendor-specific public schema is selected."}
        };
    }

    return Json::Object{
        {"vendor_profile", vendor_profile},
        {"l0_realtime_frame_fields", Json(std::move(l0_fields))},
        {"device_telemetry_fields", Json(std::move(telemetry_fields))},
        {"l3_volume_fields", Json(std::move(l3_fields))},
        {"profile_specific_mapping", profile_specific},
        {"private_protocol_warning", "A 100% clone of a commercial device frame requires vendor SDK/protocol docs or captured sample frames."},
    };
}

/**
 * @brief 用时间戳序列推断采样间隔（分钟），用于前端展示 dataset_summary。
 *
 * 计算相邻时间戳之间的时间差（分钟），再取均值并四舍五入到整数。
 * 时间戳不足 2 个时返回 0。
 *
 * @param timestamps 升序时间戳序列（ISO 字符串）。
 * @return int 采样间隔，单位分钟；样本不足时返回 0。
 */
int sampling_minutes(const std::vector<std::string>& timestamps) {
    if (timestamps.size() < 2) {
        return 0;
    }
    std::vector<double> deltas;
    for (std::size_t index = 1; index < timestamps.size(); ++index) {
        // difftime 返回秒，除以 60 转成分钟
        deltas.push_back(std::difftime(parse_timestamp(timestamps[index]), parse_timestamp(timestamps[index - 1])) / 60.0);
    }
    return static_cast<int>(std::lround(mean(deltas)));
}

/**
 * @brief 把一次主实验与各项副实验拼装成单个完整的演示 payload，用于驱动前端仪表盘。
 *
 * payload 组成：
 *  - site / dataset_summary：站点信息与采样概要（含来源构成 breakdown）；
 *  - curtain：stare 模式下的"时间-高度"幕布图数据（PM 与消光）；
 *  - ppi：最新时刻 PPI 扫描的网格化 PM 数据（前端可绘制 2D 平面图）；
 *  - qc：逐时间步的质量控制统计（SNR、时延、背景光、激光能量）；
 *  - hotspots / alerts：最新热点列表 + 全部时间步的告警；
 *  - metrics / sensitivity / ablation / failure_cases：评估指标与实验矩阵；
 *  - station_calibration / drift_monitoring / ground_series：站点校准与漂移监控及原始地面序列。
 *
 * @param primary          主实验结果。
 * @param ablation         消融实验结果数组。
 * @param sensitivity      敏感性扫描结果数组。
 * @param failure_cases    失败案例结果数组。
 * @param source_metadata  数据来源元信息。
 * @return Json 完整 payload，结构稳定且自描述。
 */
Json build_demo_payload(
    const DatasetRunResult& primary,
    const Json::Array& ablation,
    const Json::Array& sensitivity,
    const Json::Array& failure_cases,
    const Json& source_metadata
) {
    // 按 timestamp 把已处理廓线重新分组，便于按时间构建幕布图与 PPI 数据
    std::map<std::string, std::vector<const ProcessedProfile*>> grouped;
    for (const auto& profile : primary.processed_profiles) {
        grouped[profile.profile.timestamp].push_back(&profile);
    }
    std::vector<std::string> timestamps;
    timestamps.reserve(grouped.size());
    for (const auto& [timestamp, _] : grouped) {
        timestamps.push_back(timestamp);
    }

    // 取每个时间步的第一条 stare 廓线，用于构建时间-高度幕布图
    std::vector<const ProcessedProfile*> stare_profiles;
    stare_profiles.reserve(timestamps.size());
    for (const auto& timestamp : timestamps) {
        auto it = std::find_if(grouped[timestamp].begin(), grouped[timestamp].end(), [](const ProcessedProfile* profile) {
            return profile->profile.scan_mode == "stare";
        });
        if (it != grouped[timestamp].end()) {
            stare_profiles.push_back(*it);
        }
    }

    Json::Array curtain_times;
    Json::Array curtain_heights;
    Json::Array curtain_pm25;
    Json::Array curtain_extinction;
    for (const auto& timestamp : timestamps) {
        curtain_times.emplace_back(timestamp);
    }
    if (!stare_profiles.empty()) {
        // 高度轴只取一次（假设所有 stare 廓线距离门一致）
        for (const auto& point : stare_profiles.front()->enu_points_m) {
            curtain_heights.emplace_back(point[2]);
        }
        for (const auto* profile : stare_profiles) {
            curtain_pm25.emplace_back(json_array_from_double_vector(profile->pm25));
            curtain_extinction.emplace_back(json_array_from_double_vector(profile->extinction));
        }
    }

    // 最新时间步的 PPI 网格化（按方位角排序后逐距离门写 cell）
    std::string latest_timestamp = timestamps.empty() ? std::string{} : timestamps.back();
    Json::Array ppi_cells;
    Json::Array volume_voxels;
    constexpr double voxel_size_m = 100.0;
    if (!latest_timestamp.empty()) {
        std::vector<const ProcessedProfile*> latest_ppi_profiles;
        for (const auto* profile : grouped[latest_timestamp]) {
            if (profile->profile.scan_mode == "ppi") {
                latest_ppi_profiles.push_back(profile);
            }
        }
        std::sort(latest_ppi_profiles.begin(), latest_ppi_profiles.end(), [](const ProcessedProfile* left, const ProcessedProfile* right) {
            return left->profile.azimuth_deg < right->profile.azimuth_deg;
        });
        for (const auto* processed : latest_ppi_profiles) {
            for (std::size_t index = 0; index < processed->enu_points_m.size(); ++index) {
                ppi_cells.push_back(Json::Object{
                    {"x_m", processed->enu_points_m[index][0]},
                    {"y_m", processed->enu_points_m[index][1]},
                    {"z_m", processed->enu_points_m[index][2]},
                    {"pm25", processed->pm25[index]},
                    {"pm10", processed->pm10[index]},
                    {"is_true_hotspot", processed->profile.true_hotspot_mask[index] == 1},
                });
            }
        }

        struct VoxelAccumulator {
            double x_sum = 0.0;
            double y_sum = 0.0;
            double z_sum = 0.0;
            double pm25_weighted_sum = 0.0;
            double confidence_sum = 0.0;
            int count = 0;
        };
        std::map<std::string, VoxelAccumulator> voxels;
        for (const auto* processed : latest_ppi_profiles) {
            for (std::size_t index = 0; index < processed->enu_points_m.size(); ++index) {
                const auto& point = processed->enu_points_m[index];
                int ix = static_cast<int>(std::round(point[0] / voxel_size_m));
                int iy = static_cast<int>(std::round(point[1] / voxel_size_m));
                int iz = static_cast<int>(std::round(point[2] / voxel_size_m));
                std::ostringstream key;
                key << ix << ':' << iy << ':' << iz;
                double snr = index < processed->snr.size() ? processed->snr[index] : 0.0;
                double confidence = clamp(snr / 12.0, 0.05, 1.0);
                auto& voxel = voxels[key.str()];
                voxel.x_sum += point[0];
                voxel.y_sum += point[1];
                voxel.z_sum += point[2];
                voxel.pm25_weighted_sum += processed->pm25[index] * confidence;
                voxel.confidence_sum += confidence;
                ++voxel.count;
            }
        }
        for (const auto& [_, voxel] : voxels) {
            if (voxel.count <= 0 || voxel.confidence_sum <= 0.0) {
                continue;
            }
            volume_voxels.push_back(Json::Object{
                {"x_m", voxel.x_sum / static_cast<double>(voxel.count)},
                {"y_m", voxel.y_sum / static_cast<double>(voxel.count)},
                {"z_m", voxel.z_sum / static_cast<double>(voxel.count)},
                {"pm25", voxel.pm25_weighted_sum / voxel.confidence_sum},
                {"confidence", voxel.confidence_sum / static_cast<double>(voxel.count)},
                {"source_ray_count", voxel.count},
            });
        }
    }

    // 逐时间步汇总质量控制统计：取每个时间步的 stare 廓线计算近场 8 个距离门的平均 SNR
    Json::Array qc_times;
    Json::Array qc_mean_snr;
    Json::Array qc_latency;
    Json::Array qc_background;
    Json::Array qc_energy;
    for (const auto& timestamp : timestamps) {
        qc_times.emplace_back(timestamp);
        const auto& profiles_at_timestamp = grouped[timestamp];
        const ProcessedProfile* stare = nullptr;
        std::vector<double> latency_values;
        for (const auto* profile : profiles_at_timestamp) {
            latency_values.push_back(profile->latency_ms);
            if (profile->profile.scan_mode == "stare") {
                stare = profile;
            }
        }
        std::vector<double> snr_slice;
        if (stare != nullptr) {
            // 只统计前 8 个近场距离门，因为远处 SNR 容易被噪声主导
            snr_slice.assign(stare->snr.begin(), stare->snr.begin() + std::min<std::size_t>(8, stare->snr.size()));
            qc_background.emplace_back(stare->profile.background_counts);
            qc_energy.emplace_back(stare->profile.laser_energy_mj);
        } else {
            qc_background.emplace_back(0.0);
            qc_energy.emplace_back(0.0);
        }
        qc_mean_snr.emplace_back(mean(snr_slice));
        qc_latency.emplace_back(mean(latency_values));
    }

    // 全部时间步的热点（用作告警时间轴）+ 最新时间步的热点列表（用于突出展示）
    Json::Array alerts;
    for (const auto& [timestamp, hotspots] : primary.hotspots_by_timestamp) {
        for (const auto& hotspot : hotspots) {
            alerts.push_back(to_json(hotspot));
        }
    }

    Json::Array latest_hotspots;
    if (primary.hotspots_by_timestamp.contains(latest_timestamp)) {
        for (const auto& hotspot : primary.hotspots_by_timestamp.at(latest_timestamp)) {
            latest_hotspots.push_back(to_json(hotspot));
        }
    }

    // 地面序列：用于前端绘制"地面真值随时间变化"折线
    Json::Array ground_series;
    for (const auto& sample : primary.feature_table) {
        ground_series.push_back(feature_sample_to_json(sample));
    }

    Json payload = Json::Object{
        {"site", to_json(primary.site)},
        {"dataset_summary", Json::Object{
            {"profile_count", static_cast<int>(primary.processed_profiles.size())},
            {"timestamp_count", static_cast<int>(timestamps.size())},
            {"sampling_minutes", sampling_minutes(timestamps)},
            {"range_bin_count", stare_profiles.empty() ? 0 : static_cast<int>(stare_profiles.front()->profile.ranges_m.size())},
            {"train_count", static_cast<int>(primary.split.train.size())},
            {"val_count", static_cast<int>(primary.split.val.size())},
            {"test_count", static_cast<int>(primary.split.test.size())},
        }},
        {"source", source_metadata},
        {"curtain", Json::Object{
            {"times", Json(std::move(curtain_times))},
            {"heights_m", Json(std::move(curtain_heights))},
            {"pm25", Json(std::move(curtain_pm25))},
            {"extinction", Json(std::move(curtain_extinction))},
        }},
        {"ppi", Json::Object{
            {"timestamp", latest_timestamp},
            {"cells", Json(std::move(ppi_cells))},
        }},
        {"volume", Json::Object{
            {"timestamp", latest_timestamp},
            {"coordinate_system", "ENU"},
            {"voxel_size_m", voxel_size_m},
            {"voxels", Json(std::move(volume_voxels))},
        }},
        {"hotspots", Json(std::move(latest_hotspots))},
        {"qc", Json::Object{
            {"times", Json(std::move(qc_times))},
            {"mean_snr", Json(std::move(qc_mean_snr))},
            {"latency_ms", Json(std::move(qc_latency))},
            {"background_counts", Json(std::move(qc_background))},
            {"laser_energy_mj", Json(std::move(qc_energy))},
        }},
        {"alerts", Json(std::move(alerts))},
        {"device_product_schema", build_device_product_schema(source_metadata)},
        {"metrics", primary.metrics},
        {"sensitivity", Json(sensitivity)},
        {"ablation", Json(ablation)},
        {"failure_cases", Json(failure_cases)},
        {"station_calibration", station_offsets_to_json(primary.station_offsets)},
        {"drift_monitoring", primary.drift_monitoring},
        {"ground_series", Json(std::move(ground_series))},
    };

    payload["dataset_summary"]["source_mode"] = source_metadata.at("mode");
    // 按数据来源分类统计廓线来源构成，方便前端区分真实/合成数据比例
    int stare_real = 0;
    int stare_synthetic = 0;
    int ppi_hybrid = 0;
    int ppi_synthetic = 0;
    for (const auto& profile : primary.profiles) {
        if (profile.source_kind == "cloudnet_real_stare") {
            ++stare_real;
        } else if (profile.source_kind == "synthetic_stare") {
            ++stare_synthetic;
        } else if (profile.source_kind == "synthetic_ppi_hybrid") {
            ++ppi_hybrid;
        } else if (profile.source_kind == "synthetic_ppi") {
            ++ppi_synthetic;
        }
    }
    payload["dataset_summary"]["source_breakdown"] = Json::Object{
        {"stare_real", stare_real},
        {"stare_synthetic", stare_synthetic},
        {"ppi_hybrid", ppi_hybrid},
        {"ppi_synthetic", ppi_synthetic},
    };
    return payload;
}

