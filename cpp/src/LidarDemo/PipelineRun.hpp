// Internal include fragment: dataset pipeline execution.
// ---- 单次数据集全流水线运行 ----

/**
 * @brief 在一组数据（雷达廓线 + 地面观测）上完整运行反演 + PM 校准 + 热点检测 + 评估。
 *
 * 流水线步骤：
 *  1. 逐条廓线：预处理 → Fernald 反演 → （可选）湿度修正 → 转 ENU 坐标，并记录单条耗时；
 *  2. 构造特征表 → 拟合 PM 线性模型 → 求站点偏置 → 把模型应用到所有廓线；
 *  3. 按时间步对 PPI 廓线做热点检测，累积预测/真值掩码；
 *  4. 在 holdout（val+test，否则 train）时间戳上抽取地面预测，计算回归指标、
 *     分类指标、漂移监控与吞吐量指标，统一写入 result.metrics。
 *
 * @param config                流水线配置（含反演/AWI/PM/热点各类阈值）。
 * @param site                  站点信息。
 * @param profiles              原始雷达廓线。
 * @param ground_measurements   原始地面观测。
 * @param lidar_ratio_override  >0 时覆盖 config.retrieval.aerosol_lidar_ratio_sr，用于敏感性分析。
 * @param disable_humidity      为 true 时跳过湿度修正，用于消融实验。
 * @return DatasetRunResult     含全部中间产物与汇总指标。
 */
DatasetRunResult run_pipeline_on_dataset(
    const PipelineConfig& config,
    const SiteInfo& site,
    const std::vector<LidarProfile>& profiles,
    const std::vector<GroundMeasurement>& ground_measurements,
    std::optional<double> lidar_ratio_override = std::nullopt,
    bool disable_humidity = false
) {
    DatasetRunResult result;
    result.site = site;
    result.profiles = profiles;
    result.ground_measurements = ground_measurements;

    auto total_start = std::chrono::steady_clock::now();
    // 第一阶段：逐条廓线反演。这里使用类化处理链，便于把 L0->L1->L2 每一步独立替换/测试。
    RetrievalConfig retrieval_for_run = config.retrieval;
    retrieval_for_run.aerosol_lidar_ratio_sr = lidar_ratio_override.value_or(config.retrieval.aerosol_lidar_ratio_sr);
    SingleProfileProcessingChain profile_chain(retrieval_for_run, config.humidity);
    for (const auto& profile : profiles) {
        result.processed_profiles.push_back(profile_chain.process(profile, disable_humidity));
    }

    // 第二阶段：构造地面特征表 + 拟合 PM 模型 + 求偏置 + 应用到每条廓线
    result.feature_table = build_timestamp_feature_table(result.processed_profiles, ground_measurements, config.pm_calibration.surface_bin_count);
    auto fitted = fit_pm_models(result.feature_table);
    CalibrationModels models = fitted.first;
    result.split = fitted.second;
    result.station_offsets = fit_station_offsets(models, result.split);
    apply_pm_models(result.processed_profiles, models, result.feature_table, result.station_offsets);

    // 第三阶段：按 timestamp 分组对 PPI 廓线做热点检测
    std::vector<int> predicted_mask;
    std::vector<int> truth_mask;
    std::map<std::string, std::vector<ProcessedProfile*>> grouped;
    for (auto& processed : result.processed_profiles) {
        grouped[processed.profile.timestamp].push_back(&processed);
    }
    for (auto& [timestamp, profiles_at_timestamp] : grouped) {
        std::vector<ProcessedProfile*> ppi_profiles;
        for (auto* profile : profiles_at_timestamp) {
            if (profile->profile.scan_mode == "ppi") {
                ppi_profiles.push_back(profile);
            }
        }
        DetectionResult detection = detect_hotspots(
            ppi_profiles,
            config.hotspot.pm25_threshold_ugm3,
            config.hotspot.scan_relative_pm25_threshold_ugm3,
            config.hotspot.scan_relative_dry_ext_threshold,
            config.hotspot.min_cells
        );
        result.hotspots_by_timestamp[timestamp] = detection.hotspots;
        predicted_mask.insert(predicted_mask.end(), detection.predicted_mask.begin(), detection.predicted_mask.end());
        truth_mask.insert(truth_mask.end(), detection.truth_mask.begin(), detection.truth_mask.end());
    }

    // 第四阶段：在 holdout（或退回 train）时间戳上抽取地面预测以计算指标
    auto total_end = std::chrono::steady_clock::now();
    std::vector<std::string> holdout_timestamps = result.split.val_timestamps;
    holdout_timestamps.insert(holdout_timestamps.end(), result.split.test_timestamps.begin(), result.split.test_timestamps.end());
    // holdout 为空时退回训练集，保证小数据集场景仍可输出指标
    std::vector<std::string> evaluation_timestamps = holdout_timestamps.empty() ? result.split.train_timestamps : holdout_timestamps;
    PredictionSummary predictions = extract_ground_predictions(result.feature_table, models, evaluation_timestamps, result.station_offsets);
    Json hotspot_scores = classification_metrics(truth_mask, predicted_mask);
    result.drift_monitoring = summarize_drift_monitoring(predictions.residual_rows);

    // 收集每条廓线的处理时延，用于吞吐量统计
    std::vector<double> latency_values;
    latency_values.reserve(result.processed_profiles.size());
    for (const auto& profile : result.processed_profiles) {
        latency_values.push_back(profile.latency_ms);
    }
    double elapsed_s = std::chrono::duration<double>(total_end - total_start).count();
    result.metrics = Json::Object{
        {"pm25", Json::Object{
            {"mae", mae_metric(predictions.pm25_truth, predictions.pm25_pred)},
            {"rmse", rmse_metric(predictions.pm25_truth, predictions.pm25_pred)},
            {"r2", r2_metric(predictions.pm25_truth, predictions.pm25_pred)},
        }},
        {"pm10", Json::Object{
            {"mae", mae_metric(predictions.pm10_truth, predictions.pm10_pred)},
            {"rmse", rmse_metric(predictions.pm10_truth, predictions.pm10_pred)},
            {"r2", r2_metric(predictions.pm10_truth, predictions.pm10_pred)},
        }},
        {"hotspot", hotspot_scores},
        {"drift", Json::Object{
            {"station_count", static_cast<int>(result.drift_monitoring.at("stations").object_items().size())},
            {"alert_count", static_cast<int>(result.drift_monitoring.at("alerts").array_items().size())},
        }},
        {"runtime", Json::Object{
            {"mean_latency_ms", mean(latency_values)},
            {"throughput_profiles_per_s", static_cast<double>(result.processed_profiles.size()) / std::max(elapsed_s, 1e-9)},  ///< 用 max(.,1e-9) 防止零除
        }},
    };
    return result;
}

