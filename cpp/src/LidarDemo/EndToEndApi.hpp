// Public API implementation fragment: end-to-end runner.
// ---- 端到端流水线 ----

/**
 * @brief 端到端主入口：从配置出发，完成数据准备 → 主实验 → 消融 → 敏感性 → 失败案例 → 落盘。
 *
 * 这是最重要的对外 API 之一，用户通常通过它一次性获得完整的 demo payload。
 *
 * 步骤：
 *  1. 根据 source_mode 选择数据生成方式：simulation / cloudnet_hybrid（后者需 NetCDF 支持，
 *     缺失编译宏则抛错）；
 *  2. 主实验：run_pipeline_on_dataset 用默认 lidar_ratio 和湿度修正；
 *  3. 消融：再跑一次"关闭湿度修正"得到 without-humidity-correction，与主实验一起构成 ablation；
 *  4. 敏感性：逐个 lidar_ratio 跑流水线，统计 PM RMSE 的最大-最小跨度作为 retrieval_stability；
 *  5. 失败案例套件：对三种故障各跑一次，得到失败模式下的指标；
 *  6. 把结果组装成 demo payload，若有 output_root 则把 raw/l1/l2 数据写盘；
 *  7. 返回 demo payload。
 *
 * @param config      流水线配置。
 * @param output_root 若有值则把 raw/l1/l2 结果写出到该根目录下。
 * @return Json       完整 demo payload，可直接通过 render_dashboard 渲染。
 */
Json run_end_to_end(const PipelineConfig& config, const std::optional<std::filesystem::path>& output_root) {
    // 1. 数据准备
    CampaignData campaign;
    if (config.source_mode == "simulation") {
        campaign = simulate_campaign(config);
    } else if (config.source_mode == "cloudnet_hybrid") {
#if defined(LIDAR_DEMO_HAS_NETCDF)
        campaign = load_cloudnet_hybrid_campaign(config);
#else
        // 未启用 NetCDF 编译时直接报错，避免对接失败再失败
        throw std::runtime_error("Cloudnet hybrid requires a NetCDF-enabled C++ build.");
#endif
    } else {
        throw std::runtime_error("Unsupported source mode in C++ pipeline: " + config.source_mode);
    }

    // 2. 主实验
    DatasetRunResult primary = run_pipeline_on_dataset(config, campaign.site, campaign.profiles, campaign.ground_measurements);

    // 3. 消融：与主实验对比"关闭湿度修正"的影响
    Json::Array ablation;
    ablation.push_back(Json::Object{
        {"name", "full-pipeline"},
        {"pm25_rmse", primary.metrics.at("pm25").at("rmse")},
        {"hotspot_f1", primary.metrics.at("hotspot").at("f1")},
    });

    DatasetRunResult humidity_disabled = run_pipeline_on_dataset(config, campaign.site, campaign.profiles, campaign.ground_measurements, std::nullopt, true);
    ablation.push_back(Json::Object{
        {"name", "without-humidity-correction"},
        {"pm25_rmse", humidity_disabled.metrics.at("pm25").at("rmse")},
        {"hotspot_f1", humidity_disabled.metrics.at("hotspot").at("f1")},
    });

    // 4. 敏感性扫描：在配置的 lidar_ratio 列表上各跑一次
    Json::Array sensitivity;
    std::vector<double> sensitivity_rmse;
    for (double lidar_ratio : config.evaluation.sensitivity_lidar_ratios) {
        DatasetRunResult perturbed = run_pipeline_on_dataset(config, campaign.site, campaign.profiles, campaign.ground_measurements, lidar_ratio, false);
        sensitivity.push_back(Json::Object{
            {"lidar_ratio_sr", lidar_ratio},
            {"pm25_rmse", perturbed.metrics.at("pm25").at("rmse")},
            {"pm10_rmse", perturbed.metrics.at("pm10").at("rmse")},
            {"hotspot_f1", perturbed.metrics.at("hotspot").at("f1")},
        });
        sensitivity_rmse.push_back(perturbed.metrics.at("pm25").at("rmse").number_value());
    }
    // 反演稳定性：最大最小 RMSE 差，越小越稳健
    double stability_span = sensitivity_rmse.empty() ? 0.0 : *std::max_element(sensitivity_rmse.begin(), sensitivity_rmse.end()) - *std::min_element(sensitivity_rmse.begin(), sensitivity_rmse.end());
    primary.metrics["retrieval_stability"] = Json::Object{
        {"pm25_rmse_span", stability_span},
        {"reference_lidar_ratio_sr", config.retrieval.aerosol_lidar_ratio_sr},
    };

    // 5. 失败案例套件
    Json::Array failure_cases = run_failure_case_suite(config, campaign.site, campaign.profiles, campaign.ground_measurements);
    // 6. 组装最终 demo payload
    Json demo_payload = build_demo_payload(primary, ablation, sensitivity, failure_cases, campaign.source_metadata);

    // 可选：把原始 / L1 / L2 数据写到磁盘，便于后续离线分析与前端静态加载
    if (output_root.has_value()) {
        Json::Array raw_profiles;
        for (const auto& profile : primary.profiles) {
            raw_profiles.push_back(to_json(profile));
        }
        Json::Array l1_profiles;
        for (const auto& profile : primary.processed_profiles) {
            l1_profiles.push_back(to_json(profile));
        }
        write_json_file(*output_root / "data" / "raw" / "simulated_demo_campaign.json", Json(std::move(raw_profiles)));
        write_json_file(*output_root / "data" / "l1" / "demo_preprocessed.json", Json(std::move(l1_profiles)));
        write_json_file(*output_root / "data" / "l2" / "demo_results.json", demo_payload);
        write_json_file(*output_root / "data" / "vendor" / "device_product_schema.json", demo_payload.at("device_product_schema"));
    }

    return demo_payload;
}

