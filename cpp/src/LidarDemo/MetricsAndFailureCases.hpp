// Internal include fragment: metrics, drift monitoring, and failure injection.
// ---- 评估指标 (MAE / RMSE / R² / 分类指标) ----

/**
 * @brief 计算平均绝对误差 MAE（Mean Absolute Error）。
 * @param truth      真值序列。
 * @param prediction 预测序列（与 truth 等长）。
 * @return MAE；若序列为空返回 0。
 */
double mae_metric(const std::vector<double>& truth, const std::vector<double>& prediction) {
    if (truth.empty()) {
        return 0.0;
    }
    double total = 0.0;
    for (std::size_t index = 0; index < truth.size(); ++index) {
        total += std::abs(truth[index] - prediction[index]);
    }
    return total / static_cast<double>(truth.size());
}

/**
 * @brief 计算均方根误差 RMSE（Root Mean Squared Error）。
 *
 * 相比 MAE，RMSE 对较大的异常误差更敏感（先平方再开方），
 * 因此常用于突出严重偏差的影响。
 *
 * @param truth      真值序列。
 * @param prediction 预测序列。
 * @return RMSE；若序列为空返回 0。
 */
double rmse_metric(const std::vector<double>& truth, const std::vector<double>& prediction) {
    if (truth.empty()) {
        return 0.0;
    }
    double total = 0.0;
    for (std::size_t index = 0; index < truth.size(); ++index) {
        double residual = truth[index] - prediction[index];
        total += residual * residual;
    }
    return std::sqrt(total / static_cast<double>(truth.size()));
}

/**
 * @brief 计算决定系数 R²（Coefficient of Determination）。
 *
 * 公式：R² = 1 - SS_res / SS_tot，其中 SS_res 是残差平方和，SS_tot 是总平方和（相对均值）。
 * 取值范围 (-∞, 1]，越接近 1 表示模型解释力越强；若真值方差为 0（total == 0）则返回 0。
 *
 * @param truth      真值序列。
 * @param prediction 预测序列。
 * @return R²；序列为空或总方差为 0 时返回 0。
 */
double r2_metric(const std::vector<double>& truth, const std::vector<double>& prediction) {
    if (truth.empty()) {
        return 0.0;
    }
    double truth_mean = mean(truth);
    double total = 0.0;     ///< SS_tot：真值关于自身均值的总平方和
    double residual = 0.0;  ///< SS_res：预测残差平方和
    for (std::size_t index = 0; index < truth.size(); ++index) {
        total += (truth[index] - truth_mean) * (truth[index] - truth_mean);
        residual += (truth[index] - prediction[index]) * (truth[index] - prediction[index]);
    }
    if (total == 0.0) {
        return 0.0;
    }
    return 1.0 - residual / total;
}

/**
 * @brief 由真值/预测的 0/1 掩码计算二分类指标（precision、recall、F1）及混淆矩阵分量。
 *
 * 适用于热点检测这种二分类问题：1 表示热点，0 表示非热点。
 * 分母使用 std::max(..., 1) 以避免零除；当 precision 和 recall 同时为 0 时 F1 直接取 0。
 *
 * @param truth      真值 0/1 序列。
 * @param prediction 预测 0/1 序列（与 truth 等长）。
 * @return Json 含 precision、recall、f1、true_positive、false_positive、false_negative。
 */
Json classification_metrics(const std::vector<int>& truth, const std::vector<int>& prediction) {
    int true_positive = 0;
    int false_positive = 0;
    int false_negative = 0;
    for (std::size_t index = 0; index < truth.size(); ++index) {
        if (truth[index] == 1 && prediction[index] == 1) {
            ++true_positive;
        } else if (truth[index] == 0 && prediction[index] == 1) {
            ++false_positive;
        } else if (truth[index] == 1 && prediction[index] == 0) {
            ++false_negative;
        }
    }
    double precision = true_positive / static_cast<double>(std::max(true_positive + false_positive, 1));
    double recall = true_positive / static_cast<double>(std::max(true_positive + false_negative, 1));
    double f1 = (precision + recall == 0.0) ? 0.0 : 2.0 * precision * recall / (precision + recall);  ///< F1 = 2PR/(P+R)
    return Json::Object{
        {"precision", precision},
        {"recall", recall},
        {"f1", f1},
        {"true_positive", true_positive},
        {"false_positive", false_positive},
        {"false_negative", false_negative},
    };
}

// ---- 地面预测提取与漂移监控 ----

/**
 * @brief 用已拟合的模型对每个时间步的地面特征重算地面 PM 预测，构造评估/残差表。
 *
 * 对每个 timestamp 取出对应的地面特征样本，用模型预测 PM2.5/PM10 并叠加站点偏置，
 * 然后把真值/预测/残差按 timestamp 顺序填入 PredictionSummary，便于后续计算指标
 * 与滚动残差监控。
 *
 * @param feature_table   全部地面特征样本。
 * @param models          已拟合的 PM2.5/PM10 线性模型。
 * @param timestamps      需要预测的时间戳序列（需出现在 feature_table 中）。
 * @param station_offsets 站点偏置表。
 * @return PredictionSummary 含 PM 真值/预测序列及残差行表。
 */
PredictionSummary extract_ground_predictions(
    const std::vector<FeatureSample>& feature_table,
    const CalibrationModels& models,
    const std::vector<std::string>& timestamps,
    const std::map<std::string, StationOffset>& station_offsets
) {
    std::map<std::string, FeatureSample> feature_by_timestamp;
    for (const auto& sample : feature_table) {
        feature_by_timestamp[sample.timestamp] = sample;
    }

    PredictionSummary output;
    for (const auto& timestamp : timestamps) {
        const auto& sample = feature_by_timestamp.at(timestamp);
        std::string site_id = sample.site_id.empty() ? "default-site" : sample.site_id;
        StationOffset offset = station_offsets.contains(site_id) ? station_offsets.at(site_id) : StationOffset{};
        double predicted_pm25 = predict(models.pm25, sample.features) + offset.pm25_offset;
        double predicted_pm10 = predict(models.pm10, sample.features) + offset.pm10_offset;
        output.pm25_truth.push_back(sample.pm25_true);
        output.pm25_pred.push_back(predicted_pm25);
        output.pm10_truth.push_back(sample.pm10_true);
        output.pm10_pred.push_back(predicted_pm10);
        // 记录每个样本的残差，供后续分站点滚动监控
        output.residual_rows.push_back(ResidualRow{timestamp, site_id, sample.pm25_true - predicted_pm25, sample.pm10_true - predicted_pm10});
    }
    return output;
}

/**
 * @brief 按站点对残差做滑动窗口监控，捕捉模型漂移并生成告警。
 *
 * 算法：
 *  1. 按 site_id 分组，每组按时间排序；
 *  2. 用大小为 window_size（默认 3）的滑窗统计 PM2.5/PM10 平均残差（即偏差 bias）；
 *  3. 当 |pm25_bias| > 6.0 或 |pm10_bias| > 8.0 时标为 warning，否则 ok；
 *  4. 汇总每个站点的最新窗口状态，并把所有告警扁平化为 alerts 数组。
 *
 * 告警阈值反映了"在 3 个连续样本上模型系统性偏差已超出可接受范围"的工程判断。
 *
 * @param residual_rows 来自 extract_ground_predictions 的残差行序列。
 * @return Json 含 "stations"（每站点最新状态 + 全部滑窗）与 "alerts"（告警列表）。
 */
Json summarize_drift_monitoring(const std::vector<ResidualRow>& residual_rows) {
    std::map<std::string, std::vector<ResidualRow>> grouped;
    for (const auto& row : residual_rows) {
        grouped[row.site_id].push_back(row);
    }

    Json::Object stations;
    Json::Array alerts;
    for (auto& [site_id, rows] : grouped) {
        // 同站点样本按时间升序排列，保证滑窗语义正确
        std::sort(rows.begin(), rows.end(), [](const ResidualRow& left, const ResidualRow& right) {
            return left.timestamp < right.timestamp;
        });
        int window_size = std::min<int>(3, static_cast<int>(rows.size()));  ///< 滑窗长度，最多 3
        Json::Array windows;
        for (int index = 0; index <= static_cast<int>(rows.size()) - window_size; ++index) {
            std::vector<double> pm25_window;
            std::vector<double> pm10_window;
            for (int offset = 0; offset < window_size; ++offset) {
                pm25_window.push_back(rows[index + offset].pm25_residual);
                pm10_window.push_back(rows[index + offset].pm10_residual);
            }
            double pm25_bias = mean(pm25_window);
            double pm10_bias = mean(pm10_window);
            // 工程阈值：PM2.5 偏差超 6 或 PM10 偏差超 8 即视为系统漂移
            std::string alert_level = (std::abs(pm25_bias) > 6.0 || std::abs(pm10_bias) > 8.0) ? "warning" : "ok";
            Json window_summary = Json::Object{
                {"start", rows[index].timestamp},
                {"end", rows[index + window_size - 1].timestamp},
                {"pm25_bias", pm25_bias},
                {"pm10_bias", pm10_bias},
                {"alert_level", alert_level},
            };
            windows.push_back(window_summary);
            if (alert_level != "ok") {
                alerts.push_back(Json::Object{
                    {"site_id", site_id},
                    {"start", rows[index].timestamp},
                    {"end", rows[index + window_size - 1].timestamp},
                    {"pm25_bias", pm25_bias},
                    {"pm10_bias", pm10_bias},
                    {"alert_level", alert_level},
                });
            }
        }
        // 取最末窗口作为该站点的"当前状态"，无样本时给出 ok 兜底
        Json latest_window = windows.empty() ? Json(Json::Object{
            {"pm25_bias", 0.0},
            {"pm10_bias", 0.0},
            {"alert_level", "ok"},
        }) : windows.back();
        stations[site_id] = Json::Object{
            {"sample_count", static_cast<int>(rows.size())},
            {"latest_pm25_bias", latest_window.at("pm25_bias")},
            {"latest_pm10_bias", latest_window.at("pm10_bias")},
            {"latest_alert_level", latest_window.at("alert_level")},
            {"windows", Json(std::move(windows))},
        };
    }
    return Json::Object{
        {"stations", Json(std::move(stations))},
        {"alerts", Json(std::move(alerts))},
    };
}

// ---- 失败案例注入（鲁棒性压力测试） ----

/**
 * @brief 向原始观测数据注入指定的故障，用于压力测试整条流水线的鲁棒性。
 *
 * 当前支持三种典型故障模式：
 *  - `high-background-light`：模拟强背景光（如白天太阳），整体抬高基线与原始信号噪声；
 *  - `overlap-miscalibration`：把前若干距离门的几何重叠因子衰减，模拟近场校正失准；
 *  - `humidity-surge`：把所有 RH 抬高最多 22 个百分点（上限 95%），模拟湿度突变。
 *
 * 故障直接就地修改入参 profile/ground_measurements，调用方应传入副本以保留原始数据。
 * 遇到不支持的 case_name 抛出 std::runtime_error。
 *
 * @param profiles            待注入故障的雷达廓线集合（就地修改）。
 * @param ground_measurements 待注入故障的地面观测集合（就地修改）。
 * @param case_name           故障名（high-background-light / overlap-miscalibration / humidity-surge）。
 */
void apply_failure_case(std::vector<LidarProfile>& profiles, std::vector<GroundMeasurement>& ground_measurements, const std::string& case_name) {
    if (case_name == "high-background-light") {
        for (auto& profile : profiles) {
            // 背景光抬升 1.9 倍现值，原始信号抬升约 2.2 倍背景增量，模拟太阳背景淹没
            double extra_background = profile.background_counts * 1.9;
            profile.background_counts += extra_background;
            for (double& value : profile.raw_counts) {
                value += extra_background * 2.2;
            }
        }
        return;
    }
    if (case_name == "overlap-miscalibration") {
        for (auto& profile : profiles) {
            // 仅污染前 6 个近场距离门，模拟近场几何重叠失准
            for (int index = 0; index < std::min<int>(6, static_cast<int>(profile.overlap.size())); ++index) {
                profile.overlap[index] = std::max(profile.overlap[index] * 0.65, 0.1);
            }
        }
        return;
    }
    if (case_name == "humidity-surge") {
        // RH 最多抬高到 0.95（避免出现饱和水汽导致公式奇异）
        for (auto& profile : profiles) {
            profile.relative_humidity = std::min(0.95, profile.relative_humidity + 0.22);
        }
        for (auto& measurement : ground_measurements) {
            measurement.relative_humidity = std::min(0.95, measurement.relative_humidity + 0.22);
        }
        return;
    }
    throw std::runtime_error("Unsupported failure case: " + case_name);
}

// 前向声明：实际实现见本文件稍后，run_failure_case_suite 对每种 case 各跑一次完整流水线
Json::Array run_failure_case_suite(const PipelineConfig& config, const SiteInfo& site, const std::vector<LidarProfile>& profiles, const std::vector<GroundMeasurement>& ground_measurements);

