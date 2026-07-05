// Internal include fragment: JSON serialization helpers and internal result types.
// ====================================================================================
// ---- JSON 序列化辅助 / JSON serialization helpers ------------------------------------------------
// 该组函数负责把内部 C++ 结构体（矢量、标量、矩阵及业务对象）转换成 JSON 对象。
// 这些 to_json 系列是给上层 API 输出 L1/L2 数据产品用的，避免了手写重复字段名。
// ====================================================================================

/**
 * @brief 将一维 double 数组转换为 JSON 数组。
 * @param[in] values 待转换的浮点数组（通常是消光、后向散射等物理量序列）。
 * @return 与输入等长的 Json 数组，逐元素拷贝并不做物理量换算。
 */
Json json_array_from_double_vector(const std::vector<double>& values) {
    Json::Array output;
    output.reserve(values.size());
    for (double value : values) {
        output.emplace_back(value);
    }
    return Json(std::move(output));
}

/**
 * @brief 将一维 int 数组转换为 JSON 数组。
 * @param[in] values 待转换的整数数组（典型场景为热点掩膜 hotspot_mask：0/1 布尔标志）。
 * @return 与输入等长的 Json 数组。
 */
Json json_array_from_int_vector(const std::vector<int>& values) {
    Json::Array output;
    output.reserve(values.size());
    for (int value : values) {
        output.emplace_back(value);
    }
    return Json(std::move(output));
}

/**
 * @brief 将字符串数组转换为 JSON 数组。
 * @param[in] values 待转换的字符串数组（如 QC 标志、scan_id 等）。
 * @return 与输入等长的 Json 数组。
 */
Json json_array_from_string_vector(const std::vector<std::string>& values) {
    Json::Array output;
    output.reserve(values.size());
    for (const auto& value : values) {
        output.emplace_back(value);
    }
    return Json(std::move(output));
}

/**
 * @brief 将二维 double 矩阵转换为 JSON 嵌套数组（每行一个 JSON 数组）。
 * @param[in] values 二维矩阵，典型如 ENU 坐标点集合（每行 [E, N, U] 三元素）。
 * @return 嵌套的 Json 数组：外层对应行，内层对应列。
 */
Json json_array_from_matrix(const std::vector<std::vector<double>>& values) {
    Json::Array output;
    output.reserve(values.size());
    for (const auto& row : values) {
        output.emplace_back(json_array_from_double_vector(row));
    }
    return Json(std::move(output));
}

/**
 * @brief 将站点元信息 SiteInfo 序列化为 JSON。
 * @param[in] value 站点信息结构体。
 * @return 含 name/经纬度/海拔/site_id 字段的 JSON 对象，供下游标注与可视化使用。
 */
Json to_json(const SiteInfo& value) {
    return Json::Object{
        {"name", value.name},
        {"latitude_deg", value.latitude_deg},
        {"longitude_deg", value.longitude_deg},
        {"altitude_m", value.altitude_m},
        {"site_id", value.site_id},
    };
}

/**
 * @brief 将 LiDAR 单条廓线 LidarProfile 序列化为 JSON。
 *
 * 这是 L0/L1 数据产品的核心字段集合：包含扫描几何（方位角/仰角）、原始回波 raw_counts、
 * 系统参数（激光能量、背景、overlap）、气象辅助量，以及真值（molecular/true 系列）。
 * 真值字段仅在仿真模式下有意义，便于后续算法验证与误差评估。
 *
 * @param[in] value 单条 LiDAR 廓线。
 * @return 包含全部 L0/L1 字段的 JSON 对象。
 */
Json to_json(const LidarProfile& value) {
    return Json::Object{
        {"site_id", value.site_id},
        {"timestamp", value.timestamp},
        {"scan_id", value.scan_id},
        {"scan_mode", value.scan_mode},
        {"source_kind", value.source_kind},
        {"azimuth_deg", value.azimuth_deg},
        {"elevation_deg", value.elevation_deg},
        {"ranges_m", json_array_from_double_vector(value.ranges_m)},
        {"raw_counts", json_array_from_double_vector(value.raw_counts)},
        {"laser_energy_mj", value.laser_energy_mj},
        {"background_counts", value.background_counts},
        {"overlap", json_array_from_double_vector(value.overlap)},
        {"relative_humidity", value.relative_humidity},
        {"temperature_c", value.temperature_c},
        {"wind_speed_ms", value.wind_speed_ms},
        {"wind_dir_deg", value.wind_dir_deg},
        {"molecular_backscatter", json_array_from_double_vector(value.molecular_backscatter)},
        {"molecular_extinction", json_array_from_double_vector(value.molecular_extinction)},
        {"true_backscatter", json_array_from_double_vector(value.true_backscatter)},
        {"true_extinction", json_array_from_double_vector(value.true_extinction)},
        {"true_pm25", json_array_from_double_vector(value.true_pm25)},
        {"true_pm10", json_array_from_double_vector(value.true_pm10)},
        {"true_hotspot_mask", json_array_from_int_vector(value.true_hotspot_mask)},
    };
}

/**
 * @brief 将地面观测 GroundMeasurement 序列化为 JSON。
 * @param[in] value 地面站观测值（PM2.5/PM10、温湿风）。
 * @return JSON 对象，字段名带 _ugm3 后缀提示质量浓度单位。
 */
Json to_json(const GroundMeasurement& value) {
    return Json::Object{
        {"site_id", value.site_id},
        {"timestamp", value.timestamp},
        {"pm25_ugm3", value.pm25_ugm3},
        {"pm10_ugm3", value.pm10_ugm3},
        {"relative_humidity", value.relative_humidity},
        {"temperature_c", value.temperature_c},
        {"wind_speed_ms", value.wind_speed_ms},
        {"wind_dir_deg", value.wind_dir_deg},
    };
}

/**
 * @brief 将反演后的 ProcessedProfile 序列化为 JSON（L2 数据产品）。
 *
 * 包含预处理产物（L1 信号、衰减后向散射、SNR）、Fernald 反演结果（消光、干消光）、
 * PM 浓度反演结果、ENU 三维点云坐标以及质控标志与延迟统计。
 *
 * @param[in] value 处理后廓线。
 * @return L2 数据产品的 JSON 对象。
 */
Json to_json(const ProcessedProfile& value) {
    return Json::Object{
        {"profile", to_json(value.profile)},
        {"l1_signal", json_array_from_double_vector(value.l1_signal)},
        {"attenuated_backscatter", json_array_from_double_vector(value.attenuated_backscatter)},
        {"snr", json_array_from_double_vector(value.snr)},
        {"extinction", json_array_from_double_vector(value.extinction)},
        {"dry_extinction", json_array_from_double_vector(value.dry_extinction)},
        {"pm25", json_array_from_double_vector(value.pm25)},
        {"pm10", json_array_from_double_vector(value.pm10)},
        {"enu_points_m", json_array_from_matrix(value.enu_points_m)},
        {"qc_flags", json_array_from_string_vector(value.qc_flags)},
        {"latency_ms", value.latency_ms},
    };
}

/**
 * @brief 将检测到的热点 Hotspot 序列化为 JSON。
 * @param[in] value 单个热点探测结果（质心 ENU 坐标、峰值/均值 PM2.5、面积、严重度）。
 * @return JSON 对象，用于热点报告输出。
 */
Json to_json(const Hotspot& value) {
    return Json::Object{
        {"timestamp", value.timestamp},
        {"scan_id", value.scan_id},
        {"hotspot_id", value.hotspot_id},
        {"centroid_enu_m", json_array_from_double_vector(value.centroid_enu_m)},
        {"peak_pm25_ugm3", value.peak_pm25_ugm3},
        {"mean_pm25_ugm3", value.mean_pm25_ugm3},
        {"estimated_area_m2", value.estimated_area_m2},
        {"cell_count", value.cell_count},
        {"severity", value.severity},
    };
}

// ====================================================================================
// ---- 内部数据结构 / Internal data structures (analysis-time containers) ----------------------------
// 这些结构体不参与 JSON 输入输出，仅为算法各阶段之间的中间结果承载体。
// ====================================================================================

/**
 * @brief 正向仿真生成的"真值"场（仅在合成数据时存在，反演时被作为参考答案）。
 *
 * 物理量含义：
 * - molecular_extinction / molecular_backscatter：分子（Rayleigh）消光与后向散射；
 * - true_backscatter / true_extinction：分子 + 气溶胶的总后向散射与总消光；
 * - true_pm25 / true_pm10：由干气溶胶消光经经验公式还原的"真值" PM 浓度；
 * - true_hotspot_mask：烟羽热点掩膜，1 表示该距离 bin 落在强污染羽流内。
 */
struct SimulatedFields {
    std::vector<double> molecular_extinction;
    std::vector<double> molecular_backscatter;
    std::vector<double> true_backscatter;
    std::vector<double> true_extinction;
    std::vector<double> true_pm25;
    std::vector<double> true_pm10;
    std::vector<int> true_hotspot_mask;
};

/**
 * @brief PM 校准单条样本：包含特征向量与对应地面真值。
 *
 * features 字段顺序为 [1.0, 近地面干消光均值, 相对湿度, 热点代理量]，对应线性回归的截距与系数。
 */
struct FeatureSample {
    std::string timestamp;
    std::vector<double> features;
    double surface_dry_ext = 0.0;
    double surface_attenuated = 0.0;
    double hotspot_proxy = 0.0;
    std::string site_id;
    double pm25_true = 0.0;
    double pm10_true = 0.0;
    double relative_humidity = 0.0;
    double wind_speed_ms = 0.0;
};

/**
 * @brief 训练/验证/测试三段划分及对应时间戳索引，用于 PM 模型校准与评估。
 */
struct CalibrationSplit {
    std::vector<FeatureSample> train;
    std::vector<FeatureSample> val;
    std::vector<FeatureSample> test;
    std::vector<std::string> train_timestamps;
    std::vector<std::string> val_timestamps;
    std::vector<std::string> test_timestamps;
};

/**
 * @brief PM2.5 / PM10 线性回归模型系数向量。
 */
struct CalibrationModels {
    std::vector<double> pm25;
    std::vector<double> pm10;
};

/**
 * @brief 按站点统计的预测残差均值（站点偏置校正项），用以补偿不同站点的系统差。
 */
struct StationOffset {
    double pm25_offset = 0.0;
    double pm10_offset = 0.0;
    int sample_count = 0;
};

/**
 * @brief 热点检测算法的中间结果：含热点列表与逐 bin 预测/真值掩膜（一维展平的 PPI 网格）。
 */
struct DetectionResult {
    std::vector<Hotspot> hotspots;
    std::vector<int> predicted_mask;
    std::vector<int> truth_mask;
};

/**
 * @brief 单条样本的预测残差记录，用于残差分析表输出与漂移监控。
 */
struct ResidualRow {
    std::string timestamp;
    std::string site_id;
    double pm25_residual = 0.0;
    double pm10_residual = 0.0;
};

/**
 * @brief 测试集上的真值 / 预测向量对，以及逐条残差记录，聚合后用于指标计算。
 */
struct PredictionSummary {
    std::vector<double> pm25_truth;
    std::vector<double> pm25_pred;
    std::vector<double> pm10_truth;
    std::vector<double> pm10_pred;
    std::vector<ResidualRow> residual_rows;
};

/**
 * @brief 单站点完整运行结果：覆盖从原始廓线、L2 处理结果、特征表、模型偏置到漂移监控的全部产物。
 */
struct DatasetRunResult {
    SiteInfo site;
    std::vector<LidarProfile> profiles;
    std::vector<GroundMeasurement> ground_measurements;
    std::vector<ProcessedProfile> processed_profiles;
    std::vector<FeatureSample> feature_table;
    CalibrationSplit split;
    std::map<std::string, StationOffset> station_offsets;
    Json drift_monitoring;
    std::map<std::string, std::vector<Hotspot>> hotspots_by_timestamp;
    Json metrics;
};

