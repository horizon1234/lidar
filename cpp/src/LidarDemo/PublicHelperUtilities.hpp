// Internal include fragment: failure suite, chart, and JSON helpers.
// ---- 失败案例套件实现 + SVG/JSON 工具 ----

/**
 * @brief 对三种典型故障逐个跑完整流水线，得到一份"故障下的关键指标"摘要。
 *
 * 复制原始 profiles/ground_measurements 后调用 apply_failure_case 注入故障、再
 * 跑一次 run_pipeline_on_dataset，记录该 case 下的 PM2.5/PM10 RMSE、热点 F1
 * 与吞吐量。无需传 lidar_ratio_override 或 disable_humidity 标志。
 *
 * @param config             流水线配置。
 * @param site               站点信息。
 * @param profiles           原始廓线（不会被修改，内部拷贝）。
 * @param ground_measurements 原始地面观测（同上）。
 * @return Json::Array  三个 case 的指标摘要数组。
 */
Json::Array run_failure_case_suite(const PipelineConfig& config, const SiteInfo& site, const std::vector<LidarProfile>& profiles, const std::vector<GroundMeasurement>& ground_measurements) {
    Json::Array output;
    for (const std::string& case_name : {std::string("high-background-light"), std::string("overlap-miscalibration"), std::string("humidity-surge")}) {
        // 逐 case 拷贝原始数据，保证各 case 之间互不影响
        auto case_profiles = profiles;
        auto case_ground = ground_measurements;
        apply_failure_case(case_profiles, case_ground, case_name);
        DatasetRunResult case_output = run_pipeline_on_dataset(config, site, case_profiles, case_ground);
        output.push_back(Json::Object{
            {"name", case_name},
            {"pm25_rmse", case_output.metrics.at("pm25").at("rmse")},
            {"pm10_rmse", case_output.metrics.at("pm10").at("rmse")},
            {"hotspot_f1", case_output.metrics.at("hotspot").at("f1")},
            {"throughput_profiles_per_s", case_output.metrics.at("runtime").at("throughput_profiles_per_s")},
        });
    }
    return output;
}

/**
 * @brief 生成一段简易内嵌 SVG 折线图，用于 HTML 仪表盘中的 QC 趋势展示。
 *
 * 算法：把 values 等比归一化到 [10, height-10] 的垂直区间（顶部、底部各留 10px 边距），
 * 横轴按样本数等步长分布，最后输出一条带 polyline 的 SVG 字符串。空数据返回 10×10 空 SVG。
 *
 * @param values 数据点序列。
 * @param width  SVG 宽度（像素）。
 * @param height SVG 高度（像素）。
 * @param stroke polyline 的 CSS 颜色（如 "#0f766e"）。
 * @return std::string 完整的 <svg>...</svg> 字符串。
 */
std::string build_line_chart_svg(const std::vector<double>& values, int width, int height, const std::string& stroke) {
    if (values.empty()) {
        return "<svg viewBox=\"0 0 10 10\"></svg>";
    }
    double minimum = *std::min_element(values.begin(), values.end());
    double maximum = *std::max_element(values.begin(), values.end());
    double spread = maximum - minimum;
    if (spread == 0.0) {
        spread = 1.0;  ///< 数据恒定时避免 0 除
    }
    double step_x = static_cast<double>(width) / std::max<int>(static_cast<int>(values.size()) - 1, 1);
    std::ostringstream points;
    for (std::size_t index = 0; index < values.size(); ++index) {
        double x_coord = static_cast<double>(index) * step_x;
        // 反向映射：值越大 y 越小（屏幕坐标系原点在左上）
        double y_coord = static_cast<double>(height) - ((values[index] - minimum) / spread) * (height - 20) - 10;
        if (index > 0) {
            points << ' ';
        }
        points << std::fixed << std::setprecision(1) << x_coord << ',' << y_coord;
    }
    std::ostringstream svg;
    svg << "<svg viewBox=\"0 0 " << width << ' ' << height << "\" class=\"chart\"><polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"3\" points=\"" << points.str() << "\" /></svg>";
    return svg.str();
}

/**
 * @brief 把 Json 数组转换为 std::vector<double>。
 * @param value Json 数组，元素需为数字；非法元素会被 .number_value() 返回 0。
 * @return std::vector<double> 浮点序列。
 */
std::vector<double> json_to_double_vector(const Json& value) {
    std::vector<double> output;
    for (const auto& item : value.array_items()) {
        output.push_back(item.number_value());
    }
    return output;
}

