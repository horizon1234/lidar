// YLJ5 正演内部使用的最小数学和时间工具。

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

double mean(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0)
        / static_cast<double>(values.size());
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(
        values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
    double result = values[middle];
    if (values.size() % 2 == 0) {
        result = (*std::max_element(
            values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle)) + result) * 0.5;
    }
    return result;
}

std::string slugify(std::string text) {
    for (char& value : text) {
        const unsigned char byte = static_cast<unsigned char>(value);
        value = std::isalnum(byte)
            ? static_cast<char>(std::tolower(byte))
            : '-';
    }
    text.erase(std::unique(text.begin(), text.end(), [](char left, char right) {
        return left == '-' && right == '-';
    }), text.end());
    while (!text.empty() && text.front() == '-') text.erase(text.begin());
    while (!text.empty() && text.back() == '-') text.pop_back();
    return text.empty() ? "ylj5-site" : text;
}

double gaussian(double distance, double center, double sigma) {
    return std::exp(
        -((distance - center) * (distance - center))
        / std::max(2.0 * sigma * sigma, 1e-6));
}

std::string format_timestamp(std::time_t value) {
    std::tm stamp{};
    localtime_r(&value, &stamp);
    std::ostringstream output;
    output << std::put_time(&stamp, "%Y-%m-%dT%H:%M");
    return output.str();
}

/**
 * @brief 正演生成的一条射线真值场，仅在设备仿真内部使用。
 *
 * 这些字段是构造四物理通道和验证反演结果的参考量，不代表真实设备会直接
 * 输出相同数据。各向量均与射线距离轴逐 bin 对齐。
 */
struct SimulatedFields {
    std::vector<double> molecular_extinction; ///< 分子消光系数真值。
    std::vector<double> molecular_backscatter; ///< 分子后向散射系数真值。
    std::vector<double> true_backscatter; ///< 分子与气溶胶总后向散射系数真值。
    std::vector<double> true_extinction; ///< 分子与气溶胶总消光系数真值。
    std::vector<double> true_pm25; ///< PM2.5 质量浓度仿真真值。
    std::vector<double> true_pm10; ///< PM10 质量浓度仿真真值。
    std::vector<int> true_hotspot_mask; ///< 污染羽流热点掩膜，1 表示热点距离门。
};
