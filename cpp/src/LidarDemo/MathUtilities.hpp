// Internal include fragment: math, time, and scalar utilities.
// ---- 工具函数：数学/统计 ----

/**
 * @brief 将数值限制在 [lower, upper] 闭区间内。
 * @param value  待约束的输入值。
 * @param lower  允许的最小值。
 * @param upper  允许的最大值。
 * @return 若 value < lower 返回 lower；若 value > upper 返回 upper；否则返回 value。
 *
 * 通过 std::max/min 两次比较实现，避免引入额外分支；常用于约束物理量
 * (如湿度百分比、角度范围) 以免越界导致数值发散。
 */
double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

/**
 * @brief 计算 double 序列的算术平均值。
 * @param values 输入序列；允许为空。
 * @return 空序列返回 0.0，否则返回 sum/count。
 *
 * 用 std::accumulate 累加；空序列约定返回 0 而非抛错，便于对未采集到的扇区做兜底。
 */
double mean(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

/**
 * @brief 计算 int 序列的算术平均值（结果按 double 返回）。
 * @param values 输入整数序列；允许为空。
 * @return 空序列返回 0.0；否则返回累加值除以元素个数。
 *
 * 与 mean(double) 平行的整型版本，避免在调用点反复做类型转换。
 */
double mean_int(const std::vector<int>& values) {
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

/**
 * @brief 计算 double 序列的中位数。
 * @param values 按值传递的副本，函数内部会将其排序以破坏原序。
 * @return 空序列返回 0.0；奇数个返回正中元素；偶数个返回中间两元素的均值。
 *
 * 复杂度主要由 std::sort 决定 (O(n log n))。副本传参是刻意的，避免修改外部顺序。
 */
double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 0) {
        return (values[middle - 1] + values[middle]) / 2.0; ///< 偶数个时取中间两数的算术平均
    }
    return values[middle];
}

/**
 * @brief 将任意文本转为 URL/文件名安全的 slug 形式。
 * @param text 原始字符串。
 * @return 空格被替换为 '-'，其余字母统一转小写后的字符串。
 *
 * 主要用于将中英文/带空白的标题转化为可放进文件名或路径的稳定标识。
 */
std::string slugify(std::string text) {
    for (char& current : text) {
        if (current == ' ') {
            current = '-';
        } else {
            current = static_cast<char>(std::tolower(static_cast<unsigned char>(current)));
        }
    }
    return text;
}

/**
 * @brief 计算归一化高斯函数取值（峰值归一到 1）。
 * @param distance 自变量位置。
 * @param center   高斯中心位置。
 * @param sigma    标准差，决定钟形宽度。
 * @return exp(-(d-c)^2 / (2*sigma^2))；sigma 过小时以 1e-6 防除零。
 *
 * 用于模拟气溶胶/颗粒物的径向剖面或对探测器响应做加权平滑。
 */
double gaussian(double distance, double center, double sigma) {
    // 分母用 max(2*sigma^2, 1e-6) 防止 sigma=0 时除零导致 inf/NaN
    return std::exp(-((distance - center) * (distance - center)) / std::max(2.0 * sigma * sigma, 1e-6));
}

/**
 * @brief 计算两个方位角之间的最小夹角（度）。
 * @param left_deg  左侧方位（度）。
 * @param right_deg 右侧方位（度）。
 * @return 取值范围 [0, 180]，即跨越 360/0 边界后的最短角差。
 *
 * 思路：先对差值取 mod 360，再与 (360 - 差值) 取较小者，从而正确处理
 * 例如 350° 与 10° 之间实际只差 20° 的情形。
 */
double azimuth_delta(double left_deg, double right_deg) {
    double raw = std::fmod(std::abs(left_deg - right_deg), 360.0); ///< 折叠到 [0,360)
    return std::min(raw, 360.0 - raw);                              ///< 取与正向环绕的较小者
}

/**
 * @brief 线程安全地将 time_t 转换为本地时间 struct tm。
 * @param value 自 Epoch 起的秒数。
 * @return 本地时区下的时间分块 struct tm。
 *
 * Windows 平台使用 localtime_s，POSIX 平台使用 localtime_r，二者都是可重入版本，
 * 避免传统 localtime 返回静态缓冲区带来的数据竞争。
 */
std::tm safe_localtime(std::time_t value) {
    std::tm result{};
#ifdef _WIN32
    localtime_s(&result, &value); ///< Windows 安全版本：输出写入调用方缓冲
#else
    localtime_r(&value, &result); ///< POSIX 可重入版本
#endif
    return result;
}

/**
 * @brief 将 time_t 格式化为 ISO8601 分钟级时间字符串。
 * @param value 自 Epoch 起的秒数。
 * @return 形如 "YYYY-MM-DDTHH:MM" 的本地时间字符串。
 *
 * 精度截断到分钟，与 Open-Meteo/Cloudnet 公共数据的逐小时/逐分钟采样对齐。
 */
std::string format_timestamp(std::time_t value) {
    std::tm stamp = safe_localtime(value);
    std::ostringstream output;
    output << std::put_time(&stamp, "%Y-%m-%dT%H:%M");
    return output.str();
}

/**
 * @brief 解析 ISO8601 分钟级时间字符串为 time_t。
 * @param text 形如 "YYYY-MM-DDTHH:MM" 的字符串。
 * @return 对应的本地时区 time_t。
 * @throws std::runtime_error 当字符串无法按预期格式解析时抛出。
 *
 * 解析前将 tm_isdst 设为 -1，让 mktime 自动判断夏令时，避免手动猜测 DST 状态引入的误差。
 */
std::time_t parse_timestamp(const std::string& text) {
    std::tm stamp{};
    std::istringstream input(text);
    input >> std::get_time(&stamp, "%Y-%m-%dT%H:%M");
    if (input.fail()) {
        throw std::runtime_error("Failed to parse timestamp: " + text);
    }
    stamp.tm_isdst = -1; ///< 让 mktime 自行推断夏令时标志
    return std::mktime(&stamp);
}

