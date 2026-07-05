// Internal include fragment: Open-Meteo ground record utilities.
#if defined(LIDAR_DEMO_HAS_NETCDF)
// ---- 地面观测记录：结构体与字符串/数值工具 ----

/**
 * @brief 单条地面气象 / 颗粒物观测记录。
 *
 * 字段对应 Open-Meteo air-quality + weather 两个接口逐小时合并后的输出，
 * 时间戳为本地 ISO8601 分钟精度。默认值代表一个温带晴日的中性环境，
 * 用于在数据缺失时仍能驱动仿真与标定流程。
 */
struct GroundRecord {
    std::string timestamp;
    double pm25 = 0.0;                  ///< PM2.5 质量浓度 (µg/m³)
    double pm10 = 0.0;                  ///< PM10 质量浓度 (µg/m³)
    double temperature_c = 18.0;        ///< 2 m 气温 (°C)
    double relative_humidity = 60.0;    ///< 2 m 相对湿度 (%)
    double wind_speed_ms = 2.0;         ///< 10 m 风速 (m/s)
    double wind_dir_deg = 0.0;          ///< 10 m 风向 (°，气象学方位：北=0 顺时针)
};

/**
 * @brief 移除字符串中的所有连字符 '-'。
 * @param text 原始字符串。
 * @return 不含 '-' 的字符串。
 *
 * 主要用于清洗 ISO 时间戳中的 '-' 以便生成文件名片段或紧凑 key。
 */
std::string strip_hyphens(std::string text) {
    text.erase(std::remove(text.begin(), text.end(), '-'), text.end());
    return text;
}

/**
 * @brief 将文本转换为以下划线分隔的小写标识符。
 * @param text 原始字符串。
 * @return 空格→下划线、字母统一小写后的字符串。
 *
 * 与 slugify 配套：先把空格变 '-'，再把 '-' 变 '_'，得到适合作为 JSON key
 * 或 C++ 标识符风格的字段名。
 */
std::string underscore_slug(std::string text) {
    std::string output = slugify(std::move(text));
    std::replace(output.begin(), output.end(), '-', '_');
    return output;
}

/**
 * @brief 把浮点数格式化为字符串，并去掉多余的尾随 0 与孤立 '.'。
 * @param value     待格式化的数值。
 * @param precision 小数位数，默认 6。
 * @return 紧凑的字符串表示；全空时返回 "0"。
 *
 * 实现采用 fixed + setprecision 先生成定小数位文本，再裁剪末尾 0 和小数点，
 * 得到既可读又无歧义的数值字符串 (例如 1.500000 → "1.5")。
 */
std::string format_number(double value, int precision = 6) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    std::string text = stream.str();
    while (!text.empty() && text.back() == '0') {   ///< 去除末尾连续 0
        text.pop_back();
    }
    if (!text.empty() && text.back() == '.') {      ///< 去除孤立的小数点
        text.pop_back();
    }
    if (text.empty()) {
        return "0";                                 ///< 如 "-0" 之类退化情况兜底
    }
    return text;
}

/**
 * @brief 对字符串进行百分号编码 (RFC 3986 percent-encoding)。
 * @param text 原始字符串 (UTF-8 字节流)。
 * @return 编码后的字符串；字母数字及 -_.~, 保持原样，其余按 %HH 大写编码。
 *
 * 用于构造 HTTP 查询串，与 Open-Meteo 等公共 API 的兼容性要求一致。
 */
std::string url_encode(const std::string& text) {
    std::ostringstream encoded;
    encoded << std::uppercase << std::hex;          ///< 后续 %HH 使用大写十六进制
    for (unsigned char current : text) {
        if (std::isalnum(current) || current == '-' || current == '_' || current == '.' || current == '~' || current == ',') {
            encoded << static_cast<char>(current);  ///< 保留字符直接输出
        } else {
            encoded << '%' << std::setw(2) << std::setfill('0') << static_cast<int>(current); ///< 非保留字符转为 %HH
        }
    }
    return encoded.str();
}

/**
 * @brief 将键值对列表拼接为 application/x-www-form-urlencoded 查询串。
 * @param params (key, value) 形式的参数列表。
 * @return 形如 "k1=v1&k2=v2" 的查询字符串，键值均经 url_encode 处理。
 */
std::string build_query_string(const std::vector<std::pair<std::string, std::string>>& params) {
    std::ostringstream query;
    for (std::size_t index = 0; index < params.size(); ++index) {
        if (index > 0) {
            query << '&';
        }
        query << url_encode(params[index].first) << '=' << url_encode(params[index].second);
    }
    return query.str();
}

/**
 * @brief 将文本写入文件（覆盖写入，二进制模式）。
 * @param path 目标路径；父目录会被自动创建。
 * @param text 待写入内容。
 * @throws std::runtime_error 文件无法打开时抛出。
 *
 * 使用 binary 模式以避免 Windows 上对 '\n' 自动追加 '\r'，保证跨平台一致性。
 */
void write_text_file(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream handle(path, std::ios::binary);
    if (!handle) {
        throw std::runtime_error("Failed to write text file: " + path.string());
    }
    handle << text;
}

/**
 * @brief 按 CSV 规则对字段进行转义。
 * @param text 原始字段值。
 * @return 若字段不含特殊字符 (逗号/引号/换行)，原样返回；否则用双引号包起来，
 *         字段内的双引号变两个双引号。
 *
 * 遵循 RFC 4180 的最小转义规则，避免破坏下游 CSV 解析器。
 */
std::string csv_escape(const std::string& text) {
    if (text.find_first_of(",\"\n\r") == std::string::npos) {
        return text;                                ///< 不含特殊字符：无需引号
    }
    std::string output = "\"";
    for (char current : text) {
        if (current == '\"') {
            output += "\"\"";                       ///< 引号转义为两个引号
        } else {
            output.push_back(current);
        }
    }
    output.push_back('\"');
    return output;
}

/**
 * @brief 将地面观测记录序列化为 JSON 数组。
 * @param records GroundRecord 列表。
 * @return Json 数组；每个元素为一行 record 的对象表示。
 *
 * 字段顺序与 write_ground_records_csv 保持一致，方便下游交叉校验。
 */
Json ground_records_to_json(const std::vector<GroundRecord>& records) {
    Json::Array output;
    output.reserve(records.size());
    for (const auto& record : records) {
        output.push_back(Json::Object{
            {"timestamp", record.timestamp},
            {"pm25", record.pm25},
            {"pm10", record.pm10},
            {"temperature_c", record.temperature_c},
            {"relative_humidity", record.relative_humidity},
            {"wind_speed_ms", record.wind_speed_ms},
            {"wind_dir_deg", record.wind_dir_deg},
        });
    }
    return Json(std::move(output));
}

/**
 * @brief 将地面观测记录写入 CSV 文件（带表头）。
 * @param path    目标文件；父目录自动创建。
 * @param records 待写出记录列表。
 * @throws std::runtime_error 文件打开失败时抛出。
 *
 * 数值字段统一保留 3 位小数，时间戳走 csv_escape 防止其中含逗号时破坏列结构。
 */
void write_ground_records_csv(const std::filesystem::path& path, const std::vector<GroundRecord>& records) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream handle(path, std::ios::binary);
    if (!handle) {
        throw std::runtime_error("Failed to write CSV file: " + path.string());
    }
    handle << "timestamp,pm25,pm10,temperature_c,relative_humidity,wind_speed_ms,wind_dir_deg\n";
    for (const auto& record : records) {
        handle << csv_escape(record.timestamp) << ','
               << format_number(record.pm25, 3) << ','
               << format_number(record.pm10, 3) << ','
               << format_number(record.temperature_c, 3) << ','
               << format_number(record.relative_humidity, 3) << ','
               << format_number(record.wind_speed_ms, 3) << ','
               << format_number(record.wind_dir_deg, 3) << '\n';
    }
}

/**
 * @brief 从 JSON 数组中按索引取数值，越界或 null 时返回 fallback。
 * @param array    JSON 数组。
 * @param index    目标下标。
 * @param fallback 缺失时使用的默认值。
 * @return 数值元素；若 array 非 array / 越界 / 元素为 null，返回 fallback。
 *
 * Open-Meteo 接口在数据缺失时常用 null 占位，本函数将其平滑替换为可计算默认值。
 */
double json_array_number_or(const Json& array, std::size_t index, double fallback) {
    if (!array.is_array() || index >= array.array_items().size() || array.array_items()[index].is_null()) {
        return fallback;
    }
    return array.array_items()[index].number_value();
}

/**
 * @brief 合并 Open-Meteo 的空气质量与天气响应为 GroundRecord 列表。
 * @param air_quality 空气质量接口的原始 JSON，需含 hourly.pm2_5/pm10/time。
 * @param weather     天气接口的原始 JSON，需含 hourly.temperature_2m/relative_humidity_2m/
 *                    windspeed_10m/winddirection_10m。
 * @return 逐小时对齐的 GroundRecord 列表。
 * @throws std::runtime_error 任一响应缺少 hourly 字段时抛出。
 *
 * 实现细节：以所有相关数组长度的最小值为最终长度，保证每条记录字段齐全；
 * 风速由 km/h 转换为 m/s (除以 3.6)，与雷达观测坐标系一致。
 */
std::vector<GroundRecord> build_open_meteo_ground_records(const Json& air_quality, const Json& weather) {
    if (!air_quality.contains("hourly") || !weather.contains("hourly")) {
        throw std::runtime_error("Open-Meteo response missing hourly payload");
    }
    const Json& air_hourly = air_quality.at("hourly");
    const Json& weather_hourly = weather.at("hourly");
    const auto& timestamps = air_hourly.at("time").array_items();
    std::size_t count = timestamps.size();
    count = std::min(count, air_hourly.at("pm2_5").array_items().size());
    count = std::min(count, air_hourly.at("pm10").array_items().size());
    count = std::min(count, weather_hourly.at("temperature_2m").array_items().size());
    count = std::min(count, weather_hourly.at("relative_humidity_2m").array_items().size());
    count = std::min(count, weather_hourly.at("windspeed_10m").array_items().size());
    count = std::min(count, weather_hourly.at("winddirection_10m").array_items().size());

    std::vector<GroundRecord> records;
    records.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        records.push_back(GroundRecord{
            timestamps[index].string_value(),
            json_array_number_or(air_hourly.at("pm2_5"), index, 0.0),
            json_array_number_or(air_hourly.at("pm10"), index, 0.0),
            json_array_number_or(weather_hourly.at("temperature_2m"), index, 18.0),
            json_array_number_or(weather_hourly.at("relative_humidity_2m"), index, 60.0),
            json_array_number_or(weather_hourly.at("windspeed_10m"), index, 0.0) / 3.6, ///< km/h → m/s
            json_array_number_or(weather_hourly.at("winddirection_10m"), index, 0.0),
        });
    }
    return records;
}

#endif
