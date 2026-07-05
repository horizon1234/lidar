// Public API implementation fragment: public data fetch functions.
// ---- 公共数据下载：Open-Meteo 地面 PM/气象 与 Cloudnet 激光雷达样本 ----

/**
 * @brief 从 Open-Meteo 的两套公共 API（air-quality + archive）抓取并合并地面 PM 与气象数据。
 *
 * 流程：
 *  1. 构造两个查询 URL：air-quality 提供 PM10/PM2.5，archive 提供温度/湿度/风速/风向；
 *  2. 通过 http_get_text 拉取两份 JSON（参数 verify_ssl=true）；
 *  3. build_open_meteo_ground_records 按时间戳对齐两份数据，生成统一 GroundRecord 列表；
 *  4. 把对齐结果写入 JSON + CSV，并把下载元信息（坐标、日期、字段、记录数、文件路径）
 *     写入一份 manifest 文件，最终返回该 manifest。
 *
 * @param latitude_deg  纬度。
 * @param longitude_deg 经度。
 * @param start_date    起始日期（YYYY-MM-DD）。
 * @param end_date      结束日期（YYYY-MM-DD，含）。
 * @param timezone      查询时区（如 "Asia/Shanghai" 或 "UTC"）。
 * @param output_dir    输出目录（不存在会自动创建）。
 * @param prefix        输出文件前缀（与 _ground_pm_meteo.json 等拼接）。
 * @return Json         下载与对齐的元信息 manifest。
 */
Json fetch_public_ground_data(
    double latitude_deg,
    double longitude_deg,
    const std::string& start_date,
    const std::string& end_date,
    const std::string& timezone,
    const std::filesystem::path& output_dir,
    const std::string& prefix
) {
    // 空气质量 API：仅提供 hourly 的 PM10/PM2.5
    std::string air_url = "https://air-quality-api.open-meteo.com/v1/air-quality?" + build_query_string({
        {"latitude", format_number(latitude_deg)},
        {"longitude", format_number(longitude_deg)},
        {"hourly", "pm10,pm2_5"},
        {"start_date", start_date},
        {"end_date", end_date},
        {"timezone", timezone},
    });
    // 历史天气归档 API：提供温度/湿度/风速/风向
    std::string weather_url = "https://archive-api.open-meteo.com/v1/archive?" + build_query_string({
        {"latitude", format_number(latitude_deg)},
        {"longitude", format_number(longitude_deg)},
        {"start_date", start_date},
        {"end_date", end_date},
        {"hourly", "temperature_2m,relative_humidity_2m,windspeed_10m,winddirection_10m"},
        {"timezone", timezone},
    });

    Json air_quality = parse_json(http_get_text(air_url, true));
    Json weather = parse_json(http_get_text(weather_url, true));
    std::vector<GroundRecord> records = build_open_meteo_ground_records(air_quality, weather);

    // 输出目录解析为绝对路径，便于在 manifest 中记录稳定路径
    std::filesystem::path resolved_output_dir = std::filesystem::absolute(output_dir);
    std::filesystem::path json_path = resolved_output_dir / (prefix + "_ground_pm_meteo.json");
    std::filesystem::path csv_path = resolved_output_dir / (prefix + "_ground_pm_meteo.csv");
    std::filesystem::path manifest_path = resolved_output_dir / (prefix + "_fetch_manifest.json");
    write_json_file(json_path, ground_records_to_json(records));
    write_ground_records_csv(csv_path, records);

    Json manifest = Json::Object{
        {"source", "Open-Meteo"},
        {"latitude", latitude_deg},
        {"longitude", longitude_deg},
        {"start_date", start_date},
        {"end_date", end_date},
        {"record_count", static_cast<int>(records.size())},
        {"fields", json_array_from_string_vector({"timestamp", "pm25", "pm10", "temperature_c", "relative_humidity", "wind_speed_ms", "wind_dir_deg"})},
        {"ground_json", json_path.string()},
        {"ground_csv", csv_path.string()},
    };
    write_json_file(manifest_path, manifest);
    return manifest;
}

/**
 * @brief 从 Cloudnet 下载公共激光雷达样本，并同步拉取对应日期的 Open-Meteo 地面/气象数据。
 *
 * 流程：
 *  1. 必填字段校验（download_url / local_file / date）；
 *  2. 把配置中的相对/绝对 root 与 output_root 解析为最终输出根目录；
 *  3. 通过 http_get_binary 下载 Cloudnet 二进制样本，原样写入磁盘；
 *  4. 在该日期下同时拉取 Open-Meteo air-quality 与 weather（UTC）；
 *  5. 对齐两份 Open-Meteo 数据生成 GroundRecord 序列，写出 ground_pm_meteo.json/.csv 与对应 manifest；
 *  6. 最后写一份汇总 manifest，包含 Cloudnet 文件路径、各 Open-Meteo 文件路径与记录数。
 *
 * @param config     流水线配置（使用其中 source.cloudnet 与 site 字段）。
 * @param output_root 输出根目录；为空时使用当前工作目录。
 * @return Json      汇总 manifest，含全部产物路径。
 */
Json fetch_cloudnet_public_sample(const PipelineConfig& config, const std::filesystem::path& output_root) {
    // 必填字段：缺失任何一项都直接抛错，避免后续下载/写盘出问题
    if (config.source.cloudnet.download_url.empty()) {
        throw std::runtime_error("Cloudnet public sample fetch requires source.cloudnet.download_url");
    }
    if (config.source.cloudnet.local_file.empty()) {
        throw std::runtime_error("Cloudnet public sample fetch requires source.cloudnet.local_file");
    }
    if (config.source.cloudnet.date.empty()) {
        throw std::runtime_error("Cloudnet public sample fetch requires source.cloudnet.date");
    }

    // 输出根目录解析：output_root 为空则用当前目录；config.source.root 可以是绝对或相对路径
    std::filesystem::path base_root = std::filesystem::absolute(output_root.empty() ? std::filesystem::path(".") : output_root);
    std::filesystem::path configured_root = config.source.root.empty() ? std::filesystem::path(".") : std::filesystem::path(config.source.root);
    std::filesystem::path source_root = configured_root.is_absolute() ? configured_root : std::filesystem::absolute(base_root / configured_root);
    std::filesystem::path local_file = source_root / config.source.cloudnet.local_file;
    std::filesystem::create_directories(local_file.parent_path());

    // 二进制下载 Cloudnet 样本，原样落盘
    std::vector<std::uint8_t> cloudnet_bytes = http_get_binary(config.source.cloudnet.download_url, config.source.cloudnet.verify_ssl);
    {
        std::ofstream handle(local_file, std::ios::binary);
        if (!handle) {
            throw std::runtime_error("Failed to write Cloudnet sample file: " + local_file.string());
        }
        handle.write(reinterpret_cast<const char*>(cloudnet_bytes.data()), static_cast<std::streamsize>(cloudnet_bytes.size()));
    }

    // 对齐 Open-Meteo：air-quality + weather，强制 UTC 与 Cloudnet 时间匹配
    std::filesystem::path output_dir = source_root / "data" / "public" / "cloudnet";
    std::filesystem::create_directories(output_dir);
    std::string day_token = strip_hyphens(config.source.cloudnet.date);  ///< 形如 20240506 的紧凑日期
    std::string site_slug = underscore_slug(!config.source.cloudnet.site_name.empty() ? config.source.cloudnet.site_name : config.site.name);

    std::string air_url = "https://air-quality-api.open-meteo.com/v1/air-quality?" + build_query_string({
        {"latitude", format_number(config.site.latitude_deg)},
        {"longitude", format_number(config.site.longitude_deg)},
        {"start_date", config.source.cloudnet.date},
        {"end_date", config.source.cloudnet.date},
        {"hourly", "pm10,pm2_5"},
        {"timezone", "UTC"},
    });
    std::string weather_url = "https://archive-api.open-meteo.com/v1/archive?" + build_query_string({
        {"latitude", format_number(config.site.latitude_deg)},
        {"longitude", format_number(config.site.longitude_deg)},
        {"start_date", config.source.cloudnet.date},
        {"end_date", config.source.cloudnet.date},
        {"hourly", "temperature_2m,relative_humidity_2m,windspeed_10m,winddirection_10m"},
        {"timezone", "UTC"},
    });

    Json air_quality = parse_json(http_get_text(air_url, true));
    Json weather = parse_json(http_get_text(weather_url, true));

    // 落盘原始 Open-Meteo 响应（JSON），文件名含 day_token + site_slug 以便区分
    std::filesystem::path air_path = output_dir / (day_token + "_" + site_slug + "_open_meteo_air_quality.json");
    std::filesystem::path weather_path = output_dir / (day_token + "_" + site_slug + "_open_meteo_weather.json");
    write_json_file(air_path, air_quality);
    write_json_file(weather_path, weather);

    // 对齐 air-quality + weather → 统一 GroundRecord 列表，写入 JSON+CSV 与对齐 manifest
    std::vector<GroundRecord> records = build_open_meteo_ground_records(air_quality, weather);
    std::filesystem::path aligned_json_path = output_dir / ("open_meteo_" + day_token + "_ground_pm_meteo.json");
    std::filesystem::path aligned_csv_path = output_dir / ("open_meteo_" + day_token + "_ground_pm_meteo.csv");
    std::filesystem::path aligned_manifest_path = output_dir / ("open_meteo_" + day_token + "_fetch_manifest.json");
    write_json_file(aligned_json_path, ground_records_to_json(records));
    write_ground_records_csv(aligned_csv_path, records);

    Json aligned_manifest = Json::Object{
        {"source", "Open-Meteo"},
        {"latitude", config.site.latitude_deg},
        {"longitude", config.site.longitude_deg},
        {"start_date", config.source.cloudnet.date},
        {"end_date", config.source.cloudnet.date},
        {"record_count", static_cast<int>(records.size())},
        {"fields", json_array_from_string_vector({"timestamp", "pm25", "pm10", "temperature_c", "relative_humidity", "wind_speed_ms", "wind_dir_deg"})},
        {"ground_json", aligned_json_path.string()},
        {"ground_csv", aligned_csv_path.string()},
    };
    write_json_file(aligned_manifest_path, aligned_manifest);

    // 汇总 manifest：把 Cloudnet 与 Open-Meteo 各产物路径集中起来
    std::filesystem::path manifest_path = output_dir / (day_token + "_" + site_slug + "_manifest.json");
    Json manifest = Json::Object{
        {"cloudnet_lidar", local_file.string()},
        {"open_meteo_air_quality", air_path.string()},
        {"open_meteo_weather", weather_path.string()},
        {"aligned_ground_json", aligned_json_path.string()},
        {"aligned_ground_csv", aligned_csv_path.string()},
        {"aligned_ground_manifest", aligned_manifest_path.string()},
        {"record_count", static_cast<int>(records.size())},
    };
    write_json_file(manifest_path, manifest);
    return manifest;
}

