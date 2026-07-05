// Internal include fragment: HTTP download utilities.
#if defined(LIDAR_DEMO_HAS_NETCDF)
// ---- HTTP 下载层：跨平台 (Windows WinHTTP / POSIX libcurl) ----

#if defined(_WIN32)

/**
 * @brief Windows WinHTTP 句柄的 RAII 包装。
 *
 * 构造时接管一个 HINTERNET，析构时若仍持有则调用 WinHttpCloseHandle；
 * 同时禁用拷贝、实现移动语义，方便作为函数局部对象管理。
 */
struct WinHttpHandle {
    HINTERNET handle = nullptr;

    WinHttpHandle() = default;
    /** @brief 接管给定句柄。 */
    explicit WinHttpHandle(HINTERNET input) : handle(input) {}
    /** @brief 析构：若仍持有句柄则关闭之。 */
    ~WinHttpHandle() {
        if (handle != nullptr) {
            WinHttpCloseHandle(handle);
        }
    }

    WinHttpHandle(const WinHttpHandle&) = delete;            ///< 禁止拷贝
    WinHttpHandle& operator=(const WinHttpHandle&) = delete; ///< 禁止拷贝赋值

    /** @brief 移动构造：源对象句柄置空，避免双重释放。 */
    WinHttpHandle(WinHttpHandle&& other) noexcept : handle(other.handle) {
        other.handle = nullptr;
    }

    /** @brief 移动赋值：先释放自身旧句柄，再接管源句柄。 */
    WinHttpHandle& operator=(WinHttpHandle&& other) noexcept {
        if (this != &other) {
            if (handle != nullptr) {
                WinHttpCloseHandle(handle);
            }
            handle = other.handle;
            other.handle = nullptr;
        }
        return *this;
    }
};

/**
 * @brief 将 UTF-8 字符串转换为 Windows 宽字符 (UTF-16) 字符串。
 * @param text UTF-8 输入。
 * @return 对应的 std::wstring；空输入返回空串。
 * @throws std::runtime_error 转换失败时抛出。
 *
 * WinHTTP API 接受宽字符 URL / header，必须先做此转换。
 */
std::wstring wide_from_utf8(const std::string& text) {
    if (text.empty()) {
        return std::wstring();
    }
    int length = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0);
    if (length <= 0) {
        throw std::runtime_error("Failed to convert UTF-8 text to wide string");
    }
    std::wstring output(static_cast<std::size_t>(length), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), output.data(), length);
    return output;
}

/**
 * @brief 解析 HTTP(S) URL 为 host / path+query / port / secure 四元组。
 *
 * 使用 WinHttpCrackUrl 拆分 URL；path 与 extra(query/fragment) 合并为完整路径，
 * path 为空时返回 "/" 以满足 HTTP 协议要求。
 */
struct ParsedHttpUrl {
    std::wstring host;            ///< 主机名 (不含 scheme/port)
    std::wstring path_and_query;  ///< 资源路径与查询串
    INTERNET_PORT port = INTERNET_DEFAULT_HTTPS_PORT; ///< 端口 (默认 443)
    bool secure = true;           ///< 是否为 HTTPS
};

/**
 * @brief 将 URL 字符串解析为 ParsedHttpUrl。
 * @param url 完整的 http(s) URL。
 * @return 解析结果。
 * @throws std::runtime_error 当 WinHttpCrackUrl 报错时抛出。
 *
 * 显式 dwXxxLength = -1 让 WinHTTP 自动测量各段长度，避免预分配缓冲区。
 */
ParsedHttpUrl parse_http_url(const std::string& url) {
    std::wstring wide_url = wide_from_utf8(url);
    URL_COMPONENTS components{};
    components.dwStructSize = sizeof(components);
    components.dwSchemeLength = static_cast<DWORD>(-1);
    components.dwHostNameLength = static_cast<DWORD>(-1);
    components.dwUrlPathLength = static_cast<DWORD>(-1);
    components.dwExtraInfoLength = static_cast<DWORD>(-1);
    if (!WinHttpCrackUrl(wide_url.c_str(), 0, 0, &components)) {
        throw std::runtime_error("Failed to parse URL: " + url);
    }

    std::wstring path(components.lpszUrlPath, components.dwUrlPathLength);
    if (components.dwExtraInfoLength > 0 && components.lpszExtraInfo != nullptr) {
        path.append(components.lpszExtraInfo, components.dwExtraInfoLength); ///< 合并 path 与 query/fragment
    }
    if (path.empty()) {
        path = L"/"; ///< 根路径兜底
    }

    return ParsedHttpUrl{
        std::wstring(components.lpszHostName, components.dwHostNameLength),
        path,
        components.nPort,
        components.nScheme == INTERNET_SCHEME_HTTPS, ///< scheme 是否为 HTTPS
    };
}

/**
 * @brief 通过 Windows WinHTTP 执行 HTTP GET，返回二进制响应体。
 * @param url        目标 URL。
 * @param verify_ssl 是否校验 SSL 证书 (false 时忽略证书错误)。
 * @return 响应字节序列。
 * @throws std::runtime_error 任一步骤 (连接/请求/读取/状态码) 失败时抛出。
 *
 * 流程：open session → connect → open request → (可选) 关闭证书校验 → send →
 * receive → 查询状态码 (2xx) → 循环 WinHttpReadData 直到 available=0。
 * 开启 GZIP/DEFLATE 自动解压以减少带宽消耗。
 */
std::vector<std::uint8_t> http_get_binary(const std::string& url, bool verify_ssl) {
    ParsedHttpUrl parsed = parse_http_url(url);
    WinHttpHandle session(WinHttpOpen(L"atmospheric-lidar-pollution-cpp/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0));
    if (session.handle == nullptr) {
        throw std::runtime_error("WinHTTP session initialization failed for " + url);
    }

    DWORD decompression = WINHTTP_DECOMPRESSION_FLAG_GZIP | WINHTTP_DECOMPRESSION_FLAG_DEFLATE; ///< 启用 GZIP/DEFLATE 自动解压
    WinHttpSetOption(session.handle, WINHTTP_OPTION_DECOMPRESSION, &decompression, sizeof(decompression));

    WinHttpHandle connection(WinHttpConnect(session.handle, parsed.host.c_str(), parsed.port, 0));
    if (connection.handle == nullptr) {
        throw std::runtime_error("WinHTTP connect failed for " + url);
    }

    DWORD request_flags = parsed.secure ? WINHTTP_FLAG_SECURE : 0; ///< HTTPS 时启用 SECURE 标志
    WinHttpHandle request(WinHttpOpenRequest(connection.handle, L"GET", parsed.path_and_query.c_str(), nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, request_flags));
    if (request.handle == nullptr) {
        throw std::runtime_error("WinHTTP request creation failed for " + url);
    }

    if (!verify_ssl && parsed.secure) {
        // 用户显式跳过证书校验：忽略未知 CA / 过期 / 名称不匹配 / 用途不符
        DWORD security_flags = SECURITY_FLAG_IGNORE_UNKNOWN_CA | SECURITY_FLAG_IGNORE_CERT_DATE_INVALID | SECURITY_FLAG_IGNORE_CERT_CN_INVALID | SECURITY_FLAG_IGNORE_CERT_WRONG_USAGE;
        WinHttpSetOption(request.handle, WINHTTP_OPTION_SECURITY_FLAGS, &security_flags, sizeof(security_flags));
    }

    if (!WinHttpSendRequest(request.handle, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0)) {
        throw std::runtime_error("WinHTTP send failed for " + url);
    }
    if (!WinHttpReceiveResponse(request.handle, nullptr)) {
        throw std::runtime_error("WinHTTP receive failed for " + url);
    }

    DWORD status_code = 0;
    DWORD status_code_size = sizeof(status_code);
    if (!WinHttpQueryHeaders(request.handle, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &status_code, &status_code_size, WINHTTP_NO_HEADER_INDEX)) {
        throw std::runtime_error("Failed to query HTTP status for " + url);
    }
    if (status_code < 200 || status_code >= 300) {
        throw std::runtime_error("HTTP GET failed with status " + std::to_string(status_code) + " for " + url);
    }

    std::vector<std::uint8_t> output;
    for (;;) {
        DWORD available = 0;
        if (!WinHttpQueryDataAvailable(request.handle, &available)) {
            throw std::runtime_error("Failed to query response length for " + url);
        }
        if (available == 0) {
            break; ///< 没有更多数据可读，结束循环
        }
        std::vector<std::uint8_t> buffer(static_cast<std::size_t>(available), 0);
        DWORD downloaded = 0;
        if (!WinHttpReadData(request.handle, buffer.data(), available, &downloaded)) {
            throw std::runtime_error("Failed to read HTTP response for " + url);
        }
        output.insert(output.end(), buffer.begin(), buffer.begin() + downloaded); ///< 追加本次实际读到的字节
    }
    return output;
}

#else

/**
 * @brief 通过 libcurl 执行 HTTP GET，返回二进制响应体 (POSIX 路径)。
 * @param url        目标 URL。
 * @param verify_ssl 是否校验 SSL 证书 (false 时关闭对端与主机校验)。
 * @return 响应字节序列。
 * @throws std::runtime_error curl 初始化失败、传输失败或 HTTP 非 2xx 时抛出。
 *
 * 实现要点：进程级 curl_global_init/cleanup 由静态对象保证只调用一次；
 * 写回调使用 lambda 将分块数据追加到调用方提供的 vector；自动跟随重定向；
 * 校验通过 CURLINFO_RESPONSE_CODE 提取状态码，保证与平台无关的判等。
 */
std::vector<std::uint8_t> http_get_binary(const std::string& url, bool verify_ssl) {
    // 静态对象在首次进入函数时初始化全局 curl 环境，进程退出时自动清理
    static struct CurlGlobalInit {
        CurlGlobalInit() { curl_global_init(CURL_GLOBAL_DEFAULT); }
        ~CurlGlobalInit() { curl_global_cleanup(); }
    } curl_init;

    CURL* curl = curl_easy_init();
    if (curl == nullptr) {
        throw std::runtime_error("Failed to initialise libcurl for " + url);
    }

    std::vector<std::uint8_t> output;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "atmospheric-lidar-pollution-cpp/1.0");
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); ///< 自动跟随 3xx 重定向
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
        static_cast<std::size_t(*)(char*, std::size_t, std::size_t, void*)>(
            [](char* ptr, std::size_t size, std::size_t nmemb, void* userdata) -> std::size_t {
                auto* buffer = static_cast<std::vector<std::uint8_t>*>(userdata);
                std::size_t total = size * nmemb;       ///< 实际接收字节数
                buffer->insert(buffer->end(),
                               reinterpret_cast<std::uint8_t*>(ptr),
                               reinterpret_cast<std::uint8_t*>(ptr) + total);
                return total;                            ///< 返回已写字节数，不一致时 curl 会中止
            }));
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &output); ///< 把 vector 地址作为回调 userdata
    if (!verify_ssl) {
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L); ///< 跳过对端证书校验
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L); ///< 跳过主机名匹配校验
    }

    CURLcode result = curl_easy_perform(curl);

    long status_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
    curl_easy_cleanup(curl);

    if (result != CURLE_OK) {
        throw std::runtime_error("libcurl request failed for " + url + ": " + curl_easy_strerror(result));
    }
    if (status_code < 200 || status_code >= 300) {
        throw std::runtime_error("HTTP GET failed with status " + std::to_string(status_code) + " for " + url);
    }
    return output;
}

#endif

/**
 * @brief 通过 HTTP GET 获取文本响应 (二进制响应转换为字符串)。
 * @param url        目标 URL。
 * @param verify_ssl 是否校验 SSL。
 * @return 响应体按 UTF-8 解析的 std::string。
 *
 * 是 http_get_binary 的便捷包装；适用于直接消费 JSON/CSV 文本接口。
 */
std::string http_get_text(const std::string& url, bool verify_ssl) {
    std::vector<std::uint8_t> payload = http_get_binary(url, verify_ssl);
    return std::string(payload.begin(), payload.end());
}

/**
 * @brief 从 JSON 对象中读取数值字段，缺失或 null 时返回 fallback。
 * @param value    JSON 对象。
 * @param key      字段名。
 * @param fallback 缺失默认值。
 * @return 字段数值或 fallback。
 *
 * 配置/数据集中常存在可选数值，统一在此处做安全读取避免反复判 contains。
 */
double json_number_or(const Json& value, const std::string& key, double fallback) {
    if (!value.contains(key) || value.at(key).is_null()) {
        return fallback;
    }
    return value.at(key).number_value();
}

/**
 * @brief 从磁盘读取对齐好的地面观测文件 (JSON 数组) 为 GroundRecord 列表。
 * @param path JSON 文件路径 (由 fetch_public_ground_data 生成)。
 * @return 解析后的记录列表。
 * @throws std::runtime_error 文件不存在或字段缺失时抛出。
 *
 * 字段缺失时使用 json_number_or 提供工程默认值，保证在数据残缺时仍可继续。
 */
std::vector<GroundRecord> read_ground_records(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("Missing aligned Open-Meteo ground file for Cloudnet C++ path: " + path.string());
    }
    Json payload = read_json_file(path);
    std::vector<GroundRecord> records;
    for (const auto& item : payload.array_items()) {
        records.push_back(GroundRecord{
            item.at("timestamp").string_value(),
            json_number_or(item, "pm25", 12.0),
            json_number_or(item, "pm10", 20.0),
            json_number_or(item, "temperature_c", 18.0),
            json_number_or(item, "relative_humidity", 60.0),
            json_number_or(item, "wind_speed_ms", 2.0),
            json_number_or(item, "wind_dir_deg", 0.0),
        });
    }
    return records;
}

/**
 * @brief 在地面记录列表中找出时间上离 target 最近的一条。
 * @param target  目标时间 (time_t，秒)。
 * @param records 候选记录列表。
 * @return 时间差最小的记录的 const 引用。
 * @throws std::runtime_error 记录列表为空时抛出。
 *
 * 使用 |parse_timestamp(record.timestamp) - target| 作为代价，O(n) 线性扫描，
 * 适用于记录条数较少 (通常 < 24×7 条) 的逐小时地面数据。
 */
const GroundRecord& nearest_ground_record(const std::time_t target, const std::vector<GroundRecord>& records) {
    if (records.empty()) {
        throw std::runtime_error("No ground records available for Cloudnet alignment");
    }
    const GroundRecord* best_record = &records.front();
    double best_delta = std::numeric_limits<double>::infinity();
    for (const auto& record : records) {
        double delta = std::abs(std::difftime(parse_timestamp(record.timestamp), target));
        if (delta < best_delta) {
            best_delta = delta;
            best_record = &record;
        }
    }
    return *best_record;
}

std::pair<std::vector<double>, std::vector<double>> standard_molecular_fields(const std::vector<double>& ranges_m, double site_altitude_m, double elevation_deg) {
    std::vector<double> molecular_extinction;
    std::vector<double> molecular_backscatter;
    double elevation_rad = elevation_deg * std::numbers::pi / 180.0;
    for (double range_m : ranges_m) {
        double altitude_m = site_altitude_m + range_m * std::sin(elevation_rad);
        double molecular_ext = 0.012 * std::exp(-altitude_m / 8000.0);
        molecular_extinction.push_back(molecular_ext);
        molecular_backscatter.push_back(molecular_ext / 8.0);
    }
    return {molecular_extinction, molecular_backscatter};
}

std::vector<double> read_beta_row(int ncid, int beta_varid, int time_index, std::size_t range_count) {
    std::vector<double> values(range_count, 0.0);
    std::size_t start[2] = {static_cast<std::size_t>(time_index), 0};
    std::size_t count[2] = {1, range_count};
    require_netcdf(nc_get_vara_double(ncid, beta_varid, start, count, values.data()), "Failed to read Cloudnet beta row");
    for (double& value : values) {
        if (std::isnan(value) || value < 0.0) {
            value = 0.0;
        }
    }
    return values;
}

#endif
