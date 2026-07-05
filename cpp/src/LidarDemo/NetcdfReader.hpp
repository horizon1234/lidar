// Internal include fragment: NetCDF reader utilities.
#if defined(LIDAR_DEMO_HAS_NETCDF)

// ---- NetCDF 读取层：Cloudnet 数据接入 ----

/**
 * @brief 检查 netCDF 调用返回值，失败时抛出带上下文的异常。
 * @param status  netCDF API 返回的状态码 (NC_NOERR 表示成功)。
 * @param context 触发此次调用的语义描述，用于拼装错误信息。
 * @throws std::runtime_error 当 status != NC_NOERR 时抛出，附带 nc_strerror 翻译。
 *
 * 统一异常包装让上层调用者无需逐处手动检查返回码，简化错误处理路径。
 */
void require_netcdf(int status, const std::string& context) {
    if (status != NC_NOERR) {
        throw std::runtime_error(context + ": " + nc_strerror(status));
    }
}

/**
 * @brief 判断字符串是否全部为 ASCII 字符（字节值 < 128）。
 * @param value 待检测字符串。
 * @return true 表示为纯 ASCII，false 表示包含非 ASCII (如中文) 字符。
 *
 * netCDF 传统 API 仅保证接受 ASCII 路径，含 Unicode 字符的路径需先做镜像缓存。
 */
bool is_ascii_text(const std::string& value) {
    return std::all_of(value.begin(), value.end(), [](unsigned char current) {
        return current < 128;
    });
}

/**
 * @brief 确保路径的父目录存在，并返回该路径本身。
 * @param path 需要写出文件的完整路径。
 * @return 原路径；副作用是已递归创建其父目录。
 *
 * 封装 create_directories，避免在每个写文件点重复样板代码。
 */
std::filesystem::path ensure_parent_path(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    return path;
}

/**
 * @brief 将可能含非 ASCII 字符的路径转换为 netCDF 库可接受的等效路径。
 * @param input_path 用户原始路径（可能含中文等非 ASCII 字符）。
 * @return netCDF API 可直接打开的路径：原 ASCII 路径直接返回；
 *         含非 ASCII 者返回 /temp/lidar_cloudnet_cache/<basename> 处的缓存副本。
 *
 * 实现策略：在 /temp/lidar_cloudnet_cache 下按文件名镜像拷贝，文件大小不同时刷新拷贝，
 * 既规避 netCDF 对多字节路径的限制，又避免重复拷贝带来的 I/O 开销。
 */
std::filesystem::path netcdf_compatible_path(const std::filesystem::path& input_path) {
    std::filesystem::path resolved = std::filesystem::absolute(input_path);
    std::string path_text = resolved.string();
    if (is_ascii_text(path_text)) {
        return resolved;
    }

    std::filesystem::path cache_root = resolved.root_path() / "temp" / "lidar_cloudnet_cache";
    std::filesystem::path mirrored_path = ensure_parent_path(cache_root / resolved.filename());
    // 仅当目标不存在或大小不同时重新拷贝，避免对已缓存文件的冗余写
    if (!std::filesystem::exists(mirrored_path) || std::filesystem::file_size(mirrored_path) != std::filesystem::file_size(resolved)) {
        std::filesystem::copy_file(resolved, mirrored_path, std::filesystem::copy_options::overwrite_existing);
    }
    return mirrored_path;
}

/**
 * @brief netCDF 文件 RAII 句柄：构造时打开，析构时自动关闭。
 *
 * 通过封装 nc_open / nc_close，确保异常路径下文件描述符也会被释放，
 * 避免长时间运行的批量任务中泄漏 nc 标识符。
 */
class NetcdfFile {
public:
    /** @brief 以只读模式打开给定路径的 netCDF 文件。 */
    explicit NetcdfFile(const std::filesystem::path& path) {
        require_netcdf(nc_open(path.string().c_str(), NC_NOWRITE, &id_), "Failed to open netCDF file " + path.string());
    }

    /** @brief 析构时关闭文件，id_<0 表示已被移动/未占用。 */
    ~NetcdfFile() {
        if (id_ >= 0) {
            nc_close(id_);
        }
    }

    /** @return 内部 netCDF 文件标识符。 */
    int id() const { return id_; }

private:
    int id_ = -1;
};

/**
 * @brief 在 netCDF 文件中查询变量名对应的 varid。
 * @param ncid netCDF 文件 ID。
 * @param name 变量名。
 * @return 该变量的 varid。
 * @throws std::runtime_error 若变量不存在则抛出（缺失变量视作数据错误）。
 */
int variable_id(int ncid, const std::string& name) {
    int varid = -1;
    require_netcdf(nc_inq_varid(ncid, name.c_str(), &varid), "Missing netCDF variable " + name);
    return varid;
}

/**
 * @brief 判断 netCDF 文件中是否存在某变量。
 * @param ncid netCDF 文件 ID。
 * @param name 变量名。
 * @return true 表示存在，false 表示不存在（不抛异常）。
 *
 * 区别于 variable_id：用于可选字段的探测 (如 read_optional_scalar_numeric_variable)。
 */
bool has_variable(int ncid, const std::string& name) {
    int varid = -1;
    return nc_inq_varid(ncid, name.c_str(), &varid) == NC_NOERR;
}

/**
 * @brief 计算某 netCDF 变量所有维度长度的乘积（即元素总数）。
 * @param ncid  netCDF 文件 ID。
 * @param varid 变量 ID。
 * @return 该变量所含元素个数；多维变量退化为各维度逐元素相乘。
 *
 * 用于在读取二维 range/time 网格前一次性分配输出缓冲区。
 */
std::size_t variable_size(int ncid, int varid) {
    int ndims = 0;
    require_netcdf(nc_inq_varndims(ncid, varid, &ndims), "Failed to read variable rank");
    std::vector<int> dimids(static_cast<std::size_t>(ndims), 0);
    require_netcdf(nc_inq_vardimid(ncid, varid, dimids.data()), "Failed to read variable dimensions");
    std::size_t size = 1;
    for (int dimid : dimids) {
        std::size_t length = 0;
        require_netcdf(nc_inq_dimlen(ncid, dimid, &length), "Failed to read dimension length");
        size *= length;
    }
    return size;
}

/**
 * @brief 读取 netCDF 中的双精度数值变量并扁平化为 vector<double>。
 * @param ncid netCDF 文件 ID。
 * @param name 变量名。
 * @return 元素按 C 行主序排列的 vector<double>，长度等于 variable_size。
 * @throws std::runtime_error 读取失败时抛出。
 *
 * netCDF 按变量声明顺序返回数据，多维输出在调用方需自行根据维度还原索引。
 */
std::vector<double> read_numeric_variable(int ncid, const std::string& name) {
    int varid = variable_id(ncid, name);
    std::vector<double> output(variable_size(ncid, varid), 0.0);
    require_netcdf(nc_get_var_double(ncid, varid, output.data()), "Failed to read netCDF variable " + name);
    return output;
}

/**
 * @brief 读取标量数值变量（视为单元素）。
 * @param ncid netCDF 文件 ID。
 * @param name 变量名。
 * @return 变量首元素；若为 NaN 则替换为 0.0 以保证下游数学稳健。
 * @throws std::runtime_error 当变量为空时抛出。
 *
 * Cloudnet 部分全局属性以单值变量形式存储（如雷达波长），NaN 一律降级为 0。
 */
double read_scalar_numeric_variable(int ncid, const std::string& name) {
    std::vector<double> values = read_numeric_variable(ncid, name);
    if (values.empty()) {
        throw std::runtime_error("NetCDF scalar variable is empty: " + name);
    }
    return std::isnan(values.front()) ? 0.0 : values.front(); ///< NaN 杜绝：替换为 0
}

/**
 * @brief 安全读取可选标量变量，缺失时返回兜底值。
 * @param ncid     netCDF 文件 ID。
 * @param name     变量名。
 * @param fallback 变量不存在时使用的默认值。
 * @return 变量值（NaN→0）或 fallback。
 *
 * 与 read_scalar_numeric_variable 的区别：本函数不会因变量缺失而抛错，
 * 适用于 Cloudnet 不同数据集可能缺字段的鲁棒读取。
 */
double read_optional_scalar_numeric_variable(int ncid, const std::string& name, double fallback) {
    if (!has_variable(ncid, name)) {
        return fallback;
    }
    return read_scalar_numeric_variable(ncid, name);
}

/**
 * @brief 读取 netCDF 文本属性 (attribute)。
 * @param ncid  netCDF 文件 ID。
 * @param varid 所属变量 ID (NC_GLOBAL 用于全局属性)。
 * @param name  属性名。
 * @return 属性的字符串内容（不包含末尾 NUL）。
 * @throws std::runtime_error 读取失败时抛出。
 *
 * 用于读取 Cloudnet 的 time 变量 units 属性 (例如 "hours since 2022-01-01 ...")。
 */
std::string read_text_attribute(int ncid, int varid, const std::string& name) {
    std::size_t length = 0;
    require_netcdf(nc_inq_attlen(ncid, varid, name.c_str(), &length), "Failed to read netCDF attribute length " + name);
    std::string value(length, '\0');
    require_netcdf(nc_get_att_text(ncid, varid, name.c_str(), value.data()), "Failed to read netCDF attribute " + name);
    return value;
}

/**
 * @brief 去除字符串首尾的空白字符。
 * @param text 待处理字符串。
 * @return 两端无空格、制表符、换行等的字符串；全为空白则返回空。
 *
 * 提供 std::string 版本以兼容老编译器 (某些标准库的 std::string::trim 扩展不通用)。
 */
std::string trim(std::string text) {
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) {
        text.erase(text.begin());
    }
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) {
        text.pop_back();
    }
    return text;
}

/**
 * @brief 解析 Cloudnet time units 中 "since" 之后的基准时间字符串为 time_t。
 * @param time_units 形如 "hours since 2022-01-01 00:00:00 +00:00" 的 units 串。
 * @return 基准时间对应的本地时区 time_t。
 * @throws std::runtime_error 当缺少 "since" 或时间格式不可解析时抛出。
 *
 * 实现要点：定位 "since"，截去尾部时区说明 (" +offset")，先后尝试
 * "%Y-%m-%d %H:%M:%S" 与 "%Y-%m-%dT%H:%M:%S" 两种格式，兼容 CF 约定差异。
 */
std::time_t parse_cloudnet_base_time(const std::string& time_units) {
    std::size_t since_index = time_units.find("since");
    if (since_index == std::string::npos) {
        throw std::runtime_error("Unsupported Cloudnet time units: " + time_units);
    }
    std::string base_text = trim(time_units.substr(since_index + 5));   ///< "since" 之后的内容
    std::size_t timezone_index = base_text.find(" +");
    if (timezone_index != std::string::npos) {
        base_text = base_text.substr(0, timezone_index);               ///< 剥离 " +00:00" 等时区后缀
    }
    std::tm stamp{};
    std::istringstream input(base_text);
    input >> std::get_time(&stamp, "%Y-%m-%d %H:%M:%S");
    if (input.fail()) {
        std::istringstream fallback(base_text);
        fallback >> std::get_time(&stamp, "%Y-%m-%dT%H:%M:%S");        ///< 兼容 'T' 分隔的 ISO 写法
        if (fallback.fail()) {
            throw std::runtime_error("Failed to parse Cloudnet base time: " + base_text);
        }
    }
    stamp.tm_isdst = -1;
    return std::mktime(&stamp);
}

/**
 * @brief 根据 units 前缀判断 Cloudnet 时间轴一个单位对应多少秒。
 * @param time_units 形如 "hours since ..." 的 units 串。
 * @return 单位换算系数 (hours→3600, minutes→60, seconds→1, days→86400)。
 * @throws std::runtime_error 当单位词无法识别时抛出。
 */
double time_unit_seconds(const std::string& time_units) {
    std::string unit = time_units.substr(0, time_units.find(' ')); ///< 取 units 串首词作为单位
    if (unit == "hours" || unit == "hour") {
        return 3600.0;
    }
    if (unit == "minutes" || unit == "minute") {
        return 60.0;
    }
    if (unit == "seconds" || unit == "second") {
        return 1.0;
    }
    if (unit == "days" || unit == "day") {
        return 86400.0;
    }
    throw std::runtime_error("Unsupported Cloudnet time unit: " + time_units);
}

/**
 * @brief 把 Cloudnet 时间坐标值转换为分钟级 ISO 字符串。
 * @param value       时间轴上的数值（单位由 time_units 决定）。
 * @param time_units  形如 "hours since 2022-01-01 ... " 的 CF units 串。
 * @return "YYYY-MM-DDTHH:MM" 格式字符串。
 *
 * 计算流程：value × 单位秒数 → 自基准时间起经过的秒数 → 相加后格式化。
 */
std::string cloudnet_time_to_iso_minute(double value, const std::string& time_units) {
    double seconds = value * time_unit_seconds(time_units);
    return format_timestamp(parse_cloudnet_base_time(time_units) + static_cast<long long>(std::llround(seconds)));
}

/**
 * @brief 在 [0, length) 区间内均匀抽取 count 个索引。
 * @param length 序列的总长度。
 * @param count  需要选取的索引数。
 * @return 升序排列的索引 vector；length <= count 时返回 0..length-1 全集。
 *
 * 用 std::set 去重，避免浮点取整造成的重复索引；通过线性插值
 * index * (length-1)/(count-1) 保证首尾两点都被选中。
 */
std::vector<int> select_even_indices(int length, int count) {
    if (length <= count) {
        std::vector<int> output(length, 0);
        std::iota(output.begin(), output.end(), 0);
        return output;
    }
    std::set<int> chosen;
    for (int index = 0; index < count; ++index) {
        chosen.insert(static_cast<int>(std::llround(index * (length - 1.0) / std::max(count - 1, 1)))); ///< 线性映射保证首尾命中
    }
    return std::vector<int>(chosen.begin(), chosen.end());
}

/**
 * @brief 在 range 轴上筛出落在 [min,max] 区间且均匀下采样到 target_count 个的 bin 索引。
 * @param range_values_m 原始 range 数组（米）。
 * @param min_range_m    允许的最小 range（米）。
 * @param max_range_m    允许的最大 range（米）。
 * @param target_count   需要保留的 bin 数。
 * @return 满足条件的升序索引 vector。
 * @throws std::runtime_error 无任何 bin 落入区间时抛出。
 *
 * 流程：先收集满足区间条件的候选 bin，再调用 select_even_indices 在候选集中均匀采样，
 * 既保证物理范围正确又能控制输出数据量。
 */
std::vector<int> filter_range_indices(const std::vector<double>& range_values_m, double min_range_m, double max_range_m, int target_count) {
    std::vector<int> candidates;
    for (std::size_t index = 0; index < range_values_m.size(); ++index) {
        if (range_values_m[index] >= min_range_m && range_values_m[index] <= max_range_m) {
            candidates.push_back(static_cast<int>(index));
        }
    }
    if (candidates.empty()) {
        throw std::runtime_error("No Cloudnet range bins fell inside the configured min/max range");
    }
    std::vector<int> offsets = select_even_indices(static_cast<int>(candidates.size()), target_count);
    std::vector<int> output;
    output.reserve(offsets.size());
    for (int offset : offsets) {
        output.push_back(candidates[static_cast<std::size_t>(offset)]);
    }
    return output;
}

#endif
