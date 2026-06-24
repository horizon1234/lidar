/**
 * @file lidar_demo.cpp
 * @brief 大气激光雷达颗粒物监测系统 —— 全部 C++ 实现的唯一编译单元
 *
 * 本文件实现了 lidar_demo.hpp 中声明的全部对外接口，是整个项目的核心。
 * 由于内部各模块之间共享大量内部辅助类型与工具函数，因此采用"单编译单元"
 * 的组织方式：所有内部实现都放在匿名命名空间（namespace { ... }）中，
 * 仅供本文件内部使用；对外只暴露头文件中声明的那些函数。
 *
 * 文件按处理流程的先后顺序组织为以下若干大段（每段之间用分隔注释栏标注）：
 *   1. JSON 引擎（Json 类成员、递归下降解析器 JsonParser、序列化器）
 *   2. 通用数学/时间工具（clamp、mean、median、gaussian、时间戳解析等）
 *   3. NetCDF 读取层（仅在编译时启用 LIDAR_DEMO_HAS_NETCDF 时有效）
 *   4. 地面观测记录处理（Open-Meteo 数据的抓取、解析、缓存）
 *   5. HTTP 下载（Windows 用 WinHTTP，其它平台用 libcurl）
 *   6. 数据 → JSON 序列化辅助（to_json、各类 vector → Json 转换）
 *   7. 正向仿真（合成的 stare + PPI 扫描场景生成，含真值）
 *   8. 管线各阶段算法：
 *        - 预处理（背景扣除 / 能量归一化 / 重叠校正 / 距离平方校正 / SNR）
 *        - Fernald/Klett 反演（远端到近端的反向递推得到消光与后向散射）
 *        - 湿度校正（吸湿增长因子 f(RH)，把"湿"消光换算为"干"消光）
 *        - ENU 坐标转换（极坐标 range/az/el → 东/北/上）
 *        - PM 标定（最小二乘回归 + 站点偏置 + 热点加成）
 *        - 热点检测（PPI 网格上的连通域分析 + 拓扑质心）
 *        - 质量评估（RMSE / MAE / R² / 分类指标 / 漂移监测 / 失效案例）
 *   9. demo payload 组装（为仪表盘准备最终 JSON）
 *   10. 顶层公开函数（parse_pipeline_config / run_end_to_end /
 *       render_dashboard / build_summary_payload / fetch_*）
 *
 * 对应的 Python 参考实现请参见仓库早期的 lidar_core/ 包（已移除），
 * 本文件是其等价的 C++ 移植版本。
 */
#include "lidar_demo/lidar_demo.hpp"

// ============================================================================
// 标准库头文件
// ============================================================================
#include <algorithm>   ///< std::sort / std::min / std::max / std::all_of 等
#include <chrono>      ///< 性能计时（latency_ms）
#include <cmath>       ///< 数学函数：sin/cos/exp/sqrt/fmod 等
#include <cctype>      ///< isspace / isdigit 等字符判定
#include <cstdint>     ///< std::uint8_t 等定宽整型
#include <cstdio>      ///< snprintf 等
#include <ctime>       ///< 时间戳处理（mktime / localtime_r / strftime）
#include <fstream>     ///< 文件读写
#include <iomanip>     ///< put_time / get_time
#include <limits>      ///< 数值极限
#include <numbers>     ///< C++20 数学常量 pi_v
#include <numeric>     ///< std::accumulate 等
#include <queue>       ///< 热点检测连通域分析用的 BFS 队列
#include <random>      ///< 仿真用的 std::mt19937 正态分布
#include <set>         ///< 去重排序容器
#include <sstream>     ///< 字符串流（JSON 序列化、URL 拼接等）
#include <stdexcept>   ///< 异常类

// ============================================================================
// 平台相关的网络库：Windows 用 WinHTTP，其它平台（Linux/macOS）用 libcurl
// ============================================================================
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX            ///< 阻止 windows.h 定义 min/max 宏
#endif
#include <windows.h>
#include <winhttp.h>
#else
#include <curl/curl.h>
#endif

// NetCDF C API（仅在启用真实 Cloudnet 数据读取时需要）
#if defined(LIDAR_DEMO_HAS_NETCDF)
#include <netcdf.h>
#endif

namespace lidar_demo {

// ============================================================================
// Json 类成员函数实现
// 这些函数都是简单的成员初始化或类型查询，直接对应头文件中的声明。
// ============================================================================

// ---- 构造函数：把传入值直接放入 variant ----
Json::Json() : value(nullptr) {}
Json::Json(std::nullptr_t) : value(nullptr) {}
Json::Json(bool input) : value(input) {}
Json::Json(int input) : value(static_cast<double>(input)) {}
Json::Json(double input) : value(input) {}
Json::Json(const char* input) : value(std::string(input)) {}
Json::Json(const std::string& input) : value(input) {}
Json::Json(std::string&& input) : value(std::move(input)) {}
Json::Json(const array_type& input) : value(input) {}
Json::Json(array_type&& input) : value(std::move(input)) {}
Json::Json(const object_type& input) : value(input) {}
Json::Json(object_type&& input) : value(std::move(input)) {}

// ---- 类型查询：用 std::holds_alternative 判断当前存储的是哪种类型 ----
bool Json::is_null() const { return std::holds_alternative<std::nullptr_t>(value); }
bool Json::is_bool() const { return std::holds_alternative<bool>(value); }
bool Json::is_number() const { return std::holds_alternative<double>(value); }
bool Json::is_string() const { return std::holds_alternative<std::string>(value); }
bool Json::is_array() const { return std::holds_alternative<array_type>(value); }
bool Json::is_object() const { return std::holds_alternative<object_type>(value); }

// ---- 取值访问：用 std::get 取出具体类型的值，类型不符会抛 std::bad_variant_access ----
bool Json::bool_value() const { return std::get<bool>(value); }
double Json::number_value() const { return std::get<double>(value); }
int Json::int_value() const { return static_cast<int>(std::lround(number_value())); }  // 四舍五入到 int
const std::string& Json::string_value() const { return std::get<std::string>(value); }
const Json::array_type& Json::array_items() const { return std::get<array_type>(value); }
Json::array_type& Json::array_items() { return std::get<array_type>(value); }
const Json::object_type& Json::object_items() const { return std::get<object_type>(value); }
Json::object_type& Json::object_items() { return std::get<object_type>(value); }

bool Json::contains(const std::string& key) const {
    // 只有当当前值是对象时才做键查找
    return is_object() && object_items().find(key) != object_items().end();
}

const Json& Json::at(const std::string& key) const {
    // std::map::at 在键不存在时会抛 std::out_of_range
    return object_items().at(key);
}

Json& Json::operator[](const std::string& key) {
    // 非 object 类型时先切换为空对象，便于流式构建
    if (!is_object()) {
        value = object_type{};
    }
    return object_items()[key];
}

const Json& Json::operator[](const std::string& key) const {
    return at(key);
}

// ============================================================================
// 内部辅助实现（匿名命名空间，仅本文件可见）
// ============================================================================
namespace {

/**
 * @class JsonParser
 * @brief 手写的递归下降 JSON 解析器
 *
 * 支持完整的 JSON 语法：对象、数组、字符串（含 \uXXXX、\"、\\ 等转义）、
 * 数值（整数/小数/科学计数法）、true/false/null。
 * 不依赖任何第三方库，parse() 返回一个 Json 值。
 */
class JsonParser {
public:
    /** @brief 构造一个绑定到输入文本的解析器 */
    explicit JsonParser(const std::string& input) : text_(input) {}

    /**
     * @brief 执行解析
     * @return 解析得到的 Json 值
     * @throws std::runtime_error 当遇到非法 JSON 语法时抛出
     */
    Json parse() {
        Json result = parse_value();
        skip_whitespace();
        // 解析结束后若仍有残余字符，说明输入不是合法的单个 JSON 值
        if (position_ != text_.size()) {
            throw std::runtime_error("Unexpected trailing characters in JSON");
        }
        return result;
    }

private:
    const std::string& text_;   ///< 待解析的原始文本（引用，不拷贝）
    std::size_t position_ = 0;  ///< 当前解析位置（下标）

    /** @brief 跳过连续的空白字符（空格、制表、换行等） */
    void skip_whitespace() {
        while (position_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[position_]))) {
            ++position_;
        }
    }

    /** @brief 查看下一个字符但不消费；越界时抛异常 */
    char peek() const {
        if (position_ >= text_.size()) {
            throw std::runtime_error("Unexpected end of JSON input");
        }
        return text_[position_];
    }

    /** @brief 期望并消费一个指定的字符；不符则抛异常（用于 ':' ',' 等） */
    void expect(char expected) {
        skip_whitespace();
        if (peek() != expected) {
            throw std::runtime_error("Unexpected JSON token");
        }
        ++position_;
    }

    /** @brief 尝试消费一个字符；成功返回 true，否则返回 false（用于可选的结束符） */
    bool consume(char token) {
        skip_whitespace();
        if (position_ < text_.size() && text_[position_] == token) {
            ++position_;
            return true;
        }
        return false;
    }

    /** @brief 根据下一个非空字符分派到对应的子解析器 */
    Json parse_value() {
        skip_whitespace();
        switch (peek()) {
        case '{':
            return parse_object();
        case '[':
            return parse_array();
        case '"':
            return Json(parse_string());
        case 't':
            parse_literal("true");
            return Json(true);
        case 'f':
            parse_literal("false");
            return Json(false);
        case 'n':
            parse_literal("null");
            return Json(nullptr);
        default:
            return parse_number();
        }
    }

    /** @brief 解析一个 JSON 对象 { "k": v, ... } */
    Json parse_object() {
        expect('{');
        Json::object_type object;
        skip_whitespace();
        if (consume('}')) {
            return Json(std::move(object));   // 空对象 {}
        }
        while (true) {
            skip_whitespace();
            if (peek() != '"') {
                throw std::runtime_error("Expected JSON object key");
            }
            std::string key = parse_string();
            expect(':');
            object.emplace(std::move(key), parse_value());
            skip_whitespace();
            if (consume('}')) {
                break;   // 对象结束
            }
            expect(','); // 否则必须是逗号分隔的下一对
        }
        return Json(std::move(object));
    }

    /** @brief 解析一个 JSON 数组 [ v, v, ... ] */
    Json parse_array() {
        expect('[');
        Json::array_type array;
        skip_whitespace();
        if (consume(']')) {
            return Json(std::move(array));   // 空数组 []
        }
        while (true) {
            array.push_back(parse_value());
            skip_whitespace();
            if (consume(']')) {
                break;   // 数组结束
            }
            expect(',');
        }
        return Json(std::move(array));
    }

    /**
     * @brief 解析一个 JSON 字符串（已经定位到起始引号之后）
     *
     * 处理所有标准转义：\" \\ \/ \b \f \n \r \t 以及 \uXXXX。
     * 注意：\uXXXX 这里只正确处理 ASCII 范围（≤0x7F）的码点，
     * 非基本平面的字符会被替换为 '?'。
     */
    std::string parse_string() {
        expect('"');
        std::string output;
        while (position_ < text_.size()) {
            char current = text_[position_++];
            if (current == '"') {
                return output;   // 结束引号
            }
            if (current != '\\') {
                output.push_back(current);   // 普通字符
                continue;
            }
            // 处理转义序列
            if (position_ >= text_.size()) {
                throw std::runtime_error("Invalid JSON escape sequence");
            }
            char escaped = text_[position_++];
            switch (escaped) {
            case '"': output.push_back('"'); break;
            case '\\': output.push_back('\\'); break;
            case '/': output.push_back('/'); break;
            case 'b': output.push_back('\b'); break;
            case 'f': output.push_back('\f'); break;
            case 'n': output.push_back('\n'); break;
            case 'r': output.push_back('\r'); break;
            case 't': output.push_back('\t'); break;
            case 'u': {
                // \uXXXX 四位十六进制 Unicode 码点
                if (position_ + 4 > text_.size()) {
                    throw std::runtime_error("Invalid unicode escape in JSON string");
                }
                std::string hex = text_.substr(position_, 4);
                position_ += 4;
                int code_point = std::stoi(hex, nullptr, 16);
                if (code_point <= 0x7F) {
                    output.push_back(static_cast<char>(code_point));
                } else {
                    output.push_back('?');   // 超出 ASCII，简化处理
                }
                break;
            }
            default:
                throw std::runtime_error("Unsupported JSON escape sequence");
            }
        }
        throw std::runtime_error("Unterminated JSON string");
    }

    /** @brief 解析一个 JSON 数值（支持负号、小数、科学计数法） */
    Json parse_number() {
        skip_whitespace();
        std::size_t start = position_;
        if (text_[position_] == '-') {
            ++position_;
        }
        // 整数部分
        while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_]))) {
            ++position_;
        }
        // 小数部分
        if (position_ < text_.size() && text_[position_] == '.') {
            ++position_;
            while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_]))) {
                ++position_;
            }
        }
        // 科学计数法指数部分
        if (position_ < text_.size() && (text_[position_] == 'e' || text_[position_] == 'E')) {
            ++position_;
            if (position_ < text_.size() && (text_[position_] == '+' || text_[position_] == '-')) {
                ++position_;
            }
            while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_]))) {
                ++position_;
            }
        }
        return Json(std::stod(text_.substr(start, position_ - start)));
    }

    /** @brief 解析一个字面量关键字（true / false / null） */
    void parse_literal(const std::string& literal) {
        skip_whitespace();
        if (text_.substr(position_, literal.size()) != literal) {
            throw std::runtime_error("Unexpected JSON literal");
        }
        position_ += literal.size();
    }
};

/**
 * @brief 把一个字符串转义为 JSON 字符串字面量的内部内容（不含外层引号）
 *
 * 例如双引号会被替换为 \"，换行替换为 \n，控制字符替换为 \uXXXX 等。
 * @param input 原始字符串
 * @return 转义后的字符串（不含外层引号）
 */
std::string escape_json_string(const std::string& input) {
    std::ostringstream builder;
    for (char current : input) {
        switch (current) {
        case '"': builder << "\\\""; break;
        case '\\': builder << "\\\\"; break;
        case '\b': builder << "\\b"; break;
        case '\f': builder << "\\f"; break;
        case '\n': builder << "\\n"; break;
        case '\r': builder << "\\r"; break;
        case '\t': builder << "\\t"; break;
        default:
            // 控制字符（< 0x20）必须用 \uXXXX 表示
            if (static_cast<unsigned char>(current) < 0x20) {
                builder << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(current)) << std::dec;
            } else {
                builder << current;
            }
        }
    }
    return builder.str();
}

/**
 * @brief JSON 序列化的递归核心
 *
 * 把 Json 值写入输出流。当 indent>0 时输出带缩进的格式化文本，
 * 否则（indent==0）输出紧凑格式。depth 表示当前递归深度，用于计算缩进。
 *
 * 数值序列化的细则：接近整数（|x-round(x)|<1e-9）时按整数输出，
 * 否则按定点小数输出（最多 6 位有效位，并会去掉尾部的 0）。
 *
 * @param value  要序列化的 Json 值
 * @param output 目标输出流
 * @param indent 每级缩进的空格数（0 表示紧凑）
 * @param depth  当前递归深度
 */
void dump_json_impl(const Json& value, std::ostringstream& output, int indent, int depth) {
    if (value.is_null()) {
        output << "null";
        return;
    }
    if (value.is_bool()) {
        output << (value.bool_value() ? "true" : "false");
        return;
    }
    if (value.is_number()) {
        double number = value.number_value();
        if (std::abs(number - std::round(number)) < 1e-9) {
            output << static_cast<long long>(std::llround(number));
        } else {
            // 定点小数：保留 6 位，再剥离末尾多余的 0 与小数点
            std::ostringstream local;
            local << std::fixed << std::setprecision(6) << number;
            std::string text = local.str();
            while (!text.empty() && text.back() == '0') {
                text.pop_back();
            }
            if (!text.empty() && text.back() == '.') {
                text.pop_back();
            }
            output << text;
        }
        return;
    }
    if (value.is_string()) {
        output << '"' << escape_json_string(value.string_value()) << '"';
        return;
    }
    if (value.is_array()) {
        const auto& items = value.array_items();
        output << '[';
        if (!items.empty()) {
            bool pretty = indent > 0;
            for (std::size_t index = 0; index < items.size(); ++index) {
                if (pretty) {
                    output << '\n' << std::string((depth + 1) * indent, ' ');
                }
                dump_json_impl(items[index], output, indent, depth + 1);
                if (index + 1 < items.size()) {
                    output << ',';
                }
            }
            if (pretty) {
                output << '\n' << std::string(depth * indent, ' ');
            }
        }
        output << ']';
        return;
    }

    const auto& items = value.object_items();
    output << '{';
    if (!items.empty()) {
        bool pretty = indent > 0;
        std::size_t emitted = 0;
        for (const auto& [key, entry] : items) {
            if (pretty) {
                output << '\n' << std::string((depth + 1) * indent, ' ');
            }
            output << '"' << escape_json_string(key) << '"' << ':';
            if (pretty) {
                output << ' ';
            }
            dump_json_impl(entry, output, indent, depth + 1);
            ++emitted;
            if (emitted < items.size()) {
                output << ',';
            }
        }
        if (pretty) {
            output << '\n' << std::string(depth * indent, ' ');
        }
    }
    output << '}';
}

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
    Json::array_type output;
    output.reserve(records.size());
    for (const auto& record : records) {
        output.push_back(Json::object_type{
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
    Json::array_type output;
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
    Json::array_type output;
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
    Json::array_type output;
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
    Json::array_type output;
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
    return Json::object_type{
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
    return Json::object_type{
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
    return Json::object_type{
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
    return Json::object_type{
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
    return Json::object_type{
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
 * @brief 预处理（L1）结果中间结构。
 * 包含去背景的 L1 信号、范围平方校正后的衰减后向散射、信噪比和质控标志列表。
 */
struct PreprocessResult {
    std::vector<double> l1_signal;
    std::vector<double> attenuated_backscatter;
    std::vector<double> snr;
    std::vector<std::string> qc_flags;
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

// ====================================================================================
// ---- 正向物理仿真：合成 LiDAR 场景 / Forward physics simulation -----------------------------------
// 本节按"大气真实状态 → 雷达观测"的正向过程合成训练数据：先给定分子/气溶胶/PM 的真值场，
// 再代入 LiDAR 方程加上 detector 噪声，得到可被反演算法处理的"伪 raw_counts"。
// ====================================================================================

/**
 * @brief 生成期望为 mean_value、标准差为 sigma 的高斯随机数。
 *
 * 用于给仿真各通道加入观测噪声（背景噪声、激光能量抖动、温湿度随机扰动等），
 * 保证训练数据真实反映仪器统计涨落。
 *
 * @param[in,out] rng 梅森旋转随机数引擎，保证可复现。
 * @param[in] mean_value 期望。
 * @param[in] sigma 标准差（>0）。
 * @return 单次高斯抽样结果。
 */
double sample_gaussian(std::mt19937& rng, double mean_value, double sigma) {
    std::normal_distribution<double> distribution(mean_value, sigma);
    return distribution(rng);
}

/**
 * @brief 近似 LiDAR 双轴重叠因子 overlap(r) 随距离变化曲线。
 *
 * 物理含义：双轴 LiDAR 发射轴与接收视轴不完全重合，近距离处部分回波落在视场外，
 * overlap 在很近距离处接近常数下限（约 0.22）并随距离按幂函数趋近于 1。
 * 这里用 (r/180)^0.82 拟合典型几何并夹取 [0.22, 1.0]，避免近场民虚假高值。
 *
 * @param[in] ranges_m 各距离 bin 的斜距（米）。
 * @return 与 ranges_m 等长的 overlap 系数（无量纲 0~1）。
 */
std::vector<double> build_overlap(const std::vector<double>& ranges_m) {
    std::vector<double> overlap;
    overlap.reserve(ranges_m.size());
    for (double distance_m : ranges_m) {
        // 幂律拟合：r/180 的 0.82 次方，模拟典型双轴系统从近距离逐步进入全视场的过程
        double ratio = std::pow(distance_m / 180.0, 0.82);
        // 下限 0.22 防止除零导致近场发散，上限 1.0 表示远处完全进入视场
        overlap.push_back(clamp(ratio, 0.22, 1.0));
    }
    return overlap;
}

/**
 * @brief 在给定扫描几何与时间步下，仿真生成单条 LiDAR 廓线对应的真值场。
 *
 * 物理模型构成：
 * 1. 分子（Rayleigh）背景：消光系数 0.012·exp(-h/8000)（标准大气高度衰减，标高 8 km），
 *    后向散射取消光的 1/8；
 * 2. 边界层气溶胶：近地面增强项，含时间日变化（sin 相位，模拟昼夜 PBL 演化）；
 * 3. 高架抬升层（lofted）：高度约 500 m 处的高斯层，常对应区域输送；
 * 4. 两个烟羽（plume）：固定水平/方位角/高度上的三维高斯团，是测试热点检测的关键目标。
 *
 * 湿度修正：相对湿度 > 0.55 时按比例放大消光（吸湿增长）。
 * PM 反推：由干气溶胶消光经线性经验关系换算到 PM2.5/PM10，反映典型质量消光效率。
 *
 * @param[in] ranges_m 距离 bin 序列。
 * @param[in] azimuth_deg 方位角（度），用于定位烟羽横向位置（PPI 模式）。
 * @param[in] elevation_deg 仰角（度），决定每个 bin 对应的高度与水平距离。
 * @param[in] step_index 当前时间步索引，用于日变化相位。
 * @param[in] total_steps 时间步总数，归一化相位。
 * @param[in] relative_humidity 相对湿度（0~1），用于吸湿增长修正。
 * @param[in] lidar_ratio_sr 气溶胶激光比（sr），将消光换算为后向散射（典型 40~80 sr）。
 * @return 该廓线的真值场（分子/总后向散射、总消光、PM、热点掩膜）。
 */
SimulatedFields simulate_profile_fields(
    const std::vector<double>& ranges_m,
    double azimuth_deg,
    double elevation_deg,
    int step_index,
    int total_steps,
    double relative_humidity,
    double lidar_ratio_sr
) {
    SimulatedFields output;
    // 日变化相位 2π·step/total：让边界层高度与气溶胶浓度随昼夜循环变化
    double time_phase = 2.0 * std::numbers::pi * static_cast<double>(step_index) / std::max(total_steps, 1);
    // 吸湿增长因子：相对湿度越高，气溶胶散射截面越大（仅在 RH>0.55 后启动）
    double humidity_growth = 1.0 + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;

    for (double range_m : ranges_m) {
        double elevation_rad = elevation_deg * std::numbers::pi / 180.0;
        double altitude_m = range_m * std::sin(elevation_rad);
        double horizontal_m = range_m * std::cos(elevation_rad);
        // 标准 Rayleigh 分子消光：海平面 0.012，标高 8 km，随高度指数衰减
        double molecular_ext = 0.012 * std::exp(-altitude_m / 8000.0);
        // 分子后向散射约为消光的 1/8（即标准分子激光比约 8 sr）
        double molecular_beta = molecular_ext / 8.0;

        // 边界层气溶胶：含日变化振幅项，随高度按 550 m 标高衰减
        double boundary_layer = (0.017 + 0.018 * (1.0 + std::sin(time_phase)) / 2.0) * std::exp(-altitude_m / 550.0);
        // 高架抬升层：高度约 500 m 的高斯包络，常表示区域输送来的气溶胶层
        double lofted_layer = 0.018 * gaussian(altitude_m, 500.0 + 120.0 * std::sin(time_phase), 160.0);
        // 烟羽 1：近场（水平约 650 m、方位 80°、高度约 85 m）的局部强源
        double plume_1 = 0.080 * gaussian(horizontal_m, 650.0 + 80.0 * std::cos(time_phase), 130.0)
            * gaussian(azimuth_delta(azimuth_deg, 80.0), 0.0, 18.0)
            * gaussian(altitude_m, 85.0, 45.0);
        // 烟羽 2：稍远抬升的扩散源（水平约 1050 m、方位 170°、高度约 120 m）
        double plume_2 = 0.045 * gaussian(horizontal_m, 1050.0, 150.0)
            * gaussian(azimuth_delta(azimuth_deg, 170.0), 0.0, 22.0)
            * gaussian(altitude_m, 120.0, 65.0);

        // 干气溶胶消光（不含吸湿），随后再叠加湿度增长
        double aerosol_dry_ext = boundary_layer + lofted_layer + plume_1 + plume_2;
        // 实际（湿）气溶胶消光：以 humidity_growth 放大
        double aerosol_ext = aerosol_dry_ext * humidity_growth;
        // 后向散射：按给定激光比由消光换算（β = σ / S）
        double aerosol_beta = aerosol_ext / lidar_ratio_sr;

        output.molecular_extinction.push_back(molecular_ext);
        output.molecular_backscatter.push_back(molecular_beta);
        output.true_backscatter.push_back(molecular_beta + aerosol_beta);
        output.true_extinction.push_back(molecular_ext + aerosol_ext);
        // 干气溶胶消光到 PM2.5 的经验线性关系：系数 640 为典型质量消光效率倒数（约 1/0.00156 m²/g）
        // plume_1 单独加权表示局部源对细颗粒的额外贡献，常数 15 为再悬浮/本底
        output.true_pm25.push_back(640.0 * aerosol_dry_ext + 210.0 * plume_1 + 15.0);
        // PM10 略高于 PM2.5，额外含两个烟羽（粗粒贡献）和本底 22
        output.true_pm10.push_back(920.0 * aerosol_dry_ext + 260.0 * (plume_1 + plume_2) + 22.0);
        // 热点掩膜阈值：烟羽总贡献 > 0.025 标记为热点
        output.true_hotspot_mask.push_back((plume_1 + plume_2 > 0.025) ? 1 : 0);
    }
    return output;
}

/**
 * @brief 调用 LiDAR 方程将真值后向散射/消光转换为带噪声的 raw_counts。
 *
 * 实现的离散化 LiDAR 方程：
 * \f$ P(r) = C \cdot E \cdot O(r) \cdot \beta(r) \cdot \exp(-2\tau(0,r)) / r^2 + P_\text{bg} \f$
 * - C 为系统常数，E 激光能量，O(r) overlap，β 总后向散射；
 * - 指数项为双程大气透过率，τ 为累积光学厚度（沿路径积分的消光）；
 * - 1/r² 为光束几何发散衰减。
 * 同时按探测器噪声模型加入背景+信号相对扰动 5%~7%，并设置下限防止出现负计数。
 *
 * @param[in] true_backscatter 总后向散射廓线（分子+气溶胶）。
 * @param[in] true_extinction  总消光廓线，用于积分光学厚度。
 * @param[in] overlap          双轴重叠因子。
 * @param[in] ranges_m         距离 bin 序列（米）。
 * @param[in] laser_energy_mj  当前脉冲激光能量（mJ）。
 * @param[in] background_counts 探测器背景计数。
 * @param[in] system_constant  系统常数 C。
 * @param[in,out] rng          随机数引擎。
 * @return 含噪声的 raw_counts 序列。
 */
std::vector<double> simulate_raw_counts(
    const std::vector<double>& true_backscatter,
    const std::vector<double>& true_extinction,
    const std::vector<double>& overlap,
    const std::vector<double>& ranges_m,
    double laser_energy_mj,
    double background_counts,
    double system_constant,
    std::mt19937& rng
) {
    std::vector<double> raw_counts;
    raw_counts.reserve(ranges_m.size());
    double optical_depth = 0.0;
    double step_km = ranges_m.size() > 1 ? (ranges_m[1] - ranges_m[0]) / 1000.0 : 0.05;

    for (std::size_t index = 0; index < ranges_m.size(); ++index) {
        // 累积光学厚度，对应 LiDAR 方程双程透过率 exp(-2τ)
        optical_depth += true_extinction[index] * step_km;
        double range_km = ranges_m[index] / 1000.0;
        // 离散化 LiDAR 方程：C·E·O·β·exp(-2τ)/r²
        double signal = system_constant * laser_energy_mj * overlap[index] * true_backscatter[index]
            * std::exp(-2.0 * optical_depth) / std::max(range_km * range_km, 1e-6);
        // 噪声模型：取 max(背景 7%, 信号 5%) 作为高斯标准差，反映散粒噪声与背景暗计数主导两种工况
        double noise_sigma = std::max(background_counts * 0.07, signal * 0.05);
        // 信号 + 背景 + 噪声，下界为背景 + 0.1 防止负计数
        double noisy_signal = std::max(signal + background_counts + sample_gaussian(rng, 0.0, noise_sigma), background_counts + 0.1);
        raw_counts.push_back(noisy_signal);
    }
    return raw_counts;
}

/**
 * @brief 一次完整观测战役的输入聚合体：站点、原始 LiDAR 廓线、地面观测、来源元数据。
 */
struct CampaignData {
    SiteInfo site;
    std::vector<LidarProfile> profiles;
    std::vector<GroundMeasurement> ground_measurements;
    Json source_metadata;
};

/**
 * @brief 生成一整次合成观测战役（CampaignData），含 stare 与 PPI 两类扫描。
 *
 * 该函数是 demo 模式的核心：按时间步循环为每个时刻生成
 *   - 一条 zenith stare（仰角 90°，方位 0°）用于 PM 反演锚定；
 *   - 一组 PPI（仰角固定，方位角 0~360° 按步进）用于热点检测；
 *   - 一条地面观测，用于训练/校准 PM 模型。
 * 各气象量沿 time_phase 做正余弦周期变化并叠加随机扰动，模拟典型日变化。
 *
 * @param[in] config 流水线配置（含 simulation 段：seed、time_steps、ppi 步进、lidar_ratio 等）。
 * @return 完整观测战役数据（站点 + 廓线 + 地面观测 + 来源元数据）。
 */
CampaignData simulate_campaign(const PipelineConfig& config) {
    CampaignData data;
    data.site = config.site;
    // 若未显式指定 site_id，则由站点名 slugify 生成稳定标识
    data.site.site_id = data.site.site_id.empty() ? slugify(data.site.name) : data.site.site_id;

    std::mt19937 rng(config.simulation.seed);
    // 基准时间 2026-05-30 08:00（本地），作为循环起始
    std::tm base_tm{};
    base_tm.tm_year = 2026 - 1900;
    base_tm.tm_mon = 4;
    base_tm.tm_mday = 30;
    base_tm.tm_hour = 8;
    base_tm.tm_min = 0;
    base_tm.tm_isdst = -1;
    std::time_t base_time = std::mktime(&base_tm);

    // 距离 bin：均匀分布，斜距 = range_bin_m * (i+1)
    std::vector<double> ranges_m;
    for (int index = 0; index < config.simulation.range_bin_count; ++index) {
        ranges_m.push_back(config.simulation.range_bin_m * static_cast<double>(index + 1));
    }
    std::vector<double> overlap = build_overlap(ranges_m);

    for (int step_index = 0; step_index < config.simulation.time_steps; ++step_index) {
        std::string timestamp = format_timestamp(base_time + static_cast<long long>(step_index * config.simulation.minutes_per_step * 60));
        // 时间相位：0~2π 一个完整周期
        double time_phase = 2.0 * std::numbers::pi * static_cast<double>(step_index) / std::max(config.simulation.time_steps, 1);
        // 湿度：约 0.48 + 日振幅 0.20，并裁剪到 [0.28, 0.90] 合理范围
        double relative_humidity = clamp(0.48 + 0.20 * std::sin(time_phase - 0.6) + sample_gaussian(rng, 0.0, 0.015), 0.28, 0.90);
        // 温度：日变化约 ±7°C，基线 27°C，相位偏移 -0.2 错峰模拟典型午后最高
        double temperature_c = 27.0 - 7.0 * std::sin(time_phase - 0.2) + sample_gaussian(rng, 0.0, 0.3);
        // 风速：基线 2.8 m/s，截断防止出现 0 或过大值
        double wind_speed_ms = clamp(2.8 + 1.2 * std::cos(time_phase + 0.3) + sample_gaussian(rng, 0.0, 0.2), 0.6, 6.5);
        // 风向：以 120° 为基线做日摆动，加 360° 取模防止负值
        double wind_dir_deg = std::fmod(120.0 + 45.0 * std::sin(time_phase) + sample_gaussian(rng, 0.0, 4.0) + 360.0, 360.0);
        // 背景计数与激光能量：基线 + 小幅随机抖动，反映仪器状态
        double background_counts = 10.5 + sample_gaussian(rng, 0.0, 0.5);
        double laser_energy_mj = 1.0 + sample_gaussian(rng, 0.0, 0.03);

        // -- zenith stare 廓线：用于 PM 反演真值锚定 --
        SimulatedFields stare_fields = simulate_profile_fields(ranges_m, 0.0, 90.0, step_index, config.simulation.time_steps, relative_humidity, config.simulation.lidar_ratio_sr);
        std::vector<double> stare_raw_counts = simulate_raw_counts(
            stare_fields.true_backscatter,
            stare_fields.true_extinction,
            overlap,
            ranges_m,
            laser_energy_mj,
            background_counts,
            config.simulation.system_constant,
            rng
        );

        data.profiles.push_back(LidarProfile{
            data.site.site_id,
            timestamp,
            timestamp + "_stare",
            "stare",
            "synthetic_stare",
            0.0,
            90.0,
            ranges_m,
            stare_raw_counts,
            laser_energy_mj,
            background_counts,
            overlap,
            relative_humidity,
            temperature_c,
            wind_speed_ms,
            wind_dir_deg,
            stare_fields.molecular_backscatter,
            stare_fields.molecular_extinction,
            stare_fields.true_backscatter,
            stare_fields.true_extinction,
            stare_fields.true_pm25,
            stare_fields.true_pm10,
            stare_fields.true_hotspot_mask,
        });

        // -- PPI 扫描：方位角 0~360° 步进，用于热点检测 --
        std::vector<LidarProfile> ppi_profiles_for_timestamp;
        for (double azimuth = 0.0; azimuth < 360.0 - 1e-9; azimuth += config.simulation.ppi_azimuth_step_deg) {
            SimulatedFields ppi_fields = simulate_profile_fields(
                ranges_m,
                azimuth,
                config.simulation.ppi_elevation_deg,
                step_index,
                config.simulation.time_steps,
                relative_humidity,
                config.simulation.lidar_ratio_sr
            );
            std::vector<double> ppi_raw_counts = simulate_raw_counts(
                ppi_fields.true_backscatter,
                ppi_fields.true_extinction,
                overlap,
                ranges_m,
                laser_energy_mj,
                background_counts,
                config.simulation.system_constant,
                rng
            );
            // scan_id 中按方位角填充前导 0，便于在文件系统排序按 000~355 对齐
            LidarProfile profile{
                data.site.site_id,
                timestamp,
                timestamp + "_ppi_" + (azimuth < 10.0 ? "00" : (azimuth < 100.0 ? "0" : "")) + std::to_string(static_cast<int>(azimuth)),
                "ppi",
                "synthetic_ppi",
                azimuth,
                config.simulation.ppi_elevation_deg,
                ranges_m,
                ppi_raw_counts,
                laser_energy_mj,
                background_counts,
                overlap,
                relative_humidity,
                temperature_c,
                wind_speed_ms,
                wind_dir_deg,
                ppi_fields.molecular_backscatter,
                ppi_fields.molecular_extinction,
                ppi_fields.true_backscatter,
                ppi_fields.true_extinction,
                ppi_fields.true_pm25,
                ppi_fields.true_pm10,
                ppi_fields.true_hotspot_mask,
            };
            data.profiles.push_back(profile);
            ppi_profiles_for_timestamp.push_back(profile);
        }

        // -- 由真值消光反推地面观测的 PM（用于校准）--
        // 取 stare 前 6 个 bin 的真值干消光均值作为近地面代表（除以湿度增长还原为"干"值）
        std::vector<double> near_surface_values;
        for (int index = 0; index < std::min<int>(6, static_cast<int>(stare_fields.true_extinction.size())); ++index) {
            double humid_factor = 1.0 + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;
            // 除以吸湿因子：还原为"干"消光，避免双倍计入湿度
            near_surface_values.push_back(stare_fields.true_extinction[index] / humid_factor);
        }
        double near_surface_dry = mean(near_surface_values);
        // 在所有 PPI 方位中找最近 6 bin 的干消光峰值，作为该时刻的污染热点代理
        double hotspot_proxy = 0.0;
        for (const auto& profile : ppi_profiles_for_timestamp) {
            double local_peak = 0.0;
            for (int index = 0; index < std::min<int>(6, static_cast<int>(profile.true_extinction.size())); ++index) {
                double humid_factor = 1.0 + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;
                local_peak = std::max(local_peak, profile.true_extinction[index] / humid_factor);
            }
            hotspot_proxy = std::max(hotspot_proxy, local_peak);
        }

        // 地面 PM 反推：经验线性模型，含近地面干消光、热点代理、湿度项，加降噪
        // 其中 0.24·RH·100 表示湿度对颗粒物计数的额外贡献（吸湿后颗粒物质量被高估）
        data.ground_measurements.push_back(GroundMeasurement{
            data.site.site_id,
            timestamp,
            std::max(24.0, 430.0 * near_surface_dry + 305.0 * hotspot_proxy + 0.24 * relative_humidity * 100.0 + sample_gaussian(rng, 0.0, 1.8)),
            std::max(34.0, 610.0 * near_surface_dry + 410.0 * hotspot_proxy + 0.31 * relative_humidity * 100.0 + sample_gaussian(rng, 0.0, 2.4)),
            relative_humidity,
            temperature_c,
            wind_speed_ms,
            wind_dir_deg,
        });
    }

    // 来源元数据：记录该战役为纯仿真模式
    int ppi_count = 0;
    for (const auto& profile : data.profiles) {
        if (profile.scan_mode == "ppi") {
            ++ppi_count;
        }
    }
    data.source_metadata = Json::object_type{
        {"mode", "simulation"},
        {"site_id", data.site.site_id},
        {"site_name", data.site.name},
        {"real_stare_profile_count", 0},
        {"synthetic_ppi_profile_count", ppi_count},
    };
    return data;
}

#if defined(LIDAR_DEMO_HAS_NETCDF)

CampaignData load_cloudnet_hybrid_campaign(const PipelineConfig& config) {
    std::filesystem::path source_root = std::filesystem::absolute(config.source.root.empty() ? std::filesystem::path(".") : std::filesystem::path(config.source.root));
    std::filesystem::path local_file = source_root / config.source.cloudnet.local_file;
    std::string day_token = strip_hyphens(config.source.cloudnet.date);
    std::filesystem::path ground_json_path = source_root / "data" / "public" / "cloudnet" / ("open_meteo_" + day_token + "_ground_pm_meteo.json");
    if (!std::filesystem::exists(local_file) || !std::filesystem::exists(ground_json_path)) {
        fetch_cloudnet_public_sample(config, ".");
    }
    if (!std::filesystem::exists(local_file)) {
        throw std::runtime_error("Cloudnet .nc file not found for C++ path after download attempt: " + local_file.string());
    }
    if (!std::filesystem::exists(ground_json_path)) {
        throw std::runtime_error("Aligned Open-Meteo ground file not found for Cloudnet C++ path after fetch attempt: " + ground_json_path.string());
    }

    NetcdfFile dataset(netcdf_compatible_path(local_file));
    int ncid = dataset.id();
    int time_varid = variable_id(ncid, "time");
    int beta_varid = variable_id(ncid, "beta");
    std::string time_units = read_text_attribute(ncid, time_varid, "units");
    std::vector<double> time_values = read_numeric_variable(ncid, "time");
    std::vector<double> range_values_m = read_numeric_variable(ncid, "range");
    double zenith_angle_deg = read_optional_scalar_numeric_variable(ncid, "zenith_angle", 0.0);
    double elevation_deg = 90.0 - zenith_angle_deg;

    CampaignData data;
    data.site = SiteInfo{
        config.source.cloudnet.site_name,
        read_scalar_numeric_variable(ncid, "latitude"),
        read_scalar_numeric_variable(ncid, "longitude"),
        read_scalar_numeric_variable(ncid, "altitude"),
        config.source.cloudnet.site_id,
    };

    std::vector<GroundRecord> ground_records = read_ground_records(ground_json_path);

    std::vector<int> time_indices = select_even_indices(static_cast<int>(time_values.size()), config.source.cloudnet.time_steps);
    std::vector<int> range_indices = filter_range_indices(
        range_values_m,
        config.source.cloudnet.min_range_m,
        config.source.cloudnet.max_range_m,
        config.source.cloudnet.range_bin_count
    );
    std::vector<double> ranges_m;
    ranges_m.reserve(range_indices.size());
    for (int index : range_indices) {
        ranges_m.push_back(range_values_m[static_cast<std::size_t>(index)]);
    }
    std::vector<double> overlap = build_overlap(ranges_m);
    auto [molecular_extinction, molecular_backscatter] = standard_molecular_fields(ranges_m, data.site.altitude_m, elevation_deg);

    for (int time_index : time_indices) {
        std::string timestamp = cloudnet_time_to_iso_minute(time_values[static_cast<std::size_t>(time_index)], time_units);
        const GroundRecord& matched = nearest_ground_record(parse_timestamp(timestamp), ground_records);
        data.ground_measurements.push_back(GroundMeasurement{
            config.source.cloudnet.site_id,
            timestamp,
            matched.pm25,
            matched.pm10,
            matched.relative_humidity / 100.0,
            matched.temperature_c,
            matched.wind_speed_ms,
            matched.wind_dir_deg,
        });
    }

    std::map<std::string, GroundMeasurement> ground_by_timestamp;
    for (const auto& measurement : data.ground_measurements) {
        ground_by_timestamp[measurement.timestamp] = measurement;
    }

    for (std::size_t order = 0; order < time_indices.size(); ++order) {
        int time_index = time_indices[order];
        std::string timestamp = cloudnet_time_to_iso_minute(time_values[static_cast<std::size_t>(time_index)], time_units);
        const GroundMeasurement& ground = ground_by_timestamp.at(timestamp);
        std::vector<double> beta_values_full = read_beta_row(ncid, beta_varid, time_index, range_values_m.size());
        std::vector<double> beta_values;
        beta_values.reserve(range_indices.size());
        for (int index : range_indices) {
            beta_values.push_back(beta_values_full[static_cast<std::size_t>(index)]);
        }

        double background_counts = 5.0 + 0.15 * static_cast<double>(order % 5);
        double laser_energy_mj = 1.0 + 0.01 * std::sin(static_cast<double>(order));
        std::vector<double> raw_counts;
        std::vector<double> approx_extinction;
        std::vector<double> combined_backscatter;
        raw_counts.reserve(beta_values.size());
        approx_extinction.reserve(beta_values.size());
        combined_backscatter.reserve(beta_values.size());
        for (std::size_t index = 0; index < beta_values.size(); ++index) {
            double range_km = ranges_m[index] / 1000.0;
            raw_counts.push_back(background_counts + beta_values[index] * config.source.cloudnet.pseudo_signal_scale * overlap[index] / std::max(range_km * range_km, 1e-6));
            double aerosol_ext = beta_values[index] * config.retrieval.aerosol_lidar_ratio_sr;
            approx_extinction.push_back(molecular_extinction[index] + aerosol_ext);
            combined_backscatter.push_back(molecular_backscatter[index] + beta_values[index]);
        }

        data.profiles.push_back(LidarProfile{
            config.source.cloudnet.site_id,
            timestamp,
            timestamp + "_stare_real",
            "stare",
            "cloudnet_real_stare",
            0.0,
            elevation_deg,
            ranges_m,
            raw_counts,
            laser_energy_mj,
            background_counts,
            overlap,
            ground.relative_humidity,
            ground.temperature_c,
            ground.wind_speed_ms,
            ground.wind_dir_deg,
            molecular_backscatter,
            molecular_extinction,
            combined_backscatter,
            approx_extinction,
            std::vector<double>(ranges_m.size(), 0.0),
            std::vector<double>(ranges_m.size(), 0.0),
            std::vector<int>(ranges_m.size(), 0),
        });
    }

    std::mt19937 rng(config.simulation.seed + 101);
    for (std::size_t step_index = 0; step_index < data.ground_measurements.size(); ++step_index) {
        const GroundMeasurement& ground = data.ground_measurements[step_index];
        for (double azimuth = 0.0; azimuth < 360.0 - 1e-9; azimuth += config.simulation.ppi_azimuth_step_deg) {
            SimulatedFields fields = simulate_profile_fields(
                ranges_m,
                azimuth,
                config.simulation.ppi_elevation_deg,
                static_cast<int>(step_index),
                static_cast<int>(data.ground_measurements.size()),
                ground.relative_humidity,
                config.simulation.lidar_ratio_sr
            );
            double background_counts = 10.2 + sample_gaussian(rng, 0.0, 0.35);
            double laser_energy_mj = 1.0 + sample_gaussian(rng, 0.0, 0.02);
            std::vector<double> raw_counts = simulate_raw_counts(
                fields.true_backscatter,
                fields.true_extinction,
                overlap,
                ranges_m,
                laser_energy_mj,
                background_counts,
                config.simulation.system_constant,
                rng
            );
            data.profiles.push_back(LidarProfile{
                config.source.cloudnet.site_id,
                ground.timestamp,
                ground.timestamp + "_ppi_" + (azimuth < 10.0 ? "00" : (azimuth < 100.0 ? "0" : "")) + std::to_string(static_cast<int>(azimuth)),
                "ppi",
                "synthetic_ppi_hybrid",
                azimuth,
                config.simulation.ppi_elevation_deg,
                ranges_m,
                raw_counts,
                laser_energy_mj,
                background_counts,
                overlap,
                ground.relative_humidity,
                ground.temperature_c,
                ground.wind_speed_ms,
                ground.wind_dir_deg,
                fields.molecular_backscatter,
                fields.molecular_extinction,
                fields.true_backscatter,
                fields.true_extinction,
                fields.true_pm25,
                fields.true_pm10,
                fields.true_hotspot_mask,
            });
        }
    }

    std::stable_sort(data.profiles.begin(), data.profiles.end(), [](const LidarProfile& left, const LidarProfile& right) {
        if (left.timestamp != right.timestamp) {
            return left.timestamp < right.timestamp;
        }
        if (left.scan_mode != right.scan_mode) {
            return left.scan_mode < right.scan_mode;
        }
        return left.azimuth_deg < right.azimuth_deg;
    });

    int ppi_count = 0;
    for (const auto& profile : data.profiles) {
        if (profile.scan_mode == "ppi") {
            ++ppi_count;
        }
    }
    data.source_metadata = Json::object_type{
        {"mode", "cloudnet_hybrid"},
        {"site_id", data.site.site_id},
        {"site_name", data.site.name},
        {"cloudnet_file", local_file.string()},
        {"measurement_date", config.source.cloudnet.date},
        {"ground_provider", "open-meteo"},
        {"real_stare_profile_count", static_cast<int>(data.ground_measurements.size())},
        {"synthetic_ppi_profile_count", ppi_count},
    };
    return data;
}

#endif

// ====================================================================================
// ---- L1 预处理 / L1 preprocessing ----------------------------------------------------------------
// 将探测器原始计数的"伪信号"还原为可比对的物理量：
// 1. 减背景 → 2. 除以激光能量归一 → 3. 除 overlap → 4. 乘 r² 校正几何发散 → 衰减后向散射。
// 同时输出 SNR 与质控标志（近场 partial overlap、激光能量过低、近场 SNR 过低）。
// ====================================================================================

/**
 * @brief 对单条 LiDAR 廓线执行 L1 预处理。
 *
 * 处理链路：
 *   background_corrected = max(raw - background, 0)
 *   energy_normalized    = background_corrected / laser_energy
 *   overlap_corrected    = energy_normalized / overlap(r)
 *   attenuated_backscatter = overlap_corrected · r²
 *   SNR                  = background_corrected / sqrt(raw + background)
 *
 * 其中乘以 r² 反转了 LiDAR 方程中的 1/r² 几何衰减（未校正透过率，因此仍含双程大气透过率，称为"衰减"后向散射）。
 * SNR 用 Poisson 噪声近似（σ=√N），用于决定哪一段 bin 可进入反演。
 *
 * @param[in] profile 单条原始 LiDAR 廓线。
 * @return 预处理结果（L1 信号、衰减后向散射、SNR、QC 标志）。
 */
PreprocessResult preprocess_profile(const LidarProfile& profile) {
    PreprocessResult output;
    for (std::size_t index = 0; index < profile.raw_counts.size(); ++index) {
        // 减背景计数，并截断到 0，避免负值
        double background_corrected = std::max(profile.raw_counts[index] - profile.background_counts, 0.0);
        // 按脉冲能量归一，抵消每次发射能量抖动
        double energy_normalized = background_corrected / std::max(profile.laser_energy_mj, 1e-6);
        // 修正双轴 overlap，还原真实接收信号（下限 0.15 避免近场过度放大）
        double overlap_corrected = energy_normalized / std::max(profile.overlap[index], 0.15);
        double range_km = profile.ranges_m[index] / 1000.0;
        // 乘 r² 抵消 LiDAR 方程中的几何扩散衰减（至此得到含大气透过率的衰减后向散射）
        double attenuated = overlap_corrected * range_km * range_km;
        // 信噪比：基于 Poisson 计数噪声近似 σ ≈ √(raw + background)
        double signal_to_noise = background_corrected / std::max(std::sqrt(profile.raw_counts[index] + profile.background_counts), 1.0);

        output.l1_signal.push_back(background_corrected);
        output.attenuated_backscatter.push_back(std::max(attenuated, 1e-9));
        output.snr.push_back(signal_to_noise);
    }

    // ---- 质控标志 / QC flags ----
    // 近场 overlap 过低：前 3 个 bin 最小 overlap < 0.4 表示近场盲区
    if (profile.overlap.size() >= 3 && *std::min_element(profile.overlap.begin(), profile.overlap.begin() + 3) < 0.4) {
        output.qc_flags.push_back("near-range-partial-overlap");
    }
    // 激光能量过低：脉冲退化或老化导致信噪下降
    if (profile.laser_energy_mj < 0.93) {
        output.qc_flags.push_back("low-laser-energy");
    }
    // 近场 SNR 过弱：前 4 bin 平均 SNR < 3 通常意味着近距离回波质量不可用
    if (output.snr.size() >= 4 && mean(std::vector<double>(output.snr.begin(), output.snr.begin() + 4)) < 3.0) {
        output.qc_flags.push_back("weak-near-range-snr");
    }
    return output;
}

// ====================================================================================
// ---- Fernald 反演 / Fernald inversion ------------------------------------------------------------
// 由"衰减后向散射"反推气溶胶消光与后向散射廓线。
// 因 LiDAR 方程求解是 ill-posed（含双程透过率 exp 项），需要远端参考点锚定边界。
// 本实现采用从远端到近端的反向积分法。
// ====================================================================================

/**
 * @brief 对衰减后向散射执行 Fernald 反演，求气溶胶消光与气溶胶后向散射廓线。
 *
 * 利用远端（远场）假设：远端处气溶胶很弱，可以认为信号近似对应纯分子+少量气溶胶参考值。
 * 算法步骤：
 *   1. 取末端 5 个 bin 平均信号 ref_signal，配合参考点后向散射 ref_beta 求标定系数 scale = ref_beta / ref_signal；
 *   2. 将整段衰减后向散射乘 scale 得到估计的总后向散射；
 *   3. 自远端向近端反向迭代：先以透过率 exp(2τ) 复原（注意这里用 exp(+2τ)，因为衰减后向散射是被透过率压缩过的），
 *      再减去分子后向散射得到气溶胶 β；
 *   4. 由 β·S（气溶胶激光比）计算气溶胶消光，加上分子消光得到总消光；
 *   5. 限制总消光在 [分子消光, 0.45] 之间，防止数值不稳。
 *
 * @param[in] profile 单条 LiDAR 廓线（用于读取分子后向散射/消光与距离 bin）。
 * @param[in] attenuated_backscatter 预处理后的衰减后向散射廓线。
 * @param[in] aerosol_lidar_ratio_sr 气溶胶激光比（sr），典型 PM 用 40~60 sr。
 * @param[in] reference_aerosol_backscatter 远端参考点的气溶胶后向散射（1/m·sr），通常取很小的本底值。
 * @return (总消光, 气溶胶后向散射) 廓线对。
 */
std::pair<std::vector<double>, std::vector<double>> run_fernald_inversion(
    const LidarProfile& profile,
    const std::vector<double>& attenuated_backscatter,
    double aerosol_lidar_ratio_sr,
    double reference_aerosol_backscatter
) {
    // 参考点：取末端 5 个 bin（远场气溶胶稀薄）作为锚定
    int ref_index = std::max<int>(static_cast<int>(attenuated_backscatter.size()) - 5, 0);
    std::vector<double> ref_signal_slice(attenuated_backscatter.begin() + ref_index, attenuated_backscatter.end());
    std::vector<double> ref_beta_slice(profile.molecular_backscatter.begin() + ref_index, profile.molecular_backscatter.end());
    double ref_signal = std::max(mean(ref_signal_slice), 1e-9);
    // 参考点后向散射：分子 + 给定的小量气溶胶本底
    double ref_beta = mean(ref_beta_slice) + reference_aerosol_backscatter;
    // 尺度因子：把后续衰减后向散射缩放到近似总后向散射
    double scale = ref_beta / ref_signal;

    std::vector<double> extinction(attenuated_backscatter.size(), 0.0);
    std::vector<double> aerosol_backscatter(attenuated_backscatter.size(), 0.0);
    std::vector<double> scaled_signal;
    scaled_signal.reserve(attenuated_backscatter.size());
    for (double value : attenuated_backscatter) {
        scaled_signal.push_back(std::max(value * scale, 1e-9));
    }
    double step_km = profile.ranges_m.size() > 1 ? (profile.ranges_m[1] - profile.ranges_m[0]) / 1000.0 : 0.05;
    double optical_depth = 0.0;
    // 反向积分：从远端（信号最弱且气溶胶稀薄）向近端反推
    for (int index = static_cast<int>(scaled_signal.size()) - 1; index >= 0; --index) {
        // 复原总后向散射：衰减信号 × exp(+2τ) 抵消 LiDAR 方程的双程透过率
        double total_backscatter = std::max(scaled_signal[index] * std::exp(2.0 * optical_depth), profile.molecular_backscatter[index]);
        // 气溶胶后向散射 = 总 - 分子
        double aerosol_beta = std::max(total_backscatter - profile.molecular_backscatter[index], 0.0);
        // 总消光 = 分子消光 + S·气溶胶 β（S 为气溶胶激光比）
        double total_extinction = profile.molecular_extinction[index] + aerosol_lidar_ratio_sr * aerosol_beta;
        // 限制总消光：下限为分子消光（不允许低于纯大气），上限 0.45（防止数值爆炸）
        total_extinction = std::min(std::max(total_extinction, profile.molecular_extinction[index]), 0.45);
        aerosol_backscatter[index] = aerosol_beta;
        extinction[index] = total_extinction;
        // 累积光学厚度 τ（向近端积分）
        optical_depth += total_extinction * step_km;
    }
    return {extinction, aerosol_backscatter};
}

// ====================================================================================
// ---- 湿度校正 / Humidity correction ----------------------------------------------------------------
// 相对湿度升高会使气溶胶颗粒吸水增长（κ-Köhler 物理过程），增强其散射/消光截面。
// 反演过程中如果直接使用"湿"消光，会用错误的 PM 浓度。本节按典型吸湿增长因子将湿消光除回"干"参考状态。
// ====================================================================================

/**
 * @brief 计算吸湿增长因子 g(RH)（无量纲）。
 *
 * 采用简化的 κ-Köhler 形式：
 *   g(RH) = 1 + κ · max( RH/(1-RH) - RH_dry/(1-RH_dry), 0 ) · 0.18
 * 其中 RH/(1-RH) 称为水分活度对应量，κ 反映气溶胶化学组分吸湿性（硫酸盐大、矿尘小）。
 * 因子 0.18 为压缩系数（防止 g 增长过快超出消光实测范围）。
 *
 * @param[in] relative_humidity 当前 RH（0~1）。
 * @param[in] dry_reference_rh 干参考状态的 RH（约 0.35~0.4），即认为是"无吸湿"基线。
 * @param[in] hygroscopicity 吸湿性参数 κ（典型 0.1~0.5）。
 * @return 增长因子（≥ 1）。
 */
double growth_factor(double relative_humidity, double dry_reference_rh, double hygroscopicity) {
    // RH 夹到 [0.05, 0.98]：过低无意义，过高会导致 1-RH 发散
    double rh = std::min(std::max(relative_humidity, 0.05), 0.98);
    double dry_ratio = dry_reference_rh / std::max(1.0 - dry_reference_rh, 0.02);
    double humid_ratio = rh / std::max(1.0 - rh, 0.02);
    return std::max(1.0, 1.0 + hygroscopicity * std::max(humid_ratio - dry_ratio, 0.0) * 0.18);
}

/**
 * @brief 对消光廓线做吸湿校正，从"湿"消光还原为"干"消光。
 *
 * 数学：干消光 = 湿消光 / g(RH)。后续 PM 反演使用干消光更稳定（去除了 RH 引起的虚假变化）。
 *
 * @param[in] extinction 原始（湿）消光廓线。
 * @param[in] relative_humidity 当前 RH。
 * @param[in] dry_reference_rh 干参考 RH。
 * @param[in] hygroscopicity 吸湿参数 κ。
 * @return 逐 bin 的干消光廓线。
 */
std::vector<double> apply_humidity_correction(const std::vector<double>& extinction, double relative_humidity, double dry_reference_rh, double hygroscopicity) {
    double factor = growth_factor(relative_humidity, dry_reference_rh, hygroscopicity);
    std::vector<double> output;
    output.reserve(extinction.size());
    for (double value : extinction) {
        output.push_back(value / factor);
    }
    return output;
}

// ====================================================================================
// ---- ENU 坐标转换 / ENU coordinate conversion ----------------------------------------------------
// 将 LiDAR 量测的距离 bin 与方位/仰角转换为以站点为原点的 ENU（East-North-Up）笛卡尔坐标，
// 用于三维热点定位与可视化。
// ====================================================================================

/**
 * @brief 把每条距离 bin 转换为 ENU 三维点。
 *
 * 转换公式（球坐标 → 直角 ENU）：
 *   East  = r · cos(elev) · sin(azim)
 *   North = r · cos(elev) · cos(azim)
 *   Up    = r · sin(elev)
 * 其中仰角沿天顶为 90°，方位角沿正北为 0° 顺时针增加。
 *
 * @param[in] profile 含方位角、仰角、距离的 LiDAR 廓线。
 * @return ENU 坐标点列表（每行 [East_m, North_m, Up_m]）。
 */
std::vector<std::vector<double>> profile_bins_to_enu(const LidarProfile& profile) {
    std::vector<std::vector<double>> output;
    output.reserve(profile.ranges_m.size());
    double azimuth_rad = profile.azimuth_deg * std::numbers::pi / 180.0;
    double elevation_rad = profile.elevation_deg * std::numbers::pi / 180.0;
    for (double range_m : profile.ranges_m) {
        // 水平投影 = r · cos(elev)，再分解到 East/North
        double east = range_m * std::cos(elevation_rad) * std::sin(azimuth_rad);
        double north = range_m * std::cos(elevation_rad) * std::cos(azimuth_rad);
        // 铅垂分量 = r · sin(elev)
        double up = range_m * std::sin(elevation_rad);
        output.push_back({east, north, up});
    }
    return output;
}

// ====================================================================================
// ---- PM 校准特征表 / PM calibration feature table -------------------------------------------------
// 将一次完整战役中按时间戳聚合：合并 stare 反演的近地面干消光、衰减后向散射、
// 以及同时间戳下 PPI 的 hotspot 代理量，并匹配地面站真值，得到用于校准 PM 模型的特征向量。
// ====================================================================================

/**
 * @brief 按时间戳构建 PM 校准用的特征样本表。
 *
 * 流程：
 *   1. 把地面观测按时间戳建立索引；
 *   2. 把 processed_profiles 按时间戳分离为 stare / PPI 两个子集；
 *   3. 对每个有 ground 配对的 stare 时间戳：
 *      - 取近地面若干 bin（surface_bin_count）的干消光均值作为 surface feature；
 *      - 若有 PPI，用所有 PPI 方位中 surface 切片的最大干消光作为 hotspot_proxy；
 *      - 否则用 stare 切片的最大值代替 hotspot_proxy；
 *      - 组装特征向量 [1.0 (截距), 干消光均值, RH, hotspot_proxy]，并附 PM2.5/PM10 真值；
 *
 * @param[in] processed_profiles 已反演的所有处理廓线（含 stare 与 PPI）。
 * @param[in] ground_measurements 同步期地面观测序列。
 * @param[in] surface_bin_count 近地面 bin 数量（一般是 6 个，对应近 200 m）。
 * @return 时间序列特征样本列表，每条对应一个时刻。
 */
std::vector<FeatureSample> build_timestamp_feature_table(
    const std::vector<ProcessedProfile>& processed_profiles,
    const std::vector<GroundMeasurement>& ground_measurements,
    int surface_bin_count
) {
    // 1) 按 timestamp 索引地面观测
    std::map<std::string, GroundMeasurement> ground_by_timestamp;
    for (const auto& measurement : ground_measurements) {
        ground_by_timestamp[measurement.timestamp] = measurement;
    }

    // 2) 按 timestamp 分离 stare/PPI 已处理廓线（用指针避免深拷贝）
    std::map<std::string, std::vector<const ProcessedProfile*>> stare_by_timestamp;
    std::map<std::string, std::vector<const ProcessedProfile*>> ppi_by_timestamp;
    for (const auto& processed : processed_profiles) {
        if (processed.profile.scan_mode == "stare") {
            stare_by_timestamp[processed.profile.timestamp].push_back(&processed);
        } else if (processed.profile.scan_mode == "ppi") {
            ppi_by_timestamp[processed.profile.timestamp].push_back(&processed);
        }
    }

    std::vector<FeatureSample> samples;
    for (const auto& [timestamp, stare_profiles] : stare_by_timestamp) {
        // 仅当存在地面观测时才生成样本（无 ground 无法训练）
        if (stare_profiles.empty() || !ground_by_timestamp.contains(timestamp)) {
            continue;
        }
        const auto& ground = ground_by_timestamp.at(timestamp);
        const auto* stare_profile = stare_profiles.front();
        // 取近地面 bin 的干消光与衰减后向散射切片
        std::vector<double> stare_slice(stare_profile->dry_extinction.begin(), stare_profile->dry_extinction.begin() + std::min<int>(surface_bin_count, static_cast<int>(stare_profile->dry_extinction.size())));
        std::vector<double> attenuated_slice(stare_profile->attenuated_backscatter.begin(), stare_profile->attenuated_backscatter.begin() + std::min<int>(surface_bin_count, static_cast<int>(stare_profile->attenuated_backscatter.size())));
        // 热点代理：若 PPI 存在，取所有方位 surface 切片的最大值；否则退回 stare 切片最大值
        double hotspot_proxy = 0.0;
        if (ppi_by_timestamp.contains(timestamp)) {
            for (const auto* profile : ppi_by_timestamp.at(timestamp)) {
                hotspot_proxy = std::max(hotspot_proxy, *std::max_element(profile->dry_extinction.begin(), profile->dry_extinction.begin() + std::min<int>(surface_bin_count, static_cast<int>(profile->dry_extinction.size()))));
            }
        } else {
            hotspot_proxy = *std::max_element(stare_slice.begin(), stare_slice.end());
        }
        // features = [截距 1.0, 近地面干消光均值, RH, hotspot_proxy]
        samples.push_back(FeatureSample{
            timestamp,
            {1.0, mean(stare_slice), ground.relative_humidity, hotspot_proxy},
            mean(stare_slice),
            mean(attenuated_slice),
            hotspot_proxy,
            !stare_profile->profile.site_id.empty() ? stare_profile->profile.site_id : (!ground.site_id.empty() ? ground.site_id : "default-site"),
            ground.pm25_ugm3,
            ground.pm10_ugm3,
            ground.relative_humidity,
            ground.wind_speed_ms,
        });
    }
    return samples;
}

// ====================================================================================
// ---- 最小二乘求解 / Linear algebra primitives for OLS ---------------------------------------------
// PM 校准本质上是最小二乘问题 min ||Xw - y||²，其闭式解的对应正规方程为 X^T X w = X^T y。
// 这里用 Gauss-Jordan 全主元消元解该线性方程组，并给 Gram 矩阵加微小正则项防止奇异。
// ====================================================================================

/**
 * @brief 用 Gauss-Jordan 全主元消元法解线性方程组 A·x = b。
 *
 * 实现细节：
 *   - 把 b 拼接到 A 末列形成增广矩阵；
 *   - 每个主元位置选列中绝对值最大者作行交换（部分主元），改善数值稳定性；
 *   - 主元归一后，把其他行该列清零，最终得到对角单位阵，最后一列即为解。
 *
 * @param[in] matrix 系数方阵 A（按值传递以便原地修改）。
 * @param[in] vector 右端向量 b。
 * @return 解向量 x，长度等于 vector.size()。
 * @throw std::runtime_error 当矩阵奇异（主元绝对值 < 1e-9）时抛出。
 */
std::vector<double> solve_linear_system(std::vector<std::vector<double>> matrix, std::vector<double> vector) {
    int size = static_cast<int>(vector.size());
    // 构造增广矩阵：把 b 拼到 A 的最后一列
    for (int row = 0; row < size; ++row) {
        matrix[row].push_back(vector[row]);
    }
    for (int pivot = 0; pivot < size; ++pivot) {
        // 选第 pivot 列下方最大绝对值作为新主元（部分主元法）
        int max_row = pivot;
        for (int row = pivot + 1; row < size; ++row) {
            if (std::abs(matrix[row][pivot]) > std::abs(matrix[max_row][pivot])) {
                max_row = row;
            }
        }
        std::swap(matrix[pivot], matrix[max_row]);
        double pivot_value = matrix[pivot][pivot];
        if (std::abs(pivot_value) < 1e-9) {
            throw std::runtime_error("Singular matrix encountered during PM calibration");
        }
        // 归一化主元行使主元为 1
        for (int column = pivot; column <= size; ++column) {
            matrix[pivot][column] /= pivot_value;
        }
        // 用主元行消去其他行该列
        for (int row = 0; row < size; ++row) {
            if (row == pivot) {
                continue;
            }
            double factor = matrix[row][pivot];
            for (int column = pivot; column <= size; ++column) {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
        }
    }

    // 增广矩阵最后一列即为解
    std::vector<double> solution(size, 0.0);
    for (int row = 0; row < size; ++row) {
        solution[row] = matrix[row][size];
    }
    return solution;
}

/**
 * @brief 普通最小二乘（OLS）线性回归：求解 X^T X · w = X^T y。
 *
 * 实现：
 *   - 累加 Gram 矩阵 G = X^T X 与右端项 h = X^T y；
 *   - 给对角线加微小正则 1e-6（岭回归项），避免 G 退化；
 *   - 调用 solve_linear_system 得到权重 w。
 *
 * @param[in] feature_matrix 设计矩阵 X，每行为一条样本的特征向量。
 * @param[in] target 对应的标签向量 y（每条样本一个标量）。
 * @return 权重向量 w，长度等于行内特征数。
 */
std::vector<double> fit_linear_regression(const std::vector<std::vector<double>>& feature_matrix, const std::vector<double>& target) {
    int feature_count = static_cast<int>(feature_matrix.front().size());
    std::vector<std::vector<double>> gram(feature_count, std::vector<double>(feature_count, 0.0));
    std::vector<double> rhs(feature_count, 0.0);
    // 累加正式方程 X^T X 与 X^T y
    for (std::size_t row = 0; row < feature_matrix.size(); ++row) {
        for (int left = 0; left < feature_count; ++left) {
            rhs[left] += feature_matrix[row][left] * target[row];
            for (int right = 0; right < feature_count; ++right) {
                gram[left][right] += feature_matrix[row][left] * feature_matrix[row][right];
            }
        }
    }
    // L2 正则项（岭回归）：对角加 1e-6 防止 G 不可逆
    for (int diagonal = 0; diagonal < feature_count; ++diagonal) {
        gram[diagonal][diagonal] += 1e-6;
    }
    return solve_linear_system(gram, rhs);
}

/**
 * @brief 用线性模型对单条样本打分：output = Σ coef_i · feat_i。
 *
 * @param[in] coefficients 权重（含截距对应首特征 1.0）。
 * @param[in] features 与系数等长的特征向量。
 * @return 预测值（如 PM2.5 浓度）。
 */
double predict(const std::vector<double>& coefficients, const std::vector<double>& features) {
    double output = 0.0;
    for (std::size_t index = 0; index < coefficients.size(); ++index) {
        output += coefficients[index] * features[index];
    }
    return output;
}

/**
 * @brief 把样本按模 5 分组划分为 train(3)/val(1)/test(1) 三段。
 *
 * 用 index % 5 而非随机抽样，避免演示数据中加入额外随机源；同时保证时间维度上的相对均匀，
 * 有助于后续漂移测试。若某段为空，从 train 末尾移入一条避免上报告错。
 *
 * @param[in] samples 待划分样本（按生成顺序）。
 * @return 三段样本及对应时间戳索引。
 */
CalibrationSplit split_samples(const std::vector<FeatureSample>& samples) {
    CalibrationSplit split;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        int bucket = static_cast<int>(index % 5);
        if (bucket <= 2) {
            // 桶 0,1,2 → 训练（60%）
            split.train.push_back(samples[index]);
        } else if (bucket == 3) {
            // 桶 3 → 验证（20%）
            split.val.push_back(samples[index]);
        } else {
            // 桶 4 → 测试（20%）
            split.test.push_back(samples[index]);
        }
    }
    // 兜底：测试/验证为空时从 train 末尾补一条
    if (split.val.empty() && !split.train.empty()) {
        split.val.push_back(split.train.back());
        split.train.pop_back();
    }
    if (split.test.empty() && !split.train.empty()) {
        split.test.push_back(split.train.back());
        split.train.pop_back();
    }
    // 顺手记下各自时间戳，供后续漂移监控使用
    for (const auto& sample : split.train) split.train_timestamps.push_back(sample.timestamp);
    for (const auto& sample : split.val) split.val_timestamps.push_back(sample.timestamp);
    for (const auto& sample : split.test) split.test_timestamps.push_back(sample.timestamp);
    return split;
}

/**
 * @brief 端到端 PM 模型校准：划分样本 → 训练 PM2.5/PM10 各一个线性模型。
 *
 * 这是 PM 反演的"训练"流程：
 *   1. 通过 split_samples 切分训练/验证/测试集；
 *   2. 用训练集的 [1.0, 干消光均值, RH, hotspot_proxy] 拟合两个目标（PM2.5、PM10）的最小二乘直线。
 *
 * @param[in] samples 时间戳特征样本列表（来自 build_timestamp_feature_table）。
 * @return (PM2.5/PM10 模型权重, 切分集合)。后续 apply_pm_models 会使用这些模型对整条廓线推 PM。
 */
std::pair<CalibrationModels, CalibrationSplit> fit_pm_models(const std::vector<FeatureSample>& samples) {
    CalibrationSplit split = split_samples(samples);
    std::vector<std::vector<double>> features;
    std::vector<double> pm25_target;
    std::vector<double> pm10_target;
    for (const auto& sample : split.train) {
        features.push_back(sample.features);
        pm25_target.push_back(sample.pm25_true);
        pm10_target.push_back(sample.pm10_true);
    }
    CalibrationModels models;
    models.pm25 = fit_linear_regression(features, pm25_target);
    models.pm10 = fit_linear_regression(features, pm10_target);
    return {models, split};
}

std::map<std::string, StationOffset> fit_station_offsets(
    const CalibrationModels& models,
    const CalibrationSplit& split
) {
    std::map<std::string, std::vector<double>> pm25_grouped;
    std::map<std::string, std::vector<double>> pm10_grouped;
    for (const auto& sample : split.train) {
        std::string site_id = sample.site_id.empty() ? "default-site" : sample.site_id;
        pm25_grouped[site_id].push_back(sample.pm25_true - predict(models.pm25, sample.features));
        pm10_grouped[site_id].push_back(sample.pm10_true - predict(models.pm10, sample.features));
    }

    std::map<std::string, StationOffset> output;
    for (const auto& [site_id, residuals] : pm25_grouped) {
        output[site_id] = StationOffset{
            mean(residuals),
            mean(pm10_grouped[site_id]),
            static_cast<int>(residuals.size()),
        };
    }
    return output;
}

// ---- PM 浓度反演与热点检测 ----

/**
 * @brief 把已拟合的 PM2.5/PM10 线性模型应用到每条已处理廓线上，得到逐距离门的颗粒物浓度。
 *
 * 算法流程：
 *  1. 按 timestamp 把特征表组织成哈希表，O(1) 查找每条廓线对应时刻的地面特征；
 *  2. 对每条廓线，确定其 site_id（特征样本优先，廓线次之，最后回退 default-site），
 *     从而取得该站点的偏置校准 StationOffset；
 *  3. 逐距离门构造特征向量 [1, dry_ext, RH, hotspot_proxy]，用线性模型预测基线 PM 值，
 *     再叠加站点偏置；
 *  4. 当本距离门干消光高于地面背景时，按 "excess × 1250" 的线性系数叠加额外热点增强，
 *     用于补偿远场局部污染团块；最后用 std::max(0, ...) 截断负值。
 *
 * @param processed_profiles 已完成的反演廓线（dry_extinction 已就绪），结果会写入 pm25/pm10 字段。
 * @param models             由 fit_pm_models 拟合得到的 PM2.5/PM10 线性模型。
 * @param feature_table      每个时间步的地面特征样本（含 hotspot_proxy 与 surface_dry_ext）。
 * @param station_offsets    站点 ID 到偏置校准的映射，缺省时使用零偏置。
 */
void apply_pm_models(
    std::vector<ProcessedProfile>& processed_profiles,
    const CalibrationModels& models,
    const std::vector<FeatureSample>& feature_table,
    const std::map<std::string, StationOffset>& station_offsets
) {
    // 用 timestamp 做索引，便于按时间步 O(1) 取到对应的地面特征
    std::map<std::string, FeatureSample> feature_by_timestamp;
    for (const auto& sample : feature_table) {
        feature_by_timestamp[sample.timestamp] = sample;
    }

    for (auto& processed : processed_profiles) {
        const auto& timestamp_features = feature_by_timestamp.at(processed.profile.timestamp);
        // 站点 ID 的优先级：地面特征 > 廓线自身 > 默认站点
        std::string site_id = !timestamp_features.site_id.empty() ? timestamp_features.site_id : (!processed.profile.site_id.empty() ? processed.profile.site_id : "default-site");
        StationOffset offset = station_offsets.contains(site_id) ? station_offsets.at(site_id) : StationOffset{};
        processed.pm25.clear();
        processed.pm10.clear();
        for (std::size_t index = 0; index < processed.dry_extinction.size(); ++index) {
            // 本距离门相对地面背景的超额消光，>0 表示存在局部污染增强
            double local_excess = std::max(0.0, processed.dry_extinction[index] - timestamp_features.surface_dry_ext);
            // 特征向量：[偏置项, 干消光, 相对湿度, 热点代理]
            std::vector<double> features{1.0, processed.dry_extinction[index], processed.profile.relative_humidity, timestamp_features.hotspot_proxy};
            double base_pm25 = predict(models.pm25, features) + offset.pm25_offset;
            double base_pm10 = predict(models.pm10, features) + offset.pm10_offset;
            double hotspot_boost = 1250.0 * local_excess;  ///< 经验线性系数，反映超额消光到 PM 质量浓度的转换
            // PM10 与 PM2.5 的增强近似按 1.18 倍比例放大（粗细颗粒物混合比的经验值）
            processed.pm25.push_back(std::max(0.0, base_pm25 + hotspot_boost));
            processed.pm10.push_back(std::max(0.0, base_pm10 + 1.18 * hotspot_boost));
        }
    }
}

/**
 * @brief 在 PPI 扫描上检测污染热点（hotspot）。
 *
 * 算法分两大阶段：
 *  1. **逐距离门预测掩码生成**：先用绝对阈值（PM2.5 ≥ threshold_ugm3）或相对阈值
 *     （PM2.5 与 dry_extinction 同时相对基线超出）判断每个网格是否为热点；
 *  2. **连通域分析（4-邻域 BFS）**：在 (方位角 × 距离) 网格上做连通域搜索，
 *     丢弃小于 min_cells 的孤立簇，对剩余簇按 PM2.5 加权求质心、面积与峰值，
 *     并根据峰值 PM2.5 给出 "medium/high/critical" 三档严重性。
 *
 * 方位角方向使用模 N 取邻居，因此支持 PPI 环形扫描的首尾相连。面积按径向距离
 * × 方位角步长 × 距离步长 投影到水平面近似（cos(elevation)）。
 *
 * @param profiles                       指向同一时间步 PPI 廓线的指针集合。
 * @param threshold_ugm3                 绝对热点 PM2.5 阈值（µg/m³）。
 * @param relative_pm25_threshold        相对基线的 PM2.5 超出阈值。
 * @param relative_dry_ext_threshold     相对基线的干消光超出阈值（必须同时满足）。
 * @param min_cells                      一个有效热点簇所需的最小网格数。
 * @return DetectionResult               含预测/真值掩码及热点列表。
 */
DetectionResult detect_hotspots(
    const std::vector<ProcessedProfile*>& profiles,
    double threshold_ugm3,
    double relative_pm25_threshold,
    double relative_dry_ext_threshold,
    int min_cells
) {
    DetectionResult result;
    if (profiles.empty()) {
        return result;
    }

    // 按方位角排序，确保 PPI 网格在方位方向上单调
    std::vector<const ProcessedProfile*> sorted_profiles;
    sorted_profiles.reserve(profiles.size());
    for (const auto* profile : profiles) {
        sorted_profiles.push_back(profile);
    }
    std::sort(sorted_profiles.begin(), sorted_profiles.end(), [](const ProcessedProfile* left, const ProcessedProfile* right) {
        return left->profile.azimuth_deg < right->profile.azimuth_deg;
    });

    int range_count = static_cast<int>(sorted_profiles.front()->profile.ranges_m.size());
    int azimuth_count = static_cast<int>(sorted_profiles.size());
    // mask 采用 row-major 布局：azimuth_index * range_count + range_index
    result.predicted_mask.assign(range_count * azimuth_count, 0);
    result.truth_mask.assign(range_count * azimuth_count, 0);

    // 收集所有网格的 PM/消光样本，用于计算本次扫描的"基线"（中位数更鲁棒）
    std::vector<double> all_pm25;
    std::vector<double> all_dry_ext;
    for (const auto* profile : sorted_profiles) {
        all_pm25.insert(all_pm25.end(), profile->pm25.begin(), profile->pm25.end());
        all_dry_ext.insert(all_dry_ext.end(), profile->dry_extinction.begin(), profile->dry_extinction.end());
    }
    double baseline_pm25 = median(all_pm25);
    double baseline_dry_ext = median(all_dry_ext);

    // 第一阶段：逐网格判定，得到预测掩码；同时记录真值掩码用于后续评估
    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        const auto* processed = sorted_profiles[azimuth_index];
        for (int range_index = 0; range_index < range_count; ++range_index) {
            int linear_index = azimuth_index * range_count + range_index;
            result.truth_mask[linear_index] = (range_index < static_cast<int>(processed->profile.true_hotspot_mask.size()))
                ? processed->profile.true_hotspot_mask[range_index] : 0;
            bool absolute_hotspot = processed->pm25[range_index] >= threshold_ugm3;
            bool relative_hotspot = (processed->pm25[range_index] - baseline_pm25 >= relative_pm25_threshold)
                && (processed->dry_extinction[range_index] - baseline_dry_ext >= relative_dry_ext_threshold);
            if (absolute_hotspot || relative_hotspot) {
                result.predicted_mask[linear_index] = 1;
            }
        }
    }

    // 第二阶段：4-邻域连通域分析（BFS），方位角方向采用模 N 以支持环形 PPI
    std::vector<std::vector<bool>> visited(azimuth_count, std::vector<bool>(range_count, false));
    int component_index = 0;
    double azimuth_step_deg = 360.0 / std::max(azimuth_count, 1);  ///< 每条 PPI 廓线代表的方位角步长
    double range_step_m = range_count > 1 ? sorted_profiles.front()->profile.ranges_m[1] - sorted_profiles.front()->profile.ranges_m[0] : 50.0;
    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        for (int range_index = 0; range_index < range_count; ++range_index) {
            if (result.predicted_mask[azimuth_index * range_count + range_index] == 0 || visited[azimuth_index][range_index]) {
                continue;
            }

            // 经典 BFS 队列：收集当前簇内所有连通的网格
            std::queue<std::pair<int, int>> queue;
            std::vector<std::pair<int, int>> component_cells;
            queue.push({azimuth_index, range_index});
            visited[azimuth_index][range_index] = true;
            while (!queue.empty()) {
                auto [current_azimuth, current_range] = queue.front();
                queue.pop();
                component_cells.push_back({current_azimuth, current_range});
                // 4-邻域：方位角邻居使用模 N 以闭合 PPI 环；距离方向不做模运算（不允许越界）
                const std::vector<std::pair<int, int>> neighbors{
                    { (current_azimuth - 1 + azimuth_count) % azimuth_count, current_range },
                    { (current_azimuth + 1) % azimuth_count, current_range },
                    { current_azimuth, current_range - 1 },
                    { current_azimuth, current_range + 1 },
                };
                for (const auto& [next_azimuth, next_range] : neighbors) {
                    if (next_range < 0 || next_range >= range_count || visited[next_azimuth][next_range]) {
                        continue;
                    }
                    if (result.predicted_mask[next_azimuth * range_count + next_range] == 0) {
                        continue;
                    }
                    visited[next_azimuth][next_range] = true;
                    queue.push({next_azimuth, next_range});
                }
            }

            // 太小的簇视为噪声丢弃
            if (static_cast<int>(component_cells.size()) < min_cells) {
                continue;
            }

            ++component_index;
            // 为质心、面积、峰值统计累积各方向统计量
            std::vector<double> weights;
            std::vector<double> east_values;
            std::vector<double> north_values;
            std::vector<double> up_values;
            std::vector<double> pm_values;
            double area_m2 = 0.0;
            std::string scan_id = sorted_profiles[component_cells.front().first]->profile.scan_id;
            for (const auto& [component_azimuth, component_range] : component_cells) {
                const auto* processed = sorted_profiles[component_azimuth];
                double pm25_value = processed->pm25[component_range];
                const auto& point = processed->enu_points_m[component_range];
                // 用 PM2.5 值做加权，使质心更贴近高污染核心区
                weights.push_back(pm25_value);
                east_values.push_back(point[0]);
                north_values.push_back(point[1]);
                up_values.push_back(point[2]);
                pm_values.push_back(pm25_value);
                // 单元格面积：径向距离投影到水平面 × 方位角弧度 × 距离步长，最小取 10 m 防止近场退化
                double radial_distance = processed->profile.ranges_m[component_range] * std::cos(processed->profile.elevation_deg * std::numbers::pi / 180.0);
                area_m2 += std::max(radial_distance, 10.0) * (azimuth_step_deg * std::numbers::pi / 180.0) * range_step_m;
            }

            // 加权质心（East/North/Up），用 std::max 分母防止零除
            double total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);
            std::vector<double> centroid{
                std::inner_product(east_values.begin(), east_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
                std::inner_product(north_values.begin(), north_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
                std::inner_product(up_values.begin(), up_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
            };
            double peak_pm25 = *std::max_element(pm_values.begin(), pm_values.end());
            // 严重性按峰值相对绝对阈值的超出量分档：+35 µg/m³ 进入 high，+70 进入 critical
            std::string severity = peak_pm25 >= threshold_ugm3 + 70.0 ? "critical" : (peak_pm25 >= threshold_ugm3 + 35.0 ? "high" : "medium");

            std::ostringstream hotspot_id;
            hotspot_id << "hotspot-" << std::setw(3) << std::setfill('0') << component_index;  ///< 形如 hotspot-001 的固定宽度 ID
            result.hotspots.push_back(Hotspot{
                sorted_profiles[component_cells.front().first]->profile.timestamp,
                scan_id,
                hotspot_id.str(),
                centroid,
                peak_pm25,
                mean(pm_values),
                area_m2,
                static_cast<int>(component_cells.size()),
                severity,
            });
        }
    }
    return result;
}

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
    return Json::object_type{
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

    Json::object_type stations;
    Json::array_type alerts;
    for (auto& [site_id, rows] : grouped) {
        // 同站点样本按时间升序排列，保证滑窗语义正确
        std::sort(rows.begin(), rows.end(), [](const ResidualRow& left, const ResidualRow& right) {
            return left.timestamp < right.timestamp;
        });
        int window_size = std::min<int>(3, static_cast<int>(rows.size()));  ///< 滑窗长度，最多 3
        Json::array_type windows;
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
            Json window_summary = Json::object_type{
                {"start", rows[index].timestamp},
                {"end", rows[index + window_size - 1].timestamp},
                {"pm25_bias", pm25_bias},
                {"pm10_bias", pm10_bias},
                {"alert_level", alert_level},
            };
            windows.push_back(window_summary);
            if (alert_level != "ok") {
                alerts.push_back(Json::object_type{
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
        Json latest_window = windows.empty() ? Json(Json::object_type{
            {"pm25_bias", 0.0},
            {"pm10_bias", 0.0},
            {"alert_level", "ok"},
        }) : windows.back();
        stations[site_id] = Json::object_type{
            {"sample_count", static_cast<int>(rows.size())},
            {"latest_pm25_bias", latest_window.at("pm25_bias")},
            {"latest_pm10_bias", latest_window.at("pm10_bias")},
            {"latest_alert_level", latest_window.at("alert_level")},
            {"windows", Json(std::move(windows))},
        };
    }
    return Json::object_type{
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
Json::array_type run_failure_case_suite(const PipelineConfig& config, const SiteInfo& site, const std::vector<LidarProfile>& profiles, const std::vector<GroundMeasurement>& ground_measurements);

// ---- 单次数据集全流水线运行 ----

/**
 * @brief 在一组数据（雷达廓线 + 地面观测）上完整运行反演 + PM 校准 + 热点检测 + 评估。
 *
 * 流水线步骤：
 *  1. 逐条廓线：预处理 → Fernald 反演 → （可选）湿度修正 → 转 ENU 坐标，并记录单条耗时；
 *  2. 构造特征表 → 拟合 PM 线性模型 → 求站点偏置 → 把模型应用到所有廓线；
 *  3. 按时间步对 PPI 廓线做热点检测，累积预测/真值掩码；
 *  4. 在 holdout（val+test，否则 train）时间戳上抽取地面预测，计算回归指标、
 *     分类指标、漂移监控与吞吐量指标，统一写入 result.metrics。
 *
 * @param config                流水线配置（含反演/AWI/PM/热点各类阈值）。
 * @param site                  站点信息。
 * @param profiles              原始雷达廓线。
 * @param ground_measurements   原始地面观测。
 * @param lidar_ratio_override  >0 时覆盖 config.retrieval.aerosol_lidar_ratio_sr，用于敏感性分析。
 * @param disable_humidity      为 true 时跳过湿度修正，用于消融实验。
 * @return DatasetRunResult     含全部中间产物与汇总指标。
 */
DatasetRunResult run_pipeline_on_dataset(
    const PipelineConfig& config,
    const SiteInfo& site,
    const std::vector<LidarProfile>& profiles,
    const std::vector<GroundMeasurement>& ground_measurements,
    std::optional<double> lidar_ratio_override = std::nullopt,
    bool disable_humidity = false
) {
    DatasetRunResult result;
    result.site = site;
    result.profiles = profiles;
    result.ground_measurements = ground_measurements;

    auto total_start = std::chrono::steady_clock::now();
    // 第一阶段：逐条廓线反演
    for (const auto& profile : profiles) {
        auto started_at = std::chrono::steady_clock::now();
        PreprocessResult preprocessed = preprocess_profile(profile);
        auto inversion = run_fernald_inversion(
            profile,
            preprocessed.attenuated_backscatter,
            lidar_ratio_override.value_or(config.retrieval.aerosol_lidar_ratio_sr),
            config.retrieval.reference_aerosol_backscatter
        );
        std::vector<double> dry_extinction = inversion.first;
        if (!disable_humidity) {
            // 默认对反演结果做湿度修正（得到的 dry_extinction 才能用于 PM 校准）
            dry_extinction = apply_humidity_correction(
                inversion.first,
                profile.relative_humidity,
                config.humidity.dry_reference_rh,
                config.humidity.hygroscopicity
            );
        }
        auto ended_at = std::chrono::steady_clock::now();
        result.processed_profiles.push_back(ProcessedProfile{
            profile,
            preprocessed.l1_signal,
            preprocessed.attenuated_backscatter,
            preprocessed.snr,
            inversion.first,
            dry_extinction,
            {},
            {},
            profile_bins_to_enu(profile),
            preprocessed.qc_flags,
            std::chrono::duration<double, std::milli>(ended_at - started_at).count(),
        });
    }

    // 第二阶段：构造地面特征表 + 拟合 PM 模型 + 求偏置 + 应用到每条廓线
    result.feature_table = build_timestamp_feature_table(result.processed_profiles, ground_measurements, config.pm_calibration.surface_bin_count);
    auto fitted = fit_pm_models(result.feature_table);
    CalibrationModels models = fitted.first;
    result.split = fitted.second;
    result.station_offsets = fit_station_offsets(models, result.split);
    apply_pm_models(result.processed_profiles, models, result.feature_table, result.station_offsets);

    // 第三阶段：按 timestamp 分组对 PPI 廓线做热点检测
    std::vector<int> predicted_mask;
    std::vector<int> truth_mask;
    std::map<std::string, std::vector<ProcessedProfile*>> grouped;
    for (auto& processed : result.processed_profiles) {
        grouped[processed.profile.timestamp].push_back(&processed);
    }
    for (auto& [timestamp, profiles_at_timestamp] : grouped) {
        std::vector<ProcessedProfile*> ppi_profiles;
        for (auto* profile : profiles_at_timestamp) {
            if (profile->profile.scan_mode == "ppi") {
                ppi_profiles.push_back(profile);
            }
        }
        DetectionResult detection = detect_hotspots(
            ppi_profiles,
            config.hotspot.pm25_threshold_ugm3,
            config.hotspot.scan_relative_pm25_threshold_ugm3,
            config.hotspot.scan_relative_dry_ext_threshold,
            config.hotspot.min_cells
        );
        result.hotspots_by_timestamp[timestamp] = detection.hotspots;
        predicted_mask.insert(predicted_mask.end(), detection.predicted_mask.begin(), detection.predicted_mask.end());
        truth_mask.insert(truth_mask.end(), detection.truth_mask.begin(), detection.truth_mask.end());
    }

    // 第四阶段：在 holdout（或退回 train）时间戳上抽取地面预测以计算指标
    auto total_end = std::chrono::steady_clock::now();
    std::vector<std::string> holdout_timestamps = result.split.val_timestamps;
    holdout_timestamps.insert(holdout_timestamps.end(), result.split.test_timestamps.begin(), result.split.test_timestamps.end());
    // holdout 为空时退回训练集，保证小数据集场景仍可输出指标
    std::vector<std::string> evaluation_timestamps = holdout_timestamps.empty() ? result.split.train_timestamps : holdout_timestamps;
    PredictionSummary predictions = extract_ground_predictions(result.feature_table, models, evaluation_timestamps, result.station_offsets);
    Json hotspot_scores = classification_metrics(truth_mask, predicted_mask);
    result.drift_monitoring = summarize_drift_monitoring(predictions.residual_rows);

    // 收集每条廓线的处理时延，用于吞吐量统计
    std::vector<double> latency_values;
    latency_values.reserve(result.processed_profiles.size());
    for (const auto& profile : result.processed_profiles) {
        latency_values.push_back(profile.latency_ms);
    }
    double elapsed_s = std::chrono::duration<double>(total_end - total_start).count();
    result.metrics = Json::object_type{
        {"pm25", Json::object_type{
            {"mae", mae_metric(predictions.pm25_truth, predictions.pm25_pred)},
            {"rmse", rmse_metric(predictions.pm25_truth, predictions.pm25_pred)},
            {"r2", r2_metric(predictions.pm25_truth, predictions.pm25_pred)},
        }},
        {"pm10", Json::object_type{
            {"mae", mae_metric(predictions.pm10_truth, predictions.pm10_pred)},
            {"rmse", rmse_metric(predictions.pm10_truth, predictions.pm10_pred)},
            {"r2", r2_metric(predictions.pm10_truth, predictions.pm10_pred)},
        }},
        {"hotspot", hotspot_scores},
        {"drift", Json::object_type{
            {"station_count", static_cast<int>(result.drift_monitoring.at("stations").object_items().size())},
            {"alert_count", static_cast<int>(result.drift_monitoring.at("alerts").array_items().size())},
        }},
        {"runtime", Json::object_type{
            {"mean_latency_ms", mean(latency_values)},
            {"throughput_profiles_per_s", static_cast<double>(result.processed_profiles.size()) / std::max(elapsed_s, 1e-9)},  ///< 用 max(.,1e-9) 防止零除
        }},
    };
    return result;
}

// ---- 演示 payload 序列化辅助 ----

/**
 * @brief 把单个 FeatureSample 序列化为前端友好的 JSON。
 * @param sample 地面特征样本。
 * @return Json 含 timestamp、features、各类地面观测量。
 */
Json feature_sample_to_json(const FeatureSample& sample) {
    return Json::object_type{
        {"timestamp", sample.timestamp},
        {"features", json_array_from_double_vector(sample.features)},
        {"surface_dry_ext", sample.surface_dry_ext},
        {"surface_attenuated", sample.surface_attenuated},
        {"hotspot_proxy", sample.hotspot_proxy},
        {"site_id", sample.site_id},
        {"pm25_true", sample.pm25_true},
        {"pm10_true", sample.pm10_true},
        {"relative_humidity", sample.relative_humidity},
        {"wind_speed_ms", sample.wind_speed_ms},
    };
}

/**
 * @brief 把站点偏置映射序列化为以 site_id 为键的 JSON 对象。
 * @param station_offsets 站点偏置映射。
 * @return Json 每个站点含 pm25_offset / pm10_offset / sample_count。
 */
Json station_offsets_to_json(const std::map<std::string, StationOffset>& station_offsets) {
    Json::object_type output;
    for (const auto& [site_id, offset] : station_offsets) {
        output[site_id] = Json::object_type{
            {"pm25_offset", offset.pm25_offset},
            {"pm10_offset", offset.pm10_offset},
            {"sample_count", offset.sample_count},
        };
    }
    return Json(std::move(output));
}

/**
 * @brief 用时间戳序列推断采样间隔（分钟），用于前端展示 dataset_summary。
 *
 * 计算相邻时间戳之间的时间差（分钟），再取均值并四舍五入到整数。
 * 时间戳不足 2 个时返回 0。
 *
 * @param timestamps 升序时间戳序列（ISO 字符串）。
 * @return int 采样间隔，单位分钟；样本不足时返回 0。
 */
int sampling_minutes(const std::vector<std::string>& timestamps) {
    if (timestamps.size() < 2) {
        return 0;
    }
    std::vector<double> deltas;
    for (std::size_t index = 1; index < timestamps.size(); ++index) {
        // difftime 返回秒，除以 60 转成分钟
        deltas.push_back(std::difftime(parse_timestamp(timestamps[index]), parse_timestamp(timestamps[index - 1])) / 60.0);
    }
    return static_cast<int>(std::lround(mean(deltas)));
}

/**
 * @brief 把一次主实验与各项副实验拼装成单个完整的演示 payload，用于驱动前端仪表盘。
 *
 * payload 组成：
 *  - site / dataset_summary：站点信息与采样概要（含来源构成 breakdown）；
 *  - curtain：stare 模式下的"时间-高度"幕布图数据（PM 与消光）；
 *  - ppi：最新时刻 PPI 扫描的网格化 PM 数据（前端可绘制 2D 平面图）；
 *  - qc：逐时间步的质量控制统计（SNR、时延、背景光、激光能量）；
 *  - hotspots / alerts：最新热点列表 + 全部时间步的告警；
 *  - metrics / sensitivity / ablation / failure_cases：评估指标与实验矩阵；
 *  - station_calibration / drift_monitoring / ground_series：站点校准与漂移监控及原始地面序列。
 *
 * @param primary          主实验结果。
 * @param ablation         消融实验结果数组。
 * @param sensitivity      敏感性扫描结果数组。
 * @param failure_cases    失败案例结果数组。
 * @param source_metadata  数据来源元信息。
 * @return Json 完整 payload，结构稳定且自描述。
 */
Json build_demo_payload(
    const DatasetRunResult& primary,
    const Json::array_type& ablation,
    const Json::array_type& sensitivity,
    const Json::array_type& failure_cases,
    const Json& source_metadata
) {
    // 按 timestamp 把已处理廓线重新分组，便于按时间构建幕布图与 PPI 数据
    std::map<std::string, std::vector<const ProcessedProfile*>> grouped;
    for (const auto& profile : primary.processed_profiles) {
        grouped[profile.profile.timestamp].push_back(&profile);
    }
    std::vector<std::string> timestamps;
    timestamps.reserve(grouped.size());
    for (const auto& [timestamp, _] : grouped) {
        timestamps.push_back(timestamp);
    }

    // 取每个时间步的第一条 stare 廓线，用于构建时间-高度幕布图
    std::vector<const ProcessedProfile*> stare_profiles;
    stare_profiles.reserve(timestamps.size());
    for (const auto& timestamp : timestamps) {
        auto it = std::find_if(grouped[timestamp].begin(), grouped[timestamp].end(), [](const ProcessedProfile* profile) {
            return profile->profile.scan_mode == "stare";
        });
        if (it != grouped[timestamp].end()) {
            stare_profiles.push_back(*it);
        }
    }

    Json::array_type curtain_times;
    Json::array_type curtain_heights;
    Json::array_type curtain_pm25;
    Json::array_type curtain_extinction;
    for (const auto& timestamp : timestamps) {
        curtain_times.emplace_back(timestamp);
    }
    if (!stare_profiles.empty()) {
        // 高度轴只取一次（假设所有 stare 廓线距离门一致）
        for (const auto& point : stare_profiles.front()->enu_points_m) {
            curtain_heights.emplace_back(point[2]);
        }
        for (const auto* profile : stare_profiles) {
            curtain_pm25.emplace_back(json_array_from_double_vector(profile->pm25));
            curtain_extinction.emplace_back(json_array_from_double_vector(profile->extinction));
        }
    }

    // 最新时间步的 PPI 网格化（按方位角排序后逐距离门写 cell）
    std::string latest_timestamp = timestamps.empty() ? std::string{} : timestamps.back();
    Json::array_type ppi_cells;
    if (!latest_timestamp.empty()) {
        std::vector<const ProcessedProfile*> latest_ppi_profiles;
        for (const auto* profile : grouped[latest_timestamp]) {
            if (profile->profile.scan_mode == "ppi") {
                latest_ppi_profiles.push_back(profile);
            }
        }
        std::sort(latest_ppi_profiles.begin(), latest_ppi_profiles.end(), [](const ProcessedProfile* left, const ProcessedProfile* right) {
            return left->profile.azimuth_deg < right->profile.azimuth_deg;
        });
        for (const auto* processed : latest_ppi_profiles) {
            for (std::size_t index = 0; index < processed->enu_points_m.size(); ++index) {
                ppi_cells.push_back(Json::object_type{
                    {"x_m", processed->enu_points_m[index][0]},
                    {"y_m", processed->enu_points_m[index][1]},
                    {"z_m", processed->enu_points_m[index][2]},
                    {"pm25", processed->pm25[index]},
                    {"pm10", processed->pm10[index]},
                    {"is_true_hotspot", processed->profile.true_hotspot_mask[index] == 1},
                });
            }
        }
    }

    // 逐时间步汇总质量控制统计：取每个时间步的 stare 廓线计算近场 8 个距离门的平均 SNR
    Json::array_type qc_times;
    Json::array_type qc_mean_snr;
    Json::array_type qc_latency;
    Json::array_type qc_background;
    Json::array_type qc_energy;
    for (const auto& timestamp : timestamps) {
        qc_times.emplace_back(timestamp);
        const auto& profiles_at_timestamp = grouped[timestamp];
        const ProcessedProfile* stare = nullptr;
        std::vector<double> latency_values;
        for (const auto* profile : profiles_at_timestamp) {
            latency_values.push_back(profile->latency_ms);
            if (profile->profile.scan_mode == "stare") {
                stare = profile;
            }
        }
        std::vector<double> snr_slice;
        if (stare != nullptr) {
            // 只统计前 8 个近场距离门，因为远处 SNR 容易被噪声主导
            snr_slice.assign(stare->snr.begin(), stare->snr.begin() + std::min<std::size_t>(8, stare->snr.size()));
            qc_background.emplace_back(stare->profile.background_counts);
            qc_energy.emplace_back(stare->profile.laser_energy_mj);
        } else {
            qc_background.emplace_back(0.0);
            qc_energy.emplace_back(0.0);
        }
        qc_mean_snr.emplace_back(mean(snr_slice));
        qc_latency.emplace_back(mean(latency_values));
    }

    // 全部时间步的热点（用作告警时间轴）+ 最新时间步的热点列表（用于突出展示）
    Json::array_type alerts;
    for (const auto& [timestamp, hotspots] : primary.hotspots_by_timestamp) {
        for (const auto& hotspot : hotspots) {
            alerts.push_back(to_json(hotspot));
        }
    }

    Json::array_type latest_hotspots;
    if (primary.hotspots_by_timestamp.contains(latest_timestamp)) {
        for (const auto& hotspot : primary.hotspots_by_timestamp.at(latest_timestamp)) {
            latest_hotspots.push_back(to_json(hotspot));
        }
    }

    // 地面序列：用于前端绘制"地面真值随时间变化"折线
    Json::array_type ground_series;
    for (const auto& sample : primary.feature_table) {
        ground_series.push_back(feature_sample_to_json(sample));
    }

    Json payload = Json::object_type{
        {"site", to_json(primary.site)},
        {"dataset_summary", Json::object_type{
            {"profile_count", static_cast<int>(primary.processed_profiles.size())},
            {"timestamp_count", static_cast<int>(timestamps.size())},
            {"sampling_minutes", sampling_minutes(timestamps)},
            {"range_bin_count", stare_profiles.empty() ? 0 : static_cast<int>(stare_profiles.front()->profile.ranges_m.size())},
            {"train_count", static_cast<int>(primary.split.train.size())},
            {"val_count", static_cast<int>(primary.split.val.size())},
            {"test_count", static_cast<int>(primary.split.test.size())},
        }},
        {"source", source_metadata},
        {"curtain", Json::object_type{
            {"times", Json(std::move(curtain_times))},
            {"heights_m", Json(std::move(curtain_heights))},
            {"pm25", Json(std::move(curtain_pm25))},
            {"extinction", Json(std::move(curtain_extinction))},
        }},
        {"ppi", Json::object_type{
            {"timestamp", latest_timestamp},
            {"cells", Json(std::move(ppi_cells))},
        }},
        {"hotspots", Json(std::move(latest_hotspots))},
        {"qc", Json::object_type{
            {"times", Json(std::move(qc_times))},
            {"mean_snr", Json(std::move(qc_mean_snr))},
            {"latency_ms", Json(std::move(qc_latency))},
            {"background_counts", Json(std::move(qc_background))},
            {"laser_energy_mj", Json(std::move(qc_energy))},
        }},
        {"alerts", Json(std::move(alerts))},
        {"metrics", primary.metrics},
        {"sensitivity", Json(sensitivity)},
        {"ablation", Json(ablation)},
        {"failure_cases", Json(failure_cases)},
        {"station_calibration", station_offsets_to_json(primary.station_offsets)},
        {"drift_monitoring", primary.drift_monitoring},
        {"ground_series", Json(std::move(ground_series))},
    };

    payload["dataset_summary"]["source_mode"] = source_metadata.at("mode");
    // 按数据来源分类统计廓线来源构成，方便前端区分真实/合成数据比例
    int stare_real = 0;
    int stare_synthetic = 0;
    int ppi_hybrid = 0;
    int ppi_synthetic = 0;
    for (const auto& profile : primary.profiles) {
        if (profile.source_kind == "cloudnet_real_stare") {
            ++stare_real;
        } else if (profile.source_kind == "synthetic_stare") {
            ++stare_synthetic;
        } else if (profile.source_kind == "synthetic_ppi_hybrid") {
            ++ppi_hybrid;
        } else if (profile.source_kind == "synthetic_ppi") {
            ++ppi_synthetic;
        }
    }
    payload["dataset_summary"]["source_breakdown"] = Json::object_type{
        {"stare_real", stare_real},
        {"stare_synthetic", stare_synthetic},
        {"ppi_hybrid", ppi_hybrid},
        {"ppi_synthetic", ppi_synthetic},
    };
    return payload;
}

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
 * @return Json::array_type  三个 case 的指标摘要数组。
 */
Json::array_type run_failure_case_suite(const PipelineConfig& config, const SiteInfo& site, const std::vector<LidarProfile>& profiles, const std::vector<GroundMeasurement>& ground_measurements) {
    Json::array_type output;
    for (const std::string& case_name : {std::string("high-background-light"), std::string("overlap-miscalibration"), std::string("humidity-surge")}) {
        // 逐 case 拷贝原始数据，保证各 case 之间互不影响
        auto case_profiles = profiles;
        auto case_ground = ground_measurements;
        apply_failure_case(case_profiles, case_ground, case_name);
        DatasetRunResult case_output = run_pipeline_on_dataset(config, site, case_profiles, case_ground);
        output.push_back(Json::object_type{
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

// ---- JSON 引擎公开 API（匿名命名空间之外） ----
} // namespace

/**
 * @brief 解析 JSON 字符串并返回 Json 对象（公开 API）。
 *
 * 内部委托给匿名命名空间中的 JsonParser。失败时抛出 std::runtime_error。
 *
 * @param text 待解析的 JSON 文本。
 * @return Json 解析得到的对象。
 */
Json parse_json(const std::string& text) {
    return JsonParser(text).parse();
}

/**
 * @brief 把 Json 对象序列化为字符串（公开 API）。
 *
 * @param value  要序列化的 Json 对象。
 * @param indent 每层缩进的空格数，<=0 表示单行紧凑输出。
 * @return std::string 序列化后的 JSON 文本。
 */
std::string dump_json(const Json& value, int indent) {
    std::ostringstream output;
    dump_json_impl(value, output, indent, 0);
    return output.str();
}

/**
 * @brief 读取并解析一个 JSON 文件。
 *
 * 以二进制方式一次读取全部内容到内存，再调用 parse_json 解析。
 * 文件打开失败抛出 std::runtime_error。
 *
 * @param path JSON 文件路径。
 * @return Json 解析得到的对象。
 */
Json read_json_file(const std::filesystem::path& path) {
    std::ifstream handle(path, std::ios::binary);
    if (!handle) {
        throw std::runtime_error("Failed to open JSON file: " + path.string());
    }
    std::ostringstream buffer;
    buffer << handle.rdbuf();
    return parse_json(buffer.str());
}

/**
 * @brief 把 Json 对象写入磁盘文件（会自动创建父目录，使用 2 空格缩进）。
 * @param path  目标文件路径。
 * @param value 要写入的 Json 对象。
 */
void write_json_file(const std::filesystem::path& path, const Json& value) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream handle(path, std::ios::binary);
    if (!handle) {
        throw std::runtime_error("Failed to write JSON file: " + path.string());
    }
    handle << dump_json(value, 2);  ///< 固定 2 空格缩进，配合前端 pretty-print
}

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

    Json manifest = Json::object_type{
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

    Json aligned_manifest = Json::object_type{
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
    Json manifest = Json::object_type{
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

// ---- 配置解析 (JSON -> PipelineConfig) ----

/**
 * @brief 把流水线 JSON 配置反序列化为 PipelineConfig 结构（用户面对的核心入口之一）。
 *
 * 严格字段直接用 .at(...) 读取（缺失会抛错）；可选字段用 source.contains(x) ? ... : 默认值
 * 的三目模式兜底，方便用户写最小配置。本函数同时填充 config.source_mode 与
 * config.source.mode 两个等价字段，兼容老代码。
 *
 * 解析的子段：
 *  - source / source.cloudnet：数据来源方式与 Cloudnet 子项（含默认值）；
 *  - site：站点元信息（site_id 缺省时由 name slugify 自动生成）；
 *  - simulation：模拟参数（种子、时间步、PPI 仰角/方位角步长等）；
 *  - retrieval：Fernald 反演参数；
 *  - humidity：湿度修正参数；
 *  - pm_calibration：PM 模型训练比例与 bin 数；
 *  - hotspot：热点检测阈值；
 *  - evaluation：敏感性扫描使用的 lidar ratio 列表。
 *
 * @param value 已解析的根 JSON 对象。
 * @return PipelineConfig 完整配置，可直接用于后续 run_end_to_end。
 */
PipelineConfig parse_pipeline_config(const Json& value) {
    PipelineConfig config;
    // source_mode 顶层存放，便于外层快速判断走 simulation / cloudnet_hybrid 等分支
    if (value.contains("source") && value.at("source").is_object() && value.at("source").contains("mode")) {
        config.source_mode = value.at("source").at("mode").string_value();
    }
    config.source.mode = config.source_mode;  ///< 同步到 source 子结构以兼容内部代码
    if (value.contains("source") && value.at("source").is_object()) {
        const Json& source = value.at("source");
        // root 缺省为 "."（当前目录）
        config.source.root = source.contains("root") ? source.at("root").string_value() : ".";
        if (source.contains("cloudnet") && source.at("cloudnet").is_object()) {
            // Cloudnet 子段：每个字段都提供合理默认值，降低用户配置负担
            const Json& cloudnet = source.at("cloudnet");
            config.source.cloudnet.site_id = cloudnet.contains("site_id") ? cloudnet.at("site_id").string_value() : "";
            config.source.cloudnet.site_name = cloudnet.contains("site_name") ? cloudnet.at("site_name").string_value() : "";
            config.source.cloudnet.date = cloudnet.contains("date") ? cloudnet.at("date").string_value() : "";
            config.source.cloudnet.verify_ssl = cloudnet.contains("verify_ssl") ? cloudnet.at("verify_ssl").bool_value() : true;
            config.source.cloudnet.local_file = cloudnet.contains("local_file") ? cloudnet.at("local_file").string_value() : "";
            config.source.cloudnet.download_url = cloudnet.contains("download_url") ? cloudnet.at("download_url").string_value() : "";
            config.source.cloudnet.time_steps = cloudnet.contains("time_steps") ? cloudnet.at("time_steps").int_value() : 18;
            config.source.cloudnet.range_bin_count = cloudnet.contains("range_bin_count") ? cloudnet.at("range_bin_count").int_value() : 30;
            config.source.cloudnet.min_range_m = cloudnet.contains("min_range_m") ? cloudnet.at("min_range_m").number_value() : 75.0;
            config.source.cloudnet.max_range_m = cloudnet.contains("max_range_m") ? cloudnet.at("max_range_m").number_value() : 3200.0;
            config.source.cloudnet.pseudo_signal_scale = cloudnet.contains("pseudo_signal_scale") ? cloudnet.at("pseudo_signal_scale").number_value() : 600000.0;  ///< 把 Cloudnet 消光缩放到模拟器近似信号尺度的经验系数
        }
    }

    // 站点（必填字段）
    const Json& site = value.at("site");
    config.site.name = site.at("name").string_value();
    config.site.latitude_deg = site.at("latitude_deg").number_value();
    config.site.longitude_deg = site.at("longitude_deg").number_value();
    config.site.altitude_m = site.at("altitude_m").number_value();
    // site_id 缺省时用 name 的 slug 形式自动生成
    config.site.site_id = site.contains("site_id") ? site.at("site_id").string_value() : slugify(config.site.name);

    // 模拟参数（必填）
    const Json& simulation = value.at("simulation");
    config.simulation.seed = simulation.at("seed").int_value();
    config.simulation.time_steps = simulation.at("time_steps").int_value();
    config.simulation.minutes_per_step = simulation.at("minutes_per_step").int_value();
    config.simulation.range_bin_count = simulation.at("range_bin_count").int_value();
    config.simulation.range_bin_m = simulation.at("range_bin_m").number_value();
    config.simulation.ppi_elevation_deg = simulation.at("ppi_elevation_deg").number_value();
    config.simulation.ppi_azimuth_step_deg = simulation.at("ppi_azimuth_step_deg").number_value();
    config.simulation.system_constant = simulation.at("system_constant").number_value();
    config.simulation.lidar_ratio_sr = simulation.at("lidar_ratio_sr").number_value();

    // 反演参数（必填）
    const Json& retrieval = value.at("retrieval");
    config.retrieval.aerosol_lidar_ratio_sr = retrieval.at("aerosol_lidar_ratio_sr").number_value();
    config.retrieval.reference_aerosol_backscatter = retrieval.at("reference_aerosol_backscatter").number_value();

    // 湿度修正参数（必填）
    const Json& humidity = value.at("humidity");
    config.humidity.dry_reference_rh = humidity.at("dry_reference_rh").number_value();
    config.humidity.hygroscopicity = humidity.at("hygroscopicity").number_value();

    // PM 校准切分参数（必填）
    const Json& calibration = value.at("pm_calibration");
    config.pm_calibration.train_ratio = calibration.at("train_ratio").number_value();
    config.pm_calibration.val_ratio = calibration.at("val_ratio").number_value();
    config.pm_calibration.surface_bin_count = calibration.at("surface_bin_count").int_value();

    // 热点检测阈值（必填）
    const Json& hotspot = value.at("hotspot");
    config.hotspot.pm25_threshold_ugm3 = hotspot.at("pm25_threshold_ugm3").number_value();
    config.hotspot.scan_relative_pm25_threshold_ugm3 = hotspot.at("scan_relative_pm25_threshold_ugm3").number_value();
    config.hotspot.scan_relative_dry_ext_threshold = hotspot.at("scan_relative_dry_ext_threshold").number_value();
    config.hotspot.min_cells = hotspot.at("min_cells").int_value();

    // 敏感性扫描对应的 lidar_ratio 列表（数组项数可为 0）
    const Json& evaluation = value.at("evaluation");
    for (const auto& item : evaluation.at("sensitivity_lidar_ratios").array_items()) {
        config.evaluation.sensitivity_lidar_ratios.push_back(item.number_value());
    }
    return config;
}

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
    Json::array_type ablation;
    ablation.push_back(Json::object_type{
        {"name", "full-pipeline"},
        {"pm25_rmse", primary.metrics.at("pm25").at("rmse")},
        {"hotspot_f1", primary.metrics.at("hotspot").at("f1")},
    });

    DatasetRunResult humidity_disabled = run_pipeline_on_dataset(config, campaign.site, campaign.profiles, campaign.ground_measurements, std::nullopt, true);
    ablation.push_back(Json::object_type{
        {"name", "without-humidity-correction"},
        {"pm25_rmse", humidity_disabled.metrics.at("pm25").at("rmse")},
        {"hotspot_f1", humidity_disabled.metrics.at("hotspot").at("f1")},
    });

    // 4. 敏感性扫描：在配置的 lidar_ratio 列表上各跑一次
    Json::array_type sensitivity;
    std::vector<double> sensitivity_rmse;
    for (double lidar_ratio : config.evaluation.sensitivity_lidar_ratios) {
        DatasetRunResult perturbed = run_pipeline_on_dataset(config, campaign.site, campaign.profiles, campaign.ground_measurements, lidar_ratio, false);
        sensitivity.push_back(Json::object_type{
            {"lidar_ratio_sr", lidar_ratio},
            {"pm25_rmse", perturbed.metrics.at("pm25").at("rmse")},
            {"pm10_rmse", perturbed.metrics.at("pm10").at("rmse")},
            {"hotspot_f1", perturbed.metrics.at("hotspot").at("f1")},
        });
        sensitivity_rmse.push_back(perturbed.metrics.at("pm25").at("rmse").number_value());
    }
    // 反演稳定性：最大最小 RMSE 差，越小越稳健
    double stability_span = sensitivity_rmse.empty() ? 0.0 : *std::max_element(sensitivity_rmse.begin(), sensitivity_rmse.end()) - *std::min_element(sensitivity_rmse.begin(), sensitivity_rmse.end());
    primary.metrics["retrieval_stability"] = Json::object_type{
        {"pm25_rmse_span", stability_span},
        {"reference_lidar_ratio_sr", config.retrieval.aerosol_lidar_ratio_sr},
    };

    // 5. 失败案例套件
    Json::array_type failure_cases = run_failure_case_suite(config, campaign.site, campaign.profiles, campaign.ground_measurements);
    // 6. 组装最终 demo payload
    Json demo_payload = build_demo_payload(primary, ablation, sensitivity, failure_cases, campaign.source_metadata);

    // 可选：把原始 / L1 / L2 数据写到磁盘，便于后续离线分析与前端静态加载
    if (output_root.has_value()) {
        Json::array_type raw_profiles;
        for (const auto& profile : primary.profiles) {
            raw_profiles.push_back(to_json(profile));
        }
        Json::array_type l1_profiles;
        for (const auto& profile : primary.processed_profiles) {
            l1_profiles.push_back(to_json(profile));
        }
        write_json_file(*output_root / "data" / "raw" / "simulated_demo_campaign.json", Json(std::move(raw_profiles)));
        write_json_file(*output_root / "data" / "l1" / "demo_preprocessed.json", Json(std::move(l1_profiles)));
        write_json_file(*output_root / "data" / "l2" / "demo_results.json", demo_payload);
    }

    return demo_payload;
}

// ---- 前端 HTML 仪表盘渲染 ----

/**
 * @brief 用一次 run_end_to_end 得到的 demo payload 渲染单页 HTML 仪表盘（中文）。
 *
 * 这是最重要的对外 API 之一，产出可直接落盘到 web/demo_dashboard.html 风格的页面。
 * 渲染策略：
 *  1. 从 payload 的 qc.mean_snr / qc.latency_ms 提取两条时间序列，用 build_line_chart_svg
 *     各生成一张内嵌 SVG 折线图（颜色分别对应 telemetry 蓝/琥珀）；
 *  2. 把整个 payload 以紧凑 JSON 嵌入到 <script type="application/json"> 中，
 *     页面加载时一段 JS 把 source/drift_monitoring/failure_cases 三段以 <pre> 美化输出；
 *  3. 顶部三段卡片：四类关键指标（PM2.5 RMSE、PM10 RMSE、热点 F1、吞吐量）。
 *
 * CSS 内嵌在 <style> 中，单文件即可离线使用。
 *
 * @param data 完整的 demo payload（来自 run_end_to_end）。
 * @return std::string 完整的 HTML 文档字符串。
 */
std::string render_dashboard(const Json& data) {
    std::vector<double> snr_values = json_to_double_vector(data.at("qc").at("mean_snr"));
    std::vector<double> latency_values = json_to_double_vector(data.at("qc").at("latency_ms"));
    // 紧凑 JSON（无缩进）以减小 HTML 体积
    std::string embedded = dump_json(data, 0);
    std::string snr_chart = build_line_chart_svg(snr_values, 520, 180, "#0f766e");     ///< teal 主色调
    std::string latency_chart = build_line_chart_svg(latency_values, 520, 180, "#b45309"); ///< 琥珀色对比

    std::ostringstream html;
    html << "<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\" /><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />"
         << "<title>Atmospheric Lidar Demo</title><style>"
         << ":root{--bg:#f7f4ed;--panel:rgba(255,252,247,.92);--ink:#1f2937;--muted:#6b7280;--accent:#0f766e;--shadow:0 18px 42px rgba(15,23,42,.08);}"
         << "*{box-sizing:border-box}body{margin:0;font-family:'IBM Plex Sans','Segoe UI',sans-serif;background:radial-gradient(circle at top left,#fff8ec,var(--bg));color:var(--ink)}"
         << "header{padding:32px 36px 16px}main{display:grid;gap:18px;padding:0 36px 36px}.grid{display:grid;gap:18px;grid-template-columns:repeat(auto-fit,minmax(260px,1fr))}"
         << ".card{background:var(--panel);border:1px solid rgba(15,23,42,.06);border-radius:22px;padding:18px;box-shadow:var(--shadow)}.metric{font-size:28px;font-weight:700;margin-top:8px}"
         << ".muted{color:var(--muted);font-size:14px}.pill{display:inline-block;padding:4px 10px;border-radius:999px;background:#d6f0ec;color:var(--accent);font-size:12px;font-weight:600}"
         << "svg.chart{width:100%;height:180px;background:linear-gradient(180deg,rgba(15,118,110,.08),rgba(255,255,255,0));border-radius:16px}.panel{white-space:pre-wrap;line-height:1.6;font-size:13px}"
         << "@media (max-width:768px){header,main{padding-left:18px;padding-right:18px}}</style></head><body>"
         << "<header><h1>颗粒物大气激光雷达 Demo</h1><p class=\"muted\">C++ 版本输出的静态结果页面，展示指标、质量控制、站点校准和失败案例摘要。</p></header><main>"
         // 顶部四个关键指标卡片
         << "<section class=\"grid\">"
         << "<div class=\"card\"><div class=\"muted\">PM2.5 RMSE</div><div class=\"metric\">" << data.at("metrics").at("pm25").at("rmse").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">PM10 RMSE</div><div class=\"metric\">" << data.at("metrics").at("pm10").at("rmse").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">热点 F1</div><div class=\"metric\">" << data.at("metrics").at("hotspot").at("f1").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">吞吐量</div><div class=\"metric\">" << data.at("metrics").at("runtime").at("throughput_profiles_per_s").number_value() << "</div></div>"
         // QC 折线卡片
         << "</section><section class=\"grid\">"
         << "<div class=\"card\"><div class=\"pill\">平均 SNR</div>" << snr_chart << "</div>"
         << "<div class=\"card\"><div class=\"pill\">平均处理时延</div>" << latency_chart << "</div>"
         // 三个 JSON 文本面板，由下方 JS 在加载时填充
         << "</section><section class=\"grid\">"
         << "<div class=\"card\"><div class=\"pill\">Source</div><div class=\"panel\" id=\"source-panel\"></div></div>"
         << "<div class=\"card\"><div class=\"pill\">Drift Monitoring</div><div class=\"panel\" id=\"drift-panel\"></div></div>"
         << "<div class=\"card\"><div class=\"pill\">Failure Cases</div><div class=\"panel\" id=\"failure-panel\"></div></div>"
         // 把整个 payload 嵌入到 script tag，前端用 JSON.parse 读取
         << "</section><script id=\"demo-data\" type=\"application/json\">" << embedded << "</script><script>"
         << "const data=JSON.parse(document.getElementById('demo-data').textContent);"
         << "document.getElementById('source-panel').textContent=JSON.stringify(data.source,null,2);"
         << "document.getElementById('drift-panel').textContent=JSON.stringify(data.drift_monitoring,null,2);"
         << "document.getElementById('failure-panel').textContent=JSON.stringify(data.failure_cases,null,2);"
         << "</script></main></body></html>";
    return html.str();
}

/**
 * @brief 从完整的 demo payload 中提取一个轻量摘要（便于上层 API 返回）。
 *
 * 仅保留 dataset_summary、metrics、最新热点列表与告警计数四个关键字段，
 * 适合用于 REST API 的"概览"端点。
 *
 * @param results 完整的 demo payload。
 * @return Json   摘要对象。
 */
Json build_summary_payload(const Json& results) {
    return Json::object_type{
        {"dataset_summary", results.at("dataset_summary")},
        {"metrics", results.at("metrics")},
        {"latest_hotspots", results.at("hotspots")},
        {"alert_count", static_cast<int>(results.at("alerts").array_items().size())},
    };
}

// ============================================================================
// 单步处理 API —— 供实时客户端逐帧处理使用
// ============================================================================

ProcessedProfile process_single_profile(
    const LidarProfile& profile,
    const RetrievalConfig& retrieval,
    const HumidityConfig& humidity
) {
    auto started_at = std::chrono::steady_clock::now();

    PreprocessResult preprocessed = preprocess_profile(profile);
    auto inversion = run_fernald_inversion(
        profile,
        preprocessed.attenuated_backscatter,
        retrieval.aerosol_lidar_ratio_sr,
        retrieval.reference_aerosol_backscatter
    );
    std::vector<double> dry_extinction = apply_humidity_correction(
        inversion.first,
        profile.relative_humidity,
        humidity.dry_reference_rh,
        humidity.hygroscopicity
    );

    // 经验 PM 估算：用干消光 × 转换系数近似 PM2.5（无地面标定时的粗略代理）
    // 典型城市气溶胶：1 km^-1 干消光 ≈ 20-30 µg/m³ PM2.5
    // PM10 ≈ PM2.5 × 1.5（粗颗粒贡献）
    std::vector<double> pm25;
    std::vector<double> pm10;
    pm25.reserve(dry_extinction.size());
    pm10.reserve(dry_extinction.size());
    for (double ext : dry_extinction) {
        double est_pm25 = std::max(0.0, ext) * 25.0; // 经验转换系数
        pm25.push_back(est_pm25);
        pm10.push_back(est_pm25 * 1.5);
    }

    auto ended_at = std::chrono::steady_clock::now();

    return ProcessedProfile{
        profile,
        preprocessed.l1_signal,
        preprocessed.attenuated_backscatter,
        preprocessed.snr,
        inversion.first,           // extinction
        dry_extinction,            // dry_extinction
        std::move(pm25),           // pm25 — 经验估算
        std::move(pm10),           // pm10 — 经验估算
        profile_bins_to_enu(profile),
        preprocessed.qc_flags,
        std::chrono::duration<double, std::milli>(ended_at - started_at).count(),
    };
}

std::vector<Hotspot> detect_hotspots_from_processed(
    const std::vector<ProcessedProfile>& ppi_profiles,
    const HotspotConfig& hotspot_cfg
) {
    // detect_hotspots 接受 ProcessedProfile* 指针列表
    std::vector<ProcessedProfile*> ptrs;
    ptrs.reserve(ppi_profiles.size());
    for (const auto& p : ppi_profiles) {
        ptrs.push_back(const_cast<ProcessedProfile*>(&p));
    }

    DetectionResult detection = detect_hotspots(
        ptrs,
        hotspot_cfg.pm25_threshold_ugm3,
        hotspot_cfg.scan_relative_pm25_threshold_ugm3,
        hotspot_cfg.scan_relative_dry_ext_threshold,
        hotspot_cfg.min_cells
    );
    return detection.hotspots;
}

std::vector<GroundMeasurement> extract_ground_measurements(const Json& results) {
    std::vector<GroundMeasurement> output;
    if (results.contains("source") && results.at("source").contains("ground_measurements")) {
        const auto& arr = results.at("source").at("ground_measurements");
        if (arr.is_array()) {
            for (const auto& item : arr.array_items()) {
                GroundMeasurement gm;
                gm.site_id = item.contains("site_id") ? item.at("site_id").string_value() : "";
                gm.timestamp = item.contains("timestamp") ? item.at("timestamp").string_value() : "";
                gm.pm25_ugm3 = item.contains("pm25_ugm3") ? item.at("pm25_ugm3").number_value() : 0.0;
                gm.pm10_ugm3 = item.contains("pm10_ugm3") ? item.at("pm10_ugm3").number_value() : 0.0;
                gm.relative_humidity = item.contains("relative_humidity") ? item.at("relative_humidity").number_value() : 0.0;
                gm.temperature_c = item.contains("temperature_c") ? item.at("temperature_c").number_value() : 0.0;
                gm.wind_speed_ms = item.contains("wind_speed_ms") ? item.at("wind_speed_ms").number_value() : 0.0;
                gm.wind_dir_deg = item.contains("wind_dir_deg") ? item.at("wind_dir_deg").number_value() : 0.0;
                output.push_back(std::move(gm));
            }
        }
    }
    return output;
}

Json to_json_processed(const ProcessedProfile& value) {
    return to_json(value);
}

Json to_json_hotspot(const Hotspot& value) {
    return to_json(value);
}

} // namespace lidar_demo