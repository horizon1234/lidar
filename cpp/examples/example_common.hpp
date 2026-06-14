// examples/example_common.hpp
//
// 教学算例共享工具:
//   - 小数学工具 (clamp, mean, deg2rad, gaussian)
//   - 极坐标 <-> ENU 转换
//   - 可重复的伪随机数生成器 (Box-Muller 高斯)
//   - 简易 JSON 字符串构造器 (只够教学输出使用)
//   - 文件写入 + 终端彩色打印
//
// 这些工具刻意写得简单,不依赖 STL 之外的任何库,这样学习者
// 直接 cpp 文件就能读懂算法本体,不会被框架代码绕进去。

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
// 启用 Windows 终端 ANSI 转义需要 SetConsoleMode
// NOMINMAX 防止 windows.h 把 min/max 定义成宏, 否则 std::min/std::max 会被破坏
#define NOMINMAX
#include <windows.h>
#endif

namespace example_common {

constexpr double PI = 3.14159265358979323846;

// ---- 小数学工具 ----

inline double clamp(double value, double lower, double upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

inline double deg2rad(double deg) { return deg * PI / 180.0; }

inline double mean(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    double sum = 0.0;
    for (double v : values) sum += v;
    return sum / static_cast<double>(values.size());
}

inline double gaussian1d(double x, double center, double sigma) {
    return std::exp(-((x - center) * (x - center)) / (2.0 * sigma * sigma + 1e-12));
}

// ---- 极坐标 -> ENU 三维坐标 ----
// 方位角 0° 指正北, 90° 指正东, 顺时针旋转 (气象/雷达约定)
inline void polar_to_enu(double range_m, double azimuth_deg, double elevation_deg,
                         double& east, double& north, double& up) {
    double az = deg2rad(azimuth_deg);
    double el = deg2rad(elevation_deg);
    double horizontal = range_m * std::cos(el);
    east = horizontal * std::sin(az);
    north = horizontal * std::cos(az);
    up = range_m * std::sin(el);
}

// ---- 简易可重复伪随机数生成器 (xorshift64) ----
// 用 xorshift64 而不是 <random>, 这样在 Windows/MSVC 与 MinGW 上都不同种子
// 行为完全一致, 教学算例输出可复现。
struct Rng {
    std::uint64_t state;
    explicit Rng(std::uint64_t seed) : state(seed ? seed : 0x9E3779B97F4A7C15ULL) {}

    std::uint64_t next_u64() {
        std::uint64_t x = state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        state = x;
        return x;
    }

    double next_uniform() {
        // 取高位 53 位作为 [0,1) 双精度
        return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0);
    }

    double next_gauss(double mu, double sigma) {
        // Box-Muller 变换
        double u1 = next_uniform();
        double u2 = next_uniform();
        if (u1 < 1e-12) u1 = 1e-12;
        double z = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * PI * u2);
        return mu + sigma * z;
    }
};

// ---- 简易 JSON 构造器 (只支持教学输出需要的子集) ----
// 这个类故意只实现 dump, 不做 parse。教学算例只需要把中间结果写出去,
// 不需要读取, 所以保持极简。
class JsonValue {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    JsonValue() : type_(Type::Null) {}
    static JsonValue number(double v) { JsonValue j; j.type_ = Type::Number; j.num_ = v; return j; }
    static JsonValue integer(int v) { JsonValue j; j.type_ = Type::Number; j.num_ = static_cast<double>(v); return j; }
    static JsonValue boolean(bool v) { JsonValue j; j.type_ = Type::Bool; j.b_ = v; return j; }
    static JsonValue string(const std::string& v) { JsonValue j; j.type_ = Type::String; j.str_ = v; return j; }
    static JsonValue array(const std::vector<JsonValue>& v) { JsonValue j; j.type_ = Type::Array; j.arr_ = v; return j; }
    static JsonValue object(const std::vector<std::pair<std::string, JsonValue>>& v) {
        JsonValue j; j.type_ = Type::Object; j.obj_ = v; return j;
    }

    std::string dump(int indent = 2, int depth = 0) const {
        std::ostringstream os;
        switch (type_) {
            case Type::Null:    os << "null"; break;
            case Type::Bool:    os << (b_ ? "true" : "false"); break;
            case Type::Number: {
                // 整数就输出整数, 否则 6 位小数
                double intpart;
                if (std::modf(num_, &intpart) == 0.0 && std::abs(num_) < 1e15) {
                    os << static_cast<long long>(num_);
                } else {
                    os << std::fixed << std::setprecision(6) << num_;
                }
                break;
            }
            case Type::String: {
                os << '"';
                for (char c : str_) {
                    switch (c) {
                        case '"':  os << "\\\""; break;
                        case '\\': os << "\\\\"; break;
                        case '\n': os << "\\n"; break;
                        case '\r': os << "\\r"; break;
                        case '\t': os << "\\t"; break;
                        default:
                            if (static_cast<unsigned char>(c) < 0x20) {
                                os << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                                   << static_cast<int>(static_cast<unsigned char>(c))
                                   << std::dec << std::setfill(' ');
                            } else {
                                os << c;
                            }
                    }
                }
                os << '"';
                break;
            }
            case Type::Array: {
                if (arr_.empty()) { os << "[]"; break; }
                std::string pad(indent * (depth + 1), ' ');
                std::string pad_end(indent * depth, ' ');
                os << "[\n";
                for (std::size_t i = 0; i < arr_.size(); ++i) {
                    os << pad << arr_[i].dump(indent, depth + 1);
                    if (i + 1 < arr_.size()) os << ",";
                    os << "\n";
                }
                os << pad_end << "]";
                break;
            }
            case Type::Object: {
                if (obj_.empty()) { os << "{}"; break; }
                std::string pad(indent * (depth + 1), ' ');
                std::string pad_end(indent * depth, ' ');
                os << "{\n";
                for (std::size_t i = 0; i < obj_.size(); ++i) {
                    os << pad << '"' << obj_[i].first << "\": " << obj_[i].second.dump(indent, depth + 1);
                    if (i + 1 < obj_.size()) os << ",";
                    os << "\n";
                }
                os << pad_end << "}";
                break;
            }
        }
        return os.str();
    }

private:
    Type type_;
    bool b_ = false;
    double num_ = 0.0;
    std::string str_;
    std::vector<JsonValue> arr_;
    std::vector<std::pair<std::string, JsonValue>> obj_;
};

inline std::vector<double> round6(const std::vector<double>& xs) {
    std::vector<double> out;
    out.reserve(xs.size());
    for (double x : xs) {
        out.push_back(std::round(x * 1e6) / 1e6);
    }
    return out;
}

inline std::vector<JsonValue> doubles_to_json(const std::vector<double>& xs) {
    std::vector<JsonValue> out;
    out.reserve(xs.size());
    for (double x : xs) out.push_back(JsonValue::number(x));
    return out;
}

inline std::vector<JsonValue> ints_to_json(const std::vector<int>& xs) {
    std::vector<JsonValue> out;
    out.reserve(xs.size());
    for (int x : xs) out.push_back(JsonValue::integer(x));
    return out;
}

// ---- 文件 IO ----

inline void ensure_parent_dir(const std::filesystem::path& path) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
}

inline void write_text_file(const std::filesystem::path& path, const std::string& text) {
    ensure_parent_dir(path);
    std::ofstream ofs(path, std::ios::binary);
    ofs << text;
}

inline void write_json_file(const std::filesystem::path& path, const JsonValue& value) {
    write_text_file(path, value.dump(2));
}

// ---- 终端彩色打印 (Windows 10+ 支持 ANSI 转义) ----

inline void enable_ansi_color() {
#ifdef _WIN32
    // Windows 10 1511+ 默认禁用 ANSI, 需要 SetConsoleMode 启用
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (h != INVALID_HANDLE_VALUE) {
        DWORD mode = 0;
        if (GetConsoleMode(h, &mode)) {
            SetConsoleMode(h, mode | 0x0004 /* ENABLE_VIRTUAL_TERMINAL_PROCESSING */);
        }
    }
#endif
}

inline void print_header(const std::string& title) {
    std::cout << "\n";
    std::cout << "========================================================================\n";
    std::cout << " " << title << "\n";
    std::cout << "========================================================================\n";
}

inline void print_step(const std::string& label) {
    std::cout << "\n[" << label << "]\n";
}

inline std::string now_iso_utc() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[40];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return std::string(buf);
}

}  // namespace example_common
