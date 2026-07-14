/**
 * @file ExampleCommon.hpp
 * @brief 教学算例共享工具头文件 (手册第 19 章使用)
 *
 * 本头文件提供 lidar_demo 教学算例所需的全部"非核心"辅助功能,
 * 刻意保持 STL-only、不依赖 lidar_demo 主库, 这样学习者直接打开
 * cpp 文件即可读懂算法本体, 不会被框架代码绕进去。
 *
 * 涵盖内容:
 *   - 数学工具: clamp / deg2rad / mean / gaussian1d / polar_to_enu
 *   - 可重复伪随机数生成器 Rng (xorshift64 PRNG + Box-Muller 高斯),
 *     用于 Linux GCC/Clang 环境复现
 *   - 极简 dump-only JSON 构造器 JsonValue
 *     (Number / Bool / String / Array / Object)
 *   - 数值辅助: round6 (向量与标量重载), doubles_to_json, ints_to_json
 *   - 文件 IO: ensure_parent_dir / write_text_file / write_json_file
 *   - 彩色打印与时间戳: enable_ansi_color / print_header / print_step / now_iso_utc
 *
 * @note 本文件仅作为教学辅助, 性能与功能覆盖均不为生产场景设计。
 */

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

namespace example_common {

/// @brief 圆周率常数 (双精度)
constexpr double PI = 3.14159265358979323846;

// ============================================================================
// ---- 数学工具 ----
// ============================================================================

/**
 * @brief 将数值限制在 [lower, upper] 闭区间内
 * @param value 待限幅的浮点数
 * @param lower 下界
 * @param upper 上界
 * @return 若 value < lower 返回 lower; 若 value > upper 返回 upper; 否则返回 value
 */
inline double clamp(double value, double lower, double upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

/**
 * @brief 角度转弧度
 * @param deg 角度值 (单位 度)
 * @return 对应弧度值 = deg * PI / 180
 */
inline double deg2rad(double deg) { return deg * PI / 180.0; }

/**
 * @brief 计算向量算术平均值
 * @param values 输入的浮点向量 (允许为空)
 * @return 平均值; 空向量返回 0.0 以避免除零
 */
inline double mean(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    double sum = 0.0;
    for (double v : values) sum += v;
    return sum / static_cast<double>(values.size());
}

/**
 * @brief 一维高斯函数 exp(-((x-center)^2) / (2*sigma^2))
 * @param x 自变量
 * @param center 高斯峰中心位置 (均值)
 * @param sigma 标准差 sigma (不能为 0)
 * @return 高斯概率密度形状的归一化前幅值 (峰值 = 1)
 * @note 分母加 1e-12 是为了在 sigma 接近 0 时避免除零, 数值稳定保护
 */
inline double gaussian1d(double x, double center, double sigma) {
    return std::exp(-((x - center) * (x - center)) / (2.0 * sigma * sigma + 1e-12));
}

// ============================================================================
// ---- 极坐标 -> ENU 三维坐标 ----
// 方位角 0° 指正北, 90° 指正东, 顺时针旋转 (气象/雷达约定)
// ============================================================================

/**
 * @brief 将 (距离, 方位角, 仰角) 极坐标转换为 ENU (东-北-天) 笛卡尔坐标
 *
 * 采用气象/雷达约定: 方位角 0° 指正北、顺时针增加、90° 指正东。
 * 公式 (其中 horizontal = range * cos(el)):
 *   - east  = horizontal * sin(az)   ///< 东向分量
 *   - north = horizontal * cos(az)   ///< 北向分量
 *   - up    = range * sin(el)        ///< 垂直分量
 *
 * @param range_m       斜距 (米)
 * @param azimuth_deg   方位角 (度), 0°=北, 90°=东
 * @param elevation_deg 仰角 (度), 水平=0°, 垂直向上=90°
 * @param east   [out] 输出东向坐标 (米)
 * @param north  [out] 输出北向坐标 (米)
 * @param up     [out] 输出天向坐标 (米)
 */
inline void polar_to_enu(double range_m, double azimuth_deg, double elevation_deg,
                         double& east, double& north, double& up) {
    double az = deg2rad(azimuth_deg);
    double el = deg2rad(elevation_deg);
    double horizontal = range_m * std::cos(el);   ///< 水平面投影距离
    east = horizontal * std::sin(az);             ///< east = R*cos(el)*sin(az)
    north = horizontal * std::cos(az);            ///< north = R*cos(el)*cos(az)
    up = range_m * std::sin(el);                  ///< up = R*sin(el)
}

// ============================================================================
// ---- 简易可重复伪随机数生成器 (xorshift64) ----
// 用 xorshift64 而不是 <random>, 这样在 Windows/MSVC 与 MinGW 上都不同种子
// 行为完全一致, 教学算例输出可复现。
// ============================================================================

/**
 * @brief 基于 xorshift64 的可重复伪随机数生成器
 *
 * 算法 (xorshift64, Marsaglia 2003):
 *   - 三个异或移位操作: x ^= x<<13; x ^= x>>7; x ^= x<<17
 *   - 64 位状态, 周期 2^64 - 1, 速度快、实现简单
 *   - 配合 Box-Muller 变换得到标准正态分布
 *
 * 刻意自实现而非使用 <random>, 是为了保证在 MSVC / MinGW / GCC / Clang
 * 各编译器下相同种子输出完全一致, 教学算例可逐位复现。
 */
struct Rng {
    std::uint64_t state;   ///< 64 位内部状态

    /**
     * @brief 构造并初始化状态
     * @param seed 种子值; 若为 0 则使用黄金分割常数 0x9E3779B97F4A7C15 作为默认种子
     */
    explicit Rng(std::uint64_t seed) : state(seed ? seed : 0x9E3779B97F4A7C15ULL) {}

    /**
     * @brief 生成一个 64 位无符号随机数 (xorshift64 主体)
     * @return 下一随机 64 位整数
     *
     * 数学: 状态 x 经过三步异或移位 (13, 7, 17) 后回写并返回,
     * 这是 Marsaglia 给出的经典 xorshift64 参数三元组。
     */
    std::uint64_t next_u64() {
        std::uint64_t x = state;
        x ^= x << 13;   ///< 第一轮左移异或
        x ^= x >> 7;    ///< 第二轮右移异或
        x ^= x << 17;   ///< 第三轮左移异或
        state = x;
        return x;
    }

    /**
     * @brief 生成 [0, 1) 区间均匀分布的双精度随机数
     * @return [0,1) 之间的 double
     *
     * 方法: 取 next_u64() 结果的高 53 位除以 2^53, 保证双精度尾部
     * 不损失精度 (`>> 11` 跳过低 11 位尾噪声)。
     */
    double next_uniform() {
        // 取高位 53 位作为 [0,1) 双精度
        return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0);
    }

    /**
     * @brief 生成均值为 mu、标准差为 sigma 的高斯随机数 (Box-Muller 变换)
     * @param mu   目标均值
     * @param sigma 目标标准差 (sigma > 0)
     * @return 满足 N(mu, sigma^2) 分布的一次抽样值
     *
     * 数学 (Box-Muller 变换): 取两独立均匀随机数 u1, u2 ∈ (0,1), 则
     *   z = sqrt(-2 ln(u1)) * cos(2π u2)
     * 服从标准正态 N(0,1)。再线性变换: result = mu + sigma * z。
     * 对 u1 取下限 1e-12 以避免 log(0) 跳变。
     */
    double next_gauss(double mu, double sigma) {
        // Box-Muller 变换
        double u1 = next_uniform();
        double u2 = next_uniform();
        if (u1 < 1e-12) u1 = 1e-12;                                 ///< 避免 log(0)
        double z = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * PI * u2);
        return mu + sigma * z;
    }
};

// ============================================================================
// ---- 简易 JSON 构造器 (只支持教学输出需要的子集) ----
// 这个类故意只实现 dump, 不做 parse。教学算例只需要把中间结果写出去,
// 不需要读取, 所以保持极简。
// ============================================================================

/**
 * @brief 极简 dump-only JSON 值构造器
 *
 * 支持 Null / Bool / Number / String / Array / Object 六种 JSON 类型。
 * 仅实现 dump() 序列化, 不做 parse(), 因为教学算例只需把中间结果写出去。
 */
class JsonValue {
public:
    /// @brief JSON 节点支持的类型枚举
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

    /**
     * @brief 将 JSON 值序列化为字符串
     * @param indent 每一级缩进的空格数 (默认 2)
     * @param depth  当前所处嵌套深度 (递归使用, 外部调用默认 0)
     * @return JSON 文本
     *
     * - Number 类型若为整数 (|v|<1e15) 则输出整数形式, 否则 6 位小数;
     * - String 类型对 `"`、`\`、控制字符进行转义;
     * - Array / Object 多行缩进输出。
     */
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
                        case '"':  os << "\\\""; break;   ///< 转义双引号
                        case '\\': os << "\\\\"; break;   ///< 转义反斜杠
                        case '\n': os << "\\n"; break;    ///< 转义换行
                        case '\r': os << "\\r"; break;    ///< 转义回车
                        case '\t': os << "\\t"; break;    ///< 转义制表符
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
                std::string pad(indent * (depth + 1), ' ');      ///< 子元素缩进
                std::string pad_end(indent * depth, ' ');        ///< 闭括号缩进
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
                std::string pad(indent * (depth + 1), ' ');      ///< 子元素缩进
                std::string pad_end(indent * depth, ' ');        ///< 闭括号缩进
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
    Type type_;                                              ///< 当前节点类型
    bool b_ = false;                                         ///< Bool 值存储
    double num_ = 0.0;                                       ///< Number 值存储
    std::string str_;                                        ///< String 值存储
    std::vector<JsonValue> arr_;                             ///< Array 子节点
    std::vector<std::pair<std::string, JsonValue>> obj_;     ///< Object 键值对
};

/**
 * @brief 对向量逐元素保留 6 位小数 (四舍五入)
 * @param xs 输入向量
 * @return 四舍五入到 1e-6 精度的新向量
 */
inline std::vector<double> round6(const std::vector<double>& xs) {
    std::vector<double> out;
    out.reserve(xs.size());
    for (double x : xs) {
        out.push_back(std::round(x * 1e6) / 1e6);
    }
    return out;
}

/**
 * @brief 标量保留 6 位小数 (round6 的标量重载)
 * @param x 输入标量
 * @return 四舍五入到 1e-6 精度的值
 */
inline double round6(double x) {
    return std::round(x * 1e6) / 1e6;
}

/**
 * @brief 将 double 向量转换为 JsonValue 数组
 * @param xs 输入向量
 * @return 元素均为 Number 类型的 JsonValue 数组
 */
inline std::vector<JsonValue> doubles_to_json(const std::vector<double>& xs) {
    std::vector<JsonValue> out;
    out.reserve(xs.size());
    for (double x : xs) out.push_back(JsonValue::number(x));
    return out;
}

/**
 * @brief 将 int 向量转换为 JsonValue 数组
 * @param xs 输入向量
 * @return 元素均为整型 Number 的 JsonValue 数组
 */
inline std::vector<JsonValue> ints_to_json(const std::vector<int>& xs) {
    std::vector<JsonValue> out;
    out.reserve(xs.size());
    for (int x : xs) out.push_back(JsonValue::integer(x));
    return out;
}

// ============================================================================
// ---- 文件 IO ----
// ============================================================================

/**
 * @brief 确保给定路径的父目录存在 (不存在则递归创建)
 * @param path 目标文件路径
 */
inline void ensure_parent_dir(const std::filesystem::path& path) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
}

/**
 * @brief 将文本写入文件 (覆盖写, 二进制模式)
 * @param path 目标文件路径 (父目录会自动创建)
 * @param text 待写入文本
 */
inline void write_text_file(const std::filesystem::path& path, const std::string& text) {
    ensure_parent_dir(path);
    std::ofstream ofs(path, std::ios::binary);
    ofs << text;
}

/**
 * @brief 将 JsonValue 序列化并以 2 空格缩进写入文件
 * @param path  目标文件路径
 * @param value 待写入的 JSON 值
 */
inline void write_json_file(const std::filesystem::path& path, const JsonValue& value) {
    write_text_file(path, value.dump(2));
}

// ============================================================================
// ---- 终端彩色打印 ----
// ============================================================================

/**
 * @brief 启用终端 ANSI 颜色转义支持
 *
 * Linux 终端原生支持 ANSI 转义，无需额外初始化；保留函数是为了让三个教学
 * 算例使用统一入口。
 */
inline void enable_ansi_color() {}

/**
 * @brief 打印章节标题分隔栏 (用于教学算例输出可读性)
 * @param title 标题文本
 */
inline void print_header(const std::string& title) {
    std::cout << "\n";
    std::cout << "========================================================================\n";
    std::cout << " " << title << "\n";
    std::cout << "========================================================================\n";
}

/**
 * @brief 打印步骤标签前缀
 * @param label 步骤名称
 */
inline void print_step(const std::string& label) {
    std::cout << "\n[" << label << "]\n";
}

/**
 * @brief 获取当前 UTC 时间戳 (ISO 8601 格式, 精度到秒)
 * @return 形如 "2025-01-15T08:30:00Z" 的字符串
 *
 * 使用 POSIX gmtime_r 实现线程安全的时间转换，用于教学输出中的时间标注。
 */
inline std::string now_iso_utc() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[40];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return std::string(buf);
}

}  // namespace example_common
