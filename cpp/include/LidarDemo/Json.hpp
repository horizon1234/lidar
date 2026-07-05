/**
 * @file Json.hpp
 * @brief Lightweight JSON value model and JSON file I/O API.
 */
#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace lidar_demo {

/**
 * @brief 轻量级 JSON 值模型
 *
 * 使用 std::variant 承载 null / bool / number(double) / string / array / object
 * 六种 JSON 值类型。提供类型查询、取值访问与键值查找等便捷接口，
 * 不依赖任何第三方 JSON 库。与配套的 parse_json / dump_json / read_json_file /
 * write_json_file 一起构成本项目自带的 JSON 引擎。
 */
struct Json {
    using Array = std::vector<Json>;                  ///< JSON 数组类型别名
    using Object = std::map<std::string, Json>;       ///< JSON 对象（键有序映射）类型别名
    using Value = std::variant<
        std::nullptr_t, bool, double, std::string,
        Array, Object>;                               ///< 底层的标签联合体类型

    Value value;   ///< 实际存储的 JSON 值

    /** @name 构造函数 */
    ///@{
    Json();                              ///< 构造一个 null 值（默认）
    Json(std::nullptr_t);                ///< 构造一个 null 值
    Json(bool input);                    ///< 构造一个布尔值
    Json(int input);                     ///< 构造一个数值（由 int 提升为 double）
    Json(double input);                  ///< 构造一个数值
    Json(const char* input);             ///< 构造一个字符串
    Json(const std::string& input);      ///< 构造一个字符串（拷贝）
    Json(std::string&& input);           ///< 构造一个字符串（移动）
    Json(const Array& input);            ///< 构造一个数组（拷贝）
    Json(Array&& input);                 ///< 构造一个数组（移动）
    Json(const Object& input);           ///< 构造一个对象（拷贝）
    Json(Object&& input);                ///< 构造一个对象（移动）
    ///@}

    /** @name 类型查询 */
    ///@{
    bool is_null() const;                ///< 是否为 null
    bool is_bool() const;                ///< 是否为布尔值
    bool is_number() const;              ///< 是否为数值
    bool is_string() const;              ///< 是否为字符串
    bool is_array() const;               ///< 是否为数组
    bool is_object() const;              ///< 是否为对象
    ///@}

    /** @name 取值访问 */
    ///@{
    bool bool_value() const;                 ///< 取布尔值（类型不符时抛异常）
    double number_value() const;             ///< 取数值（double）
    int int_value() const;                   ///< 取数值并四舍五入为 int
    const std::string& string_value() const; ///< 取字符串引用
    const Array& array_items() const;        ///< 取数组（只读引用）
    Array& array_items();                    ///< 取数组（可写引用）
    const Object& object_items() const;      ///< 取对象（只读引用）
    Object& object_items();                  ///< 取对象（可写引用）
    ///@}

    /** @name 键值访问 */
    ///@{
    bool contains(const std::string& key) const;          ///< 判断对象中是否包含指定键
    const Json& at(const std::string& key) const;         ///< 按 key 取子值（不存在时抛异常）
    Json& operator[](const std::string& key);             ///< 按 key 取子值（不存在时自动插入空对象）
    const Json& operator[](const std::string& key) const; ///< 按 key 取子值（只读，等价于 at）
    ///@}
};

/**
 * @brief 将一段文本解析为 Json 值
 * @param text 符合 JSON 语法的字符串
 * @return 解析得到的 Json 值
 * @throws std::runtime_error 当文本不符合 JSON 语法时抛出
 */
Json parse_json(const std::string& text);

/**
 * @brief 将 Json 值序列化为字符串
 * @param value  要序列化的 Json 值
 * @param indent 缩进空格数；传 0 时输出紧凑（无空白）格式
 * @return 序列化后的字符串
 */
std::string dump_json(const Json& value, int indent = 2);

/**
 * @brief 从文件读取并解析 JSON
 * @param path JSON 文件路径
 * @return 解析得到的 Json 值
 * @throws std::runtime_error 当文件无法打开或解析失败时抛出
 */
Json read_json_file(const std::filesystem::path& path);

/**
 * @brief 将 Json 值写入文件（会自动创建父目录）
 * @param path  目标文件路径
 * @param value 要写入的 Json 值
 */
void write_json_file(const std::filesystem::path& path, const Json& value);

} // namespace lidar_demo
