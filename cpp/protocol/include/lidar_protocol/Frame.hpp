/**
 * @file frame.hpp
 * @brief 传输层帧定义 —— JSON 行协议。
 *
 * 协议规则：
 *   - 每帧 = 一条合法 JSON 对象 + 换行符 '\n'
 *   - JSON 对象中必须包含 "type" 字段标识帧类型
 *   - 支持的帧类型见 FrameType 枚举
 *   - 所有数据载体字段使用 lidar_core::Json（即 lidar_demo::Json 别名）
 *
 * 这种设计复用了已有的 JSON 引擎，零外部依赖，且人可读、易调试。
 */
#pragma once

#include <string>
#include <vector>

#include "lidar_core/LidarCore.hpp"

namespace lidar_protocol {

/**
 * @brief 帧类型枚举。
 *
 * 对应 JSON 帧 "type" 字段的字符串值。
 */
enum class FrameType {
    lidar_raw,      ///< L0 原始 LiDAR 射线数据（仿真设备 → 客户端）
    lidar_l1,       ///< L1 预处理结果（可选中转）
    ground_obs,     ///< 地面观测数据
    status,         ///< 设备/系统状态（心跳、温度等）
    telemetry,      ///< 设备运行遥测
    camera,         ///< 同步相机能力、指向和可选图像引用元数据
    lidar_product,  ///< 未标定在线派生产品
    command,        ///< 控制命令（扫描模式切换、参数调整等）
    command_result, ///< 控制命令结构化执行结果
    hotspots,       ///< 检测到的热点列表
    summary,        ///< 周期性汇总摘要
    alarm,          ///< 告警事件
    heartbeat,      ///< 心跳帧
    unknown,        ///< 无法识别的帧类型
};

/**
 * @brief 将帧类型枚举转为字符串。
 */
std::string frame_type_to_string(FrameType type);

/**
 * @brief 将字符串转为帧类型枚举，无法匹配时返回 unknown。
 */
FrameType string_to_frame_type(const std::string& text);

/**
 * @brief 将帧类型字符串转为帧类型枚举（等价于 string_to_frame_type）。
 */
inline FrameType parse_frame_type(const std::string& text) {
    return string_to_frame_type(text);
}

/**
 * @brief 一个完整的传输帧。
 *
 * 一帧 = { "type": "...", "timestamp": "...", payload... }
 * payload 字段因帧类型而异，直接展开在同一 JSON 对象中。
 */
struct Frame {
    FrameType type = FrameType::unknown;  ///< 帧类型
    std::string timestamp;                ///< ISO8601 时间戳
    lidar_core::Json payload;             ///< 帧的有效载荷（JSON 对象）

    /**
     * @brief 把帧序列化为一行 JSON 文本（不含尾部换行符）。
     */
    std::string to_json_line() const;

    /**
     * @brief 把帧序列化为完整的传输行（JSON + '\n'）。
     */
    std::string to_wire() const;
};

/**
 * @brief 构建一帧。
 *
 * @param type 帧类型
 * @param timestamp 时间戳
 * @param payload 载荷（将被合并进顶层对象）
 * @return Frame
 */
Frame make_frame(FrameType type, const std::string& timestamp, lidar_core::Json payload);

/**
 * @brief 从一行 JSON 文本解析为 Frame。
 *
 * @param line 不含换行符的 JSON 文本
 * @return Frame
 * @throws std::runtime_error 当 JSON 解析失败或缺少必要字段时
 */
Frame parse_frame(const std::string& line);

/**
 * @brief 将一段多行文本按 '\n' 切分为多帧。
 *
 * 遇到解析失败的行会跳过（不抛异常），返回成功解析的帧列表。
 * 用于从 TCP 流缓冲区中批量提取帧。
 *
 * @param buffer 包含零到多行 JSON 的文本（可能含不完整尾行）
 * @param consumed 返回被成功消费的字节数（即完整行的总长度）
 * @return 成功解析出的帧列表
 */
std::vector<Frame> parse_frames_from_buffer(const std::string& buffer, std::size_t& consumed);

// ---- 高层序列化辅助：LidarProfile → JSON 帧 payload ----

/**
 * @brief 将一条 LidarProfile 转为 JSON 载荷对象。
 *
 * 始终包含角度、距离轴、兼容主通道、四物理通道、退偏比和气象量；分子场与仿真
 * 真值由 include_truth_fields 控制。实时设备默认关闭真值，避免误当实测数据并降低带宽。
 *
 * @param profile 待序列化的原始射线。
 * @param include_truth_fields 是否附加只用于算法验证的分子场和真值场。
 * @return 可直接作为 lidar_raw 帧 payload 的 JSON 对象。
 */
lidar_core::Json profile_to_json(
    const lidar_core::LidarProfile& profile,
    bool include_truth_fields = true);

/**
 * @brief 从 JSON 载荷还原 LidarProfile。
 */
lidar_core::LidarProfile json_to_profile(const lidar_core::Json& json);

/**
 * @brief 将一条 GroundMeasurement 转为 JSON 载荷对象。
 */
lidar_core::Json ground_to_json(const lidar_core::GroundMeasurement& ground);

/**
 * @brief 从 JSON 载荷还原 GroundMeasurement。
 */
lidar_core::GroundMeasurement json_to_ground(const lidar_core::Json& json);

} // namespace lidar_protocol
