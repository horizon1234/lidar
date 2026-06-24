/**
 * @file wire_format.cpp
 * @brief 传输层帧的序列化/反序列化实现。
 */
#include "lidar_protocol/frame.hpp"

#include <algorithm>
#include <sstream>

namespace lidar_protocol {

// ---- FrameType 转换 ----

std::string frame_type_to_string(FrameType type) {
    switch (type) {
    case FrameType::lidar_raw:  return "lidar_raw";
    case FrameType::lidar_l1:   return "lidar_l1";
    case FrameType::ground_obs: return "ground_obs";
    case FrameType::status:     return "status";
    case FrameType::command:    return "command";
    case FrameType::hotspots:   return "hotspots";
    case FrameType::summary:    return "summary";
    case FrameType::alarm:      return "alarm";
    case FrameType::heartbeat:  return "heartbeat";
    default:                    return "unknown";
    }
}

FrameType string_to_frame_type(const std::string& text) {
    if (text == "lidar_raw")  return FrameType::lidar_raw;
    if (text == "lidar_l1")   return FrameType::lidar_l1;
    if (text == "ground_obs") return FrameType::ground_obs;
    if (text == "status")     return FrameType::status;
    if (text == "command")    return FrameType::command;
    if (text == "hotspots")   return FrameType::hotspots;
    if (text == "summary")    return FrameType::summary;
    if (text == "alarm")      return FrameType::alarm;
    if (text == "heartbeat")  return FrameType::heartbeat;
    return FrameType::unknown;
}

// ---- 帧构建与序列化 ----

Frame make_frame(FrameType type, const std::string& timestamp, lidar_core::Json payload) {
    Frame frame;
    frame.type = type;
    frame.timestamp = timestamp;
    frame.payload = std::move(payload);
    return frame;
}

std::string Frame::to_json_line() const {
    lidar_core::Json root = lidar_core::Json::object_type{};
    root["type"] = frame_type_to_string(type);
    root["timestamp"] = timestamp;

    // 合并 payload 字段到顶层（payload 必须是对象）
    if (payload.is_object()) {
        for (const auto& [key, value] : payload.object_items()) {
            root[key] = value;
        }
    }

    return lidar_core::dump_json(root, 0); // 紧凑格式，省带宽
}

std::string Frame::to_wire() const {
    return to_json_line() + "\n";
}

Frame parse_frame(const std::string& line) {
    lidar_core::Json root = lidar_core::parse_json(line);

    Frame frame;
    if (root.contains("type") && root.at("type").is_string()) {
        frame.type = string_to_frame_type(root.at("type").string_value());
    } else {
        throw std::runtime_error("Frame missing 'type' field");
    }

    if (root.contains("timestamp") && root.at("timestamp").is_string()) {
        frame.timestamp = root.at("timestamp").string_value();
    }

    // 提取 payload = root 去掉 type，保留 timestamp 供反序列化使用
    lidar_core::Json payload = lidar_core::Json::object_type{};
    for (const auto& [key, value] : root.object_items()) {
        if (key != "type") {
            payload[key] = value;
        }
    }
    frame.payload = std::move(payload);
    return frame;
}

std::vector<Frame> parse_frames_from_buffer(const std::string& buffer, std::size_t& consumed) {
    std::vector<Frame> frames;
    consumed = 0;

    std::size_t start = 0;
    while (true) {
        std::size_t newline_pos = buffer.find('\n', start);
        if (newline_pos == std::string::npos) {
            break; // 不完整的尾行，等待更多数据
        }

        std::string line = buffer.substr(start, newline_pos - start);
        consumed = newline_pos + 1;

        // 跳过空行
        if (line.empty() || std::all_of(line.begin(), line.end(), [](char c) { return std::isspace(static_cast<unsigned char>(c)); })) {
            start = newline_pos + 1;
            continue;
        }

        try {
            frames.push_back(parse_frame(line));
        } catch (const std::exception&) {
            // 解析失败的行跳过，不中断流
        }

        start = newline_pos + 1;
    }

    return frames;
}

// ---- LidarProfile ↔ JSON 载荷 ----

namespace {

/// 将 JSON 数组转为 double 向量
std::vector<double> json_array_to_doubles(const lidar_core::Json& json) {
    std::vector<double> output;
    if (json.is_array()) {
        for (const auto& item : json.array_items()) {
            output.push_back(item.number_value());
        }
    }
    return output;
}

/// 将 JSON 数组转为 int 向量
std::vector<int> json_array_to_ints(const lidar_core::Json& json) {
    std::vector<int> output;
    if (json.is_array()) {
        for (const auto& item : json.array_items()) {
            output.push_back(item.int_value());
        }
    }
    return output;
}

/// 将 double 向量转为 JSON 数组
lidar_core::Json doubles_to_json_array(const std::vector<double>& values) {
    lidar_core::Json::array_type output;
    output.reserve(values.size());
    for (double v : values) {
        output.emplace_back(v);
    }
    return lidar_core::Json(std::move(output));
}

} // anonymous namespace

lidar_core::Json profile_to_json(const lidar_core::LidarProfile& profile) {
    lidar_core::Json obj = lidar_core::Json::object_type{
        {"site_id", profile.site_id},
        {"timestamp", profile.timestamp},
        {"scan_id", profile.scan_id},
        {"scan_mode", profile.scan_mode},
        {"source_kind", profile.source_kind},
        {"azimuth_deg", profile.azimuth_deg},
        {"elevation_deg", profile.elevation_deg},
        {"ranges_m", doubles_to_json_array(profile.ranges_m)},
        {"raw_counts", doubles_to_json_array(profile.raw_counts)},
        {"laser_energy_mj", profile.laser_energy_mj},
        {"background_counts", profile.background_counts},
        {"overlap", doubles_to_json_array(profile.overlap)},
        {"relative_humidity", profile.relative_humidity},
        {"temperature_c", profile.temperature_c},
        {"wind_speed_ms", profile.wind_speed_ms},
        {"wind_dir_deg", profile.wind_dir_deg},
        {"molecular_backscatter", doubles_to_json_array(profile.molecular_backscatter)},
        {"molecular_extinction", doubles_to_json_array(profile.molecular_extinction)},
        {"true_backscatter", doubles_to_json_array(profile.true_backscatter)},
        {"true_extinction", doubles_to_json_array(profile.true_extinction)},
        {"true_pm25", doubles_to_json_array(profile.true_pm25)},
        {"true_pm10", doubles_to_json_array(profile.true_pm10)},
    };
    // true_hotspot_mask 是可选字段，单独添加以避免初始 map 过大
    {
        lidar_core::Json::array_type mask_arr;
        mask_arr.reserve(profile.true_hotspot_mask.size());
        for (int v : profile.true_hotspot_mask) {
            mask_arr.emplace_back(static_cast<double>(v));
        }
        obj["true_hotspot_mask"] = lidar_core::Json(std::move(mask_arr));
    }
    return obj;
}

lidar_core::LidarProfile json_to_profile(const lidar_core::Json& json) {
    lidar_core::LidarProfile profile;
    auto get_string = [&](const char* key) -> std::string {
        return (json.contains(key) && json.at(key).is_string()) ? json.at(key).string_value() : "";
    };
    auto get_number = [&](const char* key) -> double {
        return (json.contains(key) && json.at(key).is_number()) ? json.at(key).number_value() : 0.0;
    };

    profile.site_id = get_string("site_id");
    profile.timestamp = get_string("timestamp");
    profile.scan_id = get_string("scan_id");
    profile.scan_mode = get_string("scan_mode");
    profile.source_kind = get_string("source_kind");
    profile.azimuth_deg = get_number("azimuth_deg");
    profile.elevation_deg = get_number("elevation_deg");
    profile.laser_energy_mj = get_number("laser_energy_mj");
    profile.background_counts = get_number("background_counts");
    profile.relative_humidity = get_number("relative_humidity");
    profile.temperature_c = get_number("temperature_c");
    profile.wind_speed_ms = get_number("wind_speed_ms");
    profile.wind_dir_deg = get_number("wind_dir_deg");

    if (json.contains("ranges_m"))             profile.ranges_m = json_array_to_doubles(json.at("ranges_m"));
    if (json.contains("raw_counts"))           profile.raw_counts = json_array_to_doubles(json.at("raw_counts"));
    if (json.contains("overlap"))              profile.overlap = json_array_to_doubles(json.at("overlap"));
    if (json.contains("molecular_backscatter"))profile.molecular_backscatter = json_array_to_doubles(json.at("molecular_backscatter"));
    if (json.contains("molecular_extinction")) profile.molecular_extinction = json_array_to_doubles(json.at("molecular_extinction"));
    if (json.contains("true_backscatter"))     profile.true_backscatter = json_array_to_doubles(json.at("true_backscatter"));
    if (json.contains("true_extinction"))      profile.true_extinction = json_array_to_doubles(json.at("true_extinction"));
    if (json.contains("true_pm25"))            profile.true_pm25 = json_array_to_doubles(json.at("true_pm25"));
    if (json.contains("true_pm10"))            profile.true_pm10 = json_array_to_doubles(json.at("true_pm10"));
    if (json.contains("true_hotspot_mask"))    profile.true_hotspot_mask = json_array_to_ints(json.at("true_hotspot_mask"));

    return profile;
}

lidar_core::Json ground_to_json(const lidar_core::GroundMeasurement& ground) {
    return lidar_core::Json::object_type{
        {"site_id", ground.site_id},
        {"timestamp", ground.timestamp},
        {"pm25_ugm3", ground.pm25_ugm3},
        {"pm10_ugm3", ground.pm10_ugm3},
        {"relative_humidity", ground.relative_humidity},
        {"temperature_c", ground.temperature_c},
        {"wind_speed_ms", ground.wind_speed_ms},
        {"wind_dir_deg", ground.wind_dir_deg},
    };
}

lidar_core::GroundMeasurement json_to_ground(const lidar_core::Json& json) {
    lidar_core::GroundMeasurement ground;
    auto get_string = [&](const char* key) -> std::string {
        return (json.contains(key) && json.at(key).is_string()) ? json.at(key).string_value() : "";
    };
    auto get_number = [&](const char* key) -> double {
        return (json.contains(key) && json.at(key).is_number()) ? json.at(key).number_value() : 0.0;
    };

    ground.site_id = get_string("site_id");
    ground.timestamp = get_string("timestamp");
    ground.pm25_ugm3 = get_number("pm25_ugm3");
    ground.pm10_ugm3 = get_number("pm10_ugm3");
    ground.relative_humidity = get_number("relative_humidity");
    ground.temperature_c = get_number("temperature_c");
    ground.wind_speed_ms = get_number("wind_speed_ms");
    ground.wind_dir_deg = get_number("wind_dir_deg");

    return ground;
}

} // namespace lidar_protocol
