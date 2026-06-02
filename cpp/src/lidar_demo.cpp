#include "lidar_demo/lidar_demo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winhttp.h>
#endif

#if defined(LIDAR_DEMO_HAS_NETCDF)
#include <netcdf.h>
#endif

namespace lidar_demo {

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

bool Json::is_null() const { return std::holds_alternative<std::nullptr_t>(value); }
bool Json::is_bool() const { return std::holds_alternative<bool>(value); }
bool Json::is_number() const { return std::holds_alternative<double>(value); }
bool Json::is_string() const { return std::holds_alternative<std::string>(value); }
bool Json::is_array() const { return std::holds_alternative<array_type>(value); }
bool Json::is_object() const { return std::holds_alternative<object_type>(value); }

bool Json::bool_value() const { return std::get<bool>(value); }
double Json::number_value() const { return std::get<double>(value); }
int Json::int_value() const { return static_cast<int>(std::lround(number_value())); }
const std::string& Json::string_value() const { return std::get<std::string>(value); }
const Json::array_type& Json::array_items() const { return std::get<array_type>(value); }
Json::array_type& Json::array_items() { return std::get<array_type>(value); }
const Json::object_type& Json::object_items() const { return std::get<object_type>(value); }
Json::object_type& Json::object_items() { return std::get<object_type>(value); }

bool Json::contains(const std::string& key) const {
    return is_object() && object_items().find(key) != object_items().end();
}

const Json& Json::at(const std::string& key) const {
    return object_items().at(key);
}

Json& Json::operator[](const std::string& key) {
    if (!is_object()) {
        value = object_type{};
    }
    return object_items()[key];
}

const Json& Json::operator[](const std::string& key) const {
    return at(key);
}

namespace {

class JsonParser {
public:
    explicit JsonParser(const std::string& input) : text_(input) {}

    Json parse() {
        Json result = parse_value();
        skip_whitespace();
        if (position_ != text_.size()) {
            throw std::runtime_error("Unexpected trailing characters in JSON");
        }
        return result;
    }

private:
    const std::string& text_;
    std::size_t position_ = 0;

    void skip_whitespace() {
        while (position_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[position_]))) {
            ++position_;
        }
    }

    char peek() const {
        if (position_ >= text_.size()) {
            throw std::runtime_error("Unexpected end of JSON input");
        }
        return text_[position_];
    }

    void expect(char expected) {
        skip_whitespace();
        if (peek() != expected) {
            throw std::runtime_error("Unexpected JSON token");
        }
        ++position_;
    }

    bool consume(char token) {
        skip_whitespace();
        if (position_ < text_.size() && text_[position_] == token) {
            ++position_;
            return true;
        }
        return false;
    }

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

    Json parse_object() {
        expect('{');
        Json::object_type object;
        skip_whitespace();
        if (consume('}')) {
            return Json(std::move(object));
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
                break;
            }
            expect(',');
        }
        return Json(std::move(object));
    }

    Json parse_array() {
        expect('[');
        Json::array_type array;
        skip_whitespace();
        if (consume(']')) {
            return Json(std::move(array));
        }
        while (true) {
            array.push_back(parse_value());
            skip_whitespace();
            if (consume(']')) {
                break;
            }
            expect(',');
        }
        return Json(std::move(array));
    }

    std::string parse_string() {
        expect('"');
        std::string output;
        while (position_ < text_.size()) {
            char current = text_[position_++];
            if (current == '"') {
                return output;
            }
            if (current != '\\') {
                output.push_back(current);
                continue;
            }
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
                if (position_ + 4 > text_.size()) {
                    throw std::runtime_error("Invalid unicode escape in JSON string");
                }
                std::string hex = text_.substr(position_, 4);
                position_ += 4;
                int code_point = std::stoi(hex, nullptr, 16);
                if (code_point <= 0x7F) {
                    output.push_back(static_cast<char>(code_point));
                } else {
                    output.push_back('?');
                }
                break;
            }
            default:
                throw std::runtime_error("Unsupported JSON escape sequence");
            }
        }
        throw std::runtime_error("Unterminated JSON string");
    }

    Json parse_number() {
        skip_whitespace();
        std::size_t start = position_;
        if (text_[position_] == '-') {
            ++position_;
        }
        while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_]))) {
            ++position_;
        }
        if (position_ < text_.size() && text_[position_] == '.') {
            ++position_;
            while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_]))) {
                ++position_;
            }
        }
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

    void parse_literal(const std::string& literal) {
        skip_whitespace();
        if (text_.substr(position_, literal.size()) != literal) {
            throw std::runtime_error("Unexpected JSON literal");
        }
        position_ += literal.size();
    }
};

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
            if (static_cast<unsigned char>(current) < 0x20) {
                builder << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(current)) << std::dec;
            } else {
                builder << current;
            }
        }
    }
    return builder.str();
}

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

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

double mean(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double mean_int(const std::vector<int>& values) {
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 0) {
        return (values[middle - 1] + values[middle]) / 2.0;
    }
    return values[middle];
}

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

double gaussian(double distance, double center, double sigma) {
    return std::exp(-((distance - center) * (distance - center)) / std::max(2.0 * sigma * sigma, 1e-6));
}

double azimuth_delta(double left_deg, double right_deg) {
    double raw = std::fmod(std::abs(left_deg - right_deg), 360.0);
    return std::min(raw, 360.0 - raw);
}

std::tm safe_localtime(std::time_t value) {
    std::tm result{};
#ifdef _WIN32
    localtime_s(&result, &value);
#else
    localtime_r(&value, &result);
#endif
    return result;
}

std::string format_timestamp(std::time_t value) {
    std::tm stamp = safe_localtime(value);
    std::ostringstream output;
    output << std::put_time(&stamp, "%Y-%m-%dT%H:%M");
    return output.str();
}

std::time_t parse_timestamp(const std::string& text) {
    std::tm stamp{};
    std::istringstream input(text);
    input >> std::get_time(&stamp, "%Y-%m-%dT%H:%M");
    if (input.fail()) {
        throw std::runtime_error("Failed to parse timestamp: " + text);
    }
    stamp.tm_isdst = -1;
    return std::mktime(&stamp);
}

#if defined(LIDAR_DEMO_HAS_NETCDF)

void require_netcdf(int status, const std::string& context) {
    if (status != NC_NOERR) {
        throw std::runtime_error(context + ": " + nc_strerror(status));
    }
}

bool is_ascii_text(const std::string& value) {
    return std::all_of(value.begin(), value.end(), [](unsigned char current) {
        return current < 128;
    });
}

std::filesystem::path ensure_parent_path(const std::filesystem::path& path) {
    std::filesystem::create_directories(path.parent_path());
    return path;
}

std::filesystem::path netcdf_compatible_path(const std::filesystem::path& input_path) {
    std::filesystem::path resolved = std::filesystem::absolute(input_path);
    std::string path_text = resolved.string();
    if (is_ascii_text(path_text)) {
        return resolved;
    }

    std::filesystem::path cache_root = resolved.root_path() / "temp" / "lidar_cloudnet_cache";
    std::filesystem::path mirrored_path = ensure_parent_path(cache_root / resolved.filename());
    if (!std::filesystem::exists(mirrored_path) || std::filesystem::file_size(mirrored_path) != std::filesystem::file_size(resolved)) {
        std::filesystem::copy_file(resolved, mirrored_path, std::filesystem::copy_options::overwrite_existing);
    }
    return mirrored_path;
}

class NetcdfFile {
public:
    explicit NetcdfFile(const std::filesystem::path& path) {
        require_netcdf(nc_open(path.string().c_str(), NC_NOWRITE, &id_), "Failed to open netCDF file " + path.string());
    }

    ~NetcdfFile() {
        if (id_ >= 0) {
            nc_close(id_);
        }
    }

    int id() const { return id_; }

private:
    int id_ = -1;
};

int variable_id(int ncid, const std::string& name) {
    int varid = -1;
    require_netcdf(nc_inq_varid(ncid, name.c_str(), &varid), "Missing netCDF variable " + name);
    return varid;
}

bool has_variable(int ncid, const std::string& name) {
    int varid = -1;
    return nc_inq_varid(ncid, name.c_str(), &varid) == NC_NOERR;
}

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

std::vector<double> read_numeric_variable(int ncid, const std::string& name) {
    int varid = variable_id(ncid, name);
    std::vector<double> output(variable_size(ncid, varid), 0.0);
    require_netcdf(nc_get_var_double(ncid, varid, output.data()), "Failed to read netCDF variable " + name);
    return output;
}

double read_scalar_numeric_variable(int ncid, const std::string& name) {
    std::vector<double> values = read_numeric_variable(ncid, name);
    if (values.empty()) {
        throw std::runtime_error("NetCDF scalar variable is empty: " + name);
    }
    return std::isnan(values.front()) ? 0.0 : values.front();
}

double read_optional_scalar_numeric_variable(int ncid, const std::string& name, double fallback) {
    if (!has_variable(ncid, name)) {
        return fallback;
    }
    return read_scalar_numeric_variable(ncid, name);
}

std::string read_text_attribute(int ncid, int varid, const std::string& name) {
    std::size_t length = 0;
    require_netcdf(nc_inq_attlen(ncid, varid, name.c_str(), &length), "Failed to read netCDF attribute length " + name);
    std::string value(length, '\0');
    require_netcdf(nc_get_att_text(ncid, varid, name.c_str(), value.data()), "Failed to read netCDF attribute " + name);
    return value;
}

std::string trim(std::string text) {
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front()))) {
        text.erase(text.begin());
    }
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) {
        text.pop_back();
    }
    return text;
}

std::time_t parse_cloudnet_base_time(const std::string& time_units) {
    std::size_t since_index = time_units.find("since");
    if (since_index == std::string::npos) {
        throw std::runtime_error("Unsupported Cloudnet time units: " + time_units);
    }
    std::string base_text = trim(time_units.substr(since_index + 5));
    std::size_t timezone_index = base_text.find(" +");
    if (timezone_index != std::string::npos) {
        base_text = base_text.substr(0, timezone_index);
    }
    std::tm stamp{};
    std::istringstream input(base_text);
    input >> std::get_time(&stamp, "%Y-%m-%d %H:%M:%S");
    if (input.fail()) {
        std::istringstream fallback(base_text);
        fallback >> std::get_time(&stamp, "%Y-%m-%dT%H:%M:%S");
        if (fallback.fail()) {
            throw std::runtime_error("Failed to parse Cloudnet base time: " + base_text);
        }
    }
    stamp.tm_isdst = -1;
    return std::mktime(&stamp);
}

double time_unit_seconds(const std::string& time_units) {
    std::string unit = time_units.substr(0, time_units.find(' '));
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

std::string cloudnet_time_to_iso_minute(double value, const std::string& time_units) {
    double seconds = value * time_unit_seconds(time_units);
    return format_timestamp(parse_cloudnet_base_time(time_units) + static_cast<long long>(std::llround(seconds)));
}

std::vector<int> select_even_indices(int length, int count) {
    if (length <= count) {
        std::vector<int> output(length, 0);
        std::iota(output.begin(), output.end(), 0);
        return output;
    }
    std::set<int> chosen;
    for (int index = 0; index < count; ++index) {
        chosen.insert(static_cast<int>(std::llround(index * (length - 1.0) / std::max(count - 1, 1))));
    }
    return std::vector<int>(chosen.begin(), chosen.end());
}

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

struct GroundRecord {
    std::string timestamp;
    double pm25 = 0.0;
    double pm10 = 0.0;
    double temperature_c = 18.0;
    double relative_humidity = 60.0;
    double wind_speed_ms = 2.0;
    double wind_dir_deg = 0.0;
};

std::string strip_hyphens(std::string text) {
    text.erase(std::remove(text.begin(), text.end(), '-'), text.end());
    return text;
}

std::string underscore_slug(std::string text) {
    std::string output = slugify(std::move(text));
    std::replace(output.begin(), output.end(), '-', '_');
    return output;
}

std::string format_number(double value, int precision = 6) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    std::string text = stream.str();
    while (!text.empty() && text.back() == '0') {
        text.pop_back();
    }
    if (!text.empty() && text.back() == '.') {
        text.pop_back();
    }
    if (text.empty()) {
        return "0";
    }
    return text;
}

std::string url_encode(const std::string& text) {
    std::ostringstream encoded;
    encoded << std::uppercase << std::hex;
    for (unsigned char current : text) {
        if (std::isalnum(current) || current == '-' || current == '_' || current == '.' || current == '~' || current == ',') {
            encoded << static_cast<char>(current);
        } else {
            encoded << '%' << std::setw(2) << std::setfill('0') << static_cast<int>(current);
        }
    }
    return encoded.str();
}

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

void write_text_file(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream handle(path, std::ios::binary);
    if (!handle) {
        throw std::runtime_error("Failed to write text file: " + path.string());
    }
    handle << text;
}

std::string csv_escape(const std::string& text) {
    if (text.find_first_of(",\"\n\r") == std::string::npos) {
        return text;
    }
    std::string output = "\"";
    for (char current : text) {
        if (current == '\"') {
            output += "\"\"";
        } else {
            output.push_back(current);
        }
    }
    output.push_back('\"');
    return output;
}

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

double json_array_number_or(const Json& array, std::size_t index, double fallback) {
    if (!array.is_array() || index >= array.array_items().size() || array.array_items()[index].is_null()) {
        return fallback;
    }
    return array.array_items()[index].number_value();
}

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
            json_array_number_or(weather_hourly.at("windspeed_10m"), index, 0.0) / 3.6,
            json_array_number_or(weather_hourly.at("winddirection_10m"), index, 0.0),
        });
    }
    return records;
}

#if defined(_WIN32)

struct WinHttpHandle {
    HINTERNET handle = nullptr;

    WinHttpHandle() = default;
    explicit WinHttpHandle(HINTERNET input) : handle(input) {}
    ~WinHttpHandle() {
        if (handle != nullptr) {
            WinHttpCloseHandle(handle);
        }
    }

    WinHttpHandle(const WinHttpHandle&) = delete;
    WinHttpHandle& operator=(const WinHttpHandle&) = delete;

    WinHttpHandle(WinHttpHandle&& other) noexcept : handle(other.handle) {
        other.handle = nullptr;
    }

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

struct ParsedHttpUrl {
    std::wstring host;
    std::wstring path_and_query;
    INTERNET_PORT port = INTERNET_DEFAULT_HTTPS_PORT;
    bool secure = true;
};

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
        path.append(components.lpszExtraInfo, components.dwExtraInfoLength);
    }
    if (path.empty()) {
        path = L"/";
    }

    return ParsedHttpUrl{
        std::wstring(components.lpszHostName, components.dwHostNameLength),
        path,
        components.nPort,
        components.nScheme == INTERNET_SCHEME_HTTPS,
    };
}

std::vector<std::uint8_t> http_get_binary(const std::string& url, bool verify_ssl) {
    ParsedHttpUrl parsed = parse_http_url(url);
    WinHttpHandle session(WinHttpOpen(L"atmospheric-lidar-pollution-cpp/1.0", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0));
    if (session.handle == nullptr) {
        throw std::runtime_error("WinHTTP session initialization failed for " + url);
    }

    DWORD decompression = WINHTTP_DECOMPRESSION_FLAG_GZIP | WINHTTP_DECOMPRESSION_FLAG_DEFLATE;
    WinHttpSetOption(session.handle, WINHTTP_OPTION_DECOMPRESSION, &decompression, sizeof(decompression));

    WinHttpHandle connection(WinHttpConnect(session.handle, parsed.host.c_str(), parsed.port, 0));
    if (connection.handle == nullptr) {
        throw std::runtime_error("WinHTTP connect failed for " + url);
    }

    DWORD request_flags = parsed.secure ? WINHTTP_FLAG_SECURE : 0;
    WinHttpHandle request(WinHttpOpenRequest(connection.handle, L"GET", parsed.path_and_query.c_str(), nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, request_flags));
    if (request.handle == nullptr) {
        throw std::runtime_error("WinHTTP request creation failed for " + url);
    }

    if (!verify_ssl && parsed.secure) {
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
            break;
        }
        std::vector<std::uint8_t> buffer(static_cast<std::size_t>(available), 0);
        DWORD downloaded = 0;
        if (!WinHttpReadData(request.handle, buffer.data(), available, &downloaded)) {
            throw std::runtime_error("Failed to read HTTP response for " + url);
        }
        output.insert(output.end(), buffer.begin(), buffer.begin() + downloaded);
    }
    return output;
}

#else

std::vector<std::uint8_t> http_get_binary(const std::string& url, bool verify_ssl) {
    (void) url;
    (void) verify_ssl;
    throw std::runtime_error("HTTP download is currently implemented only for the Windows C++ build.");
}

#endif

std::string http_get_text(const std::string& url, bool verify_ssl) {
    std::vector<std::uint8_t> payload = http_get_binary(url, verify_ssl);
    return std::string(payload.begin(), payload.end());
}

double json_number_or(const Json& value, const std::string& key, double fallback) {
    if (!value.contains(key) || value.at(key).is_null()) {
        return fallback;
    }
    return value.at(key).number_value();
}

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

Json json_array_from_double_vector(const std::vector<double>& values) {
    Json::array_type output;
    output.reserve(values.size());
    for (double value : values) {
        output.emplace_back(value);
    }
    return Json(std::move(output));
}

Json json_array_from_int_vector(const std::vector<int>& values) {
    Json::array_type output;
    output.reserve(values.size());
    for (int value : values) {
        output.emplace_back(value);
    }
    return Json(std::move(output));
}

Json json_array_from_string_vector(const std::vector<std::string>& values) {
    Json::array_type output;
    output.reserve(values.size());
    for (const auto& value : values) {
        output.emplace_back(value);
    }
    return Json(std::move(output));
}

Json json_array_from_matrix(const std::vector<std::vector<double>>& values) {
    Json::array_type output;
    output.reserve(values.size());
    for (const auto& row : values) {
        output.emplace_back(json_array_from_double_vector(row));
    }
    return Json(std::move(output));
}

Json to_json(const SiteInfo& value) {
    return Json::object_type{
        {"name", value.name},
        {"latitude_deg", value.latitude_deg},
        {"longitude_deg", value.longitude_deg},
        {"altitude_m", value.altitude_m},
        {"site_id", value.site_id},
    };
}

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

struct SimulatedFields {
    std::vector<double> molecular_extinction;
    std::vector<double> molecular_backscatter;
    std::vector<double> true_backscatter;
    std::vector<double> true_extinction;
    std::vector<double> true_pm25;
    std::vector<double> true_pm10;
    std::vector<int> true_hotspot_mask;
};

struct PreprocessResult {
    std::vector<double> l1_signal;
    std::vector<double> attenuated_backscatter;
    std::vector<double> snr;
    std::vector<std::string> qc_flags;
};

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

struct CalibrationSplit {
    std::vector<FeatureSample> train;
    std::vector<FeatureSample> val;
    std::vector<FeatureSample> test;
    std::vector<std::string> train_timestamps;
    std::vector<std::string> val_timestamps;
    std::vector<std::string> test_timestamps;
};

struct CalibrationModels {
    std::vector<double> pm25;
    std::vector<double> pm10;
};

struct StationOffset {
    double pm25_offset = 0.0;
    double pm10_offset = 0.0;
    int sample_count = 0;
};

struct DetectionResult {
    std::vector<Hotspot> hotspots;
    std::vector<int> predicted_mask;
    std::vector<int> truth_mask;
};

struct ResidualRow {
    std::string timestamp;
    std::string site_id;
    double pm25_residual = 0.0;
    double pm10_residual = 0.0;
};

struct PredictionSummary {
    std::vector<double> pm25_truth;
    std::vector<double> pm25_pred;
    std::vector<double> pm10_truth;
    std::vector<double> pm10_pred;
    std::vector<ResidualRow> residual_rows;
};

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

double sample_gaussian(std::mt19937& rng, double mean_value, double sigma) {
    std::normal_distribution<double> distribution(mean_value, sigma);
    return distribution(rng);
}

std::vector<double> build_overlap(const std::vector<double>& ranges_m) {
    std::vector<double> overlap;
    overlap.reserve(ranges_m.size());
    for (double distance_m : ranges_m) {
        double ratio = std::pow(distance_m / 180.0, 0.82);
        overlap.push_back(clamp(ratio, 0.22, 1.0));
    }
    return overlap;
}

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
    double time_phase = 2.0 * std::numbers::pi * static_cast<double>(step_index) / std::max(total_steps, 1);
    double humidity_growth = 1.0 + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;

    for (double range_m : ranges_m) {
        double elevation_rad = elevation_deg * std::numbers::pi / 180.0;
        double altitude_m = range_m * std::sin(elevation_rad);
        double horizontal_m = range_m * std::cos(elevation_rad);
        double molecular_ext = 0.012 * std::exp(-altitude_m / 8000.0);
        double molecular_beta = molecular_ext / 8.0;

        double boundary_layer = (0.017 + 0.018 * (1.0 + std::sin(time_phase)) / 2.0) * std::exp(-altitude_m / 550.0);
        double lofted_layer = 0.018 * gaussian(altitude_m, 500.0 + 120.0 * std::sin(time_phase), 160.0);
        double plume_1 = 0.080 * gaussian(horizontal_m, 650.0 + 80.0 * std::cos(time_phase), 130.0)
            * gaussian(azimuth_delta(azimuth_deg, 80.0), 0.0, 18.0)
            * gaussian(altitude_m, 85.0, 45.0);
        double plume_2 = 0.045 * gaussian(horizontal_m, 1050.0, 150.0)
            * gaussian(azimuth_delta(azimuth_deg, 170.0), 0.0, 22.0)
            * gaussian(altitude_m, 120.0, 65.0);

        double aerosol_dry_ext = boundary_layer + lofted_layer + plume_1 + plume_2;
        double aerosol_ext = aerosol_dry_ext * humidity_growth;
        double aerosol_beta = aerosol_ext / lidar_ratio_sr;

        output.molecular_extinction.push_back(molecular_ext);
        output.molecular_backscatter.push_back(molecular_beta);
        output.true_backscatter.push_back(molecular_beta + aerosol_beta);
        output.true_extinction.push_back(molecular_ext + aerosol_ext);
        output.true_pm25.push_back(640.0 * aerosol_dry_ext + 210.0 * plume_1 + 15.0);
        output.true_pm10.push_back(920.0 * aerosol_dry_ext + 260.0 * (plume_1 + plume_2) + 22.0);
        output.true_hotspot_mask.push_back((plume_1 + plume_2 > 0.025) ? 1 : 0);
    }
    return output;
}

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
        optical_depth += true_extinction[index] * step_km;
        double range_km = ranges_m[index] / 1000.0;
        double signal = system_constant * laser_energy_mj * overlap[index] * true_backscatter[index]
            * std::exp(-2.0 * optical_depth) / std::max(range_km * range_km, 1e-6);
        double noise_sigma = std::max(background_counts * 0.07, signal * 0.05);
        double noisy_signal = std::max(signal + background_counts + sample_gaussian(rng, 0.0, noise_sigma), background_counts + 0.1);
        raw_counts.push_back(noisy_signal);
    }
    return raw_counts;
}

struct CampaignData {
    SiteInfo site;
    std::vector<LidarProfile> profiles;
    std::vector<GroundMeasurement> ground_measurements;
    Json source_metadata;
};

CampaignData simulate_campaign(const PipelineConfig& config) {
    CampaignData data;
    data.site = config.site;
    data.site.site_id = data.site.site_id.empty() ? slugify(data.site.name) : data.site.site_id;

    std::mt19937 rng(config.simulation.seed);
    std::tm base_tm{};
    base_tm.tm_year = 2026 - 1900;
    base_tm.tm_mon = 4;
    base_tm.tm_mday = 30;
    base_tm.tm_hour = 8;
    base_tm.tm_min = 0;
    base_tm.tm_isdst = -1;
    std::time_t base_time = std::mktime(&base_tm);

    std::vector<double> ranges_m;
    for (int index = 0; index < config.simulation.range_bin_count; ++index) {
        ranges_m.push_back(config.simulation.range_bin_m * static_cast<double>(index + 1));
    }
    std::vector<double> overlap = build_overlap(ranges_m);

    for (int step_index = 0; step_index < config.simulation.time_steps; ++step_index) {
        std::string timestamp = format_timestamp(base_time + static_cast<long long>(step_index * config.simulation.minutes_per_step * 60));
        double time_phase = 2.0 * std::numbers::pi * static_cast<double>(step_index) / std::max(config.simulation.time_steps, 1);
        double relative_humidity = clamp(0.48 + 0.20 * std::sin(time_phase - 0.6) + sample_gaussian(rng, 0.0, 0.015), 0.28, 0.90);
        double temperature_c = 27.0 - 7.0 * std::sin(time_phase - 0.2) + sample_gaussian(rng, 0.0, 0.3);
        double wind_speed_ms = clamp(2.8 + 1.2 * std::cos(time_phase + 0.3) + sample_gaussian(rng, 0.0, 0.2), 0.6, 6.5);
        double wind_dir_deg = std::fmod(120.0 + 45.0 * std::sin(time_phase) + sample_gaussian(rng, 0.0, 4.0) + 360.0, 360.0);
        double background_counts = 10.5 + sample_gaussian(rng, 0.0, 0.5);
        double laser_energy_mj = 1.0 + sample_gaussian(rng, 0.0, 0.03);

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

        std::vector<double> near_surface_values;
        for (int index = 0; index < std::min<int>(6, static_cast<int>(stare_fields.true_extinction.size())); ++index) {
            double humid_factor = 1.0 + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;
            near_surface_values.push_back(stare_fields.true_extinction[index] / humid_factor);
        }
        double near_surface_dry = mean(near_surface_values);
        double hotspot_proxy = 0.0;
        for (const auto& profile : ppi_profiles_for_timestamp) {
            double local_peak = 0.0;
            for (int index = 0; index < std::min<int>(6, static_cast<int>(profile.true_extinction.size())); ++index) {
                double humid_factor = 1.0 + 0.35 * std::max(relative_humidity - 0.55, 0.0) * 3.0;
                local_peak = std::max(local_peak, profile.true_extinction[index] / humid_factor);
            }
            hotspot_proxy = std::max(hotspot_proxy, local_peak);
        }

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

PreprocessResult preprocess_profile(const LidarProfile& profile) {
    PreprocessResult output;
    for (std::size_t index = 0; index < profile.raw_counts.size(); ++index) {
        double background_corrected = std::max(profile.raw_counts[index] - profile.background_counts, 0.0);
        double energy_normalized = background_corrected / std::max(profile.laser_energy_mj, 1e-6);
        double overlap_corrected = energy_normalized / std::max(profile.overlap[index], 0.15);
        double range_km = profile.ranges_m[index] / 1000.0;
        double attenuated = overlap_corrected * range_km * range_km;
        double signal_to_noise = background_corrected / std::max(std::sqrt(profile.raw_counts[index] + profile.background_counts), 1.0);

        output.l1_signal.push_back(background_corrected);
        output.attenuated_backscatter.push_back(std::max(attenuated, 1e-9));
        output.snr.push_back(signal_to_noise);
    }

    if (profile.overlap.size() >= 3 && *std::min_element(profile.overlap.begin(), profile.overlap.begin() + 3) < 0.4) {
        output.qc_flags.push_back("near-range-partial-overlap");
    }
    if (profile.laser_energy_mj < 0.93) {
        output.qc_flags.push_back("low-laser-energy");
    }
    if (output.snr.size() >= 4 && mean(std::vector<double>(output.snr.begin(), output.snr.begin() + 4)) < 3.0) {
        output.qc_flags.push_back("weak-near-range-snr");
    }
    return output;
}

std::pair<std::vector<double>, std::vector<double>> run_fernald_inversion(
    const LidarProfile& profile,
    const std::vector<double>& attenuated_backscatter,
    double aerosol_lidar_ratio_sr,
    double reference_aerosol_backscatter
) {
    int ref_index = std::max<int>(static_cast<int>(attenuated_backscatter.size()) - 5, 0);
    std::vector<double> ref_signal_slice(attenuated_backscatter.begin() + ref_index, attenuated_backscatter.end());
    std::vector<double> ref_beta_slice(profile.molecular_backscatter.begin() + ref_index, profile.molecular_backscatter.end());
    double ref_signal = std::max(mean(ref_signal_slice), 1e-9);
    double ref_beta = mean(ref_beta_slice) + reference_aerosol_backscatter;
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
    for (int index = static_cast<int>(scaled_signal.size()) - 1; index >= 0; --index) {
        double total_backscatter = std::max(scaled_signal[index] * std::exp(2.0 * optical_depth), profile.molecular_backscatter[index]);
        double aerosol_beta = std::max(total_backscatter - profile.molecular_backscatter[index], 0.0);
        double total_extinction = profile.molecular_extinction[index] + aerosol_lidar_ratio_sr * aerosol_beta;
        total_extinction = std::min(std::max(total_extinction, profile.molecular_extinction[index]), 0.45);
        aerosol_backscatter[index] = aerosol_beta;
        extinction[index] = total_extinction;
        optical_depth += total_extinction * step_km;
    }
    return {extinction, aerosol_backscatter};
}

double growth_factor(double relative_humidity, double dry_reference_rh, double hygroscopicity) {
    double rh = std::min(std::max(relative_humidity, 0.05), 0.98);
    double dry_ratio = dry_reference_rh / std::max(1.0 - dry_reference_rh, 0.02);
    double humid_ratio = rh / std::max(1.0 - rh, 0.02);
    return std::max(1.0, 1.0 + hygroscopicity * std::max(humid_ratio - dry_ratio, 0.0) * 0.18);
}

std::vector<double> apply_humidity_correction(const std::vector<double>& extinction, double relative_humidity, double dry_reference_rh, double hygroscopicity) {
    double factor = growth_factor(relative_humidity, dry_reference_rh, hygroscopicity);
    std::vector<double> output;
    output.reserve(extinction.size());
    for (double value : extinction) {
        output.push_back(value / factor);
    }
    return output;
}

std::vector<std::vector<double>> profile_bins_to_enu(const LidarProfile& profile) {
    std::vector<std::vector<double>> output;
    output.reserve(profile.ranges_m.size());
    double azimuth_rad = profile.azimuth_deg * std::numbers::pi / 180.0;
    double elevation_rad = profile.elevation_deg * std::numbers::pi / 180.0;
    for (double range_m : profile.ranges_m) {
        double east = range_m * std::cos(elevation_rad) * std::sin(azimuth_rad);
        double north = range_m * std::cos(elevation_rad) * std::cos(azimuth_rad);
        double up = range_m * std::sin(elevation_rad);
        output.push_back({east, north, up});
    }
    return output;
}

std::vector<FeatureSample> build_timestamp_feature_table(
    const std::vector<ProcessedProfile>& processed_profiles,
    const std::vector<GroundMeasurement>& ground_measurements,
    int surface_bin_count
) {
    std::map<std::string, GroundMeasurement> ground_by_timestamp;
    for (const auto& measurement : ground_measurements) {
        ground_by_timestamp[measurement.timestamp] = measurement;
    }

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
        if (stare_profiles.empty() || !ground_by_timestamp.contains(timestamp)) {
            continue;
        }
        const auto& ground = ground_by_timestamp.at(timestamp);
        const auto* stare_profile = stare_profiles.front();
        std::vector<double> stare_slice(stare_profile->dry_extinction.begin(), stare_profile->dry_extinction.begin() + std::min<int>(surface_bin_count, static_cast<int>(stare_profile->dry_extinction.size())));
        std::vector<double> attenuated_slice(stare_profile->attenuated_backscatter.begin(), stare_profile->attenuated_backscatter.begin() + std::min<int>(surface_bin_count, static_cast<int>(stare_profile->attenuated_backscatter.size())));
        double hotspot_proxy = 0.0;
        if (ppi_by_timestamp.contains(timestamp)) {
            for (const auto* profile : ppi_by_timestamp.at(timestamp)) {
                hotspot_proxy = std::max(hotspot_proxy, *std::max_element(profile->dry_extinction.begin(), profile->dry_extinction.begin() + std::min<int>(surface_bin_count, static_cast<int>(profile->dry_extinction.size()))));
            }
        } else {
            hotspot_proxy = *std::max_element(stare_slice.begin(), stare_slice.end());
        }
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

std::vector<double> solve_linear_system(std::vector<std::vector<double>> matrix, std::vector<double> vector) {
    int size = static_cast<int>(vector.size());
    for (int row = 0; row < size; ++row) {
        matrix[row].push_back(vector[row]);
    }
    for (int pivot = 0; pivot < size; ++pivot) {
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
        for (int column = pivot; column <= size; ++column) {
            matrix[pivot][column] /= pivot_value;
        }
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

    std::vector<double> solution(size, 0.0);
    for (int row = 0; row < size; ++row) {
        solution[row] = matrix[row][size];
    }
    return solution;
}

std::vector<double> fit_linear_regression(const std::vector<std::vector<double>>& feature_matrix, const std::vector<double>& target) {
    int feature_count = static_cast<int>(feature_matrix.front().size());
    std::vector<std::vector<double>> gram(feature_count, std::vector<double>(feature_count, 0.0));
    std::vector<double> rhs(feature_count, 0.0);
    for (std::size_t row = 0; row < feature_matrix.size(); ++row) {
        for (int left = 0; left < feature_count; ++left) {
            rhs[left] += feature_matrix[row][left] * target[row];
            for (int right = 0; right < feature_count; ++right) {
                gram[left][right] += feature_matrix[row][left] * feature_matrix[row][right];
            }
        }
    }
    for (int diagonal = 0; diagonal < feature_count; ++diagonal) {
        gram[diagonal][diagonal] += 1e-6;
    }
    return solve_linear_system(gram, rhs);
}

double predict(const std::vector<double>& coefficients, const std::vector<double>& features) {
    double output = 0.0;
    for (std::size_t index = 0; index < coefficients.size(); ++index) {
        output += coefficients[index] * features[index];
    }
    return output;
}

CalibrationSplit split_samples(const std::vector<FeatureSample>& samples) {
    CalibrationSplit split;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        int bucket = static_cast<int>(index % 5);
        if (bucket <= 2) {
            split.train.push_back(samples[index]);
        } else if (bucket == 3) {
            split.val.push_back(samples[index]);
        } else {
            split.test.push_back(samples[index]);
        }
    }
    if (split.val.empty() && !split.train.empty()) {
        split.val.push_back(split.train.back());
        split.train.pop_back();
    }
    if (split.test.empty() && !split.train.empty()) {
        split.test.push_back(split.train.back());
        split.train.pop_back();
    }
    for (const auto& sample : split.train) split.train_timestamps.push_back(sample.timestamp);
    for (const auto& sample : split.val) split.val_timestamps.push_back(sample.timestamp);
    for (const auto& sample : split.test) split.test_timestamps.push_back(sample.timestamp);
    return split;
}

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

void apply_pm_models(
    std::vector<ProcessedProfile>& processed_profiles,
    const CalibrationModels& models,
    const std::vector<FeatureSample>& feature_table,
    const std::map<std::string, StationOffset>& station_offsets
) {
    std::map<std::string, FeatureSample> feature_by_timestamp;
    for (const auto& sample : feature_table) {
        feature_by_timestamp[sample.timestamp] = sample;
    }

    for (auto& processed : processed_profiles) {
        const auto& timestamp_features = feature_by_timestamp.at(processed.profile.timestamp);
        std::string site_id = !timestamp_features.site_id.empty() ? timestamp_features.site_id : (!processed.profile.site_id.empty() ? processed.profile.site_id : "default-site");
        StationOffset offset = station_offsets.contains(site_id) ? station_offsets.at(site_id) : StationOffset{};
        processed.pm25.clear();
        processed.pm10.clear();
        for (std::size_t index = 0; index < processed.dry_extinction.size(); ++index) {
            double local_excess = std::max(0.0, processed.dry_extinction[index] - timestamp_features.surface_dry_ext);
            std::vector<double> features{1.0, processed.dry_extinction[index], processed.profile.relative_humidity, timestamp_features.hotspot_proxy};
            double base_pm25 = predict(models.pm25, features) + offset.pm25_offset;
            double base_pm10 = predict(models.pm10, features) + offset.pm10_offset;
            double hotspot_boost = 1250.0 * local_excess;
            processed.pm25.push_back(std::max(0.0, base_pm25 + hotspot_boost));
            processed.pm10.push_back(std::max(0.0, base_pm10 + 1.18 * hotspot_boost));
        }
    }
}

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
    result.predicted_mask.assign(range_count * azimuth_count, 0);
    result.truth_mask.assign(range_count * azimuth_count, 0);

    std::vector<double> all_pm25;
    std::vector<double> all_dry_ext;
    for (const auto* profile : sorted_profiles) {
        all_pm25.insert(all_pm25.end(), profile->pm25.begin(), profile->pm25.end());
        all_dry_ext.insert(all_dry_ext.end(), profile->dry_extinction.begin(), profile->dry_extinction.end());
    }
    double baseline_pm25 = median(all_pm25);
    double baseline_dry_ext = median(all_dry_ext);

    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        const auto* processed = sorted_profiles[azimuth_index];
        for (int range_index = 0; range_index < range_count; ++range_index) {
            int linear_index = azimuth_index * range_count + range_index;
            result.truth_mask[linear_index] = processed->profile.true_hotspot_mask[range_index];
            bool absolute_hotspot = processed->pm25[range_index] >= threshold_ugm3;
            bool relative_hotspot = (processed->pm25[range_index] - baseline_pm25 >= relative_pm25_threshold)
                && (processed->dry_extinction[range_index] - baseline_dry_ext >= relative_dry_ext_threshold);
            if (absolute_hotspot || relative_hotspot) {
                result.predicted_mask[linear_index] = 1;
            }
        }
    }

    std::vector<std::vector<bool>> visited(azimuth_count, std::vector<bool>(range_count, false));
    int component_index = 0;
    double azimuth_step_deg = 360.0 / std::max(azimuth_count, 1);
    double range_step_m = range_count > 1 ? sorted_profiles.front()->profile.ranges_m[1] - sorted_profiles.front()->profile.ranges_m[0] : 50.0;
    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        for (int range_index = 0; range_index < range_count; ++range_index) {
            if (result.predicted_mask[azimuth_index * range_count + range_index] == 0 || visited[azimuth_index][range_index]) {
                continue;
            }

            std::queue<std::pair<int, int>> queue;
            std::vector<std::pair<int, int>> component_cells;
            queue.push({azimuth_index, range_index});
            visited[azimuth_index][range_index] = true;
            while (!queue.empty()) {
                auto [current_azimuth, current_range] = queue.front();
                queue.pop();
                component_cells.push_back({current_azimuth, current_range});
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

            if (static_cast<int>(component_cells.size()) < min_cells) {
                continue;
            }

            ++component_index;
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
                weights.push_back(pm25_value);
                east_values.push_back(point[0]);
                north_values.push_back(point[1]);
                up_values.push_back(point[2]);
                pm_values.push_back(pm25_value);
                double radial_distance = processed->profile.ranges_m[component_range] * std::cos(processed->profile.elevation_deg * std::numbers::pi / 180.0);
                area_m2 += std::max(radial_distance, 10.0) * (azimuth_step_deg * std::numbers::pi / 180.0) * range_step_m;
            }

            double total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);
            std::vector<double> centroid{
                std::inner_product(east_values.begin(), east_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
                std::inner_product(north_values.begin(), north_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
                std::inner_product(up_values.begin(), up_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
            };
            double peak_pm25 = *std::max_element(pm_values.begin(), pm_values.end());
            std::string severity = peak_pm25 >= threshold_ugm3 + 70.0 ? "critical" : (peak_pm25 >= threshold_ugm3 + 35.0 ? "high" : "medium");

            std::ostringstream hotspot_id;
            hotspot_id << "hotspot-" << std::setw(3) << std::setfill('0') << component_index;
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

double r2_metric(const std::vector<double>& truth, const std::vector<double>& prediction) {
    if (truth.empty()) {
        return 0.0;
    }
    double truth_mean = mean(truth);
    double total = 0.0;
    double residual = 0.0;
    for (std::size_t index = 0; index < truth.size(); ++index) {
        total += (truth[index] - truth_mean) * (truth[index] - truth_mean);
        residual += (truth[index] - prediction[index]) * (truth[index] - prediction[index]);
    }
    if (total == 0.0) {
        return 0.0;
    }
    return 1.0 - residual / total;
}

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
    double f1 = (precision + recall == 0.0) ? 0.0 : 2.0 * precision * recall / (precision + recall);
    return Json::object_type{
        {"precision", precision},
        {"recall", recall},
        {"f1", f1},
        {"true_positive", true_positive},
        {"false_positive", false_positive},
        {"false_negative", false_negative},
    };
}

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
        output.residual_rows.push_back(ResidualRow{timestamp, site_id, sample.pm25_true - predicted_pm25, sample.pm10_true - predicted_pm10});
    }
    return output;
}

Json summarize_drift_monitoring(const std::vector<ResidualRow>& residual_rows) {
    std::map<std::string, std::vector<ResidualRow>> grouped;
    for (const auto& row : residual_rows) {
        grouped[row.site_id].push_back(row);
    }

    Json::object_type stations;
    Json::array_type alerts;
    for (auto& [site_id, rows] : grouped) {
        std::sort(rows.begin(), rows.end(), [](const ResidualRow& left, const ResidualRow& right) {
            return left.timestamp < right.timestamp;
        });
        int window_size = std::min<int>(3, static_cast<int>(rows.size()));
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

void apply_failure_case(std::vector<LidarProfile>& profiles, std::vector<GroundMeasurement>& ground_measurements, const std::string& case_name) {
    if (case_name == "high-background-light") {
        for (auto& profile : profiles) {
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
            for (int index = 0; index < std::min<int>(6, static_cast<int>(profile.overlap.size())); ++index) {
                profile.overlap[index] = std::max(profile.overlap[index] * 0.65, 0.1);
            }
        }
        return;
    }
    if (case_name == "humidity-surge") {
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

Json::array_type run_failure_case_suite(const PipelineConfig& config, const SiteInfo& site, const std::vector<LidarProfile>& profiles, const std::vector<GroundMeasurement>& ground_measurements);

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

    result.feature_table = build_timestamp_feature_table(result.processed_profiles, ground_measurements, config.pm_calibration.surface_bin_count);
    auto fitted = fit_pm_models(result.feature_table);
    CalibrationModels models = fitted.first;
    result.split = fitted.second;
    result.station_offsets = fit_station_offsets(models, result.split);
    apply_pm_models(result.processed_profiles, models, result.feature_table, result.station_offsets);

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

    auto total_end = std::chrono::steady_clock::now();
    std::vector<std::string> holdout_timestamps = result.split.val_timestamps;
    holdout_timestamps.insert(holdout_timestamps.end(), result.split.test_timestamps.begin(), result.split.test_timestamps.end());
    std::vector<std::string> evaluation_timestamps = holdout_timestamps.empty() ? result.split.train_timestamps : holdout_timestamps;
    PredictionSummary predictions = extract_ground_predictions(result.feature_table, models, evaluation_timestamps, result.station_offsets);
    Json hotspot_scores = classification_metrics(truth_mask, predicted_mask);
    result.drift_monitoring = summarize_drift_monitoring(predictions.residual_rows);

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
            {"throughput_profiles_per_s", static_cast<double>(result.processed_profiles.size()) / std::max(elapsed_s, 1e-9)},
        }},
    };
    return result;
}

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

int sampling_minutes(const std::vector<std::string>& timestamps) {
    if (timestamps.size() < 2) {
        return 0;
    }
    std::vector<double> deltas;
    for (std::size_t index = 1; index < timestamps.size(); ++index) {
        deltas.push_back(std::difftime(parse_timestamp(timestamps[index]), parse_timestamp(timestamps[index - 1])) / 60.0);
    }
    return static_cast<int>(std::lround(mean(deltas)));
}

Json build_demo_payload(
    const DatasetRunResult& primary,
    const Json::array_type& ablation,
    const Json::array_type& sensitivity,
    const Json::array_type& failure_cases,
    const Json& source_metadata
) {
    std::map<std::string, std::vector<const ProcessedProfile*>> grouped;
    for (const auto& profile : primary.processed_profiles) {
        grouped[profile.profile.timestamp].push_back(&profile);
    }
    std::vector<std::string> timestamps;
    timestamps.reserve(grouped.size());
    for (const auto& [timestamp, _] : grouped) {
        timestamps.push_back(timestamp);
    }

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
        for (const auto& point : stare_profiles.front()->enu_points_m) {
            curtain_heights.emplace_back(point[2]);
        }
        for (const auto* profile : stare_profiles) {
            curtain_pm25.emplace_back(json_array_from_double_vector(profile->pm25));
            curtain_extinction.emplace_back(json_array_from_double_vector(profile->extinction));
        }
    }

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

Json::array_type run_failure_case_suite(const PipelineConfig& config, const SiteInfo& site, const std::vector<LidarProfile>& profiles, const std::vector<GroundMeasurement>& ground_measurements) {
    Json::array_type output;
    for (const std::string& case_name : {std::string("high-background-light"), std::string("overlap-miscalibration"), std::string("humidity-surge")}) {
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

std::string build_line_chart_svg(const std::vector<double>& values, int width, int height, const std::string& stroke) {
    if (values.empty()) {
        return "<svg viewBox=\"0 0 10 10\"></svg>";
    }
    double minimum = *std::min_element(values.begin(), values.end());
    double maximum = *std::max_element(values.begin(), values.end());
    double spread = maximum - minimum;
    if (spread == 0.0) {
        spread = 1.0;
    }
    double step_x = static_cast<double>(width) / std::max<int>(static_cast<int>(values.size()) - 1, 1);
    std::ostringstream points;
    for (std::size_t index = 0; index < values.size(); ++index) {
        double x_coord = static_cast<double>(index) * step_x;
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

std::vector<double> json_to_double_vector(const Json& value) {
    std::vector<double> output;
    for (const auto& item : value.array_items()) {
        output.push_back(item.number_value());
    }
    return output;
}

} // namespace

Json parse_json(const std::string& text) {
    return JsonParser(text).parse();
}

std::string dump_json(const Json& value, int indent) {
    std::ostringstream output;
    dump_json_impl(value, output, indent, 0);
    return output.str();
}

Json read_json_file(const std::filesystem::path& path) {
    std::ifstream handle(path, std::ios::binary);
    if (!handle) {
        throw std::runtime_error("Failed to open JSON file: " + path.string());
    }
    std::ostringstream buffer;
    buffer << handle.rdbuf();
    return parse_json(buffer.str());
}

void write_json_file(const std::filesystem::path& path, const Json& value) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream handle(path, std::ios::binary);
    if (!handle) {
        throw std::runtime_error("Failed to write JSON file: " + path.string());
    }
    handle << dump_json(value, 2);
}

Json fetch_public_ground_data(
    double latitude_deg,
    double longitude_deg,
    const std::string& start_date,
    const std::string& end_date,
    const std::string& timezone,
    const std::filesystem::path& output_dir,
    const std::string& prefix
) {
    std::string air_url = "https://air-quality-api.open-meteo.com/v1/air-quality?" + build_query_string({
        {"latitude", format_number(latitude_deg)},
        {"longitude", format_number(longitude_deg)},
        {"hourly", "pm10,pm2_5"},
        {"start_date", start_date},
        {"end_date", end_date},
        {"timezone", timezone},
    });
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

Json fetch_cloudnet_public_sample(const PipelineConfig& config, const std::filesystem::path& output_root) {
    if (config.source.cloudnet.download_url.empty()) {
        throw std::runtime_error("Cloudnet public sample fetch requires source.cloudnet.download_url");
    }
    if (config.source.cloudnet.local_file.empty()) {
        throw std::runtime_error("Cloudnet public sample fetch requires source.cloudnet.local_file");
    }
    if (config.source.cloudnet.date.empty()) {
        throw std::runtime_error("Cloudnet public sample fetch requires source.cloudnet.date");
    }

    std::filesystem::path base_root = std::filesystem::absolute(output_root.empty() ? std::filesystem::path(".") : output_root);
    std::filesystem::path configured_root = config.source.root.empty() ? std::filesystem::path(".") : std::filesystem::path(config.source.root);
    std::filesystem::path source_root = configured_root.is_absolute() ? configured_root : std::filesystem::absolute(base_root / configured_root);
    std::filesystem::path local_file = source_root / config.source.cloudnet.local_file;
    std::filesystem::create_directories(local_file.parent_path());

    std::vector<std::uint8_t> cloudnet_bytes = http_get_binary(config.source.cloudnet.download_url, config.source.cloudnet.verify_ssl);
    {
        std::ofstream handle(local_file, std::ios::binary);
        if (!handle) {
            throw std::runtime_error("Failed to write Cloudnet sample file: " + local_file.string());
        }
        handle.write(reinterpret_cast<const char*>(cloudnet_bytes.data()), static_cast<std::streamsize>(cloudnet_bytes.size()));
    }

    std::filesystem::path output_dir = source_root / "data" / "public" / "cloudnet";
    std::filesystem::create_directories(output_dir);
    std::string day_token = strip_hyphens(config.source.cloudnet.date);
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

    std::filesystem::path air_path = output_dir / (day_token + "_" + site_slug + "_open_meteo_air_quality.json");
    std::filesystem::path weather_path = output_dir / (day_token + "_" + site_slug + "_open_meteo_weather.json");
    write_json_file(air_path, air_quality);
    write_json_file(weather_path, weather);

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

PipelineConfig parse_pipeline_config(const Json& value) {
    PipelineConfig config;
    if (value.contains("source") && value.at("source").is_object() && value.at("source").contains("mode")) {
        config.source_mode = value.at("source").at("mode").string_value();
    }
    config.source.mode = config.source_mode;
    if (value.contains("source") && value.at("source").is_object()) {
        const Json& source = value.at("source");
        config.source.root = source.contains("root") ? source.at("root").string_value() : ".";
        if (source.contains("cloudnet") && source.at("cloudnet").is_object()) {
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
            config.source.cloudnet.pseudo_signal_scale = cloudnet.contains("pseudo_signal_scale") ? cloudnet.at("pseudo_signal_scale").number_value() : 600000.0;
        }
    }

    const Json& site = value.at("site");
    config.site.name = site.at("name").string_value();
    config.site.latitude_deg = site.at("latitude_deg").number_value();
    config.site.longitude_deg = site.at("longitude_deg").number_value();
    config.site.altitude_m = site.at("altitude_m").number_value();
    config.site.site_id = site.contains("site_id") ? site.at("site_id").string_value() : slugify(config.site.name);

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

    const Json& retrieval = value.at("retrieval");
    config.retrieval.aerosol_lidar_ratio_sr = retrieval.at("aerosol_lidar_ratio_sr").number_value();
    config.retrieval.reference_aerosol_backscatter = retrieval.at("reference_aerosol_backscatter").number_value();

    const Json& humidity = value.at("humidity");
    config.humidity.dry_reference_rh = humidity.at("dry_reference_rh").number_value();
    config.humidity.hygroscopicity = humidity.at("hygroscopicity").number_value();

    const Json& calibration = value.at("pm_calibration");
    config.pm_calibration.train_ratio = calibration.at("train_ratio").number_value();
    config.pm_calibration.val_ratio = calibration.at("val_ratio").number_value();
    config.pm_calibration.surface_bin_count = calibration.at("surface_bin_count").int_value();

    const Json& hotspot = value.at("hotspot");
    config.hotspot.pm25_threshold_ugm3 = hotspot.at("pm25_threshold_ugm3").number_value();
    config.hotspot.scan_relative_pm25_threshold_ugm3 = hotspot.at("scan_relative_pm25_threshold_ugm3").number_value();
    config.hotspot.scan_relative_dry_ext_threshold = hotspot.at("scan_relative_dry_ext_threshold").number_value();
    config.hotspot.min_cells = hotspot.at("min_cells").int_value();

    const Json& evaluation = value.at("evaluation");
    for (const auto& item : evaluation.at("sensitivity_lidar_ratios").array_items()) {
        config.evaluation.sensitivity_lidar_ratios.push_back(item.number_value());
    }
    return config;
}

Json run_end_to_end(const PipelineConfig& config, const std::optional<std::filesystem::path>& output_root) {
    CampaignData campaign;
    if (config.source_mode == "simulation") {
        campaign = simulate_campaign(config);
    } else if (config.source_mode == "cloudnet_hybrid") {
#if defined(LIDAR_DEMO_HAS_NETCDF)
        campaign = load_cloudnet_hybrid_campaign(config);
#else
        throw std::runtime_error("Cloudnet hybrid requires a NetCDF-enabled C++ build.");
#endif
    } else {
        throw std::runtime_error("Unsupported source mode in C++ pipeline: " + config.source_mode);
    }

    DatasetRunResult primary = run_pipeline_on_dataset(config, campaign.site, campaign.profiles, campaign.ground_measurements);

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
    double stability_span = sensitivity_rmse.empty() ? 0.0 : *std::max_element(sensitivity_rmse.begin(), sensitivity_rmse.end()) - *std::min_element(sensitivity_rmse.begin(), sensitivity_rmse.end());
    primary.metrics["retrieval_stability"] = Json::object_type{
        {"pm25_rmse_span", stability_span},
        {"reference_lidar_ratio_sr", config.retrieval.aerosol_lidar_ratio_sr},
    };

    Json::array_type failure_cases = run_failure_case_suite(config, campaign.site, campaign.profiles, campaign.ground_measurements);
    Json demo_payload = build_demo_payload(primary, ablation, sensitivity, failure_cases, campaign.source_metadata);

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

std::string render_dashboard(const Json& data) {
    std::vector<double> snr_values = json_to_double_vector(data.at("qc").at("mean_snr"));
    std::vector<double> latency_values = json_to_double_vector(data.at("qc").at("latency_ms"));
    std::string embedded = dump_json(data, 0);
    std::string snr_chart = build_line_chart_svg(snr_values, 520, 180, "#0f766e");
    std::string latency_chart = build_line_chart_svg(latency_values, 520, 180, "#b45309");

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
         << "<section class=\"grid\">"
         << "<div class=\"card\"><div class=\"muted\">PM2.5 RMSE</div><div class=\"metric\">" << data.at("metrics").at("pm25").at("rmse").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">PM10 RMSE</div><div class=\"metric\">" << data.at("metrics").at("pm10").at("rmse").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">热点 F1</div><div class=\"metric\">" << data.at("metrics").at("hotspot").at("f1").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">吞吐量</div><div class=\"metric\">" << data.at("metrics").at("runtime").at("throughput_profiles_per_s").number_value() << "</div></div>"
         << "</section><section class=\"grid\">"
         << "<div class=\"card\"><div class=\"pill\">平均 SNR</div>" << snr_chart << "</div>"
         << "<div class=\"card\"><div class=\"pill\">平均处理时延</div>" << latency_chart << "</div>"
         << "</section><section class=\"grid\">"
         << "<div class=\"card\"><div class=\"pill\">Source</div><div class=\"panel\" id=\"source-panel\"></div></div>"
         << "<div class=\"card\"><div class=\"pill\">Drift Monitoring</div><div class=\"panel\" id=\"drift-panel\"></div></div>"
         << "<div class=\"card\"><div class=\"pill\">Failure Cases</div><div class=\"panel\" id=\"failure-panel\"></div></div>"
         << "</section><script id=\"demo-data\" type=\"application/json\">" << embedded << "</script><script>"
         << "const data=JSON.parse(document.getElementById('demo-data').textContent);"
         << "document.getElementById('source-panel').textContent=JSON.stringify(data.source,null,2);"
         << "document.getElementById('drift-panel').textContent=JSON.stringify(data.drift_monitoring,null,2);"
         << "document.getElementById('failure-panel').textContent=JSON.stringify(data.failure_cases,null,2);"
         << "</script></main></body></html>";
    return html.str();
}

Json build_summary_payload(const Json& results) {
    return Json::object_type{
        {"dataset_summary", results.at("dataset_summary")},
        {"metrics", results.at("metrics")},
        {"latest_hotspots", results.at("hotspots")},
        {"alert_count", static_cast<int>(results.at("alerts").array_items().size())},
    };
}

} // namespace lidar_demo