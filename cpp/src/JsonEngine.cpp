/**
 * @file JsonEngine.cpp
 * @brief Lightweight JSON value, parser, serializer, and file I/O implementation.
 */
#include "LidarDemo/Json.hpp"

#include <cmath>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lidar_demo {

Json::Json() : value(nullptr) {}
Json::Json(std::nullptr_t) : value(nullptr) {}
Json::Json(bool input) : value(input) {}
Json::Json(int input) : value(static_cast<double>(input)) {}
Json::Json(double input) : value(input) {}
Json::Json(const char* input) : value(std::string(input)) {}
Json::Json(const std::string& input) : value(input) {}
Json::Json(std::string&& input) : value(std::move(input)) {}
Json::Json(const Array& input) : value(input) {}
Json::Json(Array&& input) : value(std::move(input)) {}
Json::Json(const Object& input) : value(input) {}
Json::Json(Object&& input) : value(std::move(input)) {}

bool Json::is_null() const { return std::holds_alternative<std::nullptr_t>(value); }
bool Json::is_bool() const { return std::holds_alternative<bool>(value); }
bool Json::is_number() const { return std::holds_alternative<double>(value); }
bool Json::is_string() const { return std::holds_alternative<std::string>(value); }
bool Json::is_array() const { return std::holds_alternative<Array>(value); }
bool Json::is_object() const { return std::holds_alternative<Object>(value); }

bool Json::bool_value() const { return std::get<bool>(value); }
double Json::number_value() const { return std::get<double>(value); }
int Json::int_value() const { return static_cast<int>(std::lround(number_value())); }
const std::string& Json::string_value() const { return std::get<std::string>(value); }
const Json::Array& Json::array_items() const { return std::get<Array>(value); }
Json::Array& Json::array_items() { return std::get<Array>(value); }
const Json::Object& Json::object_items() const { return std::get<Object>(value); }
Json::Object& Json::object_items() { return std::get<Object>(value); }

bool Json::contains(const std::string& key) const {
    return is_object() && object_items().find(key) != object_items().end();
}

const Json& Json::at(const std::string& key) const {
    return object_items().at(key);
}

Json& Json::operator[](const std::string& key) {
    if (!is_object()) {
        value = Object{};
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
        Json::Object object;
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
        Json::Array array;
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
                output.push_back(code_point <= 0x7F ? static_cast<char>(code_point) : '?');
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
                builder << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(current)) << std::dec;
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

} // namespace lidar_demo
