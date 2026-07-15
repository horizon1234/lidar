#include "lidar_log/Logger.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <string>

int main() {
    const auto directory = std::filesystem::temp_directory_path() / "lidar_log_test";
    std::error_code error;
    std::filesystem::remove_all(directory, error);

    lidar_log::Logger::initialize({
        .file_path = directory / "test.log",
        .minimum_level = lidar_log::Level::trace,
        .console = false,
        .file = true,
        .max_file_size = 256,
        .max_files = 3,
    });

    for (int i = 0; i < 100; ++i) {
        LIDAR_LOG_INFO("rotation message #", i, " ", std::string(32, 'x'));
    }
    LIDAR_LOG_INFO("oversized payload: ", std::string(4096, 'y'));
    lidar_log::Logger::shutdown();

    std::size_t file_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            ++file_count;
            assert(std::filesystem::file_size(entry.path()) <= 256);
        }
    }
    assert(file_count <= 3);
    assert(std::filesystem::exists(directory / "test.log"));

    bool has_info = false;
    bool has_rotation_message = false;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        std::ifstream file(entry.path());
        const std::string contents((std::istreambuf_iterator<char>(file)),
                                   std::istreambuf_iterator<char>());
        has_info = has_info || contents.find("[INFO]") != std::string::npos;
        has_rotation_message = has_rotation_message
            || contents.find("rotation message") != std::string::npos;
    }
    assert(has_info);
    assert(has_rotation_message);

    std::filesystem::remove_all(directory, error);
    return 0;
}
