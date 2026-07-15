/**
 * @file Logger.hpp
 * @brief Small, thread-safe logger shared by all LiDAR processes and libraries.
 *
 * The logger writes to stderr and to a size-rotated file by default.  A process
 * normally only needs to configure it once at startup, then use one of the
 * LIDAR_LOG_* macros:
 *
 *   lidar_log::Logger::initialize({.file_path = "logs/server.log"});
 *   LIDAR_LOG_INFO("server started on port ", port);
 */
#pragma once

#include <cstddef>
#include <filesystem>
#include <source_location>
#include <sstream>
#include <string>
#include <string_view>

namespace lidar_log {

enum class Level {
    trace,
    debug,
    info,
    warn,
    error,
    fatal,

    // PascalCase aliases make the enum convenient for callers using either
    // the project's existing naming style or conventional logging names.
    Trace = trace,
    Debug = debug,
    Info = info,
    Warn = warn,
    Error = error,
    Fatal = fatal,
};

struct Config {
    /// Current log file.  Parent directories are created automatically.
    std::filesystem::path file_path = "logs/lidar.log";
    /// Messages below this level are discarded by both sinks.
    Level minimum_level = Level::info;
    /// Write to stderr in addition to the file.
    bool console = true;
    /// Disable this to run console-only, for example in a short-lived test.
    bool file = true;
    /// Maximum size of one file, including the current file, in bytes.
    std::size_t max_file_size = 100ULL * 1024ULL * 1024ULL;
    /// Maximum number of files kept, including the current file.
    std::size_t max_files = 10;
};

class Logger final {
public:
    static Logger& instance() noexcept;

    /// Configure or reconfigure the process-wide logger. Safe while logging.
    static void initialize(const Config& config = {});

    /// Flush and close the configured file. Logging remains safe afterwards;
    /// a later message will only be sent to the console until reinitialized.
    static void shutdown() noexcept;

    void set_minimum_level(Level level) noexcept;
    [[nodiscard]] Level minimum_level() const noexcept;

    [[nodiscard]] bool should_log(Level level) const noexcept;

    // The source_location overload is the implementation-friendly API for
    // callers that do not use the convenience macros.
    void log(Level level,
             std::string_view message,
             std::source_location location = std::source_location::current());

    template <typename... Args>
    void log(Level level,
             std::source_location location,
             const Args&... args) {
        if (!should_log(level)) {
            return;
        }
        log(level, detail::join(args...), location);
    }

private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void open_file_locked();

    struct Impl;
    Impl* impl_;

    struct detail {
        template <typename... Args>
        static std::string join(const Args&... args) {
            std::ostringstream stream;
            if constexpr (sizeof...(Args) > 0) {
                (stream << ... << args);
            }
            return stream.str();
        }
    };
};

std::string_view level_name(Level level) noexcept;

} // namespace lidar_log

#define LIDAR_LOG_TRACE(...) \
    ::lidar_log::Logger::instance().log(::lidar_log::Level::trace, \
                                         std::source_location::current(), __VA_ARGS__)
#define LIDAR_LOG_DEBUG(...) \
    ::lidar_log::Logger::instance().log(::lidar_log::Level::debug, \
                                         std::source_location::current(), __VA_ARGS__)
#define LIDAR_LOG_INFO(...) \
    ::lidar_log::Logger::instance().log(::lidar_log::Level::info, \
                                         std::source_location::current(), __VA_ARGS__)
#define LIDAR_LOG_WARN(...) \
    ::lidar_log::Logger::instance().log(::lidar_log::Level::warn, \
                                         std::source_location::current(), __VA_ARGS__)
#define LIDAR_LOG_ERROR(...) \
    ::lidar_log::Logger::instance().log(::lidar_log::Level::error, \
                                         std::source_location::current(), __VA_ARGS__)
#define LIDAR_LOG_FATAL(...) \
    ::lidar_log::Logger::instance().log(::lidar_log::Level::fatal, \
                                         std::source_location::current(), __VA_ARGS__)
