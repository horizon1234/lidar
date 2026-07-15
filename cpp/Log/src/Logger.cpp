#include "lidar_log/Logger.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

namespace lidar_log {

namespace {

std::string format_time(const std::chrono::system_clock::time_point now) {
    const auto time = std::chrono::system_clock::to_time_t(now);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch()) % 1000;
    std::tm local_time{};
#if defined(_WIN32)
    localtime_s(&local_time, &time);
#else
    localtime_r(&time, &local_time);
#endif

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
           << '.' << std::setfill('0') << std::setw(3) << millis.count();
    return stream.str();
}

std::string file_name_for(const std::filesystem::path& path, std::size_t index) {
    if (index == 0) {
        return path.string();
    }
    return path.string() + '.' + std::to_string(index);
}

} // namespace

struct Logger::Impl {
    mutable std::mutex mutex;
    Config config;
    std::ofstream file;
    std::size_t current_size = 0;
    bool initialized = false;
};

Logger::Logger() : impl_(new Impl) {
    std::lock_guard lock(impl_->mutex);
    impl_->initialized = true;
}

Logger::~Logger() {
    {
        std::lock_guard lock(impl_->mutex);
        if (impl_->file.is_open()) {
            impl_->file.flush();
            impl_->file.close();
        }
    }
    delete impl_;
}

Logger& Logger::instance() noexcept {
    static Logger logger;
    return logger;
}

void Logger::initialize(const Config& requested) {
    auto& logger = instance();
    std::lock_guard lock(logger.impl_->mutex);

    logger.impl_->config = requested;
    if (logger.impl_->config.max_file_size == 0) {
        logger.impl_->config.max_file_size = 100ULL * 1024ULL * 1024ULL;
    }
    if (logger.impl_->config.max_files == 0) {
        logger.impl_->config.max_files = 1;
    }

    if (logger.impl_->file.is_open()) {
        logger.impl_->file.flush();
        logger.impl_->file.close();
    }
    logger.impl_->current_size = 0;
    logger.impl_->initialized = true;

    logger.open_file_locked();
}

void Logger::open_file_locked() {
    if (!impl_->config.file) {
        return;
    }

    try {
        const auto parent = impl_->config.file_path.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
        std::error_code error;
        const auto existing_size = std::filesystem::file_size(impl_->config.file_path, error);
        if (!error) {
            impl_->current_size = static_cast<std::size_t>(existing_size);
        }
        impl_->file.open(impl_->config.file_path,
                         std::ios::out | std::ios::app | std::ios::binary);
        if (!impl_->file.is_open()) {
            impl_->config.file = false;
            std::fprintf(stderr, "[logger] failed to open log file '%s'\n",
                         impl_->config.file_path.c_str());
        }
    } catch (const std::exception& error) {
        impl_->config.file = false;
        std::fprintf(stderr, "[logger] failed to initialize file sink: %s\n", error.what());
    }
}

void Logger::shutdown() noexcept {
    auto& logger = instance();
    std::lock_guard lock(logger.impl_->mutex);
    if (logger.impl_->file.is_open()) {
        logger.impl_->file.flush();
        logger.impl_->file.close();
    }
    logger.impl_->initialized = false;
    logger.impl_->current_size = 0;
}

void Logger::set_minimum_level(const Level level) noexcept {
    std::lock_guard lock(impl_->mutex);
    impl_->config.minimum_level = level;
}

Level Logger::minimum_level() const noexcept {
    std::lock_guard lock(impl_->mutex);
    return impl_->config.minimum_level;
}

bool Logger::should_log(const Level level) const noexcept {
    std::lock_guard lock(impl_->mutex);
    return static_cast<int>(level) >= static_cast<int>(impl_->config.minimum_level);
}

void Logger::log(const Level level,
                 const std::string_view message,
                 const std::source_location location) {
    auto& logger = *this;
    std::lock_guard lock(logger.impl_->mutex);
    if (static_cast<int>(level) < static_cast<int>(logger.impl_->config.minimum_level)) {
        return;
    }

    std::ostringstream line;
    line << format_time(std::chrono::system_clock::now())
         << " [" << level_name(level) << "]"
         << " [tid=" << std::this_thread::get_id() << "] "
         << message << " (" << location.file_name() << ':' << location.line() << ')';
    const std::string rendered = line.str();

    if (logger.impl_->config.console) {
        std::cerr << rendered << '\n';
        std::cerr.flush();
    }

    if (!logger.impl_->config.file || !logger.impl_->initialized) {
        return;
    }
    // Open lazily for callers that use a macro before explicitly configuring
    // the logger. This avoids creating the default file merely by referencing
    // Logger::instance() during startup.
    if (!logger.impl_->file.is_open()) {
        logger.open_file_locked();
    }
    if (!logger.impl_->file.is_open()) {
        return;
    }

    // A caller can pass an accidentally huge payload. Keep the file sink's
    // hard size limit in that case; the console sink above still gets the
    // complete message for immediate diagnosis.
    std::string file_line = rendered;
    if (file_line.size() + 1 > logger.impl_->config.max_file_size) {
        file_line.resize(logger.impl_->config.max_file_size > 0
                             ? logger.impl_->config.max_file_size - 1
                             : 0);
    }
    const std::size_t bytes = file_line.size() + 1;
    if (logger.impl_->current_size > 0
        && logger.impl_->current_size + bytes > logger.impl_->config.max_file_size) {
        logger.impl_->file.flush();
        logger.impl_->file.close();

        // Keep at most max_files files in total: the active file plus backups
        // named .1, .2, ... .(max_files - 1).
        for (std::size_t index = logger.impl_->config.max_files; index-- > 1;) {
            const auto source = file_name_for(logger.impl_->config.file_path, index - 1);
            const auto destination = file_name_for(logger.impl_->config.file_path, index);
            std::error_code error;
            std::filesystem::remove(destination, error);
            std::filesystem::rename(source, destination, error);
        }
        std::error_code error;
        std::filesystem::remove(logger.impl_->config.file_path, error);
        logger.impl_->file.open(logger.impl_->config.file_path,
                                std::ios::out | std::ios::trunc | std::ios::binary);
        logger.impl_->current_size = 0;
    }

    if (logger.impl_->file.is_open()) {
        logger.impl_->file << file_line << '\n';
        logger.impl_->file.flush();
        logger.impl_->current_size += bytes;
    }
}

std::string_view level_name(const Level level) noexcept {
    switch (level) {
    case Level::trace: return "TRACE";
    case Level::debug: return "DEBUG";
    case Level::info: return "INFO";
    case Level::warn: return "WARN";
    case Level::error: return "ERROR";
    case Level::fatal: return "FATAL";
    }
    return "UNKNOWN";
}

} // namespace lidar_log
