/**
 * @file ApiServer.cpp
 * @brief 基于原生 socket 的极简 HTTP API 服务器。
 *
 * 本文件实现一个无第三方依赖（不使用 Boost.Asio / cpp-httplib）的 HTTP API：
 * - 通过 POSIX socket 或 Windows Winsock 提供同步阻塞的请求-响应服务；
 * - 暴露两个只读 JSON 路由：
 *     · GET /api/summary   —— 返回 metrics + curtain + ppi 等综合摘要；
 *     · GET /api/hotspots   —— 返回 latest_hotspots 热点列表；
 * - 每个请求都会即时执行流水线（run_end_to_end），适合 demo / 开发联调；
 * - 通过 SocketGuard（RAII）保证 socket 在异常路径下也会被关闭；
 * - WinsockSession 用 RAII 包裹 WSAStartup/WSACleanup，实现跨平台一致性。
 *
 * 架构概览：
 *   main() -> 解析 CLI -> (可选) --once 单次输出 / serve_forever() 常驻服务
 *   serve_forever() -> create_listen_socket() -> accept 循环
 *       -> receive_request() -> route_request() -> send_all()
 */

#include "LidarDemo/LidarDemo.hpp"

#include <array>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

// === 跨平台 socket 头部：Windows 走 Winsock2，POSIX 走 BSD socket ===
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX // 阻止 windows.h 定义 min/max 宏，避免与 std::min/max 冲突
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

// === 平台无关的 socket 句柄别名：抽象 Windows(SOCKET) 与 POSIX(int) 的差异 ===
#ifdef _WIN32
using SocketHandle = SOCKET; // Windows：句柄类型为 SOCKET（无符号）
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET; // Windows 的无效句柄常量
#else
using SocketHandle = int; // POSIX：句柄就是文件描述符
constexpr SocketHandle kInvalidSocket = -1; // POSIX 的无效句柄常量
#endif

/**
 * @brief 跨平台关闭 socket 句柄。
 *
 * Windows 调用 closesocket()，POSIX 调用 close()；对无效句柄做幂等保护。
 *
 * @param handle 待关闭的 socket 句柄。
 */
void close_socket(SocketHandle handle) {
#ifdef _WIN32
    if (handle != INVALID_SOCKET) {
        closesocket(handle);
    }
#else
    if (handle >= 0) {
        close(handle);
    }
#endif
}

/**
 * @brief socket 的 RAII 包装：析构时自动关闭，防止异常路径下句柄泄漏。
 *
 * 该类型只可移动（move-only），不可拷贝，避免同一句柄被多次 close。
 * 提供 release() 方法用于"交出所有权"的语义。
 */
struct SocketGuard {
    SocketHandle handle = kInvalidSocket; ///< 被托管的 socket 句柄

    SocketGuard() = default;
    explicit SocketGuard(SocketHandle input) : handle(input) {}
    ~SocketGuard() {
        close_socket(handle); // 析构时自动关闭，确保异常安全
    }

    SocketGuard(const SocketGuard&) = delete; // 禁止拷贝，避免双重 close
    SocketGuard& operator=(const SocketGuard&) = delete;

    SocketGuard(SocketGuard&& other) noexcept : handle(other.handle) {
        other.handle = kInvalidSocket; // 移动后源对象置为无效，避免被重复 close
    }

    SocketGuard& operator=(SocketGuard&& other) noexcept {
        if (this != &other) {
            close_socket(handle); // 先释放当前持有的句柄
            handle = other.handle;
            other.handle = kInvalidSocket;
        }
        return *this;
    }

    /**
     * @brief 放弃所有权并返回句柄。
     *
     * 调用后本对象不再负责关闭该句柄，调用方需自行管理其生命周期。
     *
     * @return SocketHandle 释放出的句柄值。
     */
    SocketHandle release() {
        SocketHandle value = handle;
        handle = kInvalidSocket;
        return value;
    }
};

#ifdef _WIN32
/**
 * @brief Winsock 会话的 RAII 包装：在进程级生命周期内完成 WSAStartup/WSACleanup。
 *
 * Windows 在使用任何 socket API 前必须调用 WSAStartup；构造时初始化，
 * 析构时调用 WSACleanup。一处构造、与 main 同寿命即可。
 */
struct WinsockSession {
    WSADATA data{}; // Winsock 返回的版本与能力信息

    WinsockSession() {
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) { // 请求 Winsock 2.2
            throw std::runtime_error("Failed to initialize Winsock");
        }
    }

    ~WinsockSession() {
        WSACleanup(); // 配对释放
    }
};
#endif

/**
 * @brief 构造平台相关的错误前缀字符串，便于在异常消息中定位根因。
 *
 * Windows 输出 WSAGetLastError()，POSIX 输出 strerror(errno)。
 *
 * @param message 用户提供的错误描述。
 * @return std::string 拼接后的完整错误字符串。
 */
std::string socket_error_prefix(const std::string& message) {
#ifdef _WIN32
    return message + " (winsock error " + std::to_string(WSAGetLastError()) + ")";
#else
    return message + ": " + std::strerror(errno);
#endif
}

/**
 * @brief 执行流水线并构造 API 摘要负载。
 *
 * 每次请求都会重新读取配置并端到端跑一次：开发/演示语义，请勿在生产用此模式。
 *
 * @param config_path 流水线 JSON 配置文件路径。
 * @return lidar_demo::Json 包含 metrics/curtain/ppi/latest_hotspots 等的摘要。
 */
lidar_demo::Json build_summary(const std::filesystem::path& config_path) {
    lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(lidar_demo::read_json_file(config_path));
    return lidar_demo::build_summary_payload(lidar_demo::run_end_to_end(config));
}

/**
 * @brief 构造一个最小可用的 HTTP/1.1 响应字符串。
 *
 * 包含状态行、Content-Type、Content-Length、Connection: close、Cache-Control: no-store，
 * 然后是空行（\r\n\r\n）与响应体。本服务器采用短连接策略：每个响应后关闭连接。
 *
 * @param status_code  HTTP 状态码（如 200、404）。
 * @param status_text  状态描述（如 "OK"、"Not Found"）。
 * @param content_type MIME 类型（如 "application/json"）。
 * @param body         响应体内容。
 * @return std::string 完整的 HTTP 响应字符串。
 */
std::string build_response(
    int status_code,
    const std::string& status_text,
    const std::string& content_type,
    const std::string& body
) {
    std::ostringstream response;
    response << "HTTP/1.1 " << status_code << ' ' << status_text << "\r\n";
    response << "Content-Type: " << content_type << "\r\n";
    response << "Content-Length: " << body.size() << "\r\n";
    response << "Connection: close\r\n"; // 短连接：响应完毕即关闭
    response << "Cache-Control: no-store\r\n\r\n"; // 实时计算结果，禁用缓存
    response << body;
    return response.str();
}

/**
 * @brief 持续发送直到整个 payload 写完，处理部分写（partial write）。
 *
 * 流式 send 可能一次只写入部分字节，需要循环直至全部发送完毕；
 * 出错或对端关闭时抛出带平台错误信息的异常。
 *
 * @param handle  目标 socket 句柄。
 * @param payload 待发送的完整字符串。
 */
void send_all(SocketHandle handle, const std::string& payload) {
    std::size_t sent = 0;
    while (sent < payload.size()) {
#ifdef _WIN32
        int count = send(handle, payload.data() + sent, static_cast<int>(payload.size() - sent), 0);
#else
        ssize_t count = send(handle, payload.data() + sent, payload.size() - sent, 0);
#endif
        if (count <= 0) { // 出错或对端关闭：抛异常带错误细节
            throw std::runtime_error(socket_error_prefix("Failed to send HTTP response"));
        }
        sent += static_cast<std::size_t>(count);
    }
}

/**
 * @brief 读取 HTTP 请求头直到遇到空行（\r\n\r\n）。
 *
 * 采用增量 recv 策略：每次读到 4KB 缓冲，拼接到请求字符串；
 * 检测到头尾分隔符即停止读取，避免无谓等待 body；
 * 设有 16KB 上限以防恶意超长请求耗尽内存。
 *
 * @param handle 客户端 socket 句柄。
 * @return std::string 原始请求字符串；对端提前关闭时返回已读部分（可能为空）。
 */
std::string receive_request(SocketHandle handle) {
    std::string request;
    std::array<char, 4096> buffer{};
    while (request.find("\r\n\r\n") == std::string::npos) { // 一直读到头尾分隔符
#ifdef _WIN32
        int count = recv(handle, buffer.data(), static_cast<int>(buffer.size()), 0);
#else
        ssize_t count = recv(handle, buffer.data(), buffer.size(), 0);
#endif
        if (count < 0) { // 读取出错
            throw std::runtime_error(socket_error_prefix("Failed to read HTTP request"));
        }
        if (count == 0) { // 对端主动关闭连接
            break;
        }
        request.append(buffer.data(), static_cast<std::size_t>(count));
        if (request.size() > 16384) { // 防 DoS：请求头超过 16KB 即截断
            break;
        }
    }
    return request;
}

/**
 * @brief 路由分发：根据请求行解析 method/target 并匹配已知路由。
 *
 * 路由规则：
 *   - 仅支持 GET，其他方法返回 405 Method Not Allowed；
 *   - 仅支持 /api/summary 与 /api/hotspots，其余返回 404 Not Found；
 *   - 查询串（?key=value）会被剥离后再做路径匹配；
 *   - 路由内部出现异常时返回 500 并以 JSON 形式回传错误描述。
 *
 * @param request     原始 HTTP 请求字符串。
 * @param config_path 流水线配置文件路径（用于即时构建结果）。
 * @return std::string 完整的 HTTP 响应字符串（含状态行、头部、正文）。
 */
std::string route_request(const std::string& request, const std::filesystem::path& config_path) {
    std::istringstream stream(request);
    std::string method;
    std::string target;
    std::string version;
    stream >> method >> target >> version; // 解析请求行："GET /api/summary HTTP/1.1"

    if (method.empty() || target.empty()) { // 请求行不完整
        return build_response(400, "Bad Request", "text/plain; charset=utf-8", "Bad Request");
    }
    if (method != "GET") { // 非 GET 一律拒绝
        return build_response(405, "Method Not Allowed", "text/plain; charset=utf-8", "Method Not Allowed");
    }

    // === 剥离查询串：只保留路径部分用于匹配 ===
    std::size_t query_index = target.find('?');
    if (query_index != std::string::npos) {
        target = target.substr(0, query_index);
    }

    if (target != "/api/summary" && target != "/api/hotspots") { // 仅暴露两个只读路由
        return build_response(404, "Not Found", "text/plain; charset=utf-8", "Not Found");
    }

    try {
        lidar_demo::Json summary = build_summary(config_path); // 即时端到端计算
        const lidar_demo::Json& payload = target == "/api/summary" ? summary : summary.at("latest_hotspots"); // 选择子树
        return build_response(200, "OK", "application/json; charset=utf-8", lidar_demo::dump_json(payload, 2));
    } catch (const std::exception& error) {
        // === 内部错误：以 JSON 体回传错误描述，便于前端展示 ===
        lidar_demo::Json body = lidar_demo::Json::Object{{"error", error.what()}};
        return build_response(500, "Internal Server Error", "application/json; charset=utf-8", lidar_demo::dump_json(body, 2));
    }
}

/**
 * @brief 创建并绑定监听 socket（getaddrinfo + bind + listen）。
 *
 * 使用 getaddrinfo 获取候选地址族（IPv4/IPv6 均可，因 AF_UNSPEC），
 * 逐个尝试 socket()->setsockopt(SO_REUSEADDR)->bind()->listen()，第一个成功者即返回。
 * SO_REUSEADDR 用于开发期快速重启，避免 TIME_WAIT 导致端口占用。
 *
 * @param host 监听主机；为空时绑定所有接口（nullptr + AI_PASSIVE）。
 * @param port 监听端口。
 * @return SocketGuard 持有已进入 LISTEN 状态的 socket。
 */
SocketGuard create_listen_socket(const std::string& host, int port) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC; // 同时允许 IPv4/IPv6
    hints.ai_socktype = SOCK_STREAM; // TCP
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE; // 用于 bind：host 为空时绑定 INADDR_ANY

    addrinfo* result = nullptr;
    std::string port_text = std::to_string(port);
    const char* node = host.empty() ? nullptr : host.c_str(); // nullptr + AI_PASSIVE 表示所有接口
    int status = getaddrinfo(node, port_text.c_str(), &hints, &result);
    if (status != 0) {
#ifdef _WIN32
        throw std::runtime_error("getaddrinfo failed for host binding");
#else
        throw std::runtime_error(std::string("getaddrinfo failed: ") + gai_strerror(status));
#endif
    }

    SocketGuard listen_socket;
    for (addrinfo* current = result; current != nullptr; current = current->ai_next) {
        SocketHandle handle = socket(current->ai_family, current->ai_socktype, current->ai_protocol);
        if (handle == kInvalidSocket) {
            continue; // 该地址族不可用，尝试下一个候选
        }

        int reuse = 1;
        setsockopt(handle, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse)); // 允许端口快速重用
        if (bind(handle, current->ai_addr, static_cast<int>(current->ai_addrlen)) == 0 && listen(handle, SOMAXCONN) == 0) {
            listen_socket = SocketGuard(handle); // 成功：交给 RAII 托管
            break;
        }
        close_socket(handle); // 绑定/监听失败：关闭后尝试下一候选
    }
    freeaddrinfo(result); // 释放 getaddrinfo 的链表

    if (listen_socket.handle == kInvalidSocket) { // 全部候选都失败
        throw std::runtime_error(socket_error_prefix("Failed to bind HTTP server socket"));
    }
    return listen_socket;
}

/**
 * @brief 主服务循环：accept 连接并同步处理请求。
 *
 * 同步阻塞模型，单连接串行处理：适合 demo / 低并发场景。
 * 每条连接由 SocketGuard 托管，异常或循环退出都能正确释放。
 *
 * @param host        监听主机。
 * @param port        监听端口。
 * @param config_path 流水线配置文件路径。
 */
void serve_forever(const std::string& host, int port, const std::filesystem::path& config_path) {
    SocketGuard listen_socket = create_listen_socket(host, port);
    std::cout << "Serving on http://" << host << ':' << port << "/api/summary" << std::endl;

    for (;;) { // accept 循环：每次取一个连接同步处理
        SocketHandle client_handle = accept(listen_socket.handle, nullptr, nullptr);
        if (client_handle == kInvalidSocket) {
            throw std::runtime_error(socket_error_prefix("Failed to accept HTTP connection"));
        }
        SocketGuard client(client_handle); // 客户端 socket 交给 RAII
        std::string request = receive_request(client.handle);
        if (request.empty()) { // 对端未发送有效请求，跳过
            continue;
        }
        std::string response = route_request(request, config_path);
        send_all(client.handle, response);
    }
}

} // namespace

/**
 * @brief HTTP API 服务器主入口。
 *
 * 支持以下命令行参数：
 *   --config <path>   流水线 JSON 配置，默认 configs/DefaultPipeline.json
 *   --host <ip>       监听地址，默认 127.0.0.1（仅本机）
 *   --port <int>      监听端口，默认 8765
 *   --once            一次性输出摘要并退出（不进入常驻服务，便于调试/脚本）
 *
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组。
 * @return int 0 表示正常退出；1 表示异常。
 */
int main(int argc, char** argv) {
    try {
        // === 默认参数：仅本机访问，默认端口 8765 ===
        std::filesystem::path config_path = "configs/DefaultPipeline.json"; ///< 流水线配置文件
        std::string host = "127.0.0.1"; ///< 监听地址
        int port = 8765; ///< 监听端口
        bool once = false; ///< 是否仅输出一次（--once）

        // === 命令行参数解析 ===
        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--config" && index + 1 < argc) {
                config_path = argv[++index];
            } else if (argument == "--host" && index + 1 < argc) {
                host = argv[++index];
            } else if (argument == "--port" && index + 1 < argc) {
                port = std::stoi(argv[++index]); // 字符串转 int（非法值会抛异常被外层捕获）
            } else if (argument == "--once") {
                once = true; // 此次只做一次摘要并退出
            }
        }

        if (once) {
            // === 单次模式：直接打印摘要并退出，便于与脚本/调试对接 ===
            lidar_demo::Json summary = build_summary(config_path);
            std::cout << lidar_demo::dump_json(summary, 2) << std::endl;
            return 0;
        }

#ifdef _WIN32
        WinsockSession winsock; // Windows 必须先初始化 Winsock
#endif
        serve_forever(host, port, config_path); // 进入常驻服务循环
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}