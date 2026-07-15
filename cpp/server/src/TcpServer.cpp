/**
 * @file TcpServer.cpp
 * @brief Linux POSIX TCP 服务器实现。
 */
#include "lidar_server/TcpServer.hpp"

#include <cstring>
#include <stdexcept>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "lidar_log/Logger.hpp"
using SocketHandle = int;
constexpr SocketHandle InvalidSocketValue = -1;

namespace lidar_server {

namespace {

void close_socket(int sock) {
    if (sock < 0) return;
    ::close(sock);
}

void shutdown_socket(int sock) {
    // 先 shutdown 再 close，确保另一线程中阻塞的 accept/recv 能被可靠唤醒。
    if (sock < 0) return;
    ::shutdown(sock, SHUT_RDWR);
}

} // anonymous namespace

TcpServer::TcpServer(std::uint16_t port)
    : port_(port), listen_socket_(-1), client_socket_(-1), running_(false) {
}

TcpServer::~TcpServer() {
    stop();
}

void TcpServer::start(const std::function<void(const std::string&)>& handler) {
    // 创建 listen socket
    SocketHandle listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd == InvalidSocketValue) {
        throw std::runtime_error("Failed to create listen socket");
    }

    // 允许地址重用
    int opt = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);
    if (::bind(listen_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close_socket(static_cast<int>(listen_fd));
        throw std::runtime_error("Failed to bind port " + std::to_string(port_));
    }

    // 监听
    if (::listen(listen_fd, 1) < 0) {
        close_socket(static_cast<int>(listen_fd));
        throw std::runtime_error("Failed to listen on port " + std::to_string(port_));
    }

    listen_socket_.store(static_cast<int>(listen_fd));
    running_.store(true);

    LIDAR_LOG_INFO("[tcp_server] Listening on port ", port_, " ...");

    while (running_) {
        // 接受连接
        SocketHandle client_fd = ::accept(listen_fd, nullptr, nullptr);
        if (client_fd == InvalidSocketValue) {
            if (running_) {
                LIDAR_LOG_ERROR("[tcp_server] accept() failed");
            }
            break;
        }

        client_socket_ = static_cast<int>(client_fd);
        LIDAR_LOG_INFO("[tcp_server] Client connected.");

        // 逐行读取
        std::string buffer;
        char chunk[4096];
        while (running_) {
            int bytes = static_cast<int>(::recv(client_fd, chunk, sizeof(chunk), 0));
            if (bytes <= 0) {
                break; // 连接关闭或出错
            }
            buffer.append(chunk, static_cast<std::size_t>(bytes));

            // 按行分割
            std::size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                if (handler) {
                    handler(line);
                }
            }
        }

        // 原子地取出 fd 并置 -1，避免与 stop() 的并发 close 冲突
        if (int fd = client_socket_.exchange(-1); fd >= 0) {
            close_socket(fd);
        }
        LIDAR_LOG_INFO("[tcp_server] Client disconnected.");
    }

    if (int fd = listen_socket_.exchange(-1); fd >= 0) {
        close_socket(fd);
    }
}

void TcpServer::stop() {
    running_.store(false);

    // exchange 原子地取值并置 -1：即使与 start() 并发，也只有一个线程会 close
    if (int fd = listen_socket_.exchange(-1); fd >= 0) {
        shutdown_socket(fd);
        close_socket(fd);
    }
    if (int fd = client_socket_.exchange(-1); fd >= 0) {
        shutdown_socket(fd);
        close_socket(fd);
    }
}

bool TcpServer::send_line(const std::string& line) {
    // 加锁串行化：主线程推送数据帧与 handler 回调推送 ACK 可能同时调用，
    // 不加锁会导致两次 send() 的字节交错，客户端解析 JSON 行失败。
    std::lock_guard<std::mutex> lk(send_mutex_);

    int fd = client_socket_.load();
    if (fd < 0) {
        return false;
    }
    std::string data = line + "\n";
    int total_sent = 0;
    int remaining = static_cast<int>(data.size());
    const char* ptr = data.c_str();
    while (remaining > 0) {
        int sent = static_cast<int>(::send(fd, ptr + total_sent, remaining, 0));
        if (sent <= 0) {
            return false;
        }
        total_sent += sent;
        remaining -= sent;
    }
    return true;
}

bool TcpServer::has_client() const {
    return client_socket_.load() >= 0;
}

bool TcpServer::is_listening() const {
    return listen_socket_.load() >= 0;
}

} // namespace lidar_server
