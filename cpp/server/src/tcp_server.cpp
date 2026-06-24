/**
 * @file tcp_server.cpp
 * @brief 跨平台 TCP 服务器实现。
 */
#include "lidar_server/tcp_server.hpp"

#include <cstring>
#include <iostream>
#include <stdexcept>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using socket_type = SOCKET;
constexpr socket_type INVALID_SOCKET_VALUE = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_type = int;
constexpr socket_type INVALID_SOCKET_VALUE = -1;
#endif

namespace lidar_server {

namespace {

/// 全局 Winsock 初始化计数（Windows only）
#ifdef _WIN32
struct WinsockInit {
    WinsockInit() {
        WSADATA wsa_data;
        WSAStartup(MAKEWORD(2, 2), &wsa_data);
    }
    ~WinsockInit() {
        WSACleanup();
    }
};
WinsockInit g_winsock_init;
#endif

void close_socket(int sock) {
    if (sock < 0) return;
#ifdef _WIN32
    closesocket(static_cast<socket_type>(sock));
#else
    ::close(sock);
#endif
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
    socket_type listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd == INVALID_SOCKET_VALUE) {
        throw std::runtime_error("Failed to create listen socket");
    }

    // 允许地址重用
    int opt = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));

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

    listen_socket_ = static_cast<int>(listen_fd);
    running_ = true;

    std::cerr << "[tcp_server] Listening on port " << port_ << " ...\n";

    while (running_) {
        // 接受连接
        socket_type client_fd = ::accept(listen_fd, nullptr, nullptr);
        if (client_fd == INVALID_SOCKET_VALUE) {
            if (running_) {
                std::cerr << "[tcp_server] accept() failed\n";
            }
            break;
        }

        client_socket_ = static_cast<int>(client_fd);
        std::cerr << "[tcp_server] Client connected.\n";

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

        close_socket(client_socket_);
        client_socket_ = -1;
        std::cerr << "[tcp_server] Client disconnected.\n";
    }

    close_socket(listen_socket_);
    listen_socket_ = -1;
}

void TcpServer::stop() {
    running_ = false;
    if (listen_socket_ >= 0) {
        close_socket(listen_socket_);
        listen_socket_ = -1;
    }
    if (client_socket_ >= 0) {
        close_socket(client_socket_);
        client_socket_ = -1;
    }
}

bool TcpServer::send_line(const std::string& line) {
    if (client_socket_ < 0) {
        return false;
    }
    std::string data = line + "\n";
    int total_sent = 0;
    int remaining = static_cast<int>(data.size());
    const char* ptr = data.c_str();
    while (remaining > 0) {
        int sent = static_cast<int>(::send(client_socket_, ptr + total_sent, remaining, 0));
        if (sent <= 0) {
            return false;
        }
        total_sent += sent;
        remaining -= sent;
    }
    return true;
}

bool TcpServer::has_client() const {
    return client_socket_ >= 0;
}

bool TcpServer::is_listening() const {
    return listen_socket_ >= 0;
}

} // namespace lidar_server
