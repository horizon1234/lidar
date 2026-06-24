/**
 * @file tcp_client.cpp
 * @brief 跨平台 TCP 客户端实现。
 */
#include "lidar_client/tcp_client.hpp"

#include <algorithm>
#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socket_t = SOCKET;
    constexpr socket_t INVALID_S = INVALID_SOCKET;
    constexpr int CLOSE_ERR = SOCKET_ERROR;

    // Winsock 全局初始化（进程级，只执行一次）
    struct WinsockInit {
        WinsockInit() {
            WSADATA data;
            WSAStartup(MAKEWORD(2, 2), &data);
        }
        ~WinsockInit() {
            WSACleanup();
        }
    };
    static WinsockInit g_winsock_init;
#else
    #include <arpa/inet.h>
    #include <netdb.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <unistd.h>
    using socket_t = int;
    constexpr socket_t INVALID_S = -1;
    constexpr int CLOSE_ERR = -1;
#endif

namespace lidar_client {

static void close_socket(int sock) {
#ifdef _WIN32
    ::closesocket(sock);
#else
    ::close(sock);
#endif
}

TcpClient::TcpClient()
    : socket_(static_cast<int>(INVALID_S)) {
}

TcpClient::~TcpClient() {
    disconnect();
}

bool TcpClient::connect(const std::string& host, std::uint16_t port) {
    // 先断开旧连接
    disconnect();

    socket_t sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_S) {
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    // 解析主机名 / IP
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
        // 尝试 DNS 解析
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* result = nullptr;
        if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result) != 0 || result == nullptr) {
            close_socket(static_cast<int>(sock));
            return false;
        }
        sockaddr_in* resolved = reinterpret_cast<sockaddr_in*>(result->ai_addr);
        addr.sin_addr = resolved->sin_addr;
        freeaddrinfo(result);
    }

    if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close_socket(static_cast<int>(sock));
        return false;
    }

    socket_ = static_cast<int>(sock);
    buffer_.clear();
    return true;
}

void TcpClient::disconnect() {
    if (socket_ != static_cast<int>(INVALID_S)) {
        close_socket(socket_);
        socket_ = static_cast<int>(INVALID_S);
    }
    buffer_.clear();
}

bool TcpClient::is_connected() const {
    return socket_ != static_cast<int>(INVALID_S);
}

bool TcpClient::send_line(const std::string& line) {
    if (!is_connected()) {
        return false;
    }

    std::string data = line + "\n";
    const char* ptr = data.data();
    std::size_t remaining = data.size();

    while (remaining > 0) {
#ifdef _WIN32
        int sent = ::send(socket_, ptr, static_cast<int>(remaining), 0);
#else
        ssize_t sent = ::send(socket_, ptr, remaining, 0);
#endif
        if (sent <= 0) {
            return false;
        }
        ptr += sent;
        remaining -= static_cast<std::size_t>(sent);
    }
    return true;
}

bool TcpClient::read_line(std::string& line) {
    if (!is_connected()) {
        return false;
    }

    while (true) {
        // 检查缓冲区中是否已有完整行
        std::size_t newline_pos = buffer_.find('\n');
        if (newline_pos != std::string::npos) {
            line = buffer_.substr(0, newline_pos);
            buffer_.erase(0, newline_pos + 1);
            return true;
        }

        // 缓冲区不完整，从 socket 读取更多数据
        char recv_buf[8192];
#ifdef _WIN32
        int received = ::recv(socket_, recv_buf, sizeof(recv_buf), 0);
#else
        ssize_t received = ::recv(socket_, recv_buf, sizeof(recv_buf), 0);
#endif
        if (received <= 0) {
            // 连接关闭或出错
            // 如果缓冲区还有残余数据，先返回
            if (!buffer_.empty()) {
                line = buffer_;
                buffer_.clear();
                return true;
            }
            return false;
        }
        buffer_.append(recv_buf, static_cast<std::size_t>(received));
    }
}

} // namespace lidar_client
