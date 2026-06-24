/**
 * @file tcp_client.hpp
 * @brief 跨平台 TCP 客户端：连接服务器、按行读取帧、发送帧。
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace lidar_client {

/**
 * @brief TCP 客户端。
 *
 * 生命周期：
 *   TcpClient client;
 *   client.connect(host, port);
 *   client.run_line_loop(handler);  // 阻塞，逐行回调
 *   client.disconnect();
 */
class TcpClient {
public:
    TcpClient();
    ~TcpClient();

    TcpClient(const TcpClient&) = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    /**
     * @brief 连接到服务器。
     * @param host 主机名或 IP 地址
     * @param port 端口
     * @return 是否连接成功
     */
    bool connect(const std::string& host, std::uint16_t port);

    /**
     * @brief 断开连接。
     */
    void disconnect();

    /**
     * @brief 是否已连接。
     */
    bool is_connected() const;

    /**
     * @brief 发送一行文本（自动追加 '\n'）。
     */
    bool send_line(const std::string& line);

    /**
     * @brief 阻塞读取一行文本。
     *
     * 内部维护缓冲区，按 '\n' 切分。当连接关闭时返回空字符串。
     *
     * @param line 输出参数：读取到的一行（不含 '\n'）
     * @return 是否成功读取（false 表示连接已断开）
     */
    bool read_line(std::string& line);

private:
    int socket_;
    std::string buffer_;
};

} // namespace lidar_client
