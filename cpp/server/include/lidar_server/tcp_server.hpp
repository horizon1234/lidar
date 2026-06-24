/**
 * @file tcp_server.hpp
 * @brief 跨平台 TCP 服务器：接受连接、按行读取帧、发送帧。
 *
 * Windows 使用 Winsock2，POSIX 使用标准 socket API。
 * 设计为单线程阻塞模型（sim server 场景下客户端通常只有一个），
 * 可被 worker 线程包裹实现多客户端。
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace lidar_server {

/**
 * @brief TCP 服务器。
 *
 * 生命周期：
 *   TcpServer server(port);
 *   server.start(handler);  // 阻塞直到 stop()
 *
 * handler 回调签名：void(const std::string& line)
 *   - 每收到一行（'\n' 分隔）就调用一次
 *   - 在 handler 中可调用 server.send_line() 向该连接回写数据
 */
class TcpServer {
public:
    /**
     * @brief 构造并绑定端口。
     * @param port 监听端口
     */
    explicit TcpServer(std::uint16_t port);
    ~TcpServer();

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;

    /**
     * @brief 启动监听并接受连接。
     *
     * 阻塞调用线程，循环接受客户端连接。每接受一个连接，
     * 在当前线程中逐行读取数据并调用 handler。
     *
     * @param handler 行回调函数
     */
    void start(const std::function<void(const std::string&)>& handler);

    /**
     * @brief 停止服务器（关闭 listen socket）。
     */
    void stop();

    /**
     * @brief 向当前连接的客户端发送一行文本（自动追加 '\n'）。
     * @param line 要发送的文本行（不含 '\n'）
     * @return 发送是否成功
     */
    bool send_line(const std::string& line);

    /**
     * @brief 获取监听端口。
     */
    std::uint16_t port() const { return port_; }

    /**
     * @brief 判断当前是否有客户端连接。
     */
    bool has_client() const;

private:
    std::uint16_t port_;
    int listen_socket_;  ///< POSIX: fd; Windows: SOCKET cast to int
    int client_socket_;  ///< 当前活跃客户端
    bool running_;
};

} // namespace lidar_server
