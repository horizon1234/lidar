/**
 * @file tcp_server.hpp
 * @brief Linux POSIX TCP 服务器：接受连接、按行读取帧、发送帧。
 *
 * 线程模型：start() 在 worker 线程中阻塞运行（accept + recv 循环），
 * 主线程可并发调用 has_client() / is_listening() / send_line()。
 * 内部共享状态使用 std::atomic 保证可见性，send_line() 使用 mutex
 * 串行化并发发送，避免两条 JSON 行的字节交错。
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
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

    /**
     * @brief 判断是否已成功绑定并开始监听。
     */
    bool is_listening() const;

private:
    std::uint16_t port_;              ///< 当前 POSIX TCP 监听端口。
    std::atomic<int> listen_socket_;  ///< 当前监听 socket 文件描述符。
    std::atomic<int> client_socket_;  ///< 当前活跃客户端（原子：子线程写、主线程读）
    std::atomic<bool> running_;       ///< 服务器运行标志
    std::mutex send_mutex_;           ///< 串行化 send_line()，防止并发 send() 字节交错
};

} // namespace lidar_server
