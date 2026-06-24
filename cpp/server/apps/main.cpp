/**
 * @file main.cpp
 * @brief 仿真 LiDAR TCP 服务器入口。
 *
 * 启动后：
 *   1. 创建 SimDevice，一次性生成全部仿真数据
 *   2. 创建 TcpServer 监听指定端口
 *   3. 客户端连接后，周期性推送每个时间步的扫描帧
 *   4. 客户端断开后等待重新连接
 *
 * 用法：
 *   lidar_sim_server [port] [step_delay_ms]
 *
 * 默认端口 19850，默认帧间延迟 50ms。
 */
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "lidar_server/sim_device.hpp"
#include "lidar_server/tcp_server.hpp"
#include "lidar_protocol/frame.hpp"

static std::atomic<bool> g_should_stop{false};

void signal_handler(int /*sig*/) {
    g_should_stop = true;
}

int main(int argc, char* argv[]) {
    std::uint16_t port = 19850;
    int step_delay_ms = 500; // 每个时间步之间的间隔

    if (argc >= 2) {
        port = static_cast<std::uint16_t>(std::stoi(argv[1]));
    }
    if (argc >= 3) {
        step_delay_ms = std::stoi(argv[2]);
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ---- 初始化仿真设备 ----
    lidar_server::SimDeviceConfig device_config;
    // 如果命令行参数可以覆盖更多配置，这里可以扩展
    lidar_server::SimDevice device(device_config);

    if (device.total_steps() == 0) {
        std::cerr << "[server] SimDevice initialization failed, exiting.\n";
        return 1;
    }

    std::cerr << "[server] SimDevice ready: " << device.total_steps() << " steps, "
              << "site=" << device.site_info().site_id << "\n";

    // ---- 创建 TCP 服务器 ----
    lidar_server::TcpServer server(port);

    // 行处理回调（接收客户端命令，目前仅打日志）
    auto handler = [&server](const std::string& line) {
        std::cerr << "[server] Received from client: " << line << "\n";
        try {
            auto frame = lidar_protocol::parse_frame(line);
            if (frame.type == lidar_protocol::FrameType::command) {
                std::cerr << "[server] Command received: "
                          << lidar_core::dump_json(frame.payload) << "\n";
                // 简单回 ACK
                server.send_line("{\"type\":\"status\",\"status\":\"ack\"}");
            }
        } catch (const std::exception& e) {
            // 非 JSON 行，忽略
        }
    };

    // ---- 在独立线程中运行服务器 ----
    std::atomic<std::string*> startup_error{nullptr};
    std::string error_msg;
    std::thread server_thread([&]() {
        try {
            server.start(handler);
        } catch (const std::exception& e) {
            error_msg = e.what();
            startup_error.store(&error_msg);
        }
    });

    // 等待服务器启动（或失败），最多 2 秒
    for (int i = 0; i < 20 && !startup_error.load(); ++i) {
        if (server.is_listening()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (startup_error.load()) {
        std::string* err = startup_error.load();
        std::cerr << "\n[server] 启动失败: " << *err << "\n";
        std::cerr << "[server] 提示: 端口 " << port
                  << " 可能已被占用。可以用以下命令排查:\n"
                  << "         lsof -i :" << port
                  << "   或   ps aux | grep lidar_sim_server\n"
                  << "         kill <PID>   或   kill -9 <PID>\n";
        g_should_stop = true;
        server.stop();
        if (server_thread.joinable()) server_thread.join();
        return 1;
    }

    std::cerr << "[server] Listening on port " << port << ", waiting for client...\n";

    // ---- 主循环：等待客户端连接后推送数据 ----
    int current_step = 0;
    while (!g_should_stop) {
        if (!server.has_client()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // 客户端已连接，开始推送数据
        // 先发送状态帧
        {
            lidar_core::Json status_payload = lidar_core::Json::object_type{
                {"site_id", device.site_info().site_id},
                {"site_name", device.site_info().name},
                {"total_steps", device.total_steps()},
                {"minutes_per_step", device.minutes_per_step()},
            };
            auto status_frame = lidar_protocol::make_frame(
                lidar_protocol::FrameType::status,
                "",
                std::move(status_payload)
            );
            server.send_line(status_frame.to_json_line());
        }

        // 逐时间步推送
        for (int step = 0; step < device.total_steps() && !g_should_stop; ++step) {
            // 客户端断开则跳出
            if (!server.has_client()) {
                std::cerr << "[server] Client disconnected mid-stream.\n";
                break;
            }

            auto frames = device.produce_scan_cycle(step);
            int frame_delay = device.inter_frame_delay_ms();

            for (const auto& frame : frames) {
                if (!server.send_line(frame.to_json_line())) {
                    std::cerr << "[server] Send failed, client may have disconnected.\n";
                    break;
                }
                if (frame_delay > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(frame_delay));
                }
            }

            // 每个时间步后发送一个心跳帧
            if (server.has_client()) {
                auto heartbeat = lidar_protocol::make_frame(
                    lidar_protocol::FrameType::heartbeat,
                    "",
                    lidar_core::Json::object_type{{"step", step + 1}, {"total", device.total_steps()}}
                );
                server.send_line(heartbeat.to_json_line());
            }

            // 时间步间隔
            if (step_delay_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(step_delay_ms));
            }
        }

        // 一轮播完后，如果客户端还连着，继续循环
        if (server.has_client()) {
            std::cerr << "[server] Completed one full cycle, looping...\n";
        }
    }

    std::cerr << "[server] Shutting down...\n";
    server.stop();
    if (server_thread.joinable()) {
        server_thread.join();
    }

    return 0;
}
