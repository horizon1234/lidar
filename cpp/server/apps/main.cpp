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

        // ============================================================
        // 客户端已连接，开始推送数据。共发送 4 种帧：
        //   ① status      —— 站点元信息（每轮发一次，开头）
        //   ② lidar_raw   —— 每条射线的原始光子计数（核心数据，来自 produce_scan_cycle）
        //   ③ ground_obs  —— 地面观测 PM/气象量（每步 1 条，来自 produce_scan_cycle）
        //   ④ heartbeat   —— 时间步进度心跳（每步末尾发 1 条）
        // ============================================================

        // ① 状态帧：站点元信息，客户端据此知道这是哪个站、总共会收到多少步数据
        //    数据来源：SimDeviceConfig 里的 site_id / site_name / time_steps / minutes_per_step
        {
            lidar_core::Json status_payload = lidar_core::Json::object_type{
                {"site_id", device.site_info().site_id},         // 站点 ID
                {"site_name", device.site_info().name},          // 站点可读名称
                {"total_steps", device.total_steps()},          // 总时间步数（18）
                {"minutes_per_step", device.minutes_per_step()}, // 步间隔（20 分钟）
            };
            auto status_frame = lidar_protocol::make_frame(
                lidar_protocol::FrameType::status,
                "",
                std::move(status_payload)
            );
            server.send_line(status_frame.to_json_line());
        }

        // 逐时间步推送：每个时间步产出 74 条帧（73 lidar_raw + 1 ground_obs）
        // ②③ 的数据都由 produce_scan_cycle() 从缓存中取出并封装为帧
        for (int step = 0; step < device.total_steps() && !g_should_stop; ++step) {
            // 客户端断开则跳出
            if (!server.has_client()) {
                std::cerr << "[server] Client disconnected mid-stream.\n";
                break;
            }

            // produce_scan_cycle 返回该时间步的全部帧：
            //   ② lidar_raw：1 条 stare（天顶凝视）+ 72 条 PPI（6 仰角 × 12 方位）= 73 条
            //      每条帧包含 ranges_m[60]、raw_counts[60]、true_backscatter[60] 等数组
            //      数据来源：构造时 lidar_core::run_end_to_end() 正向仿真生成并缓存
            //   ③ ground_obs：1 条地面观测，含地面 PM2.5/PM10/T/RH/风速风向
            //      数据来源：仿真地面模型生成，存于 ground_measurements_ 缓存
            auto frames = device.produce_scan_cycle(step);
            int frame_delay = device.inter_frame_delay_ms();

            for (const auto& frame : frames) {
                if (!server.send_line(frame.to_json_line())) {
                    std::cerr << "[server] Send failed, client may have disconnected.\n";
                    break;
                }
                // 帧间延迟（默认 50ms），模拟真实雷达逐射线扫描节奏
                if (frame_delay > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(frame_delay));
                }
            }

            // ④ 心跳帧：报告当前播放进度，客户端据此显示进度条
            //    只有 step/total 两个字段，体积很小
            if (server.has_client()) {
                auto heartbeat = lidar_protocol::make_frame(
                    lidar_protocol::FrameType::heartbeat,
                    "",
                    lidar_core::Json::object_type{{"step", step + 1}, {"total", device.total_steps()}}
                );
                server.send_line(heartbeat.to_json_line());
            }

            // 时间步间隔（默认 500ms），区分不同时间步的节奏
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
