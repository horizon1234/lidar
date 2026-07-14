/**
 * @file LidarSimServer.cpp
 * @brief YLJ5 公开规格仿真设备的 TCP JSONL 服务入口。
 *
 * 本进程模拟一台“中科光电 YLJ5 / AGHJ-I-LIDAR(MPL)”微脉冲颗粒物激光雷达，
 * 以 TCP + JSONL（每行一个 JSON 帧）的形式向客户端持续推送数据。
 *
 * @section 数据流总览
 * 客户端连接后，一个“扫描周期（cycle/step）”的发送顺序如下：
 *
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │  ① status_frame       —— 仅在新连接的首个周期发送一次（设备能力/规格）  │
 *   │  ② telemetry_frame    —— 每个周期开头发送（设备健康/天气/链路遥测）      │
 *   │  ③ lidar_raw × N      —— 本周期所有射线（stare + PPI 各方位）           │
 *   │     可选: camera / lidar_product / ground_obs                       │
 *   │  ④ heartbeat          —— 周期结束标记（step/total/device_state）       │
 *   └──────────────────────────────────────────────────────────────────────┘
 *
 * 之后 current_step++ 进入下一周期，循环直到客户端断开或收到 stop 命令。
 *
 * @section 帧间隔时序
 * 各 lidar_raw 帧之间的发送间隔由真实采集耗时 ÷ 加速倍率决定（见
 * playback_delay_ms_for_frame）：
 *   - stare 帧：等待 vertical_stare_dwell_s / playback_time_scale（默认 30s ÷ scale）
 *   - PPI 帧：  等待 (line_dwell_s + movement_seconds_per_ray) / playback_time_scale
 *   - 周期首个非 raw 帧：额外等待 scan_overhead_s（体扫回扫/上报余量）
 * 若 inter_frame_delay_ms > 0，则固定使用该值覆盖上述自适应间隔。
 *
 * @section 命令通道
 * 主循环只负责“推数据”；命令处理在 server.start 的接收回调里异步进行。
 * 客户端可发送 command 帧（如 start/stop/pause/resume/set_param），
 * 设备返回 command_result 帧；start 命令将设备状态置为 scanning 后，
 * 主循环才会开始推送 lidar_raw 帧。
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "lidar_protocol/Frame.hpp"
#include "lidar_server/DevicePlaybackTiming.hpp"
#include "lidar_server/SimDevice.hpp"
#include "lidar_server/TcpServer.hpp"

namespace {

/// 全局停止标志，由信号处理器置位，通知主循环退出。
std::atomic<bool> should_stop{false};

/**
 * @brief SIGINT/SIGTERM 处理器：置位停止标志，让主循环优雅退出。
 * @note 只做原子赋值，不做任何阻塞操作（信号处理函数里不安全）。
 */
void signal_handler(int) {
    should_stop = true;
}

} // namespace

/**
 * @brief 主入口：启动 YLJ5 仿真设备 TCP 服务并循环推送扫描周期。
 *
 * @param argv 用法: LidarSimServer [port] [cycle_delay_ms] [playback_time_scale]
 *             - port                 监听端口，默认 19850
 *             - cycle_delay_ms       相邻周期之间的额外间隔(ms)，默认 0
 *             - playback_time_scale  播放加速倍率，默认 1.0（实时）；>1 加速，<1 慢放
 * @return 0 正常退出；1 启动失败（端口占用/配置校验失败）。
 */
int main(int argc, char* argv[]) {
    // ---- 解析命令行参数 ----
    std::uint16_t port = 19850;          // 默认监听端口（YLJ5 约定端口）
    int cycle_delay_ms = 0;              // 周期间额外延迟，默认 0 表示无额外等待
    double playback_time_scale = 1.0;    // 1.0 = 按真实采集耗时推送；>1 加速回放
    if (argc >= 2) {
        port = static_cast<std::uint16_t>(std::stoi(argv[1]));
    }
    if (argc >= 3) {
        cycle_delay_ms = std::stoi(argv[2]);
    }
    if (argc >= 4) {
        playback_time_scale = std::stod(argv[3]);
    }

    // ---- 注册信号处理，支持 Ctrl+C 优雅退出 ----
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    lidar_server::SimDeviceConfig device_config;
    device_config.stream.playback_time_scale = playback_time_scale;
    // 构造时只执行公开规格校验；若配置非法 total_steps 为 0。
    lidar_server::SimDevice device(device_config);
    if (device.total_steps() == 0) {
        std::cerr << "[server] YLJ5 emulator configuration failed validation.\n";
        return 1;
    }

    // ---- 启动 TCP 服务器，端口由命令行指定 ----
    lidar_server::TcpServer server(port);
    // ---- 命令处理回调：客户端发来的每一行 JSON 在这里被解析和响应 ----
    // 主循环只管“推数据”，命令处理在这个接收回调里异步进行。
    auto handler = [&server, &device](const std::string& line) {
        try {
            auto frame = lidar_protocol::parse_frame(line);
            // 只处理 command 帧（如 start/stop/pause/resume/set_param）
            if (frame.type == lidar_protocol::FrameType::command) {
                // 执行命令并立即回送 command_result 帧（同步）
                server.send_line(device.handle_command(frame.payload).to_json_line());
            }
        } catch (const std::exception& error) {
            // 帧解析失败：回送 error 结果，保持连接不中断
            auto result = lidar_protocol::make_frame(
                lidar_protocol::FrameType::command_result,
                "",
                lidar_core::Json::Object{
                    {"accepted", false},
                    {"result", "error"},
                    {"message", std::string("invalid frame: ") + error.what()},
                });
            server.send_line(result.to_json_line());
        }
    };

    // ---- 在独立线程启动 TCP 监听，避免阻塞主推流循环 ----
    std::atomic<bool> startup_failed{false};
    std::string startup_error;
    std::thread server_thread([&]() {
        try {
            server.start(handler);
        } catch (const std::exception& error) {
            startup_error = error.what();
            startup_failed = true;
        }
    });

    // 等待监听就绪，最多重试 20 次 × 100ms = 2s
    for (int retry = 0; retry < 20 && !startup_failed && !server.is_listening(); ++retry) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (startup_failed || !server.is_listening()) {
        std::cerr << "[server] Startup failed on port " << port;
        if (!startup_error.empty()) {
            std::cerr << ": " << startup_error;
        }
        std::cerr << "\n";
        should_stop = true;
    } else {
        std::cerr << "[server] YLJ5 / AGHJ-I-LIDAR(MPL) emulator listening on port "
                  << port << ", site=" << device.site_info().site_id << "\n";
    }

    // ==================== 主推流循环 ====================
    // 状态机：无客户端 → 等待；有客户端但未发过 status → 发 status；
    //         有客户端且设备 scanning → 按周期推送 telemetry + raw×N + heartbeat。
    int current_step = 0;
    bool status_sent_for_connection = false;  // 标记当前连接是否已发过 status 帧
    while (!should_stop) {
        // ---- 无客户端时：重置 status 标记，空转等待 ----
        if (!server.has_client()) {
            status_sent_for_connection = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        // ---- 新连接：仅发一次 status 帧（设备能力/规格/校准状态） ----
        // 之后断开重连才会再次发送。
        if (!status_sent_for_connection) {
            server.send_line(device.status_frame(current_step).to_json_line());
            status_sent_for_connection = true;
        }
        // ---- 设备未进入 scanning 状态（如刚就绪等待 start 命令）：空转 ----
        if (!device.is_streaming()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        const int total_steps = device.total_steps();
        if (total_steps <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }
        // 当前周期索引，超过总数后取模循环（模拟 24/7 连续运行）
        const int step = current_step % total_steps;

        // ---- ② 每周期开头：发送 telemetry 帧（设备健康/天气/链路状态） ----
        server.send_line(device.telemetry_frame(step).to_json_line());

        bool scan_overhead_waited = false;  // 标记本周期是否已等待过体扫回扫余量
        // ---- ③ 逐帧推送本周期所有 lidar_raw 射线（及可选 camera/product/ground） ----
        // 用 stream_scan_cycle 的回调流式发送，避免把整周期（~180 条大 JSON）同时驻留内存。
        // 逐帧序列化、等待并发送，避免把 180 条大 JSON 帧同时保存在内存中。
        const bool cycle_completed = device.stream_scan_cycle(
            step,
            [&](lidar_protocol::Frame&& frame) {
                // —— 暂停响应：若设备被 pause，在此空转等待 resume，不中断周期 ——
                while (server.has_client()
                       && !device.is_streaming()
                       && device.run_state() == lidar_server::DeviceRunState::paused
                       && !should_stop) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                // —— 中止条件：客户端断开 / 设备停止 / 进程退出 ——
                if (!server.has_client() || !device.is_streaming() || should_stop) {
                    return false;
                }

                const auto config = device.config();
                // —— 周期首个非 raw 帧前，等待体扫回扫/上报余量 scan_overhead_s ——
                // （加速倍率下等比例缩短：overhead_ms = scan_overhead_s × 1000 / scale）
                if (frame.type != lidar_protocol::FrameType::lidar_raw
                    && !scan_overhead_waited
                    && config.scan.scan_overhead_s > 0.0
                    && config.stream.playback_time_scale > 0.0) {
                    const int overhead_ms = std::max(1, static_cast<int>(std::llround(
                        config.scan.scan_overhead_s * 1000.0
                        / config.stream.playback_time_scale)));
                    std::this_thread::sleep_for(std::chrono::milliseconds(overhead_ms));
                    scan_overhead_waited = true;
                }
                // —— 帧间间隔：按真实采集耗时 ÷ 加速倍率计算 ——
                //   stare 帧: vertical_stare_dwell_s / scale
                //   PPI 帧:   (line_dwell_s + movement_seconds_per_ray) / scale
                //   若 inter_frame_delay_ms > 0，则固定使用该值覆盖
                const int delay_ms = lidar_server::playback_delay_ms_for_frame(config, frame);
                if (delay_ms > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
                }
                // —— 发送本帧；发送失败（客户端断开）则中止本周期 ——
                if (!server.send_line(frame.to_json_line())) {
                    return false;
                }
                return true;
            });

        // ---- ④ 周期完整结束后：发送 heartbeat 帧（周期进度标记） ----
        // 仅在周期未被打断（客户端未断、设备仍 scanning）时发送。
        if (cycle_completed && server.has_client() && device.is_streaming()) {
            server.send_line(lidar_protocol::make_frame(
                lidar_protocol::FrameType::heartbeat,
                "",
                lidar_core::Json::Object{
                    {"step", step + 1},
                    {"total", total_steps},
                    {"device_state", "scanning"},
                }).to_json_line());
            // 推进到下一周期（取模循环，模拟全天连续运行）
            current_step = (step + 1) % total_steps;
        }
        // ---- 周期间额外延迟（命令行指定 cycle_delay_ms，默认 0） ----
        if (cycle_delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(cycle_delay_ms));
        }
    }

    // ==================== 优雅退出 ====================
    server.stop();
    if (server_thread.joinable()) {
        server_thread.join();
    }
    return startup_failed ? 1 : 0;
}
