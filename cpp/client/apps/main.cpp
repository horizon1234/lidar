/**
 * @file main.cpp
 * @brief 主控客户端（无头模式）：连接仿真服务器，实时处理帧，输出结果。
 *
 * 用法：
 *   lidar_control_client [host] [port]
 *
 * 默认连接 127.0.0.1:19850。
 *
 * 工作流程：
 *   1. 连接到仿真服务器
 *   2. 逐行读取帧，交给 FrameProcessor 处理
 *   3. 每个时间步完成后输出摘要 + 热点信息
 *   4. 可选：把处理结果写到 JSON 文件
 */
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "lidar_client/tcp_client.hpp"
#include "lidar_client/frame_processor.hpp"
#include "lidar_client/hotspot_tracker.hpp"
#include "lidar_client/disposal_linkage.hpp"
#include "lidar_client/report_generator.hpp"
#include "lidar_protocol/frame.hpp"

static std::atomic<bool> g_should_stop{false};

void signal_handler(int /*sig*/) {
    g_should_stop = true;
}

int main(int argc, char* argv[]) {
    std::string host = "127.0.0.1";
    std::uint16_t port = 19850;
    std::string output_dir = "data/client_output";

    if (argc >= 2) host = argv[1];
    if (argc >= 3) port = static_cast<std::uint16_t>(std::stoi(argv[2]));
    if (argc >= 4) output_dir = argv[3];

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 创建输出目录
    std::filesystem::create_directories(output_dir);

    // 初始化帧处理器
    lidar_client::ProcessorConfig proc_config;
    proc_config.retrieval.aerosol_lidar_ratio_sr = 45.0;
    proc_config.retrieval.reference_aerosol_backscatter = 0.0004;
    proc_config.humidity.dry_reference_rh = 0.45;
    proc_config.humidity.hygroscopicity = 1.1;
    proc_config.hotspot.pm25_threshold_ugm3 = 50.0;
    proc_config.hotspot.min_cells = 3;

    int step_counter = 0;
    int total_expected_steps = -1; // 从 status 帧获取

    // ---- 闭环管理组件 ----
    lidar_client::TrackerConfig tracker_config;
    tracker_config.alarm.confirm_consecutive_steps = 2;
    tracker_config.alarm.resolve_consecutive_steps = 3;
    tracker_config.alarm.severity_threshold_ugm3 = 50.0;
    tracker_config.match_distance_m = 300.0;

    lidar_client::HotspotTracker tracker(tracker_config);

    lidar_client::LinkageConfig linkage_config;
    linkage_config.auto_trigger = true;
    linkage_config.trigger_threshold_ugm3 = 50.0;
    linkage_config.stop_on_resolve = true;

    lidar_client::DisposalLinkage linkage(linkage_config);
    // 注册默认设备（以站点中心为原点的环形布局）
    linkage.register_default_devices({0.0, 0.0, 0.0}, 800.0, 4);

    lidar_client::ReportConfig report_config;
    report_config.site_id = "demo_site";
    report_config.site_name = "LiDAR Demo Monitoring Site";
    lidar_client::ReportGenerator report_gen(report_config);

    lidar_client::FrameProcessor processor(proc_config);

    processor.set_step_complete_callback([&](const lidar_client::StepResult& result) {
        std::cerr << "\n========================================\n";
        std::cerr << "Step " << step_counter << " | " << result.timestamp << "\n";
        std::cerr << "  Rays: " << result.raw_count << "\n";
        std::cerr << "  Ground: " << result.ground_measurements.size() << "\n";

        if (!result.ground_measurements.empty()) {
            const auto& gm = result.ground_measurements.front();
            std::cerr << "  PM2.5: " << gm.pm25_ugm3 << " µg/m³, PM10: " << gm.pm10_ugm3 << " µg/m³\n";
            std::cerr << "  RH: " << gm.relative_humidity * 100 << "%, Temp: " << gm.temperature_c << "°C\n";
        }

        std::cerr << "  Hotspots: " << result.hotspots.size() << "\n";
        for (const auto& hs : result.hotspots) {
            std::cerr << "    [" << hs.severity << "] "
                      << hs.hotspot_id
                      << " peak=" << hs.peak_pm25_ugm3 << " µg/m³"
                      << " cells=" << hs.cell_count
                      << " area=" << hs.estimated_area_m2 << " m²\n";
        }

        // ---- 闭环处理 ----
        auto tracker_summary = tracker.process_step(result.timestamp, result.hotspots);
        std::cerr << "  [tracker] active_events=" << tracker_summary.active_events
                  << " new=" << tracker_summary.new_events
                  << " resolved=" << tracker_summary.resolved_events
                  << " changed=" << tracker_summary.state_changed_events << "\n";

        // 检查活跃事件 → 触发处置联动
        for (const auto& evt : tracker.active_events()) {
            if (evt->state() == lidar_client::AlarmState::active) {
                int triggered = linkage.on_event_activated(
                    evt->event_id(), result.timestamp,
                    evt->centroid_enu_m(), evt->current_pm25_ugm3());
                if (triggered > 0) {
                    tracker.mark_event_mitigating(evt->event_id());
                    std::cerr << "  [linkage] Triggered disposal for event "
                              << evt->event_id() << "\n";
                }
            }
        }

        // 检查关闭的事件 → 停止处置
        auto active_tasks = linkage.active_tasks();
        for (const auto& task : active_tasks) {
            auto evt = tracker.find_event(task.event_id);
            if (evt && evt->is_terminal()) {
                linkage.on_event_resolved(task.event_id, result.timestamp,
                                          evt->current_pm25_ugm3());
                std::cerr << "  [linkage] Stopped disposal for resolved event "
                          << task.event_id << "\n";
            }
        }

        // 显示当前活跃任务
        if (!linkage.active_tasks().empty()) {
            std::cerr << "  [linkage] Active tasks: " << linkage.active_tasks().size() << "\n";
        }

        // 收集步骤摘要
        lidar_client::StepResultSummary step_summary;
        step_summary.timestamp = result.timestamp;
        step_summary.raw_count = result.raw_count;
        step_summary.processed_count = static_cast<int>(result.processed_profiles.size());
        step_summary.hotspot_count = static_cast<int>(result.hotspots.size());
        step_summary.max_pm25_ugm3 = tracker_summary.max_pm25_ugm3;
        report_gen.add_step_result(step_summary);

        std::cerr << "========================================\n\n";

        // 写出该步的 JSON 结果
        lidar_core::Json step_json = lidar_core::Json::object_type{
            {"timestamp", result.timestamp},
            {"raw_count", result.raw_count},
        };

        // 处理后的射线（序列化为 L2 JSON）
        lidar_core::Json::array_type processed_arr;
        for (const auto& p : result.processed_profiles) {
            processed_arr.push_back(lidar_core::to_json_processed(p));
        }
        step_json["processed_profiles"] = lidar_core::Json(std::move(processed_arr));

        // 地面观测
        lidar_core::Json::array_type ground_arr;
        for (const auto& gm : result.ground_measurements) {
            ground_arr.push_back(lidar_protocol::ground_to_json(gm));
        }
        step_json["ground_measurements"] = lidar_core::Json(std::move(ground_arr));

        // 热点
        lidar_core::Json::array_type hotspot_arr;
        for (const auto& hs : result.hotspots) {
            hotspot_arr.push_back(lidar_core::to_json_hotspot(hs));
        }
        step_json["hotspots"] = lidar_core::Json(std::move(hotspot_arr));

        // 写盘
        std::string safe_ts = result.timestamp;
        std::replace(safe_ts.begin(), safe_ts.end(), ':', '-');
        std::filesystem::path step_file = std::filesystem::path(output_dir) / ("step_" + safe_ts + ".json");
        lidar_core::write_json_file(step_file, step_json);

        ++step_counter;

        // 如果所有步骤都处理完毕，自动断开连接
        if (total_expected_steps > 0 && step_counter >= total_expected_steps) {
            std::cerr << "[client] All " << total_expected_steps
                      << " steps processed, disconnecting...\n";
            g_should_stop = true;
        }
    });

    // 连接服务器
    lidar_client::TcpClient client;
    std::cerr << "[client] Connecting to " << host << ":" << port << "...\n";

    int retry_count = 0;
    while (!g_should_stop) {
        if (client.connect(host, port)) {
            break;
        }
        ++retry_count;
        if (retry_count > 10) {
            std::cerr << "[client] Failed to connect after 10 retries. Exiting.\n";
            return 1;
        }
        std::cerr << "[client] Connection failed, retrying (" << retry_count << "/10)...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (g_should_stop) {
        return 0;
    }

    std::cerr << "[client] Connected!\n";

    // 主循环：逐行读取并处理
    std::string line;
    while (!g_should_stop && client.is_connected()) {
        if (!client.read_line(line)) {
            std::cerr << "[client] Connection lost.\n";
            break;
        }

        if (line.empty()) {
            continue;
        }

        try {
            auto frame = lidar_protocol::parse_frame(line);

            // 拦截 status 帧，提取 total_steps
            if (frame.type == lidar_protocol::FrameType::status &&
                frame.payload.contains("total_steps")) {
                total_expected_steps = static_cast<int>(
                    frame.payload.at("total_steps").number_value());
                std::cerr << "[client] Expected total steps: "
                          << total_expected_steps << "\n";
            }

            processor.handle_frame(frame);
        } catch (const std::exception& e) {
            std::cerr << "[client] Frame parse error: " << e.what() << "\n";
        }
    }

    // 最终处理
    processor.finalize_step();

    std::cerr << "[client] Disconnected. Total steps processed: "
              << processor.total_steps_completed()
              << ", total rays: " << processor.total_processed() << "\n";

    // ---- 生成最终报表 ----
    if (processor.total_steps_completed() > 0) {
        std::cerr << "\n[closed_loop] Generating final reports...\n";

        // JSON 报表
        auto json_report = report_gen.generate_json_report(tracker, linkage);
        auto json_path = std::filesystem::path(output_dir) / "final_report.json";
        lidar_core::write_json_file(json_path, json_report);
        std::cerr << "[closed_loop] JSON report written to " << json_path << "\n";

        // 文本报表
        std::string text_report = report_gen.generate_text_report(tracker, linkage);
        auto text_path = std::filesystem::path(output_dir) / "final_report.txt";
        std::ofstream ofs(text_path);
        ofs << text_report;
        ofs.close();
        std::cerr << "[closed_loop] Text report written to " << text_path << "\n";

        // 在终端打印摘要
        std::cerr << "\n" << text_report << "\n";
    }

    client.disconnect();
    return 0;
}
