/**
 * @file test_closed_loop.cpp
 * @brief 闭环管理组件的单元测试：告警状态机、热点追踪器、处置联动、报表生成器。
 *
 * 测试场景：
 * 1. 状态机状态转换：candidate → confirmed → active → mitigating → resolved
 * 2. 热点追踪器跨步骤关联：同一区域的热点被关联到同一事件
 * 3. 处置联动：active 事件触发设备任务，resolved 事件停止任务
 * 4. 报表生成：JSON 和文本报表包含正确的统计数据
 */

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "lidar_core/lidar_core.hpp"
#include "lidar_client/alarm_state_machine.hpp"
#include "lidar_client/hotspot_tracker.hpp"
#include "lidar_client/disposal_linkage.hpp"
#include "lidar_client/report_generator.hpp"

using namespace lidar_client;
using lidar_core::Hotspot;

// ---------------------------------------------------------------------------
// 辅助函数
// ---------------------------------------------------------------------------

static Hotspot make_hotspot(const std::string& id,
                            const std::string& ts,
                            double peak_pm25,
                            double cx, double cy, double cz) {
    Hotspot h;
    h.hotspot_id = id;
    h.timestamp = ts;
    h.peak_pm25_ugm3 = peak_pm25;
    h.severity = (peak_pm25 >= 75.0) ? "high" : (peak_pm25 >= 50.0 ? "medium" : "low");
    h.cell_count = 5;
    h.estimated_area_m2 = 10000.0;
    h.centroid_enu_m = {cx, cy, cz};
    return h;
}

static int test_count = 0;
static int test_passed = 0;

#define CHECK(cond, msg) do { \
    ++test_count; \
    if (!(cond)) { \
        std::cerr << "FAIL [" << #cond << "]: " << msg << "\n"; \
        return false; \
    } \
    ++test_passed; \
} while(0)

// ---------------------------------------------------------------------------
// 测试用例
// ---------------------------------------------------------------------------

/// 测试1: 状态机基本转换
bool test_state_machine_transitions() {
    std::cerr << "  test_state_machine_transitions...\n";

    AlarmConfig config;
    config.confirm_consecutive_steps = 2;
    config.resolve_consecutive_steps = 3;
    config.severity_threshold_ugm3 = 50.0;

    // 初始状态：candidate
    auto hs = make_hotspot("hs-001", "2026-01-01T10:00", 35.0, 100, 200, 50);
    HotspotEvent evt("EVT-001", hs);

    CHECK(evt.state() == AlarmState::candidate, "initial state should be candidate");
    CHECK(evt.peak_pm25_ugm3() == 35.0, "peak pm25 should be 35.0");
    CHECK(evt.consecutive_detections() == 1, "should have 1 consecutive detection");

    // 第2次检测：应该进入 confirmed（连续2次）
    auto hs2 = make_hotspot("hs-001", "2026-01-01T10:05", 36.0, 101, 201, 51);
    bool changed = evt.update_detected(hs2, config);
    CHECK(changed, "should have changed state");
    CHECK(evt.state() == AlarmState::confirmed, "should be confirmed after 2 detections");
    CHECK(evt.peak_pm25_ugm3() == 36.0, "peak should update to 36.0");

    // 第3次检测：PM2.5 超过阈值，进入 active
    auto hs3 = make_hotspot("hs-001", "2026-01-01T10:10", 55.0, 102, 202, 52);
    changed = evt.update_detected(hs3, config);
    CHECK(changed, "should change to active");
    CHECK(evt.state() == AlarmState::active, "should be active when threshold exceeded");
    CHECK(evt.peak_pm25_ugm3() == 55.0, "peak should be 55.0");

    // 手动标记 mitigating
    evt.mark_mitigating();
    CHECK(evt.state() == AlarmState::mitigating, "should be mitigating");

    // 连续3次未检测：resolved
    evt.update_missed("2026-01-01T10:15", config);
    evt.update_missed("2026-01-01T10:20", config);
    evt.update_missed("2026-01-01T10:25", config);
    CHECK(evt.state() == AlarmState::resolved, "should be resolved after 3 misses");
    CHECK(evt.is_terminal(), "resolved is terminal");

    // 检查历史记录
    CHECK(evt.history().size() >= 4, "should have at least 4 state changes");

    return true;
}

/// 测试2: 热点追踪器空间关联
bool test_tracker_spatial_matching() {
    std::cerr << "  test_tracker_spatial_matching...\n";

    TrackerConfig config;
    config.match_distance_m = 300.0;
    config.enable_tracking = true;

    HotspotTracker tracker(config);

    // 步骤1：检测到2个热点（一个近，一个远）
    std::vector<Hotspot> step1 = {
        make_hotspot("hs-A", "2026-01-01T10:00", 30.0, 100, 200, 50),
        make_hotspot("hs-B", "2026-01-01T10:00", 40.0, 800, 900, 100),
    };
    auto s1 = tracker.process_step("2026-01-01T10:00", step1);
    CHECK(s1.new_events == 2, "step1 should create 2 new events");
    CHECK(s1.active_events == 2, "step1 should have 2 active events");

    // 步骤2：同一区域的热点（位置略有偏移），应该被关联
    std::vector<Hotspot> step2 = {
        make_hotspot("hs-A", "2026-01-01T10:05", 32.0, 110, 210, 55),
        make_hotspot("hs-B", "2026-01-01T10:05", 38.0, 810, 910, 105),
    };
    auto s2 = tracker.process_step("2026-01-01T10:05", step2);
    CHECK(s2.new_events == 0, "step2 should create 0 new events (matched)");
    CHECK(s2.active_events == 2, "step2 should still have 2 active events");

    // 步骤3：仅检测到1个热点，另一个missed
    std::vector<Hotspot> step3 = {
        make_hotspot("hs-A", "2026-01-01T10:10", 35.0, 120, 220, 60),
    };
    auto s3 = tracker.process_step("2026-01-01T10:10", step3);
    CHECK(s3.new_events == 0, "step3 should create 0 new events");
    CHECK(tracker.events().size() == 2, "total events should still be 2");

    // 步骤4-6：都没有检测到，两个事件都应该被 resolved
    tracker.process_step("2026-01-01T10:15", {});
    tracker.process_step("2026-01-01T10:20", {});
    auto s6 = tracker.process_step("2026-01-01T10:25", {});
    CHECK(s6.resolved_events > 0, "should have resolved events after misses");
    CHECK(tracker.active_events().empty(), "no active events after resolution");

    return true;
}

/// 测试3: 处置联动
bool test_disposal_linkage() {
    std::cerr << "  test_disposal_linkage...\n";

    LinkageConfig config;
    config.auto_trigger = true;
    config.trigger_threshold_ugm3 = 50.0;
    config.stop_on_resolve = true;

    DisposalLinkage linkage(config);
    linkage.register_default_devices({0.0, 0.0, 0.0}, 600.0, 4);

    CHECK(linkage.devices().size() == 4, "should have 4 devices registered");
    CHECK(linkage.tasks().empty(), "no tasks initially");

    // 触发处置（PM2.5 超过阈值）
    int triggered = linkage.on_event_activated(
        "EVT-001", "2026-01-01T10:00", {100, 100, 50}, 60.0);
    CHECK(triggered == 1, "should trigger 1 disposal task");
    CHECK(linkage.tasks().size() == 1, "should have 1 task");
    CHECK(linkage.active_tasks().size() == 1, "should have 1 active task");

    // 设备状态检查
    int active_devs = 0;
    for (const auto& dev : linkage.devices()) {
        if (dev.state == DeviceState::active) ++active_devs;
    }
    CHECK(active_devs == 1, "1 device should be active");

    // 低于阈值不触发
    int triggered2 = linkage.on_event_activated(
        "EVT-002", "2026-01-01T10:00", {200, 200, 50}, 30.0);
    CHECK(triggered2 == 0, "should not trigger below threshold");

    // 事件解决 → 停止任务
    int stopped = linkage.on_event_resolved("EVT-001", "2026-01-01T10:30", 15.0);
    CHECK(stopped == 1, "should stop 1 task");
    CHECK(linkage.active_tasks().empty(), "no active tasks after resolve");

    // 设备恢复 idle
    active_devs = 0;
    for (const auto& dev : linkage.devices()) {
        if (dev.state == DeviceState::active) ++active_devs;
    }
    CHECK(active_devs == 0, "0 devices active after stop");

    // 检查 JSON 序列化
    auto json = linkage.to_json();
    CHECK(json.contains("devices"), "json should have devices");
    CHECK(json.contains("tasks"), "json should have tasks");
    CHECK(json.contains("statistics"), "json should have statistics");

    return true;
}

/// 测试4: 报表生成
bool test_report_generation() {
    std::cerr << "  test_report_generation...\n";

    // 先走一遍 tracker + linkage
    TrackerConfig tconfig;
    tconfig.match_distance_m = 300.0;
    HotspotTracker tracker(tconfig);

    LinkageConfig lconfig;
    lconfig.trigger_threshold_ugm3 = 50.0;
    DisposalLinkage linkage(lconfig);
    linkage.register_default_devices({0, 0, 0}, 600.0, 4);

    ReportConfig rconfig;
    rconfig.site_id = "test_site";
    rconfig.site_name = "Test Site";
    ReportGenerator report_gen(rconfig);

    // 模拟3个时间步
    for (int i = 0; i < 3; ++i) {
        std::string ts = "2026-01-01T10:" + std::to_string(i * 5).substr(0, 2);
        if (i == 0) ts = "2026-01-01T10:00";
        else if (i == 1) ts = "2026-01-01T10:05";
        else ts = "2026-01-01T10:10";

        std::vector<Hotspot> hots = {
            make_hotspot("hs-1", ts, 55.0 + i * 5, 100.0 + i, 200.0, 50.0),
        };

        auto summary = tracker.process_step(ts, hots);

        // 处置联动
        for (const auto& evt : tracker.active_events()) {
            if (evt->state() == AlarmState::active) {
                linkage.on_event_activated(evt->event_id(), ts,
                                           evt->centroid_enu_m(),
                                           evt->current_pm25_ugm3());
            }
        }

        StepResultSummary sr;
        sr.timestamp = ts;
        sr.raw_count = 13;
        sr.processed_count = 12;
        sr.hotspot_count = static_cast<int>(hots.size());
        sr.max_pm25_ugm3 = summary.max_pm25_ugm3;
        report_gen.add_step_result(sr);
    }

    // JSON 报表
    auto json_report = report_gen.generate_json_report(tracker, linkage);
    CHECK(json_report.contains("header"), "report should have header");
    CHECK(json_report.contains("summary"), "report should have summary");
    CHECK(json_report.contains("steps"), "report should have steps");
    CHECK(json_report.contains("events"), "report should have events");
    CHECK(json_report.contains("disposal"), "report should have disposal");

    // 验证 header
    CHECK(json_report.at("header").at("site_id").string_value() == "test_site",
          "site_id should be test_site");

    // 验证步骤数
    CHECK(json_report.at("steps").array_items().size() == 3,
          "should have 3 steps in report");

    // 文本报表
    std::string text = report_gen.generate_text_report(tracker, linkage);
    CHECK(text.find("LiDAR PM Monitoring") != std::string::npos,
          "text report should have title");
    CHECK(text.find("Test Site") != std::string::npos,
          "text report should have site name");
    CHECK(text.find("PER-STEP TIMELINE") != std::string::npos,
          "text report should have timeline section");

    return true;
}

/// 测试5: HotspotEvent JSON 序列化
bool test_event_json_serialization() {
    std::cerr << "  test_event_json_serialization...\n";

    auto hs = make_hotspot("hs-001", "2026-01-01T10:00", 60.0, 100, 200, 50);
    HotspotEvent evt("EVT-001", hs);

    auto json = evt.to_json();
    CHECK(json.at("event_id").string_value() == "EVT-001", "event_id mismatch");
    CHECK(json.at("state").string_value() == "candidate", "state mismatch");
    CHECK(json.at("peak_pm25_ugm3").number_value() == 60.0, "peak_pm25 mismatch");
    CHECK(json.at("history").array_items().size() >= 1, "should have history");

    return true;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::cerr << "=== Closed-Loop Component Tests ===\n\n";

    std::vector<bool (*)()> tests = {
        test_state_machine_transitions,
        test_tracker_spatial_matching,
        test_disposal_linkage,
        test_report_generation,
        test_event_json_serialization,
    };

    bool all_passed = true;
    for (auto& test : tests) {
        if (!test()) {
            all_passed = false;
        }
    }

    std::cerr << "\n=== Results: " << test_passed << "/" << test_count
              << " assertions passed ===\n";

    if (all_passed) {
        std::cerr << "ALL TESTS PASSED\n";
        return 0;
    } else {
        std::cerr << "SOME TESTS FAILED\n";
        return 1;
    }
}
