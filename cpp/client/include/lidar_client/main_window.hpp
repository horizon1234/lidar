/**
 * @file main_window.hpp
 * @brief 主窗口：集成 PPI 热力图、告警面板、处置状态栏。
 *
 * 布局：
 *   ┌─────────────────────────────────────────────────┐
 *   │  工具栏（连接/断开、时间步控制）                    │
 *   ├──────────────────────────┬──────────────────────┤
 *   │                          │  告警面板             │
 *   │  PPI 热力图               │  (事件列表+详情)      │
 *   │                          │                      │
 *   ├──────────────────────────┤                      │
 *   │  处置状态栏               │                      │
 *   │  (设备列表+任务状态)       │                      │
 *   └──────────────────────────┴──────────────────────┘
 */
#pragma once

#ifdef LIDAR_ENABLE_QT

#include <QLabel>
#include <QMainWindow>
#include <QTimer>
#include <memory>

#include "lidar_core/lidar_core.hpp"
#include "lidar_client/frame_processor.hpp"
#include "lidar_client/hotspot_tracker.hpp"
#include "lidar_client/disposal_linkage.hpp"
#include "lidar_client/report_generator.hpp"
#include "lidar_client/ppi_widget.hpp"
#include "lidar_client/alarm_panel.hpp"

class QToolBar;
class QAction;

namespace lidar_client {

class TcpClient;

/**
 * @brief 主窗口。
 */
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    /**
     * @brief 连接到仿真服务器。
     */
    void on_connect();

    /**
     * @brief 断开连接。
     */
    void on_disconnect();

    /**
     * @brief 主定时器回调（读取数据、刷新 UI）。
     */
    void on_tick();

    /**
     * @brief 生成报表。
     */
    void on_generate_report();

private:
    // UI 组件
    PpiWidget* ppi_widget_;
    AlarmPanel* alarm_panel_;
    QLabel* status_bar_;
    QToolBar* toolbar_;
    QAction* connect_action_;
    QAction* disconnect_action_;
    QAction* report_action_;

    // 数据处理
    std::unique_ptr<TcpClient> tcp_client_;
    std::unique_ptr<FrameProcessor> processor_;
    std::unique_ptr<HotspotTracker> tracker_;
    std::unique_ptr<DisposalLinkage> linkage_;
    std::unique_ptr<ReportGenerator> report_gen_;

    // 定时器（驱动 UI 刷新 + TCP 读取）
    QTimer* timer_;

    // 连接参数
    std::string host_ = "127.0.0.1";
    uint16_t port_ = 19850;
    int step_counter_ = 0;
    int total_expected_steps_ = -1;

    /**
     * @brief 初始化 UI 布局。
     */
    void setup_ui();

    /**
     * @brief 初始化数据处理组件。
     */
    void setup_pipeline();
};

} // namespace lidar_client

#endif // LIDAR_ENABLE_QT
