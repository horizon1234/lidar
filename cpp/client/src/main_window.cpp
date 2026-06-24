/**
 * @file main_window.cpp
 * @brief 主窗口实现。
 */
#ifdef LIDAR_ENABLE_QT

#include "lidar_client/main_window.hpp"

#include <QAction>
#include <QApplication>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QSplitter>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

#include "lidar_client/tcp_client.hpp"

namespace lidar_client {

// =========================================================================
// MainWindow
// =========================================================================

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setup_ui();
    setup_pipeline();
}

MainWindow::~MainWindow() = default;

void MainWindow::setup_ui() {
    setWindowTitle("激光雷达颗粒物监测系统");
    resize(1200, 800);

    // 工具栏
    toolbar_ = addToolBar("主工具栏");

    connect_action_ = new QAction("连接", this);
    connect_action_->setShortcut(Qt::Key_F5);
    toolbar_->addAction(connect_action_);

    disconnect_action_ = new QAction("断开", this);
    disconnect_action_->setShortcut(Qt::Key_Escape);
    toolbar_->addAction(disconnect_action_);

    toolbar_->addSeparator();

    report_action_ = new QAction("生成报表", this);
    toolbar_->addAction(report_action_);

    // 中心 Widget
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* main_layout = new QHBoxLayout(central);

    // 左侧：PPI 热力图
    ppi_widget_ = new PpiWidget(this);
    ppi_widget_->set_title("PPI 热力图 — PM2.5");

    // 右侧：告警面板
    alarm_panel_ = new AlarmPanel(this);

    // 分割
    auto* splitter = new QSplitter(Qt::Horizontal, central);
    splitter->addWidget(ppi_widget_);
    splitter->addWidget(alarm_panel_);
    splitter->setSizes({700, 500});
    main_layout->addWidget(splitter);

    // 状态栏
    status_bar_ = new QLabel("就绪。按 F5 连接到仿真服务器。", this);
    statusBar()->addWidget(status_bar_);

    // 定时器
    timer_ = new QTimer(this);
    timer_->setInterval(100); // 10 FPS

    // 信号连接
    connect(connect_action_, &QAction::triggered, this, &MainWindow::on_connect);
    connect(disconnect_action_, &QAction::triggered, this, &MainWindow::on_disconnect);
    connect(report_action_, &QAction::triggered, this, &MainWindow::on_generate_report);
    connect(timer_, &QTimer::timeout, this, &MainWindow::on_tick);
}

void MainWindow::setup_pipeline() {
    TrackerConfig tracker_cfg;
    tracker_cfg.match_distance_m = 300.0;
    tracker_cfg.alarm.confirm_consecutive_steps = 2;
    tracker_cfg.alarm.resolve_consecutive_steps = 3;
    tracker_cfg.alarm.severity_threshold_ugm3 = 50.0;

    tracker_ = std::make_unique<HotspotTracker>(tracker_cfg);

    LinkageConfig linkage_cfg;
    linkage_cfg.auto_trigger = true;
    linkage_cfg.trigger_threshold_ugm3 = 50.0;

    linkage_ = std::make_unique<DisposalLinkage>(linkage_cfg);
    linkage_->register_default_devices(4, 800.0);

    ReportConfig report_cfg;
    report_cfg.site_id = "GUI-001";
    report_cfg.site_name = "GUI Monitor Station";
    report_gen_ = std::make_unique<ReportGenerator>(report_cfg);

    processor_ = std::make_unique<FrameProcessor>();

    // 设置回调
    processor_->set_step_complete_callback([this](
        const std::vector<ProcessedProfile>& profiles,
        const std::vector<Hotspot>& hotspots,
        const std::string& timestamp) {

        // 更新 PPI 热力图
        QMetaObject::invokeMethod(this, [this, profiles, hotspots]() {
            ppi_widget_->set_data(profiles, "pm25");
            ppi_widget_->set_hotspots(hotspots);
        }, Qt::QueuedConnection);

        // 更新追踪器
        auto active_before = tracker_->active_events();
        auto summary = tracker_->process_step(timestamp, hotspots);
        auto active_after = tracker_->active_events();

        // 处理处置联动
        for (const auto& evt : active_after) {
            if (evt->current_state() == AlarmState::active) {
                linkage_->on_event_activated(*evt);
            }
        }
        for (const auto& evt : active_after) {
            if (evt->current_state() == AlarmState::resolved) {
                linkage_->on_event_resolved(evt->event_id());
            }
        }

        // 更新告警面板
        QMetaObject::invokeMethod(this, [this]() {
            alarm_panel_->update_data(*tracker_, *linkage_);
        }, Qt::QueuedConnection);

        // 收集步骤结果
        double max_pm25 = 0.0;
        double avg_pm25 = 0.0;
        int count = 0;
        for (const auto& p : profiles) {
            for (double v : p.pm25) {
                max_pm25 = std::max(max_pm25, v);
                avg_pm25 += v;
                ++count;
            }
        }
        if (count > 0) avg_pm25 /= count;

        report_gen_->add_step_result({
            timestamp,
            static_cast<int>(profiles.size()),
            static_cast<int>(profiles.size()),
            static_cast<int>(hotspots.size()),
            max_pm25,
            avg_pm25
        });

        step_counter_++;
        QString status = QString("步骤 %1 | 时间: %2 | 热点: %3 | 事件: %4 | 峰值 PM2.5: %5 µg/m³")
            .arg(step_counter_)
            .arg(QString::fromStdString(timestamp))
            .arg(hotspots.size())
            .arg(active_after.size())
            .arg(max_pm25, 0, 'f', 1);
        QMetaObject::invokeMethod(this, [this, status]() {
            status_bar_->setText(status);
        }, Qt::QueuedConnection);
    });
}

// =========================================================================
// 槽函数
// =========================================================================

void MainWindow::on_connect() {
    bool ok = false;
    QString host_q = QInputDialog::getText(this, "连接", "服务器地址:",
                                           QLineEdit::Normal, "127.0.0.1", &ok);
    if (!ok || host_q.isEmpty()) return;

    int port_q = QInputDialog::getInt(this, "连接", "端口:",
                                      19850, 1, 65535, 1, &ok);
    if (!ok) return;

    host_ = host_q.toStdString();
    port_ = static_cast<uint16_t>(port_q);

    tcp_client_ = std::make_unique<TcpClient>(host_, port_);
    if (!tcp_client_->connect()) {
        QMessageBox::warning(this, "连接失败",
                             "无法连接到 " + host_q + ":" + QString::number(port_q));
        tcp_client_.reset();
        return;
    }

    status_bar_->setText("已连接到 " + host_q + ":" + QString::number(port_q));
    timer_->start();
    step_counter_ = 0;
}

void MainWindow::on_disconnect() {
    if (timer_) timer_->stop();
    if (tcp_client_) {
        tcp_client_->disconnect();
        tcp_client_.reset();
    }
    status_bar_->setText("已断开。共处理 " + QString::number(step_counter_) + " 步。");
}

void MainWindow::on_tick() {
    if (!tcp_client_ || !tcp_client_->is_connected()) {
        return;
    }

    // 尝试读取一帧
    auto frame_opt = tcp_client_->read_frame();
    if (frame_opt.has_value()) {
        processor_->handle_frame(frame_opt.value());
    }
}

void MainWindow::on_generate_report() {
    if (report_gen_->step_count() == 0) {
        QMessageBox::information(this, "报表", "尚无数据，请先采集数据。");
        return;
    }

    QString file = QFileDialog::getSaveFileName(
        this, "保存报表", "report.txt", "文本文件 (*.txt);;JSON (*.json)");
    if (file.isEmpty()) return;

    if (file.endsWith(".json")) {
        auto json = report_gen_->generate_json_report(*tracker_, *linkage_);
        QFile f(file);
        if (f.open(QIODevice::WriteOnly)) {
            f.write(json.dump(2).c_str());
        }
    } else {
        auto text = report_gen_->generate_text_report(*tracker_, *linkage_);
        QFile f(file);
        if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
            f.write(text.c_str());
        }
    }

    QMessageBox::information(this, "报表", "报表已保存到:\n" + file);
}

} // namespace lidar_client

#endif // LIDAR_ENABLE_QT
