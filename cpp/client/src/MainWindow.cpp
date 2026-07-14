/**
 * @file MainWindow.cpp
 * @brief Linux Qt 主窗口实现。
 */
#include "lidar_client/MainWindow.hpp"

#include <QAction>
#include <QFileDialog>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QTabWidget>
#include <QToolBar>
#include <QVBoxLayout>

#include "lidar_client/AlarmPanel.hpp"
#include "lidar_client/PpiWidget.hpp"
#include "lidar_client/VerticalProfileWidget.hpp"

namespace lidar_client {

namespace {

/** @brief 创建状态带中的标题和值双行单元。 */
QWidget* make_status_item(const QString& caption, QLabel*& value_label, QWidget* parent) {
    auto* container = new QWidget(parent);
    auto* layout = new QVBoxLayout(container);
    layout->setContentsMargins(12, 7, 12, 7);
    layout->setSpacing(2);
    auto* caption_label = new QLabel(caption, container);
    caption_label->setObjectName(QStringLiteral("statusCaption"));
    value_label = new QLabel(QStringLiteral("--"), container);
    value_label->setObjectName(QStringLiteral("statusValue"));
    value_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    value_label->setMinimumWidth(150);
    layout->addWidget(caption_label);
    layout->addWidget(value_label);
    return container;
}

/** @brief 把协议扫描模式转换为操作员可读名称。 */
QString scan_pattern_name(const std::string& pattern) {
    if (pattern == "conical" || pattern == "conical_scan") {
        return QStringLiteral("锥形方位扫描");
    }
    if (pattern == "horizontal" || pattern == "horizontal_scan") {
        return QStringLiteral("水平扫描");
    }
    if (pattern.empty()) return QStringLiteral("模式待同步");
    return QString::fromStdString(pattern);
}

} // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setup_ui();
    setup_worker();
}

MainWindow::~MainWindow() {
    if (worker_ != nullptr && worker_thread_.isRunning()) {
        QMetaObject::invokeMethod(worker_, "shutdown", Qt::BlockingQueuedConnection);
    }
    worker_thread_.quit();
    worker_thread_.wait();
}

void MainWindow::set_endpoint(const QString& host, quint16 port) {
    host_edit_->setText(host);
    port_spin_->setValue(static_cast<int>(port));
}

void MainWindow::connect_to_configured_server() {
    on_connect();
}

void MainWindow::setup_ui() {
    setWindowTitle(QStringLiteral("YLJ5 大气颗粒物激光雷达监控终端"));
    resize(1480, 900);
    setMinimumSize(1080, 680);

    auto* toolbar = addToolBar(QStringLiteral("设备控制"));
    toolbar->setMovable(false);
    toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    toolbar->addWidget(new QLabel(QStringLiteral("地址"), toolbar));
    host_edit_ = new QLineEdit(QStringLiteral("127.0.0.1"), toolbar);
    host_edit_->setMinimumWidth(150);
    host_edit_->setMaximumWidth(230);
    host_edit_->setClearButtonEnabled(true);
    toolbar->addWidget(host_edit_);
    toolbar->addWidget(new QLabel(QStringLiteral("端口"), toolbar));
    port_spin_ = new QSpinBox(toolbar);
    port_spin_->setRange(1, 65535);
    port_spin_->setValue(19850);
    port_spin_->setMinimumWidth(92);
    toolbar->addWidget(port_spin_);
    toolbar->addSeparator();

    connect_action_ = toolbar->addAction(
        style()->standardIcon(QStyle::SP_DialogApplyButton), QStringLiteral("连接"));
    connect_action_->setToolTip(QStringLiteral("连接设备服务"));
    disconnect_action_ = toolbar->addAction(
        style()->standardIcon(QStyle::SP_DialogCloseButton), QStringLiteral("断开"));
    disconnect_action_->setToolTip(QStringLiteral("断开当前设备"));
    toolbar->addSeparator();
    start_action_ = toolbar->addAction(
        style()->standardIcon(QStyle::SP_MediaPlay), QStringLiteral("开始"));
    start_action_->setToolTip(QStringLiteral("启动扫描程序"));
    pause_action_ = toolbar->addAction(
        style()->standardIcon(QStyle::SP_MediaPause), QStringLiteral("暂停"));
    pause_action_->setToolTip(QStringLiteral("暂停扫描程序"));
    stop_action_ = toolbar->addAction(
        style()->standardIcon(QStyle::SP_MediaStop), QStringLiteral("停止"));
    stop_action_->setToolTip(QStringLiteral("停止扫描程序"));
    toolbar->addSeparator();
    calibration_action_ = toolbar->addAction(
        style()->standardIcon(QStyle::SP_DialogOpenButton), QStringLiteral("加载标定"));
    calibration_action_->setToolTip(QStringLiteral("加载接收机与站点 PM 组合标定 JSON"));

    auto* central = new QWidget(this);
    auto* central_layout = new QVBoxLayout(central);
    central_layout->setContentsMargins(10, 10, 10, 10);
    central_layout->setSpacing(10);
    setCentralWidget(central);

    auto* status_band = new QFrame(central);
    status_band->setObjectName(QStringLiteral("statusBand"));
    auto* status_layout = new QHBoxLayout(status_band);
    status_layout->setContentsMargins(0, 0, 0, 0);
    status_layout->setSpacing(0);
    status_layout->addWidget(make_status_item(QStringLiteral("连接"), connection_label_, status_band));
    status_layout->addWidget(make_status_item(QStringLiteral("设备"), device_label_, status_band));
    status_layout->addWidget(make_status_item(QStringLiteral("扫描"), scan_label_, status_band));
    status_layout->addWidget(make_status_item(QStringLiteral("指向"), pointing_label_, status_band));
    status_layout->addWidget(make_status_item(QStringLiteral("PM 标定"), calibration_label_, status_band));
    status_layout->addWidget(make_status_item(QStringLiteral("传输"), transport_label_, status_band));
    status_layout->setStretch(1, 2);
    status_layout->setStretch(2, 2);
    central_layout->addWidget(status_band);

    connection_label_->setText(QStringLiteral("未连接"));
    device_label_->setText(QStringLiteral("等待设备状态"));
    scan_label_->setText(QStringLiteral("等待扫描程序"));
    pointing_label_->setText(QStringLiteral("方位 -- / 仰角 --"));
    calibration_label_->setText(QStringLiteral("未加载，仅输出光学产品"));
    transport_label_->setText(QStringLiteral("0 帧 / 0 错误 / 0 MiB"));

    auto* products = new QTabWidget(central);
    ppi_widget_ = new PpiWidget(products);
    profile_widget_ = new VerticalProfileWidget(products);
    products->addTab(ppi_widget_, QStringLiteral("方位扫描"));
    products->addTab(profile_widget_, QStringLiteral("垂直廓线"));

    alarm_panel_ = new AlarmPanel(central);
    alarm_panel_->setMinimumWidth(360);
    auto* splitter = new QSplitter(Qt::Horizontal, central);
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(products);
    splitter->addWidget(alarm_panel_);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({1040, 400});
    central_layout->addWidget(splitter, 1);

    statusBar()->showMessage(QStringLiteral("就绪：连接设备后等待状态帧和完整扫描周期"));
    update_action_state(false);

    connect(connect_action_, &QAction::triggered, this, &MainWindow::on_connect);
    connect(disconnect_action_, &QAction::triggered, this, &MainWindow::on_disconnect);
    connect(start_action_, &QAction::triggered, this, [this]() {
        emit command_requested(QStringLiteral("start"));
    });
    connect(pause_action_, &QAction::triggered, this, [this]() {
        emit command_requested(QStringLiteral("pause"));
    });
    connect(stop_action_, &QAction::triggered, this, [this]() {
        emit command_requested(QStringLiteral("stop"));
    });
    connect(calibration_action_, &QAction::triggered, this, &MainWindow::on_load_calibration);
    connect(host_edit_, &QLineEdit::returnPressed, this, &MainWindow::on_connect);
}

void MainWindow::setup_worker() {
    ProcessorConfig processor_config;
    worker_ = new LidarClientWorker(std::move(processor_config));
    worker_thread_.setObjectName(QStringLiteral("YLJ5-Network-Processing"));
    worker_->moveToThread(&worker_thread_);

    connect(this, &MainWindow::connect_requested,
            worker_, &LidarClientWorker::connect_to_server, Qt::QueuedConnection);
    connect(this, &MainWindow::disconnect_requested,
            worker_, &LidarClientWorker::disconnect_from_server, Qt::QueuedConnection);
    connect(this, &MainWindow::command_requested,
            worker_, &LidarClientWorker::send_command, Qt::QueuedConnection);
    connect(this, &MainWindow::calibration_requested,
            worker_, &LidarClientWorker::load_pm_calibration, Qt::QueuedConnection);
    connect(worker_, &LidarClientWorker::step_ready,
            this, &MainWindow::on_step_ready, Qt::QueuedConnection);
    connect(worker_, &LidarClientWorker::device_status_ready,
            this, &MainWindow::on_device_status_ready, Qt::QueuedConnection);
    connect(worker_, &LidarClientWorker::connection_changed,
            this, &MainWindow::on_connection_changed, Qt::QueuedConnection);
    connect(worker_, &LidarClientWorker::transport_stats,
            this, &MainWindow::on_transport_stats, Qt::QueuedConnection);
    connect(worker_, &LidarClientWorker::calibration_changed,
            this, &MainWindow::on_calibration_changed, Qt::QueuedConnection);
    connect(worker_, &LidarClientWorker::worker_error,
            this, &MainWindow::on_worker_error, Qt::QueuedConnection);
    connect(worker_, &LidarClientWorker::operation_message,
            this, [this](const QString& message) {
                statusBar()->showMessage(message, 8000);
            }, Qt::QueuedConnection);
    connect(&worker_thread_, &QThread::finished, worker_, &QObject::deleteLater);
    connect(worker_, &QObject::destroyed, this, [this]() { worker_ = nullptr; });
    worker_thread_.start();
}

void MainWindow::update_action_state(bool connected) {
    connect_action_->setEnabled(!connected);
    disconnect_action_->setEnabled(connected);
    start_action_->setEnabled(connected);
    pause_action_->setEnabled(connected);
    stop_action_->setEnabled(connected);
    host_edit_->setEnabled(!connected);
    port_spin_->setEnabled(!connected);
}

void MainWindow::on_connect() {
    const QString host = host_edit_->text().trimmed();
    if (host.isEmpty()) {
        statusBar()->showMessage(QStringLiteral("请输入设备地址"), 5000);
        host_edit_->setFocus();
        return;
    }
    connect_action_->setEnabled(false);
    connection_label_->setText(QStringLiteral("正在连接 %1:%2").arg(host).arg(port_spin_->value()));
    emit connect_requested(host, static_cast<quint16>(port_spin_->value()));
}

void MainWindow::on_disconnect() {
    emit disconnect_requested();
}

void MainWindow::on_load_calibration() {
    const QString file_path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("加载接收机与站点 PM 组合标定"),
        QString(),
        QStringLiteral("JSON 标定文件 (*.json);;所有文件 (*)"));
    if (!file_path.isEmpty()) {
        emit calibration_requested(file_path);
    }
}

void MainWindow::on_step_ready(StepResultPtr result, DisplaySnapshotPtr display) {
    if (!result || !display) return;
    ++completed_steps_;
    ppi_widget_->set_snapshot(*display);
    profile_widget_->set_snapshot(*display);
    alarm_panel_->update_data(*result);

    statusBar()->showMessage(
        QStringLiteral("周期 %1 | %2 | %3 条有效射线，%4 条拒绝，%5 个仰角层")
            .arg(completed_steps_)
            .arg(QString::fromStdString(result->timestamp))
            .arg(static_cast<qulonglong>(result->processed_profiles.size()))
            .arg(result->rejected_count)
            .arg(result->elevation_layer_count));
}

void MainWindow::on_device_status_ready(DeviceStatusSnapshotPtr status) {
    if (!status || !status->valid) return;
    const QString model = status->device_model.empty()
        ? QStringLiteral("型号未知")
        : QString::fromStdString(status->device_model);
    device_label_->setText(QStringLiteral("%1 / %2")
        .arg(model, QString::fromStdString(status->device_state)));

    const QString program = status->scan_program_mode.empty()
        ? QStringLiteral("程序待同步")
        : QString::fromStdString(status->scan_program_mode);
    scan_label_->setText(QStringLiteral("%1 / %2 / %3 s")
        .arg(program, scan_pattern_name(status->active_scan_pattern))
        .arg(status->full_scan_cycle_s, 0, 'f', 1));

    const int schedule_count = static_cast<int>(status->scheduled_elevations_deg.size());
    pointing_label_->setText(QStringLiteral("方位 %1° / 仰角 %2° / 层 %3/%4")
        .arg(status->gimbal_azimuth_deg, 0, 'f', 1)
        .arg(status->gimbal_elevation_deg, 0, 'f', 1)
        .arg(schedule_count > 0 ? status->elevation_schedule_index + 1 : 0)
        .arg(schedule_count));
}

void MainWindow::on_connection_changed(bool connected, const QString& description) {
    connection_label_->setText(description);
    const bool connecting = !connected && description.startsWith(QStringLiteral("正在连接"));
    if (connecting) {
        connect_action_->setEnabled(false);
        disconnect_action_->setEnabled(true);
        start_action_->setEnabled(false);
        pause_action_->setEnabled(false);
        stop_action_->setEnabled(false);
        host_edit_->setEnabled(false);
        port_spin_->setEnabled(false);
    } else {
        update_action_state(connected);
    }
    statusBar()->showMessage(description, 8000);
}

void MainWindow::on_transport_stats(quint64 frames, quint64 errors, quint64 bytes) {
    transport_label_->setText(QStringLiteral("%1 帧 / %2 错误 / %3 MiB")
        .arg(frames)
        .arg(errors)
        .arg(static_cast<double>(bytes) / (1024.0 * 1024.0), 0, 'f', 2));
}

void MainWindow::on_calibration_changed(bool valid, const QString& description) {
    calibration_label_->setText(description);
    calibration_label_->setProperty("validCalibration", valid);
    calibration_label_->style()->unpolish(calibration_label_);
    calibration_label_->style()->polish(calibration_label_);
    statusBar()->showMessage(description, 10000);
}

void MainWindow::on_worker_error(const QString& description) {
    statusBar()->showMessage(description, 12000);
}

} // namespace lidar_client
