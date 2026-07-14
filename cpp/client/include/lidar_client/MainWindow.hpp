/**
 * @file MainWindow.hpp
 * @brief Linux Qt 主窗口：只负责设备控制和不可变结果快照的可视化。
 */
#pragma once

#include <QMainWindow>
#include <QString>
#include <QThread>

#include "lidar_client/LidarClientWorker.hpp"

class QAction;
class QLabel;
class QLineEdit;
class QSpinBox;

namespace lidar_client {

class AlarmPanel;
class PpiWidget;
class VerticalProfileWidget;

/**
 * @brief YLJ5 监控客户端主窗口。
 *
 * 主窗口不读取 socket，也不执行反演或热点检测。所有命令通过排队信号发送给工作线程，
 * 页面只消费工作线程发布的 StepResult 和 DeviceStatusSnapshot 只读快照。
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    /** @brief 设置工具栏中的目标地址，主要供命令行启动参数使用。 */
    void set_endpoint(const QString& host, quint16 port);

    /** @brief 使用工具栏当前地址发起异步连接。 */
    void connect_to_configured_server();

signals:
    /** @brief 排队到工作线程的连接请求。 */
    void connect_requested(const QString& host, quint16 port);

    /** @brief 排队到工作线程的断开请求。 */
    void disconnect_requested();

    /** @brief 排队到工作线程的设备控制命令。 */
    void command_requested(const QString& command);

    /** @brief 排队到工作线程的 PM 标定加载请求。 */
    void calibration_requested(const QString& file_path);

private slots:
    /** @brief 校验地址并请求工作线程连接设备。 */
    void on_connect();

    /** @brief 请求工作线程断开设备。 */
    void on_disconnect();

    /** @brief 选择站点 PM 标定 JSON 并交给工作线程加载。 */
    void on_load_calibration();

    /** @brief 渲染一个完成扫描周期的处理结果。 */
    void on_step_ready(
        lidar_client::StepResultPtr result,
        lidar_client::DisplaySnapshotPtr display);

    /** @brief 刷新设备能力、扫描程序和云台遥测。 */
    void on_device_status_ready(lidar_client::DeviceStatusSnapshotPtr status);

    /** @brief 刷新连接状态并同步工具栏可用性。 */
    void on_connection_changed(bool connected, const QString& description);

    /** @brief 刷新累计帧、错误和字节统计。 */
    void on_transport_stats(quint64 frames, quint64 errors, quint64 bytes);

    /** @brief 刷新当前 PM 标定状态。 */
    void on_calibration_changed(bool valid, const QString& description);

    /** @brief 在状态栏展示工作线程的非致命错误。 */
    void on_worker_error(const QString& description);

private:
    /** @brief 创建工具栏、状态带和产品视图。 */
    void setup_ui();

    /** @brief 创建工作器并建立所有跨线程排队连接。 */
    void setup_worker();

    /** @brief 根据连接状态统一更新设备命令可用性。 */
    void update_action_state(bool connected);

    QThread worker_thread_;                    ///< 网络、协议和 L0-L2 处理专用线程。
    LidarClientWorker* worker_ = nullptr;      ///< 迁移到专用线程的客户端工作器。
    QLineEdit* host_edit_ = nullptr;           ///< 目标设备 IPv4、IPv6 或主机名输入框。
    QSpinBox* port_spin_ = nullptr;            ///< 目标设备 TCP 端口输入框。
    QAction* connect_action_ = nullptr;        ///< 发起异步连接的工具栏动作。
    QAction* disconnect_action_ = nullptr;     ///< 主动断开连接的工具栏动作。
    QAction* start_action_ = nullptr;          ///< 向设备发送 start 命令的工具栏动作。
    QAction* pause_action_ = nullptr;          ///< 向设备发送 pause 命令的工具栏动作。
    QAction* stop_action_ = nullptr;           ///< 向设备发送 stop 命令的工具栏动作。
    QAction* calibration_action_ = nullptr;    ///< 选择并加载站点 PM 标定的工具栏动作。
    PpiWidget* ppi_widget_ = nullptr;          ///< 水平或锥形扫描产品视图。
    VerticalProfileWidget* profile_widget_ = nullptr; ///< 天顶消光与退偏比廓线视图。
    AlarmPanel* alarm_panel_ = nullptr;        ///< 已标定热点和周期 QC 面板。
    QLabel* connection_label_ = nullptr;       ///< 当前网络连接状态摘要。
    QLabel* device_label_ = nullptr;           ///< 设备型号和运行状态摘要。
    QLabel* scan_label_ = nullptr;             ///< 扫描程序、模式和周期摘要。
    QLabel* pointing_label_ = nullptr;         ///< 当前云台方位、仰角和调度层摘要。
    QLabel* calibration_label_ = nullptr;      ///< PM 标定批次和有效性摘要。
    QLabel* transport_label_ = nullptr;        ///< TCP 接收帧、错误和流量摘要。
    bool connected_ = false;                   ///< GUI 最近收到的网络连接状态。
    quint64 completed_steps_ = 0;              ///< GUI 已渲染的完整扫描周期数。
};

} // namespace lidar_client
