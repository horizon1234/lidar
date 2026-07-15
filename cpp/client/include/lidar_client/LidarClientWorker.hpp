/**
 * @file LidarClientWorker.hpp
 * @brief Linux Qt 客户端工作线程：异步网络、协议解析和设备数据处理。
 */
#pragma once

#include <QByteArray>
#include <QMetaType>
#include <QObject>
#include <QString>

#include <memory>

#include "lidar_client/DeviceStatusModel.hpp"
#include "lidar_client/DisplaySnapshot.hpp"
#include "lidar_client/FrameProcessor.hpp"
#include "lidar_client/ScanCycleMonitor.hpp"

class QTcpSocket;

namespace lidar_client {

using StepResultPtr = std::shared_ptr<const StepResult>;
using DeviceStatusSnapshotPtr = std::shared_ptr<const DeviceStatusSnapshot>;

/**
 * @brief 必须移动到专用 QThread 的网络与处理工作器。
 *
 * QTcpSocket 在 connect_to_server 槽中创建，确保 socket、JSON 解析、四通道处理和
 * Fernald 反演全部拥有工作线程亲和性。GUI 只接收 shared_ptr 指向的不可变快照。
 */
class LidarClientWorker : public QObject {
    Q_OBJECT

public:
    explicit LidarClientWorker(ProcessorConfig config, QObject* parent = nullptr);
    ~LidarClientWorker() override;

public slots:
    /** @brief 异步连接目标设备服务。 */
    void connect_to_server(const QString& host, quint16 port);

    /** @brief 主动断开当前连接。 */
    void disconnect_from_server();

    /** @brief 向设备发送 start/pause/stop/get_status 等命令。 */
    void send_command(const QString& command);

    /** @brief 在工作线程读取并加载站点 PM 标定 JSON。 */
    void load_pm_calibration(const QString& file_path);

    /** @brief 线程退出前同步关闭 socket 并封口当前扫描周期。 */
    void shutdown();

signals:
    /** @brief 网络连接状态发生变化。 */
    void connection_changed(bool connected, const QString& description);

    /** @brief 完成一个扫描周期的处理；科学结果和预计算显示快照均为只读。 */
    void step_ready(
        lidar_client::StepResultPtr result,
        lidar_client::DisplaySnapshotPtr display);

    /** @brief 设备状态或遥测发生增量更新。 */
    void device_status_ready(lidar_client::DeviceStatusSnapshotPtr status);

    /** @brief 传输统计更新。 */
    void transport_stats(quint64 frames_received, quint64 parse_errors, quint64 bytes_received);

    /** @brief 标定加载结果。 */
    void calibration_changed(bool valid, const QString& description);

    /** @brief 可展示给操作员的非致命错误。 */
    void worker_error(const QString& description);

    /** @brief 设备确认命令成功等普通操作消息。 */
    void operation_message(const QString& description);

private slots:
    /** @brief 消费 QTcpSocket 当前所有可读数据。 */
    void handle_ready_read();

    /** @brief 处理 socket 错误并同步连接状态。 */
    void handle_socket_error();

private:
    /** @brief 创建具有当前工作线程亲和性的 socket。 */
    void ensure_socket();

    /** @brief 解析并分发一条完整 JSONL 帧。 */
    void process_line(const QByteArray& line);

    QTcpSocket* socket_ = nullptr;          ///< 工作线程拥有的异步 TCP socket。
    QByteArray receive_buffer_;             ///< 尚未形成完整 JSONL 行的接收缓存。
    FrameProcessor processor_;              ///< 四通道 L0-L2 处理器，仅在工作线程调用。
    DeviceStatusModel device_status_;       ///< 状态与遥测增量合并模型。
    ScanCycleMonitor scan_monitor_;         ///< 扫描周期原始射线缺失和重复接收监控器。
    quint64 frames_received_ = 0;            ///< 成功解析协议帧数量。
    quint64 parse_errors_ = 0;               ///< JSON 或协议解析失败数量。
    quint64 bytes_received_ = 0;             ///< socket 累计接收字节数。
    bool shutting_down_ = false;             ///< 防止线程关闭时触发自动状态处理。
};

} // namespace lidar_client

Q_DECLARE_METATYPE(lidar_client::StepResultPtr)
Q_DECLARE_METATYPE(lidar_client::DeviceStatusSnapshotPtr)
