/**
 * @file LidarClientWorker.cpp
 * @brief Linux Qt 客户端工作线程实现。
 */
#include "lidar_client/LidarClientWorker.hpp"

#include <QAbstractSocket>
#include <QFileInfo>
#include <QTcpSocket>
#include <QThread>

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "lidar_log/Logger.hpp"

namespace lidar_client {

namespace {

constexpr qsizetype MaximumBufferedBytes = 64 * 1024 * 1024;

double required_number(const lidar_core::Json& json, const char* key) {
    // 接收机和 PM 系数属于强合同：字段缺失时拒绝整份标定，不能用 0 静默回退。
    if (!json.contains(key) || !json.at(key).is_number()) {
        throw std::runtime_error(std::string("missing calibration field: ") + key);
    }
    return json.at(key).number_value();
}

std::string optional_string(const lidar_core::Json& json, const char* key) {
    return json.contains(key) && json.at(key).is_string()
        ? json.at(key).string_value()
        : "";
}

/**
 * @brief 把组合标定 JSON 中的 receiver_calibration 对象解析为强类型模型。
 *
 * 除结构检查外还验证参数的基本物理范围。这里不判断标定是否真的来自实验室；valid=true
 * 表示文件合同完整且调用方明确选择加载，标定 ID 负责后续审计追踪。
 */
ReceiverCalibrationModel parse_receiver_calibration(const lidar_core::Json& json) {
    if (!json.is_object()) {
        throw std::runtime_error("receiver_calibration must be an object");
    }
    ReceiverCalibrationModel model;
    model.calibration_id = optional_string(json, "calibration_id");
    model.detector_mode = optional_string(json, "detector_mode");
    model.signal_unit = optional_string(json, "signal_unit");
    model.range_zero_offset_m = required_number(json, "range_zero_offset_m");
    model.minimum_valid_range_m = required_number(json, "minimum_valid_range_m");
    model.minimum_retrieval_overlap = required_number(json, "minimum_retrieval_overlap");
    model.minimum_quantitative_overlap = required_number(
        json, "minimum_quantitative_overlap");
    model.saturation_counts = required_number(json, "saturation_counts");
    model.saturation_guard_fraction = required_number(json, "saturation_guard_fraction");
    model.dead_time_loss_per_count = required_number(json, "dead_time_loss_per_count");
    model.maximum_dead_time_occupancy = required_number(
        json, "maximum_dead_time_occupancy");
    // 默认模型自带仿真 kernel，加载实机文件时必须清空并完全采用文件内容。
    model.afterpulse_kernel.clear();
    if (!json.contains("afterpulse_kernel") || !json.at("afterpulse_kernel").is_array()) {
        throw std::runtime_error("missing calibration field: afterpulse_kernel");
    }
    double kernel_sum = 0.0;
    for (const auto& item : json.at("afterpulse_kernel").array_items()) {
        // 单个泄漏系数必须非负且小于 0.5，避免递推反卷积明显不稳定。
        if (!item.is_number() || item.number_value() < 0.0
            || item.number_value() >= 0.5) {
            throw std::runtime_error("afterpulse_kernel contains an invalid coefficient");
        }
        model.afterpulse_kernel.push_back(item.number_value());
        kernel_sum += item.number_value();
    }
    if (model.calibration_id.empty()) {
        throw std::runtime_error("receiver calibration_id is empty");
    }
    if (model.signal_unit != "mean_counts_per_pulse"
        && model.signal_unit != "integrated_counts") {
        throw std::runtime_error("receiver signal_unit is unsupported");
    }
    if (model.detector_mode != "photon_counting" && model.detector_mode != "analog") {
        throw std::runtime_error("receiver detector_mode is unsupported");
    }
    /*
     * overlap 反演阈值不得高于定量阈值；占用率必须严格小于数学奇点 1；kernel 总和限制
     * 为 0.8，保证前序泄漏不会主导当前距离门。更严格的设备验收应由标定流程完成。
     */
    if (model.minimum_valid_range_m < 0.0
        || !std::isfinite(model.range_zero_offset_m)
        || model.minimum_retrieval_overlap <= 0.0
        || model.minimum_retrieval_overlap > model.minimum_quantitative_overlap
        || model.minimum_quantitative_overlap > 1.0
        || model.saturation_counts <= 0.0
        || model.saturation_guard_fraction < 0.5
        || model.saturation_guard_fraction > 1.0
        || model.dead_time_loss_per_count < 0.0
        || model.maximum_dead_time_occupancy <= 0.0
        || model.maximum_dead_time_occupancy >= 1.0
        || kernel_sum >= 0.8) {
        throw std::runtime_error("receiver calibration values are outside physical limits");
    }
    model.valid = true;
    return model;
}

} // namespace

LidarClientWorker::LidarClientWorker(ProcessorConfig config, QObject* parent)
    : QObject(parent),
      processor_(std::move(config)) {
    qRegisterMetaType<StepResultPtr>("lidar_client::StepResultPtr");
    qRegisterMetaType<DeviceStatusSnapshotPtr>("lidar_client::DeviceStatusSnapshotPtr");
    qRegisterMetaType<DisplaySnapshotPtr>("lidar_client::DisplaySnapshotPtr");
    processor_.set_step_complete_callback([this](StepResult result) {
        const auto summary = scan_monitor_.take_summary_for_timestamp(result.timestamp);
        if (summary.expected_rays > 0 && !summary.complete) {
            result.qc_flags.emplace_back("scan-cycle-incomplete");
        }
        auto result_snapshot = std::make_shared<const StepResult>(std::move(result));
        auto display_snapshot = std::make_shared<const DisplaySnapshot>(
            build_display_snapshot(*result_snapshot));
        emit step_ready(std::move(result_snapshot), std::move(display_snapshot));
    });
}

LidarClientWorker::~LidarClientWorker() {
    if (socket_ != nullptr) {
        socket_->abort();
    }
}

void LidarClientWorker::ensure_socket() {
    if (socket_ != nullptr) {
        return;
    }
    socket_ = new QTcpSocket(this);
    connect(socket_, &QTcpSocket::readyRead, this, &LidarClientWorker::handle_ready_read);
    connect(socket_, &QTcpSocket::connected, this, [this]() {
        LIDAR_LOG_INFO("[client] connected to ", socket_->peerName().toStdString(), ":",
                       socket_->peerPort());
        emit connection_changed(
            true,
            QStringLiteral("已连接 %1:%2")
                .arg(socket_->peerName())
                .arg(socket_->peerPort()));
        send_command(QStringLiteral("get_status"));
    });
    connect(socket_, &QTcpSocket::disconnected, this, [this]() {
        LIDAR_LOG_INFO("[client] disconnected from device");
        if (!shutting_down_) {
            emit connection_changed(false, QStringLiteral("连接已断开"));
        }
    });
    connect(socket_, &QTcpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError) {
        LIDAR_LOG_ERROR("[client] socket error: ", socket_->errorString().toStdString());
        handle_socket_error();
    });
}

void LidarClientWorker::connect_to_server(const QString& host, quint16 port) {
    if (QThread::currentThread() != thread()) {
        emit worker_error(QStringLiteral("网络连接请求未在工作线程执行"));
        return;
    }
    shutting_down_ = false;
    ensure_socket();
    receive_buffer_.clear();
    scan_monitor_.reset();
    if (socket_->state() != QAbstractSocket::UnconnectedState) {
        socket_->abort();
    }
    emit connection_changed(false, QStringLiteral("正在连接 %1:%2").arg(host).arg(port));
    socket_->connectToHost(host, port);
}

void LidarClientWorker::disconnect_from_server() {
    processor_.finalize_step();
    if (socket_ != nullptr) {
        socket_->disconnectFromHost();
        if (socket_->state() != QAbstractSocket::UnconnectedState) {
            socket_->abort();
        }
    }
    receive_buffer_.clear();
    scan_monitor_.reset();
    emit connection_changed(false, QStringLiteral("已主动断开"));
}

void LidarClientWorker::send_command(const QString& command) {
    if (socket_ == nullptr || socket_->state() != QAbstractSocket::ConnectedState) {
        emit worker_error(QStringLiteral("设备未连接，无法发送命令"));
        return;
    }
    const auto frame = lidar_protocol::make_frame(
        lidar_protocol::FrameType::command,
        "",
        lidar_core::Json::Object{{"command", command.toStdString()}});
    const std::string wire = frame.to_wire();
    if (socket_->write(wire.data(), static_cast<qint64>(wire.size())) < 0) {
        emit worker_error(QStringLiteral("命令发送失败：%1").arg(socket_->errorString()));
    }
}

void LidarClientWorker::load_pm_calibration(const QString& file_path) {
    try {
        // 文件读取和全部字段验证都在工作线程执行，GUI 主线程不会因磁盘或 JSON 解析阻塞。
        const auto json = lidar_core::read_json_file(std::filesystem::path(file_path.toStdString()));
        PmCalibrationModel model;
        model.calibration_id = optional_string(json, "calibration_id");
        model.valid_from = optional_string(json, "valid_from");
        model.pm25_intercept_ugm3 = required_number(json, "pm25_intercept_ugm3");
        model.pm25_slope_ugm3_per_km = required_number(json, "pm25_slope_ugm3_per_km");
        model.pm10_intercept_ugm3 = required_number(json, "pm10_intercept_ugm3");
        model.pm10_slope_ugm3_per_km = required_number(json, "pm10_slope_ugm3_per_km");
        if (model.calibration_id.empty()) {
            throw std::runtime_error("calibration_id is empty");
        }
        model.valid = true;
        // 定量 PM 必须和接收机幅度/有效区标定成套加载，不接受只有 PM 斜率的半份文件。
        if (!json.contains("receiver_calibration")) {
            throw std::runtime_error("missing calibration field: receiver_calibration");
        }
        auto receiver = parse_receiver_calibration(json.at("receiver_calibration"));

        // 只有 PM 与接收机两部分都成功解析后才修改处理器，避免异常文件造成半更新状态。
        processor_.set_pm_calibration(model);
        processor_.set_receiver_calibration(receiver);
        emit calibration_changed(
            true,
            QStringLiteral("PM %1 / 接收机 %2")
                .arg(QString::fromStdString(model.calibration_id))
                .arg(QString::fromStdString(receiver.calibration_id)));
    } catch (const std::exception& error) {
        emit calibration_changed(
            false,
            QStringLiteral("标定文件无效：%1").arg(QString::fromUtf8(error.what())));
    }
}

void LidarClientWorker::shutdown() {
    shutting_down_ = true;
    processor_.finalize_step();
    if (socket_ != nullptr) {
        socket_->abort();
    }
    receive_buffer_.clear();
}

void LidarClientWorker::handle_ready_read() {
    if (socket_ == nullptr) {
        return;
    }
    const QByteArray chunk = socket_->readAll();
    LIDAR_LOG_INFO("收到激光雷达数据，本次接收 ", chunk.size(), " 字节");
    bytes_received_ += static_cast<quint64>(chunk.size());
    receive_buffer_.append(chunk);
    if (receive_buffer_.size() > MaximumBufferedBytes) {
        LIDAR_LOG_WARN("接收缓存过大，当前缓存为 ", receive_buffer_.size(),
                       " 字节，已丢弃未封口数据");
        receive_buffer_.clear();
        ++parse_errors_;
        emit worker_error(QStringLiteral("接收缓存超过 64 MiB，已丢弃未封口数据"));
        return;
    }

    if (receive_buffer_.contains('\n')) {
        LIDAR_LOG_INFO("开始解析激光雷达数据");
    }
    qsizetype newline = -1;
    while ((newline = receive_buffer_.indexOf('\n')) >= 0) {
        QByteArray line = receive_buffer_.left(newline).trimmed();
        receive_buffer_.remove(0, newline + 1);
        if (!line.isEmpty()) {
            process_line(line);
        }
    }
    emit transport_stats(frames_received_, parse_errors_, bytes_received_);
}

void LidarClientWorker::process_line(const QByteArray& line) {
    try {
        // 一行 JSONL 必须先完整解析成 Frame；失败帧不会进入状态机或科学算法。
        const auto frame = lidar_protocol::parse_frame(line.toStdString());
        ++frames_received_;

        // 周期监控先观察原始 ray_index/rays_in_cycle，即使该射线随后科学处理失败也能统计到达情况。
        scan_monitor_.observe_frame(frame);

        // 状态模型消费 status/telemetry/command_result 的增量字段，并向 GUI 发布不可变快照。
        if (device_status_.update_from_frame(frame)) {
            emit device_status_ready(
                std::make_shared<const DeviceStatusSnapshot>(device_status_.snapshot()));
        }
        if (frame.type == lidar_protocol::FrameType::command_result
            && frame.payload.contains("message")
            && frame.payload.at("message").is_string()) {
            const QString message = QString::fromStdString(frame.payload.at("message").string_value());
            bool success = true;
            if (frame.payload.contains("success") && frame.payload.at("success").is_bool()) {
                success = frame.payload.at("success").bool_value();
            }
            if (frame.payload.contains("accepted") && frame.payload.at("accepted").is_bool()) {
                success = success && frame.payload.at("accepted").bool_value();
            }
            if (frame.payload.contains("result") && frame.payload.at("result").is_string()) {
                success = success && frame.payload.at("result").string_value() != "error";
            }
            if (success) {
                emit operation_message(message);
            } else {
                emit worker_error(message);
            }
        }
        // 所有帧最后交给 FrameProcessor：raw 走 L0-L2，ground/status 更新上下文，heartbeat 封口。
        processor_.handle_frame(frame);
    } catch (const std::exception& error) {
        ++parse_errors_;
        LIDAR_LOG_WARN("[client] protocol frame parse failed: ", error.what());
        emit worker_error(QStringLiteral("协议帧解析失败：%1").arg(QString::fromUtf8(error.what())));
    }
}

void LidarClientWorker::handle_socket_error() {
    if (socket_ == nullptr || shutting_down_) {
        return;
    }
    emit connection_changed(false, QStringLiteral("网络错误：%1").arg(socket_->errorString()));
}

} // namespace lidar_client
