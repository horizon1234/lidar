/**
 * @file LidarClientWorker.cpp
 * @brief Linux Qt 客户端工作线程实现。
 */
#include "lidar_client/LidarClientWorker.hpp"

#include <QAbstractSocket>
#include <QFileInfo>
#include <QTcpSocket>
#include <QThread>

#include <filesystem>
#include <stdexcept>

namespace lidar_client {

namespace {

constexpr qsizetype MaximumBufferedBytes = 64 * 1024 * 1024;

double required_number(const lidar_core::Json& json, const char* key) {
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
        emit connection_changed(
            true,
            QStringLiteral("已连接 %1:%2")
                .arg(socket_->peerName())
                .arg(socket_->peerPort()));
        send_command(QStringLiteral("get_status"));
    });
    connect(socket_, &QTcpSocket::disconnected, this, [this]() {
        if (!shutting_down_) {
            emit connection_changed(false, QStringLiteral("连接已断开"));
        }
    });
    connect(socket_, &QTcpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError) {
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
        processor_.set_pm_calibration(model);
        emit calibration_changed(
            true,
            QStringLiteral("已加载站点标定：%1")
                .arg(QString::fromStdString(model.calibration_id)));
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
    bytes_received_ += static_cast<quint64>(chunk.size());
    receive_buffer_.append(chunk);
    if (receive_buffer_.size() > MaximumBufferedBytes) {
        receive_buffer_.clear();
        ++parse_errors_;
        emit worker_error(QStringLiteral("接收缓存超过 64 MiB，已丢弃未封口数据"));
        return;
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
        const auto frame = lidar_protocol::parse_frame(line.toStdString());
        ++frames_received_;
        scan_monitor_.observe_frame(frame);
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
        processor_.handle_frame(frame);
    } catch (const std::exception& error) {
        ++parse_errors_;
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
