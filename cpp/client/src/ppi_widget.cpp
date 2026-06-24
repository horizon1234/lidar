/**
 * @file ppi_widget.cpp
 * @brief PPI 热力图 Widget 实现。
 */
#ifdef LIDAR_ENABLE_QT

#include "lidar_client/ppi_widget.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <cmath>

namespace lidar_client {

// =========================================================================
// 辅助：turbo 色表
// =========================================================================

QColor PpiWidget::value_to_color(double normalized) {
    // clamp to [0, 1]
    normalized = std::max(0.0, std::min(1.0, normalized));

    // 简化的 jet 色表
    // 蓝(0) → 青(0.25) → 绿(0.5) → 黄(0.75) → 红(1.0)
    double r, g, b;
    if (normalized < 0.25) {
        // 蓝 → 青
        double t = normalized / 0.25;
        r = 0.0;
        g = t;
        b = 1.0;
    } else if (normalized < 0.5) {
        // 青 → 绿
        double t = (normalized - 0.25) / 0.25;
        r = 0.0;
        g = 1.0;
        b = 1.0 - t;
    } else if (normalized < 0.75) {
        // 绿 → 黄
        double t = (normalized - 0.5) / 0.25;
        r = t;
        g = 1.0;
        b = 0.0;
    } else {
        // 黄 → 红
        double t = (normalized - 0.75) / 0.25;
        r = 1.0;
        g = 1.0 - t;
        b = 0.0;
    }

    return QColor(static_cast<int>(r * 255),
                  static_cast<int>(g * 255),
                  static_cast<int>(b * 255));
}

// =========================================================================
// PpiWidget
// =========================================================================

PpiWidget::PpiWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(400, 400);
}

void PpiWidget::set_title(const QString& title) {
    title_ = title;
    update();
}

void PpiWidget::set_hotspots(const std::vector<lidar_core::Hotspot>& hotspots) {
    hotspots_ = hotspots;
    update();
}

void PpiWidget::set_data(const std::vector<lidar_core::ProcessedProfile>& profiles,
                          const std::string& field,
                          double max_range_m) {
    max_range_m_ = max_range_m;

    // 构建一个极坐标 → 笛卡尔的渲染
    int size = 500;
    canvas_ = QImage(size, size, QImage::Format_RGB32);
    canvas_.fill(Qt::black);

    double center = size / 2.0;
    double scale = (size / 2.0 - 20) / max_range_m; // 像素/米

    // 找全局最大值用于归一化
    double global_max = 1e-6;
    for (const auto& p : profiles) {
        const std::vector<double>* data = nullptr;
        if (field == "pm25") data = &p.pm25;
        else if (field == "extinction") data = &p.extinction;
        else if (field == "dry_extinction") data = &p.dry_extinction;
        if (!data) continue;
        for (double v : *data) {
            global_max = std::max(global_max, v);
        }
    }

    QPainter painter(&canvas_);
    painter.setRenderHint(QPainter::Antialiasing, false);

    // 渲染每条射线的每个距离 bin
    for (const auto& p : profiles) {
        double az = p.profile.azimuth_deg * M_PI / 180.0;
        double el = p.profile.elevation_deg * M_PI / 180.0;

        const std::vector<double>* data = nullptr;
        if (field == "pm25") data = &p.pm25;
        else if (field == "extinction") data = &p.extinction;
        else if (field == "dry_extinction") data = &p.dry_extinction;
        if (!data) continue;

        for (size_t i = 0; i < data->size() && i < p.profile.ranges_m.size(); ++i) {
            double r = p.profile.ranges_m[i];
            if (r > max_range_m) continue;

            // 极坐标 → 笛卡尔（PPI 投影，忽略 elevation 的 z 分量）
            double x = center + r * std::cos(az) * scale;
            double y = center - r * std::sin(az) * scale;

            double normalized = (*data)[i] / global_max;
            QColor color = value_to_color(normalized);

            // 画一个矩形代表一个 bin
            int bin_size = 3;
            painter.fillRect(QRectF(x - bin_size/2.0, y - bin_size/2.0,
                                    bin_size, bin_size),
                             color);
        }
    }

    // 画热点标注
    painter.setPen(QPen(Qt::white, 2));
    painter.setBrush(Qt::NoBrush);
    for (const auto& hs : hotspots_) {
        if (hs.centroid_enu_m.size() >= 2) {
            double x = center + hs.centroid_enu_m[0] * scale;
            double y = center - hs.centroid_enu_m[1] * scale;
            painter.drawEllipse(QPointF(x, y), 15, 15);
            painter.setPen(Qt::red);
            painter.drawText(QPointF(x + 18, y),
                             QString::fromStdString(hs.hotspot_id) +
                             " (" + QString::number(hs.peak_pm25_ugm3, 'f', 1) + ")");
            painter.setPen(QPen(Qt::white, 2));
        }
    }

    update();
}

QSize PpiWidget::minimumSizeHint() const {
    return QSize(400, 400);
}

QSize PpiWidget::sizeHint() const {
    return QSize(600, 600);
}

void PpiWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);

    // 背景
    painter.fillRect(rect(), Qt::black);

    // 绘制缓存的画布
    if (!canvas_.isNull()) {
        // 缩放适配
        QPixmap pixmap = QPixmap::fromImage(canvas_);
        painter.drawPixmap(rect(), pixmap);
    }

    // 标题
    painter.setPen(Qt::white);
    QFont font = painter.font();
    font.setBold(true);
    font.setPointSize(12);
    painter.setFont(font);
    painter.drawText(10, 20, title_);

    // 距离环
    painter.setPen(QPen(QColor(100, 100, 100, 150), 1, Qt::DashLine));
    int cx = width() / 2;
    int cy = height() / 2;
    int max_r = std::min(cx, cy) - 20;
    for (int i = 1; i <= 4; ++i) {
        int r = max_r * i / 4;
        painter.drawEllipse(QPoint(cx, cy), r, r);
        painter.setPen(QColor(120, 120, 120));
        painter.drawText(cx + r + 2, cy - 2,
                         QString::number(max_range_m_ * i / 4 / 1000.0, 'f', 1) + " km");
        painter.setPen(QPen(QColor(100, 100, 100, 150), 1, Qt::DashLine));
    }

    // 十字准线
    painter.drawLine(cx, 20, cx, height() - 20);
    painter.drawLine(20, cy, width() - 20, cy);
}

} // namespace lidar_client

#endif // LIDAR_ENABLE_QT
