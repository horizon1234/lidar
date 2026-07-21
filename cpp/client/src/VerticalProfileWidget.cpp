/**
 * @file VerticalProfileWidget.cpp
 * @brief 垂直观测廓线视图实现。
 */
#include "lidar_client/VerticalProfileWidget.hpp"

#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace lidar_client {

VerticalProfileWidget::VerticalProfileWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(520, 420);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void VerticalProfileWidget::set_snapshot(const DisplaySnapshot& snapshot) {
    heights_m_ = snapshot.vertical_heights_m;
    dry_extinction_ = snapshot.vertical_dry_extinction;
    depolarization_ = snapshot.vertical_depolarization;
    update();
}

QSize VerticalProfileWidget::minimumSizeHint() const {
    return QSize(520, 420);
}

void VerticalProfileWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(247, 249, 250));
    painter.setPen(QColor(31, 45, 55));
    QFont title = painter.font();
    title.setBold(true);
    title.setPointSize(11);
    painter.setFont(title);
    painter.drawText(QRect(18, 10, width() - 36, 24), Qt::AlignLeft, QStringLiteral("垂直观测廓线"));

    const QRectF plot(72, 48, std::max(width() - 118, 1), std::max(height() - 92, 1));
    painter.setPen(QPen(QColor(170, 181, 188), 1));
    painter.drawRect(plot);
    if (heights_m_.empty()) {
        painter.setPen(QColor(110, 121, 128));
        painter.drawText(plot, Qt::AlignCenter, QStringLiteral("等待垂直观测"));
        return;
    }

    const double max_height = std::max(heights_m_.back(), 1.0);
    double max_extinction = 1e-9;
    for (double value : dry_extinction_) {
        if (std::isfinite(value)) max_extinction = std::max(max_extinction, value);
    }
    double max_depolarization = 0.01;
    for (double value : depolarization_) {
        if (std::isfinite(value)) max_depolarization = std::max(max_depolarization, value);
    }

    QPainterPath extinction_path;
    QPainterPath depolarization_path;
    bool extinction_started = false;
    bool depolarization_started = false;
    for (std::size_t index = 0; index < heights_m_.size(); ++index) {
        const double y = plot.bottom() - heights_m_[index] / max_height * plot.height();
        if (index < dry_extinction_.size() && std::isfinite(dry_extinction_[index])) {
            const double x_ext = plot.left()
                + std::clamp(dry_extinction_[index] / max_extinction, 0.0, 1.0)
                    * plot.width();
            if (!extinction_started) extinction_path.moveTo(x_ext, y);
            else extinction_path.lineTo(x_ext, y);
            extinction_started = true;
        } else {
            extinction_started = false;
        }
        if (index < depolarization_.size() && std::isfinite(depolarization_[index])) {
            const double x_dep = plot.left() + std::clamp(depolarization_[index] / max_depolarization, 0.0, 1.0) * plot.width();
            if (!depolarization_started) depolarization_path.moveTo(x_dep, y);
            else depolarization_path.lineTo(x_dep, y);
            depolarization_started = true;
        } else {
            depolarization_started = false;
        }
    }
    painter.setPen(QPen(QColor(24, 128, 146), 2));
    painter.drawPath(extinction_path);
    painter.setPen(QPen(QColor(211, 112, 49), 2, Qt::DashLine));
    painter.drawPath(depolarization_path);

    painter.setPen(QColor(85, 99, 108));
    painter.setFont(QFont());
    painter.drawText(QRectF(8, plot.top(), 58, 20), Qt::AlignRight, QString::number(max_height / 1000.0, 'f', 1) + " km");
    painter.drawText(QRectF(8, plot.bottom() - 18, 58, 20), Qt::AlignRight, QStringLiteral("0 km"));
    painter.drawText(QRectF(plot.left(), plot.bottom() + 8, plot.width(), 20), Qt::AlignLeft,
                     QStringLiteral("干气溶胶消光  |  -- 退偏比"));
}

} // namespace lidar_client
