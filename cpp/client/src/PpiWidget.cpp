/**
 * @file PpiWidget.cpp
 * @brief 方位扫描产品视图实现。
 */
#include "lidar_client/PpiWidget.hpp"

#include <QPaintEvent>
#include <QPainter>

namespace lidar_client {

PpiWidget::PpiWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(520, 420);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void PpiWidget::set_snapshot(const DisplaySnapshot& snapshot) {
    canvas_ = snapshot.ppi_image;
    title_ = snapshot.ppi_title;
    field_label_ = snapshot.ppi_field_label;
    hotspots_ = snapshot.hotspots;
    max_range_m_ = snapshot.ppi_max_range_m;
    color_max_ = snapshot.ppi_color_max;
    ray_count_ = snapshot.ppi_ray_count;
    update();
}

QSize PpiWidget::minimumSizeHint() const {
    return QSize(520, 420);
}

QSize PpiWidget::sizeHint() const {
    return QSize(760, 640);
}

void PpiWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.fillRect(rect(), QColor(247, 249, 250));
    const int top = 42;
    const int bottom = 32;
    const int side = 16;
    const QRect target(side, top, width() - side * 2, height() - top - bottom);
    if (!canvas_.isNull()) {
        const QPixmap image = QPixmap::fromImage(canvas_).scaled(
            target.size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        const QPoint origin(
            target.x() + (target.width() - image.width()) / 2,
            target.y() + (target.height() - image.height()) / 2);
        painter.drawPixmap(origin, image);

        const double scale = static_cast<double>(image.width()) / 720.0;
        const QPointF center(origin.x() + image.width() * 0.5, origin.y() + image.height() * 0.5);
        painter.setPen(QPen(QColor(190, 202, 210), 1));
        for (double fraction : {0.25, 0.5, 0.75, 1.0}) {
            painter.drawEllipse(center, 342.0 * scale * fraction, 342.0 * scale * fraction);
        }
        painter.drawLine(QPointF(center.x(), origin.y()), QPointF(center.x(), origin.y() + image.height()));
        painter.drawLine(QPointF(origin.x(), center.y()), QPointF(origin.x() + image.width(), center.y()));

        painter.setPen(QPen(QColor(176, 43, 55), 2));
        for (const auto& hotspot : hotspots_) {
            if (hotspot.centroid_enu_m.size() < 2) continue;
            const double pixel_per_m = 342.0 * scale / max_range_m_;
            const QPointF point(
                center.x() + hotspot.centroid_enu_m[0] * pixel_per_m,
                center.y() - hotspot.centroid_enu_m[1] * pixel_per_m);
            painter.drawEllipse(point, 8, 8);
        }
    }

    painter.setPen(QColor(31, 45, 55));
    QFont title_font = painter.font();
    title_font.setBold(true);
    title_font.setPointSize(11);
    painter.setFont(title_font);
    painter.drawText(QRect(16, 8, width() - 32, 24), Qt::AlignLeft | Qt::AlignVCenter, title_);
    painter.setFont(QFont());
    painter.setPen(QColor(85, 99, 108));
    painter.drawText(
        QRect(16, height() - 26, width() - 32, 20),
        Qt::AlignLeft | Qt::AlignVCenter,
        QStringLiteral("%1  |  %2 条射线  |  色标上限 %3")
            .arg(field_label_)
            .arg(ray_count_)
            .arg(color_max_, 0, 'g', 4));
}

} // namespace lidar_client
