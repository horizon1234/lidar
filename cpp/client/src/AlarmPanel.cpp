/**
 * @file AlarmPanel.cpp
 * @brief 已标定热点与质量控制面板实现。
 */
#include "lidar_client/AlarmPanel.hpp"

#include <QHeaderView>
#include <QSplitter>
#include <QVBoxLayout>

namespace lidar_client {

AlarmPanel::AlarmPanel(QWidget* parent)
    : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    auto* splitter = new QSplitter(Qt::Vertical, this);
    table_ = new QTableWidget(splitter);
    table_->setColumnCount(4);
    table_->setHorizontalHeaderLabels({
        QStringLiteral("热点"),
        QStringLiteral("等级"),
        QStringLiteral("峰值 PM2.5"),
        QStringLiteral("面积")});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->verticalHeader()->setVisible(false);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    detail_view_ = new QTextEdit(splitter);
    detail_view_->setReadOnly(true);
    detail_view_->setHtml(QStringLiteral(
        "<h3>周期质量控制</h3><p>等待设备完成首个扫描周期。</p>"));
    splitter->addWidget(table_);
    splitter->addWidget(detail_view_);
    splitter->setSizes({260, 260});
    layout->addWidget(splitter);
    connect(table_, &QTableWidget::cellClicked, this, &AlarmPanel::on_event_selected);
}

void AlarmPanel::update_data(const StepResult& result) {
    hotspots_ = result.hotspots;
    qc_flags_ = result.qc_flags;
    pm_calibrated_ = result.pm_calibrated;
    table_->setRowCount(static_cast<int>(hotspots_.size()));
    for (int row = 0; row < static_cast<int>(hotspots_.size()); ++row) {
        const auto& hotspot = hotspots_[static_cast<std::size_t>(row)];
        table_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(hotspot.hotspot_id)));
        table_->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(hotspot.severity)));
        table_->setItem(row, 2, new QTableWidgetItem(QString::number(hotspot.peak_pm25_ugm3, 'f', 1)));
        table_->setItem(row, 3, new QTableWidgetItem(QString::number(hotspot.estimated_area_m2, 'f', 0) + " m²"));
    }

    QString html = QStringLiteral("<h3>周期质量控制</h3>");
    html += QStringLiteral(
        "<p>原始射线：%1<br>拒绝射线：%2<br>有效距离门：%3<br>屏蔽距离门：%4"
        "<br>平均处理：%5 ms</p>")
        .arg(result.raw_count)
        .arg(result.rejected_count)
        .arg(result.valid_bin_count)
        .arg(result.masked_bin_count)
        .arg(result.mean_processing_latency_ms, 0, 'f', 2);
    if (!pm_calibrated_) {
        html += QStringLiteral("<p><b>PM 未标定：</b>当前仅展示消光与退偏产品，不生成浓度告警。</p>");
    }
    html += QStringLiteral("<ul>");
    for (const auto& flag : qc_flags_) {
        html += QStringLiteral("<li>%1</li>").arg(QString::fromStdString(flag).toHtmlEscaped());
    }
    html += QStringLiteral("</ul>");
    detail_view_->setHtml(html);
}

void AlarmPanel::on_event_selected(int row) {
    if (row < 0 || row >= static_cast<int>(hotspots_.size())) return;
    const auto& hotspot = hotspots_[static_cast<std::size_t>(row)];
    QString coordinates = QStringLiteral("未知");
    if (hotspot.centroid_enu_m.size() >= 3) {
        coordinates = QStringLiteral("E %1 m / N %2 m / U %3 m")
            .arg(hotspot.centroid_enu_m[0], 0, 'f', 1)
            .arg(hotspot.centroid_enu_m[1], 0, 'f', 1)
            .arg(hotspot.centroid_enu_m[2], 0, 'f', 1);
    }
    detail_view_->setHtml(QStringLiteral(
        "<h3>%1</h3><p>等级：%2<br>峰值 PM2.5：%3 µg/m³<br>平均 PM2.5：%4 µg/m³"
        "<br>质心：%5<br>像元：%6</p>")
        .arg(QString::fromStdString(hotspot.hotspot_id).toHtmlEscaped())
        .arg(QString::fromStdString(hotspot.severity).toHtmlEscaped())
        .arg(hotspot.peak_pm25_ugm3, 0, 'f', 1)
        .arg(hotspot.mean_pm25_ugm3, 0, 'f', 1)
        .arg(coordinates)
        .arg(hotspot.cell_count));
}

} // namespace lidar_client
