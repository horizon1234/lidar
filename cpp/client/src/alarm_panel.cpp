/**
 * @file alarm_panel.cpp
 * @brief 告警面板 Widget 实现。
 */
#ifdef LIDAR_ENABLE_QT

#include "lidar_client/alarm_panel.hpp"

#include <QHeaderView>
#include <QSplitter>
#include <QVBoxLayout>
#include <sstream>

namespace lidar_client {

// =========================================================================
// AlarmPanel
// =========================================================================

AlarmPanel::AlarmPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);

    auto* splitter = new QSplitter(Qt::Vertical, this);

    // 事件表格
    table_ = new QTableWidget(this);
    table_->setColumnCount(5);
    table_->setHorizontalHeaderLabels({"事件ID", "状态", "PM2.5 (µg/m³)",
                                        "首次检测", "更新次数"});
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);

    // 详情视图
    detail_view_ = new QTextEdit(this);
    detail_view_->setReadOnly(true);
    detail_view_->setPlaceholderText("选择一个事件以查看详情...");

    splitter->addWidget(table_);
    splitter->addWidget(detail_view_);
    splitter->setSizes({250, 200});

    layout->addWidget(splitter);

    connect(table_, &QTableWidget::cellClicked, this, &AlarmPanel::on_event_selected);
}

void AlarmPanel::update_data(const HotspotTracker& tracker,
                              const DisposalLinkage& linkage) {
    // 保存当前事件列表
    current_events_ = tracker.active_events();

    // 也可以包含最近 resolved 的事件
    // 这里只显示活跃事件，保持简洁

    table_->setRowCount(static_cast<int>(current_events_.size()));

    for (int i = 0; i < static_cast<int>(current_events_.size()); ++i) {
        const auto& evt = current_events_[i];
        table_->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(evt->event_id())));
        table_->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(
                                evt->current_state_string())));
        table_->setItem(i, 2, new QTableWidgetItem(
                                QString::number(evt->peak_pm25_ugm3(), 'f', 1)));
        table_->setItem(i, 3, new QTableWidgetItem(QString::fromStdString(
                                evt->first_detected_timestamp())));
        table_->setItem(i, 4, new QTableWidgetItem(QString::number(
                                evt->update_count())));
    }
}

void AlarmPanel::on_event_selected(int row) {
    if (row < 0 || row >= static_cast<int>(current_events_.size())) {
        return;
    }

    const auto& evt = current_events_[row];

    std::ostringstream oss;
    oss << "<h2>事件 " << evt->event_id() << "</h2>";
    oss << "<p><b>当前状态:</b> " << evt->current_state_string() << "</p>";
    oss << "<p><b>峰值 PM2.5:</b> " << std::fixed << std::setprecision(1)
        << evt->peak_pm25_ugm3() << " µg/m³</p>";
    oss << "<p><b>首次检测:</b> " << evt->first_detected_timestamp() << "</p>";
    oss << "<p><b>更新次数:</b> " << evt->update_count() << "</p>";

    // 质心坐标
    oss << "<h3>质心坐标 (ENU)</h3><ul>";
    if (evt->centroid_enu_m().size() >= 2) {
        oss << "<li>East: " << evt->centroid_enu_m()[0] << " m</li>";
        oss << "<li>North: " << evt->centroid_enu_m()[1] << " m</li>";
    }
    oss << "</ul>";

    // 状态变更历史
    oss << "<h3>状态变更历史</h3>";
    auto history = evt->state_changes();
    oss << "<table border='1' cellpadding='4' style='border-collapse:collapse;'>";
    oss << "<tr><th>时间</th><th>从</th><th>到</th><th>原因</th><th>PM2.5</th></tr>";
    for (const auto& sc : history) {
        oss << "<tr>";
        oss << "<td>" << sc.timestamp << "</td>";
        oss << "<td>" << sc.from_state << "</td>";
        oss << "<td>" << sc.to_state << "</td>";
        oss << "<td>" << sc.reason << "</td>";
        oss << "<td>" << std::fixed << std::setprecision(1) << sc.peak_pm25_ugm3
            << "</td>";
        oss << "</tr>";
    }
    oss << "</table>";

    detail_view_->setHtml(QString::fromStdString(oss.str()));
}

} // namespace lidar_client

#endif // LIDAR_ENABLE_QT
