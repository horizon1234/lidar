/**
 * @file alarm_panel.hpp
 * @brief 告警面板 Widget：告警列表 + 事件详情 + 状态机可视化。
 *
 * 显示内容：
 * - 左侧：告警事件表格（事件ID、状态、PM2.5、首次检测时间）
 * - 右侧：选中事件的详细信息（状态变更历史、质心坐标、处置状态）
 */
#pragma once

#ifdef LIDAR_ENABLE_QT

#include <QTableWidget>
#include <QTextEdit>
#include <QWidget>
#include <memory>

#include "lidar_core/lidar_core.hpp"
#include "lidar_client/hotspot_tracker.hpp"
#include "lidar_client/disposal_linkage.hpp"

namespace lidar_client {

/**
 * @brief 告警面板 Widget。
 */
class AlarmPanel : public QWidget {
    Q_OBJECT
public:
    explicit AlarmPanel(QWidget* parent = nullptr);

    /**
     * @brief 更新告警数据。
     * @param tracker 热点追踪器
     * @param linkage 处置联动
     */
    void update_data(const HotspotTracker& tracker,
                     const DisposalLinkage& linkage);

private slots:
    /**
     * @brief 表格行选择变化时，显示事件详情。
     */
    void on_event_selected(int row);

private:
    QTableWidget* table_;
    QTextEdit* detail_view_;
    std::vector<HotspotTracker::EventPtr> current_events_;
};

} // namespace lidar_client

#endif // LIDAR_ENABLE_QT
