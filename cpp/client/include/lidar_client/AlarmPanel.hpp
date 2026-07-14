/**
 * @file AlarmPanel.hpp
 * @brief 已标定热点与周期质量控制面板。
 */
#pragma once

#include <QTableWidget>
#include <QTextEdit>
#include <QWidget>

#include <vector>

#include "lidar_client/FrameProcessor.hpp"

namespace lidar_client {

/** @brief GUI 线程中的只读热点和 QC 展示控件。 */
class AlarmPanel : public QWidget {
    Q_OBJECT

public:
    explicit AlarmPanel(QWidget* parent = nullptr);

    /** @brief 用工作线程生成的不可变周期快照更新页面。 */
    void update_data(const StepResult& result);

private slots:
    /** @brief 展示当前选中热点的空间和浓度信息。 */
    void on_event_selected(int row);

private:
    QTableWidget* table_ = nullptr;              ///< 当前周期热点表格。
    QTextEdit* detail_view_ = nullptr;           ///< 热点详情和 QC 汇总区。
    std::vector<lidar_core::Hotspot> hotspots_;  ///< 当前周期已标定热点副本。
    std::vector<std::string> qc_flags_;          ///< 当前周期质量标志副本。
    bool pm_calibrated_ = false;                 ///< 当前周期是否具备定量 PM 标定。
};

} // namespace lidar_client
