/**
 * @file VerticalProfileWidget.hpp
 * @brief 垂直观测的消光和退偏比廓线视图。
 */
#pragma once

#include <QWidget>

#include <vector>

#include "lidar_client/DisplaySnapshot.hpp"

namespace lidar_client {

/** @brief 显示最近一条天顶观测的高度廓线。 */
class VerticalProfileWidget : public QWidget {
    Q_OBJECT

public:
    explicit VerticalProfileWidget(QWidget* parent = nullptr);

    /** @brief 接收工作线程选择并整理好的垂直廓线。 */
    void set_snapshot(const DisplaySnapshot& snapshot);

    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    std::vector<double> heights_m_;       ///< 相对设备的观测高度（米）。
    std::vector<double> dry_extinction_;  ///< 干态气溶胶消光廓线（每千米）。
    std::vector<double> depolarization_;  ///< 体退偏比廓线。
};

} // namespace lidar_client
