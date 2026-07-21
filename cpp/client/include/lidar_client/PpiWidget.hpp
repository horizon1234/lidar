/**
 * @file PpiWidget.hpp
 * @brief 水平/锥形扫描的极坐标产品视图。
 */
#pragma once

#include <QImage>
#include <QWidget>

#include <vector>

#include "lidar_client/DisplaySnapshot.hpp"

namespace lidar_client {

/** @brief 用 ENU 投影显示方位扫描回波、消光或已标定 PM。 */
class PpiWidget : public QWidget {
    Q_OBJECT

public:
    explicit PpiWidget(QWidget* parent = nullptr);

    /** @brief 接收工作线程预计算的方位栅格和色标元数据。 */
    void set_snapshot(const DisplaySnapshot& snapshot);

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage canvas_;                            ///< 当前周期预渲染的极坐标图像。
    QString title_ = QStringLiteral("方位扫描"); ///< 当前图层标题。
    QString field_label_ = QStringLiteral("干气溶胶消光"); ///< 当前显示物理量名称。
    std::vector<lidar_core::Hotspot> hotspots_; ///< 当前已标定热点叠加层。
    double max_range_m_ = 5000.0;             ///< 当前显示半径（米）。
    double color_max_ = 0.0;                  ///< 当前色标 98% 分位上限。
    int ray_count_ = 0;                       ///< 当前参与绘制的方位射线数。
    int valid_bin_count_ = 0;                 ///< 当前有效距离门数量。
    int masked_bin_count_ = 0;                ///< 当前被屏蔽距离门数量。
};

} // namespace lidar_client
