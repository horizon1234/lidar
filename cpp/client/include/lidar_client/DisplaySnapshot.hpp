/**
 * @file DisplaySnapshot.hpp
 * @brief 工作线程生成、GUI 线程只读消费的周期显示快照。
 */
#pragma once

#include <QImage>
#include <QMetaType>
#include <QString>

#include <memory>
#include <vector>

#include "lidar_client/FrameProcessor.hpp"

namespace lidar_client {

/** @brief 一个扫描周期预计算后的 PPI 和垂直廓线显示数据。 */
struct DisplaySnapshot {
    QImage ppi_image;                              ///< 工作线程生成的方位扫描栅格。
    QString ppi_title;                             ///< 当前方位图标题。
    QString ppi_field_label;                       ///< 当前色标物理量及单位。
    std::vector<lidar_core::Hotspot> hotspots;    ///< 已标定 PM 热点叠加数据。
    double ppi_max_range_m = 5000.0;              ///< 方位视图显示半径（米）。
    double ppi_color_max = 0.0;                   ///< 方位色标 98% 分位上限。
    int ppi_ray_count = 0;                        ///< 参与栅格化的方位射线数量。
    int ppi_valid_bin_count = 0;                  ///< 实际参与绘制的有效距离门数量。
    int ppi_masked_bin_count = 0;                 ///< 被质量控制排除的距离门数量。
    std::vector<double> vertical_heights_m;       ///< 最近垂直廓线的相对高度（米）。
    std::vector<double> vertical_dry_extinction;  ///< 最近垂直廓线的干消光。
    std::vector<double> vertical_depolarization;  ///< 最近垂直廓线的体退偏比。
};

using DisplaySnapshotPtr = std::shared_ptr<const DisplaySnapshot>;

/** @brief 在工作线程把科学产品转换为轻量 GUI 显示快照。 */
DisplaySnapshot build_display_snapshot(
    const StepResult& result,
    double ppi_max_range_m = 5000.0);

} // namespace lidar_client

Q_DECLARE_METATYPE(lidar_client::DisplaySnapshotPtr)
