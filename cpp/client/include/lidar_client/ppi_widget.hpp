/**
 * @file ppi_widget.hpp
 * @brief PPI 热力图 Widget：用 QPainter 将消光/PM 数据渲染为 2D 热力图。
 *
 * 数据来源：一个时间步内多条 PPI 射线的处理结果（极坐标 → 屏幕像素）。
 * 颜色映射：低值=蓝/绿，高值=黄/红（jet 或 turbo 色表）。
 */
#pragma once

#ifdef LIDAR_ENABLE_QT

#include <QImage>
#include <QWidget>
#include <vector>

#include "lidar_core/lidar_core.hpp"

namespace lidar_client {

/**
 * @brief PPI 热力图显示 Widget。
 */
class PpiWidget : public QWidget {
    Q_OBJECT
public:
    explicit PpiWidget(QWidget* parent = nullptr);

    /**
     * @brief 设置当前时间步的处理结果。
     * @param profiles 所有 PPI 射线（已处理）
     * @param field 要显示的数据字段："pm25", "extinction", "dry_extinction"
     * @param max_range_m 最大显示距离（米）
     */
    void set_data(const std::vector<lidar_core::ProcessedProfile>& profiles,
                  const std::string& field = "pm25",
                  double max_range_m = 5000.0);

    /**
     * @brief 叠加显示热点。
     */
    void set_hotspots(const std::vector<lidar_core::Hotspot>& hotspots);

    /**
     * @brief 设置标题。
     */
    void set_title(const QString& title);

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage canvas_;
    QString title_ = "PPI Heatmap";
    std::vector<lidar_core::Hotspot> hotspots_;
    double max_range_m_ = 5000.0;

    /**
     * @brief 将一个数值映射为颜色（turbo 色表）。
     */
    static QColor value_to_color(double normalized);
};

} // namespace lidar_client

#endif // LIDAR_ENABLE_QT
