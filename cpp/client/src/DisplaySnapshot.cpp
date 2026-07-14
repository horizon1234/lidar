/**
 * @file DisplaySnapshot.cpp
 * @brief 周期显示快照的工作线程预计算实现。
 */
#include "lidar_client/DisplaySnapshot.hpp"

#include <QColor>
#include <QPainter>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace lidar_client {

namespace {

/** @brief 选择当前方位图使用的物理产品数组。 */
const std::vector<double>& ppi_field(
    const lidar_core::ProcessedProfile& profile,
    bool display_pm) {
    return display_pm ? profile.pm25 : profile.dry_extinction;
}

bool display_bin_usable(
    const lidar_core::ProcessedProfile& profile,
    std::size_t index,
    bool display_pm) {
    if (profile.bin_quality.empty()) return true;
    if (index >= profile.bin_quality.size()) return false;
    return display_pm
        ? lidar_core::bin_is_usable_for_quantitative_product(profile.bin_quality[index])
        : lidar_core::bin_is_usable_for_retrieval(profile.bin_quality[index]);
}

/** @brief 计算正有限值的 98% 分位，抑制孤立异常点拉伸色标。 */
double percentile_98(std::vector<double> values) {
    if (values.empty()) return 1.0;
    const std::size_t index = static_cast<std::size_t>(
        std::floor(0.98 * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
    return std::max(values[index], 1e-12);
}

/** @brief 把归一化产品值映射为感知较均匀的分段色表。 */
QColor value_to_color(double normalized) {
    normalized = std::clamp(normalized, 0.0, 1.0);
    const struct Stop { double position; int red; int green; int blue; } stops[] = {
        {0.00, 18, 32, 47},
        {0.20, 24, 92, 118},
        {0.45, 32, 153, 138},
        {0.68, 224, 187, 73},
        {0.84, 229, 108, 52},
        {1.00, 176, 43, 55},
    };
    constexpr std::size_t StopCount = sizeof(stops) / sizeof(stops[0]);
    for (std::size_t index = 1; index < StopCount; ++index) {
        if (normalized <= stops[index].position) {
            const auto& left = stops[index - 1];
            const auto& right = stops[index];
            const double ratio = (normalized - left.position)
                / (right.position - left.position);
            return QColor(
                static_cast<int>(left.red + ratio * (right.red - left.red)),
                static_cast<int>(left.green + ratio * (right.green - left.green)),
                static_cast<int>(left.blue + ratio * (right.blue - left.blue)));
        }
    }
    return QColor(176, 43, 55);
}

/** @brief 从当前周期选择最近一条垂直观测。 */
void select_vertical_profile(const StepResult& result, DisplaySnapshot& snapshot) {
    for (auto iterator = result.processed_profiles.rbegin();
         iterator != result.processed_profiles.rend(); ++iterator) {
        if (iterator->profile.scan_mode != "stare") continue;
        const std::size_t count = std::min(
            iterator->profile.ranges_m.size(), iterator->dry_extinction.size());
        snapshot.vertical_heights_m.reserve(count);
        snapshot.vertical_dry_extinction.assign(
            iterator->dry_extinction.begin(), iterator->dry_extinction.begin() + count);
        const double elevation = iterator->profile.elevation_deg * std::acos(-1.0) / 180.0;
        for (std::size_t index = 0; index < count; ++index) {
            if (!display_bin_usable(*iterator, index, false)) {
                snapshot.vertical_dry_extinction[index] =
                    std::numeric_limits<double>::quiet_NaN();
            }
            if (index < iterator->enu_points_m.size()
                && iterator->enu_points_m[index].size() >= 3) {
                snapshot.vertical_heights_m.push_back(iterator->enu_points_m[index][2]);
            } else {
                snapshot.vertical_heights_m.push_back(
                    iterator->profile.ranges_m[index] * std::sin(elevation));
            }
        }
        if (iterator->profile.depolarization_ratio.size() >= count) {
            snapshot.vertical_depolarization.assign(
                iterator->profile.depolarization_ratio.begin(),
                iterator->profile.depolarization_ratio.begin() + count);
            for (std::size_t index = 0; index < count; ++index) {
                if (!display_bin_usable(*iterator, index, false)) {
                    snapshot.vertical_depolarization[index] =
                        std::numeric_limits<double>::quiet_NaN();
                }
            }
        }
        return;
    }
}

} // namespace

DisplaySnapshot build_display_snapshot(const StepResult& result, double ppi_max_range_m) {
    constexpr int CanvasSize = 720;
    DisplaySnapshot snapshot;
    snapshot.ppi_max_range_m = std::max(ppi_max_range_m, 100.0);
    snapshot.hotspots = result.hotspots;
    const bool display_pm = result.pm_calibrated;
    snapshot.ppi_title = display_pm
        ? QStringLiteral("方位扫描 - 已标定 PM2.5")
        : QStringLiteral("方位扫描 - 干消光（PM 未标定）");
    snapshot.ppi_field_label = display_pm
        ? QStringLiteral("PM2.5 (µg/m³)")
        : QStringLiteral("干消光 (km⁻¹)");

    std::vector<double> finite_values;
    for (const auto& profile : result.processed_profiles) {
        if (profile.profile.scan_mode != "ppi") continue;
        ++snapshot.ppi_ray_count;
        const auto& values = ppi_field(profile, display_pm);
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (!display_bin_usable(profile, index, display_pm)
                || !std::isfinite(values[index])) {
                ++snapshot.ppi_masked_bin_count;
                continue;
            }
            ++snapshot.ppi_valid_bin_count;
            if (values[index] > 0.0) finite_values.push_back(values[index]);
        }
    }
    snapshot.ppi_color_max = percentile_98(std::move(finite_values));
    snapshot.ppi_image = QImage(
        CanvasSize, CanvasSize, QImage::Format_ARGB32_Premultiplied);
    snapshot.ppi_image.fill(QColor(12, 20, 28));

    const double center = CanvasSize * 0.5;
    const double scale = (CanvasSize * 0.5 - 18.0) / snapshot.ppi_max_range_m;
    QPainter painter(&snapshot.ppi_image);
    painter.setRenderHint(QPainter::Antialiasing, false);
    for (const auto& profile : result.processed_profiles) {
        if (profile.profile.scan_mode != "ppi") continue;
        const auto& values = ppi_field(profile, display_pm);
        const std::size_t count = std::min(values.size(), profile.enu_points_m.size());
        for (std::size_t index = 0; index < count; ++index) {
            if (!display_bin_usable(profile, index, display_pm)
                || !std::isfinite(values[index])
                || profile.enu_points_m[index].size() < 2) continue;
            const double east = profile.enu_points_m[index][0];
            const double north = profile.enu_points_m[index][1];
            if (std::hypot(east, north) > snapshot.ppi_max_range_m) continue;
            painter.fillRect(
                QRectF(center + east * scale - 1.7, center - north * scale - 1.7, 3.4, 3.4),
                value_to_color(values[index] / snapshot.ppi_color_max));
        }
    }
    painter.end();
    select_vertical_profile(result, snapshot);
    return snapshot;
}

} // namespace lidar_client
