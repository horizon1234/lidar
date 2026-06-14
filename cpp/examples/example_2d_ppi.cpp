// cpp/examples/example_2d_ppi.cpp
//
// 第 19.17-19.27 节二维 PPI 算例: 水平扫描面 + 连通域热点 + 喷雾目标角。
// 对应《激光颗粒物监测系统入门手册》第 19.17-19.27 节。
//
// 数据来源: 直接复用文档 19.18 节的 PM2.5 矩阵 (已经做完反演湿度修正
// 之后的结果), 这样学习者对照文档表格读代码最直接。
//
// 完整步骤:
//   1. 二维 PM2.5 矩阵: 3 方位 × 4 距离 = 12 cell
//   2. 阈值分割 -> 二值 mask
//   3. 8 邻域连通域分析 (BFS) -> 同一热点判定
//   4. 每个 cell 转 ENU 三维坐标
//   5. PM2.5 加权质心
//   6. 质心 -> 喷雾目标方位角 + 仰角
//   7. 形成热点事件 + ASCII 伪热力图打印
//   8. 写 JSON 结果到 data/examples/2d_ppi_result.json

#include "example_common.hpp"

#include <cstdio>
#include <deque>
#include <string>
#include <vector>

using namespace example_common;

namespace {

constexpr double ELEVATION_DEG = 10.0;
constexpr double PM25_THRESHOLD = 60.0;

struct PpiMatrix {
    std::vector<double> azimuths_deg;
    std::vector<double> ranges_m;
    // pm25[i][j] = 第 i 个方位角, 第 j 个距离的 PM2.5
    std::vector<std::vector<double>> pm25;
};

PpiMatrix load_ppi_matrix() {
    // 文档 19.18 节
    PpiMatrix m;
    m.azimuths_deg = {38.0, 40.0, 42.0};
    m.ranges_m     = {60.0, 90.0, 120.0, 150.0};
    m.pm25 = {
        {34.0, 46.0, 78.0, 58.0},   // 38 deg
        {37.0, 51.0, 82.0, 66.0},   // 40 deg
        {35.0, 49.0, 79.0, 63.0},   // 42 deg
    };
    return m;
}

std::vector<std::vector<int>> threshold_matrix(const PpiMatrix& m, double threshold) {
    std::vector<std::vector<int>> mask(m.azimuths_deg.size(),
                                       std::vector<int>(m.ranges_m.size(), 0));
    for (std::size_t i = 0; i < m.azimuths_deg.size(); ++i) {
        for (std::size_t j = 0; j < m.ranges_m.size(); ++j) {
            mask[i][j] = (m.pm25[i][j] > threshold) ? 1 : 0;
        }
    }
    return mask;
}

// 8 邻域连通域分析, 返回每个域的 cell 列表
std::vector<std::vector<std::pair<int, int>>> find_components(const std::vector<std::vector<int>>& mask) {
    int na = static_cast<int>(mask.size());
    int nr = static_cast<int>(mask[0].size());
    std::vector<std::vector<char>> visited(na, std::vector<char>(nr, 0));
    std::vector<std::vector<std::pair<int, int>>> components;

    int dz_arr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dr_arr[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    for (int az = 0; az < na; ++az) {
        for (int r = 0; r < nr; ++r) {
            if (mask[az][r] == 0 || visited[az][r]) continue;
            std::deque<std::pair<int, int>> q;
            q.push_back({az, r});
            visited[az][r] = 1;
            std::vector<std::pair<int, int>> cells;
            while (!q.empty()) {
                auto [ca, cr] = q.front();
                q.pop_front();
                cells.push_back({ca, cr});
                for (int k = 0; k < 8; ++k) {
                    int na_idx = ca + dz_arr[k];
                    int nr_idx = cr + dr_arr[k];
                    if (na_idx < 0 || na_idx >= na || nr_idx < 0 || nr_idx >= nr) continue;
                    if (mask[na_idx][nr_idx] == 0 || visited[na_idx][nr_idx]) continue;
                    visited[na_idx][nr_idx] = 1;
                    q.push_back({na_idx, nr_idx});
                }
            }
            components.push_back(cells);
        }
    }
    return components;
}

struct Centroid3D { double x, y, z; };

Centroid3D weighted_centroid(const std::vector<std::pair<int, int>>& cells,
                             const PpiMatrix& m, double elevation_deg) {
    double sx = 0, sy = 0, sz = 0, sw = 0;
    for (auto [a, r] : cells) {
        double pm = m.pm25[a][r];
        double az_deg = m.azimuths_deg[a];
        double range_m = m.ranges_m[r];
        double east, north, up;
        polar_to_enu(range_m, az_deg, elevation_deg, east, north, up);
        sx += pm * east;
        sy += pm * north;
        sz += pm * up;
        sw += pm;
    }
    if (sw <= 0) return {0, 0, 0};
    return {sx / sw, sy / sw, sz / sw};
}

struct TargetAngles { double az, el, horizontal; };

TargetAngles centroid_to_target(const Centroid3D& c) {
    double horizontal = std::sqrt(c.x * c.x + c.y * c.y);
    double az = std::atan2(c.x, c.y) * 180.0 / PI;
    double el = (horizontal > 1e-6) ? std::atan2(c.z, horizontal) * 180.0 / PI : 0.0;
    return {az, el, horizontal};
}

void render_ascii(const PpiMatrix& m, const std::vector<std::vector<int>>& mask) {
    printf("az\\R |");
    for (double r : m.ranges_m) printf(" %4dm |", static_cast<int>(r));
    printf("\n");
    printf("------");
    for (std::size_t j = 0; j < m.ranges_m.size(); ++j) printf("--------");
    printf("\n");
    for (std::size_t i = 0; i < m.azimuths_deg.size(); ++i) {
        printf("%3.0f  |", m.azimuths_deg[i]);
        for (std::size_t j = 0; j < m.ranges_m.size(); ++j) {
            char tag = 'L';
            if (m.pm25[i][j] >= 50.0) tag = 'M';
            if (mask[i][j] == 1)      tag = 'H';
            printf(" %3.0f%c |", m.pm25[i][j], tag);
        }
        printf("\n");
    }
}

void run_2d_ppi_demo() {
    print_header("2D PPI example (sections 19.17-19.27)");
    printf("    fixed elevation = %.1f deg\n", ELEVATION_DEG);

    PpiMatrix m = load_ppi_matrix();
    printf("\n[input] PM2.5 matrix shape = (%zu, %zu)\n",
           m.azimuths_deg.size(), m.ranges_m.size());
    std::vector<std::vector<int>> mask_empty(m.azimuths_deg.size(),
                                             std::vector<int>(m.ranges_m.size(), 0));
    render_ascii(m, mask_empty);

    // 2. 阈值
    auto mask = threshold_matrix(m, PM25_THRESHOLD);
    int hot_count = 0;
    for (auto& row : mask) for (int v : row) hot_count += v;
    printf("\n[2] threshold split (PM2.5 > %.0f) -> %d cells\n", PM25_THRESHOLD, hot_count);
    render_ascii(m, mask);

    // 3. 连通域
    auto components = find_components(mask);
    printf("\n[3] 8-neighbour connected components -> %zu comp(s)\n", components.size());
    for (std::size_t k = 0; k < components.size(); ++k) {
        printf("    component #%zu (%zu cells):", k, components[k].size());
        for (auto [a, r] : components[k]) {
            printf(" (az=%.0f, R=%.0f, PM=%.0f)", m.azimuths_deg[a], m.ranges_m[r], m.pm25[a][r]);
        }
        printf("\n");
    }
    std::vector<std::vector<std::pair<int, int>>> valid;
    for (auto& c : components) if (c.size() >= 3) valid.push_back(c);
    printf("    min_cells=3 -> %zu valid hotspot(s)\n", valid.size());

    // 4-7. 每个有效热点 -> 事件
    for (std::size_t idx = 0; idx < valid.size(); ++idx) {
        auto& cells = valid[idx];
        Centroid3D c = weighted_centroid(cells, m, ELEVATION_DEG);
        TargetAngles t = centroid_to_target(c);

        // peak / mean / area
        double peak = 0;
        double sum = 0;
        double min_up = 1e9, max_up = -1e9;
        for (auto [a, r] : cells) {
            peak = std::max(peak, m.pm25[a][r]);
            sum += m.pm25[a][r];
            double east, north, up;
            polar_to_enu(m.ranges_m[r], m.azimuths_deg[a], ELEVATION_DEG, east, north, up);
            min_up = std::min(min_up, up);
            max_up = std::max(max_up, up);
        }
        double mean = sum / static_cast<double>(cells.size());

        // 面积近似: 圆环扇区 (azimuth_step * range_step)
        double az_step_deg = (m.azimuths_deg.size() > 1)
            ? (m.azimuths_deg[1] - m.azimuths_deg[0]) : 1.0;
        double range_step_m = (m.ranges_m.size() > 1)
            ? (m.ranges_m[1] - m.ranges_m[0]) : 30.0;
        double arc_len = (az_step_deg / 360.0) * 2.0 * PI * t.horizontal;
        double area = arc_len * range_step_m * static_cast<double>(cells.size());

        std::string action = (mean > 60.0) ? "spray" : "monitor";
        printf("\n[hotspot #%zu] target az=%.2f deg el=%.2f deg peak=%.1f action=%s\n",
               idx, t.az, t.el, peak, action.c_str());
        printf("    centroid ENU = (%.2f, %.2f, %.2f) m  area=%.1f m^2  cells=%zu\n",
               c.x, c.y, c.z, area, cells.size());
    }

    // 写 JSON
    std::vector<JsonValue> azimuths_json;
    for (double v : m.azimuths_deg) azimuths_json.push_back(JsonValue::number(v));
    std::vector<JsonValue> ranges_json;
    for (double v : m.ranges_m) ranges_json.push_back(JsonValue::number(v));

    std::vector<JsonValue> pm25_rows;
    for (auto& row : m.pm25) {
        std::vector<JsonValue> rj;
        for (double v : row) rj.push_back(JsonValue::number(v));
        pm25_rows.push_back(JsonValue::array(rj));
    }
    std::vector<JsonValue> mask_rows;
    for (auto& row : mask) {
        std::vector<JsonValue> rj;
        for (int v : row) rj.push_back(JsonValue::integer(v));
        mask_rows.push_back(JsonValue::array(rj));
    }
    std::vector<JsonValue> components_json;
    for (auto& comp : components) {
        std::vector<JsonValue> cells_json;
        for (auto [a, r] : comp) {
            cells_json.push_back(JsonValue::array({
                JsonValue::integer(a), JsonValue::integer(r)
            }));
        }
        components_json.push_back(JsonValue::array(cells_json));
    }
    std::vector<JsonValue> hotspot_events;
    for (std::size_t idx = 0; idx < valid.size(); ++idx) {
        auto& cells = valid[idx];
        Centroid3D c = weighted_centroid(cells, m, ELEVATION_DEG);
        TargetAngles t = centroid_to_target(c);
        double peak = 0; double sum = 0;
        for (auto [a, r] : cells) { peak = std::max(peak, m.pm25[a][r]); sum += m.pm25[a][r]; }
        double mean = sum / static_cast<double>(cells.size());
        double az_step = (m.azimuths_deg.size() > 1)
            ? (m.azimuths_deg[1] - m.azimuths_deg[0]) : 1.0;
        double range_step = (m.ranges_m.size() > 1)
            ? (m.ranges_m[1] - m.ranges_m[0]) : 30.0;
        double arc = (az_step / 360.0) * 2.0 * PI * t.horizontal;
        double area = arc * range_step * static_cast<double>(cells.size());
        std::vector<JsonValue> cells_pm_json;
        for (auto [a, r] : cells) {
            cells_pm_json.push_back(JsonValue::object({
                {"azimuth_deg", JsonValue::number(m.azimuths_deg[a])},
                {"range_m",     JsonValue::number(m.ranges_m[r])},
                {"pm25",        JsonValue::number(m.pm25[a][r])}
            }));
        }
        hotspot_events.push_back(JsonValue::object({
            {"event_id",          JsonValue::string("dust_ppi_" + std::to_string(idx))},
            {"cell_count",        JsonValue::integer(static_cast<int>(cells.size()))},
            {"centroid_enu_m",    JsonValue::array({
                JsonValue::number(round6(c.x)),
                JsonValue::number(round6(c.y)),
                JsonValue::number(round6(c.z))
            })},
            {"target_azimuth_deg",   JsonValue::number(round6(t.az))},
            {"target_elevation_deg", JsonValue::number(round6(t.el))},
            {"peak_pm25_ugm3",       JsonValue::number(round6(peak))},
            {"mean_pm25_ugm3",       JsonValue::number(round6(mean))},
            {"estimated_area_m2",    JsonValue::number(round6(area))},
            {"recommended_action",   JsonValue::string(mean > 60.0 ? "spray" : "monitor")},
            {"cells",                JsonValue::array(cells_pm_json)}
        }));
    }

    JsonValue root = JsonValue::object({
        {"scan_mode",       JsonValue::string("ppi")},
        {"elevation_deg",   JsonValue::number(ELEVATION_DEG)},
        {"azimuths_deg",    JsonValue::array(azimuths_json)},
        {"ranges_m",        JsonValue::array(ranges_json)},
        {"pm25_matrix",     JsonValue::array(pm25_rows)},
        {"threshold_ugm3",  JsonValue::number(PM25_THRESHOLD)},
        {"binary_mask",     JsonValue::array(mask_rows)},
        {"components",      JsonValue::array(components_json)},
        {"hotspots",        JsonValue::array(hotspot_events)}
    });

    std::filesystem::path out = std::filesystem::current_path() / "data" / "examples" / "2d_ppi_result.json";
    write_json_file(out, root);
    std::cout << "\n[OK] JSON written to " << out.string() << "\n";
}

}  // namespace

int main() {
    enable_ansi_color();
    try {
        run_2d_ppi_demo();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
    return 0;
}
