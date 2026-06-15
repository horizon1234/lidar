/**
 * @file example_2d_ppi.cpp
 * @brief 《激光颗粒物监测系统入门手册》第 19.17–19.27 节 二维 PPI 算例。
 *
 * PPI (Plan-Position Indicator): 固定仰角, 方位角扫一圈, 形成"水平扫描面"。
 * 与 RHI (固定方位角、扫仰角) 互补, 两者叠加可以重建三维污染场。
 *
 * 本例直接复用手册 §19.18 节给出的 PM2.5 矩阵 (3 方位 × 4 距离 = 12 cell),
 * 即假设反演 + 湿度修正已完成; 学习重点放在 **后端处理链**:
 *   - §19.19 阈值分割               mask[i][j] = (PM2.5 > 60) ? 1 : 0
 *   - §19.20 8 邻域连通域分析 (BFS)  把相邻 hot cell 聚合成同一热点团块
 *   - §19.21 每个 cell 转 ENU 三维   (E, N, U) = polar_to_enu(R, az, el)
 *   - §19.22 PM2.5 加权质心          C = Σ w·r / Σ w
 *   - §19.23 质心 -> 喷雾目标方位/仰角 az = atan2(E, N), el = atan2(U, √(E²+N²))
 *   - §19.24–25 热点事件 + 面积估算 (圆环扇区)
 *   - §19.26 ASCII 伪热力图打印
 *   - §19.27 写 JSON 结果到 data/examples/2d_ppi_result.json
 *
 * 编译运行:
 * @code
 *   cmake --build build --target lidar_example_2d_ppi
 *   build/lidar_example_2d_ppi
 * @endcode
 */

#include "example_common.hpp"

#include <cstdio>
#include <deque>
#include <string>
#include <vector>

using namespace example_common;

namespace {

constexpr double ELEVATION_DEG = 10.0;   ///< PPI 扫描的固定仰角 (度)
constexpr double PM25_THRESHOLD = 60.0;  ///< 热点阈值 (μg/m³)

/**
 * @brief PPI 二维扫描矩阵。
 *
 * `pm25[i][j]` 表示第 i 个方位角、第 j 个距离 bin 的 PM2.5 (μg/m³)。
 */
struct PpiMatrix {
    std::vector<double> azimuths_deg;   ///< 方位角序列 (度, 北顺时针)
    std::vector<double> ranges_m;       ///< 距离 bin 中心点序列 (m)
    /// pm25[i][j] = 第 i 个方位角, 第 j 个距离的 PM2.5
    std::vector<std::vector<double>> pm25;
};

/**
 * @brief 装入手册 §19.18 的示例 PM2.5 矩阵 (3 方位 × 4 距离)。
 *
 * 该矩阵体现"中等污染团块": 120 m 处浓度最高 (72–82 μg/m³),
 * 这正是后续阈值分割后能聚成连通热点团块的位置。
 *
 * @return PpiMatrix (不需要参数, 是手册内嵌的示例数据)
 */
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

/**
 * @brief §19.19 阈值分割: 把 PM2.5 矩阵二值化。
 *
 * PM2.5 > threshold 视为热点 cell (mask=1), 否则为背景 (mask=0)。
 * 这是后续连通域分析的输入。
 *
 * @param m         PM2.5 矩阵
 * @param threshold 阈值 (μg/m³)
 * @return 与 m 同 shape 的二值 mask
 */
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

/**
 * @brief §19.20 用 8 邻域 BFS 做连通域分析。
 *
 * 把 mask 上所有标记为 1 的 cell 按 8 邻接 (水平/垂直/对角) 聚合,
 * 同一连通域视为一个独立的污染团块 (热点)。
 *
 * 算法:
 *   - 对每个未访问的 hot cell, 用 deque 做广度优先搜索
 *   - 8 个邻居方向 (dz, dr) 覆盖上下左右与四个对角
 *   - 所有访问过且 mask=1 的 cell 一起加入同一连通域
 *
 * @param mask 二值 mask (方位 × 距离)
 * @return 连通域列表, 每个域是一个 (az_idx, range_idx) cell 序列
 */
std::vector<std::vector<std::pair<int, int>>> find_components(const std::vector<std::vector<int>>& mask) {
    int na = static_cast<int>(mask.size());
    int nr = static_cast<int>(mask[0].size());
    std::vector<std::vector<char>> visited(na, std::vector<char>(nr, 0));
    std::vector<std::vector<std::pair<int, int>>> components;

    // 8 邻域方向: (dz 方位, dr 距离) 覆盖 8 个相邻 bin
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
                // 探测 8 个邻居
                for (int k = 0; k < 8; ++k) {
                    int na_idx = ca + dz_arr[k];
                    int nr_idx = cr + dr_arr[k];
                    // 越界 / 背景 / 已访问的均跳过
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

/**
 * @brief 三维加权质心 (E, N, U)。
 */
struct Centroid3D { double x, y, z; };

/**
 * @brief §19.22 对一个热点连通域求 PM2.5 加权质心。
 *
 * 算法:
 *   - 每个 cell 用 polar_to_enu 转 (E, N, U)
 *   - 加权累加, 权重 w = PM2.5 (浓度越高的 bin 对质心贡献越大)
 *   - C = (Σw·E, Σw·N, Σw·U) / Σw
 *
 * @param cells         连通域的所有 cell 索引
 * @param m             PM2.5 矩阵 (取坐标与权重)
 * @param elevation_deg PPI 固定仰角
 * @return 加权质心 (E, N, U) 米; 若权重和为 0 返回原点
 */
Centroid3D weighted_centroid(const std::vector<std::pair<int, int>>& cells,
                             const PpiMatrix& m, double elevation_deg) {
    double sx = 0, sy = 0, sz = 0, sw = 0;
    for (auto [a, r] : cells) {
        double pm = m.pm25[a][r];          // 权重 = PM2.5
        double az_deg = m.azimuths_deg[a];
        double range_m = m.ranges_m[r];
        double east, north, up;
        polar_to_enu(range_m, az_deg, elevation_deg, east, north, up);
        sx += pm * east;   // Σ w·E
        sy += pm * north;  // Σ w·N
        sz += pm * up;     // Σ w·U
        sw += pm;          // Σ w
    }
    if (sw <= 0) return {0, 0, 0};   // 退化保护
    return {sx / sw, sy / sw, sz / sw};
}

/**
 * @brief §19.23 由质心指向求喷雾目标方位角与仰角。
 *
 * 几何关系:
 *   - 水平距离    D  = √(E² + N²)
 *   - 方位角 (北顺) az = atan2(E, N)
 *   - 仰角        el = atan2(U, D)
 *
 * @param c 加权质心
 * @return 目标 (az_deg, el_deg, 水平距离)
 */
struct TargetAngles { double az, el, horizontal; };

TargetAngles centroid_to_target(const Centroid3D& c) {
    double horizontal = std::sqrt(c.x * c.x + c.y * c.y);
    double az = std::atan2(c.x, c.y) * 180.0 / PI;        // 北顺 (从 N 向 E)
    double el = (horizontal > 1e-6) ? std::atan2(c.z, horizontal) * 180.0 / PI : 0.0;
    return {az, el, horizontal};
}

/**
 * @brief §19.26 打印 ASCII 伪热力图。
 *
 * 每个 cell 显示 PM2.5 数值, 后跟浓度等级标签:
 *   - L: <50 (低)
 *   - M: 50–60 (中等)
 *   - H: 阈值命中 (高, 已超过 60)
 *
 * @param m    PM2.5 矩阵
 * @param mask 阈值 mask (用于决定 'H' 标签)
 */
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
            // 标签: L < 50, M ≥ 50, H 表示命中阈值
            char tag = 'L';
            if (m.pm25[i][j] >= 50.0) tag = 'M';
            if (mask[i][j] == 1)      tag = 'H';
            printf(" %3.0f%c |", m.pm25[i][j], tag);
        }
        printf("\n");
    }
}

/**
 * @brief 2D PPI 算例的主驱动: 完整运行 §19.17–§19.27 的后端处理链。
 *
 * 流程:
 *   1. 装入手册 §19.18 的 PM2.5 矩阵并打印 (现 mask=空 -> 仅显示浓度等级)
 *   2. §19.19 阈值分割得到二值 mask
 *   3. §19.20 8 邻域 BFS 连通域分析, 过滤 cell 数 ≥ 3 的连通域
 *   4. §19.21–25 对每个有效热点求加权质心、目标角、面积、动作建议
 *   5. §19.27 写出完整 JSON 结果 (含 mask / components / hotspots)
 */
void run_2d_ppi_demo() {
    print_header("2D PPI example (sections 19.17-19.27)");
    printf("    fixed elevation = %.1f deg\n", ELEVATION_DEG);

    // ===== §19.18 装入示例 PM2.5 矩阵 =====
    PpiMatrix m = load_ppi_matrix();
    printf("\n[input] PM2.5 matrix shape = (%zu, %zu)\n",
           m.azimuths_deg.size(), m.ranges_m.size());
    std::vector<std::vector<int>> mask_empty(m.azimuths_deg.size(),
                                             std::vector<int>(m.ranges_m.size(), 0));
    render_ascii(m, mask_empty);

    // ===== §19.19 阈值分割 =====
    auto mask = threshold_matrix(m, PM25_THRESHOLD);
    int hot_count = 0;
    for (auto& row : mask) for (int v : row) hot_count += v;
    printf("\n[2] threshold split (PM2.5 > %.0f) -> %d cells\n", PM25_THRESHOLD, hot_count);
    render_ascii(m, mask);

    // ===== §19.20 连通域分析 =====
    auto components = find_components(mask);
    printf("\n[3] 8-neighbour connected components -> %zu comp(s)\n", components.size());
    for (std::size_t k = 0; k < components.size(); ++k) {
        printf("    component #%zu (%zu cells):", k, components[k].size());
        for (auto [a, r] : components[k]) {
            printf(" (az=%.0f, R=%.0f, PM=%.0f)", m.azimuths_deg[a], m.ranges_m[r], m.pm25[a][r]);
        }
        printf("\n");
    }
    // 过滤: cell 数 ≥ 3 才视为有效热点 (剔除 1–2 cell 的零散噪声)
    std::vector<std::vector<std::pair<int, int>>> valid;
    for (auto& c : components) if (c.size() >= 3) valid.push_back(c);
    printf("    min_cells=3 -> %zu valid hotspot(s)\n", valid.size());

    // ===== §19.21–25 每个有效热点 -> 事件 =====
    for (std::size_t idx = 0; idx < valid.size(); ++idx) {
        auto& cells = valid[idx];
        // §19.22 加权质心
        Centroid3D c = weighted_centroid(cells, m, ELEVATION_DEG);
        // §19.23 目标角
        TargetAngles t = centroid_to_target(c);

        // peak / mean / 高度范围 统计
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

        // §19.24 面积估算: 把 cell 看作 (圆周弧长 × 径向步长) 的扇环
        //   弧长 = (方位步长 / 360°) · 2π · D
        double az_step_deg = (m.azimuths_deg.size() > 1)
            ? (m.azimuths_deg[1] - m.azimuths_deg[0]) : 1.0;
        double range_step_m = (m.ranges_m.size() > 1)
            ? (m.ranges_m[1] - m.ranges_m[0]) : 30.0;
        double arc_len = (az_step_deg / 360.0) * 2.0 * PI * t.horizontal;
        double area = arc_len * range_step_m * static_cast<double>(cells.size());

        // §19.25 动作建议: mean > 60 μg/m³ -> 启动喷雾
        std::string action = (mean > 60.0) ? "spray" : "monitor";
        printf("\n[hotspot #%zu] target az=%.2f deg el=%.2f deg peak=%.1f action=%s\n",
               idx, t.az, t.el, peak, action.c_str());
        printf("    centroid ENU = (%.2f, %.2f, %.2f) m  area=%.1f m^2  cells=%zu\n",
               c.x, c.y, c.z, area, cells.size());
    }

    // ===== §19.27 写 JSON 结果 =====
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

/**
 * @brief 二维 PPI 算例入口点。
 * @return 0=成功, 1=异常退出
 */
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
