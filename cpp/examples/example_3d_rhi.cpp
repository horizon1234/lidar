// cpp/examples/example_3d_rhi.cpp
//
// 第 19.28-19.37 节 RHI 二维算例: 垂直剖面 + 层顶层底 + 羽流抬升。
// 对应《激光颗粒物监测系统入门手册》第 19.28-19.37 节。
//
// RHI = Range-Height Indicator 固定方位, 仰角上下扫描。
// 与 PPI 互补: RHI 解决"羽流抬得多高、层底层顶在哪"。
//
// 完整步骤:
//   1. 二维 PM2.5 矩阵: 4 仰角 × 4 距离 = 16 cell
//   2. 阈值分割 -> 二值 mask
//   3. RHI 格点 -> (地面距离 s, 高度 h)  (要加设备高度 18 m)
//   4. 估算层底 / 层顶 / 垂直厚度
//   5. PM2.5 加权剖面质心 (s, h)
//   6. 剖面质心 -> 三维 ENU (代入固定方位 40°)
//   7. 形成剖面形态描述 + ASCII 伪热力图
//   8. 写 JSON 到 data/examples/3d_rhi_result.json

#include "example_common.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace example_common;

namespace {

constexpr double FIXED_AZIMUTH_DEG = 40.0;
constexpr double DEVICE_HEIGHT_M   = 18.0;
constexpr double PM25_THRESHOLD    = 60.0;

struct RhiMatrix {
    std::vector<double> elevations_deg;
    std::vector<double> ranges_m;
    std::vector<std::vector<double>> pm25;  // pm25[i][j] 仰角 i, 距离 j
};

RhiMatrix load_rhi_matrix() {
    // 文档 19.29 节
    RhiMatrix m;
    m.elevations_deg = {4.0, 8.0, 12.0, 16.0};
    m.ranges_m       = {60.0, 90.0, 120.0, 150.0};
    m.pm25 = {
        {28.0, 45.0, 72.0, 65.0},  // 4 deg
        {32.0, 58.0, 95.0, 84.0},  // 8 deg
        {30.0, 55.0, 88.0, 76.0},  // 12 deg
        {24.0, 41.0, 62.0, 54.0},  // 16 deg
    };
    return m;
}

std::vector<std::vector<int>> threshold_matrix(const RhiMatrix& m, double threshold) {
    std::vector<std::vector<int>> mask(m.elevations_deg.size(),
                                       std::vector<int>(m.ranges_m.size(), 0));
    for (std::size_t i = 0; i < m.elevations_deg.size(); ++i) {
        for (std::size_t j = 0; j < m.ranges_m.size(); ++j) {
            mask[i][j] = (m.pm25[i][j] > threshold) ? 1 : 0;
        }
    }
    return mask;
}

struct ShPoint { double s, h; };

ShPoint cell_to_sh(double range_m, double elevation_deg, double device_height_m) {
    double el = deg2rad(elevation_deg);
    return {range_m * std::cos(el), device_height_m + range_m * std::sin(el)};
}

struct CentroidSH { double s, h; };

void run_3d_rhi_demo() {
    print_header("2D RHI example (sections 19.28-19.37)");
    printf("    fixed azimuth = %.1f deg  device_height = %.1f m\n",
           FIXED_AZIMUTH_DEG, DEVICE_HEIGHT_M);

    RhiMatrix m = load_rhi_matrix();
    printf("\n[input] PM2.5 matrix shape = (%zu, %zu)\n",
           m.elevations_deg.size(), m.ranges_m.size());
    std::vector<std::vector<int>> mask_empty(m.elevations_deg.size(),
                                             std::vector<int>(m.ranges_m.size(), 0));

    auto render_ascii = [&](const std::vector<std::vector<int>>& mask) {
        printf("el\\R |");
        for (double r : m.ranges_m) printf(" %4dm |", static_cast<int>(r));
        printf("\n------");
        for (std::size_t j = 0; j < m.ranges_m.size(); ++j) printf("--------");
        printf("\n");
        for (std::size_t i = 0; i < m.elevations_deg.size(); ++i) {
            printf("%3.0f  |", m.elevations_deg[i]);
            for (std::size_t j = 0; j < m.ranges_m.size(); ++j) {
                char tag = 'L';
                if (m.pm25[i][j] >= 50.0) tag = 'M';
                if (mask[i][j] == 1)      tag = 'H';
                printf(" %3.0f%c |", m.pm25[i][j], tag);
            }
            printf("\n");
        }
    };
    render_ascii(mask_empty);

    // 2. 阈值
    auto mask = threshold_matrix(m, PM25_THRESHOLD);
    int hot = 0;
    for (auto& row : mask) for (int v : row) hot += v;
    printf("\n[2] threshold split (PM2.5 > %.0f) -> %d cells\n", PM25_THRESHOLD, hot);
    render_ascii(mask);

    // 3. 每个 hot cell -> (s, h)
    printf("\n[3] hot cells -> (ground_range s, height h):\n");
    std::vector<std::pair<int, int>> cells;
    std::vector<ShPoint> shs;
    std::vector<double> weights;
    for (std::size_t i = 0; i < m.elevations_deg.size(); ++i) {
        for (std::size_t j = 0; j < m.ranges_m.size(); ++j) {
            if (mask[i][j] == 0) continue;
            cells.push_back({static_cast<int>(i), static_cast<int>(j)});
            ShPoint sh = cell_to_sh(m.ranges_m[j], m.elevations_deg[i], DEVICE_HEIGHT_M);
            shs.push_back(sh);
            weights.push_back(m.pm25[i][j]);
            printf("    %3.0f deg, %4.0f m  PM2.5=%5.1f  s=%6.2f m  h=%5.2f m\n",
                   m.elevations_deg[i], m.ranges_m[j], m.pm25[i][j], sh.s, sh.h);
        }
    }

    // 4. 层底/层顶/厚度
    double h_bottom = 1e9, h_top = -1e9;
    for (auto& sh : shs) {
        h_bottom = std::min(h_bottom, sh.h);
        h_top    = std::max(h_top, sh.h);
    }
    double depth = h_top - h_bottom;
    printf("\n[4] layer bottom/top\n");
    printf("    h_bottom = %.2f m\n", h_bottom);
    printf("    h_top    = %.2f m\n", h_top);
    printf("    thickness = %.2f m\n", depth);

    // 5. 加权剖面质心
    double ss = 0, sh2 = 0, sw = 0;
    for (std::size_t k = 0; k < shs.size(); ++k) {
        ss += weights[k] * shs[k].s;
        sh2 += weights[k] * shs[k].h;
        sw += weights[k];
    }
    CentroidSH c_sh = (sw > 0) ? CentroidSH{ss / sw, sh2 / sw} : CentroidSH{0, 0};
    printf("\n[5] PM2.5-weighted section centroid\n");
    printf("    (s_c, h_c) = (%.2f, %.2f) m\n", c_sh.s, c_sh.h);

    // 6. 转三维 ENU
    double az_rad = deg2rad(FIXED_AZIMUTH_DEG);
    double cx = c_sh.s * std::sin(az_rad);
    double cy = c_sh.s * std::cos(az_rad);
    double cz = c_sh.h;
    printf("\n[6] section centroid -> 3D ENU (azimuth=%.0f deg)\n", FIXED_AZIMUTH_DEG);
    printf("    (x, y, z) = (%.2f, %.2f, %.2f) m\n", cx, cy, cz);

    // 7. peak / mean / 形态描述
    double peak = 0, sum = 0;
    for (double w : weights) { peak = std::max(peak, w); sum += w; }
    double mean_v = weights.empty() ? 0 : sum / weights.size();
    std::string action = (c_sh.h > 35.0) ? "spray_high" : "spray_ground";
    std::string interpret = "plume from " + std::to_string(static_cast<int>(h_bottom)) +
                            "m to " + std::to_string(static_cast<int>(h_top)) +
                            "m (depth " + std::to_string(static_cast<int>(depth)) +
                            "m); main mass center at " +
                            std::to_string(static_cast<int>(c_sh.h)) +
                            "m; not ground-hugging thin dust but vertically lofted plume.";
    printf("\n[7] section interpretation\n");
    printf("    %s\n", interpret.c_str());
    printf("    recommended_action = %s\n", action.c_str());

    // 写 JSON
    std::vector<JsonValue> sh_cells_json;
    for (auto& sh : shs) {
        sh_cells_json.push_back(JsonValue::array({
            JsonValue::number(round6(sh.s)),
            JsonValue::number(round6(sh.h))
        }));
    }
    std::vector<JsonValue> weights_json;
    for (double w : weights) weights_json.push_back(JsonValue::number(round6(w)));

    std::vector<JsonValue> elevations_json;
    for (double v : m.elevations_deg) elevations_json.push_back(JsonValue::number(v));
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

    JsonValue root = JsonValue::object({
        {"scan_mode",         JsonValue::string("rhi")},
        {"fixed_azimuth_deg", JsonValue::number(FIXED_AZIMUTH_DEG)},
        {"device_height_m",   JsonValue::number(DEVICE_HEIGHT_M)},
        {"elevations_deg",    JsonValue::array(elevations_json)},
        {"ranges_m",          JsonValue::array(ranges_json)},
        {"pm25_matrix",       JsonValue::array(pm25_rows)},
        {"threshold_ugm3",    JsonValue::number(PM25_THRESHOLD)},
        {"binary_mask",       JsonValue::array(mask_rows)},
        {"hot_cells_sh",      JsonValue::array(sh_cells_json)},
        {"hot_cells_pm25",    JsonValue::array(weights_json)},
        {"layer_bounds",      JsonValue::object({
            {"bottom_m",    JsonValue::number(round6(h_bottom))},
            {"top_m",       JsonValue::number(round6(h_top))},
            {"thickness_m", JsonValue::number(round6(depth))}
        })},
        {"section_centroid",  JsonValue::object({
            {"ground_range_m", JsonValue::number(round6(c_sh.s))},
            {"height_m",       JsonValue::number(round6(c_sh.h))}
        })},
        {"centroid_enu_m",    JsonValue::array({
            JsonValue::number(round6(cx)),
            JsonValue::number(round6(cy)),
            JsonValue::number(round6(cz))
        })},
        {"plume_event",       JsonValue::object({
            {"event_id",                  JsonValue::string("dust_rhi_0")},
            {"type",                      JsonValue::string("dust_plume_vertical")},
            {"scan_mode",                 JsonValue::string("rhi")},
            {"fixed_azimuth_deg",         JsonValue::number(FIXED_AZIMUTH_DEG)},
            {"device_height_m",           JsonValue::number(DEVICE_HEIGHT_M)},
            {"section_centroid_ground_range_m", JsonValue::number(round6(c_sh.s))},
            {"section_centroid_height_m",       JsonValue::number(round6(c_sh.h))},
            {"centroid_enu_m",            JsonValue::array({
                JsonValue::number(round6(cx)),
                JsonValue::number(round6(cy)),
                JsonValue::number(round6(cz))
            })},
            {"layer_bottom_m",            JsonValue::number(round6(h_bottom))},
            {"layer_top_m",               JsonValue::number(round6(h_top))},
            {"vertical_thickness_m",      JsonValue::number(round6(depth))},
            {"peak_pm25_ugm3",            JsonValue::number(round6(peak))},
            {"mean_pm25_ugm3",            JsonValue::number(round6(mean_v))},
            {"hot_cell_count",            JsonValue::integer(static_cast<int>(cells.size()))},
            {"interpretation",            JsonValue::string(interpret)},
            {"recommended_action",        JsonValue::string(action)}
        })}
    });

    std::filesystem::path out = std::filesystem::current_path() / "data" / "examples" / "3d_rhi_result.json";
    write_json_file(out, root);
    std::cout << "\n[OK] JSON written to " << out.string() << "\n";
}

}  // namespace

int main() {
    enable_ansi_color();
    try {
        run_3d_rhi_demo();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
    return 0;
}
