/**
 * @file Example1dRayOutput.hpp
 * @brief One-dimensional ray teaching example reporting and demo driver.
 */
#pragma once

#include "Example1dRayPhysics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace example_1d_ray {
using namespace example_common;

/**
 * @brief 热点事件: 描述一个被识别出的污染团块与对应喷雾动作建议。
 *
 * 在本一维算例中只产生 0 或 1 个事件 (所有 hot bin 合并为一个加权质心)。
 * 在 PPI/RHI 二维算例中, 每个连通域各产生一个事件。
 */
struct HotspotEvent {
    int cell_count = 0;             ///< 命中阈值的 bin 数量
    double centroid_east = 0.0;     ///< 加权质心东向分量 (m)
    double centroid_north = 0.0;    ///< 加权质心北向分量 (m)
    double centroid_up = 0.0;       ///< 加权质心上向分量 (m)
    double target_az_deg = 0.0;     ///< 喷雾目标方位角 (度, 北顺时针)
    double target_el_deg = 0.0;     ///< 喷雾目标仰角 (度)
    double peak_pm25 = 0.0;         ///< 热点区峰值 PM2.5 (μg/m³)
    double mean_pm25 = 0.0;         ///< 热点区平均 PM2.5 (μg/m³)
    std::string action;             ///< 推荐动作: "spray" 或 "monitor"
};

/**
 * @brief 工具: 以固定精度打印一个 double 向量。
 * @param label     行标签
 * @param v         数据向量
 * @param precision 小数位数
 */
inline void print_vec(const std::string& label, const std::vector<double>& v, int precision = 4) {
    std::cout << "    " << label << " = [";
    for (std::size_t i = 0; i < v.size(); ++i) {
        std::cout << std::fixed << std::setprecision(precision) << v[i];
        if (i + 1 < v.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

/**
 * @brief 工具: 打印二值 mask, 并附命中 bin 数。
 */
inline void print_mask(const std::string& label, const std::vector<int>& v) {
    std::cout << "    " << label << " = [";
    for (std::size_t i = 0; i < v.size(); ++i) {
        std::cout << v[i];
        if (i + 1 < v.size()) std::cout << ", ";
    }
    int sum = 0; for (int x : v) sum += x;
    std::cout << "]  hot bin count = " << sum << "\n";
}

/**
 * @brief 主驱动: 单射线 13 步算法链的总入口, 完成仿真→反演→事件→写盘。
 *
 * 顺序按手册 §19.1–§19.13 推进, 每步打印中间向量。
 * 最终把整条链路的所有中间量与热点事件汇总后写入
 * `data/examples/1d_ray_result.json`, 供 web dashboard 调用。
 *
 * @return 单个 HotspotEvent (本算例中所有 hot bin 只合成一个质心)
 */
inline HotspotEvent run_1d_ray_demo() {
    print_header("1D ray example (sections 19.1-19.15)");

    Rng rng(7);    // 固定种子让结果可复现 (seed=7)
    // 距离序列: 30 m ~ 180 m 共 6 个 bin, 步长 30 m
    std::vector<double> ranges = {30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
    double azimuth_deg = 40.0, elevation_deg = 10.0;

    // ===== §19.1 仿真: 物理模型正向 + Gaussian plume + LiDAR 方程 + 射击噪声 =====
    SingleRayProfile sim = simulate_single_ray(ranges, azimuth_deg, elevation_deg, rng);
    std::cout << "[1] scenario + raw signal\n";
        std::cout << "    azimuth=" << azimuth_deg << " deg  elevation=" << elevation_deg
                  << " deg  device_height=" << DEVICE_HEIGHT_M << " m\n";
        std::cout << "    ranges m = [30, 60, 90, 120, 150, 180]\n";
        print_vec("raw photon counts", sim.raw_counts, 1);
        std::cout << "    background = " << sim.background << "  laser_energy = "
                  << sim.energy_mj << " mJ\n";

        // ===== §19.2 背景扣除: N' = N - N_bg =====
        std::vector<double> p1 = step2_background_subtract(sim.raw_counts, sim.background);
        print_step("2");
        print_vec("P1 (after background)", p1, 3);

        // ===== §19.3 能量归一: P1 = N' / E =====
        std::vector<double> p2 = step3_energy_normalize(p1, sim.energy_mj);
        print_step("3");
        print_vec("P2 (per mJ)", p2, 3);

        // ===== §19.4 RCS 距离平方校正: X = P1 · R² =====
        std::vector<double> rcs = step4_rcs(p2, ranges);
        print_step("4");
        print_vec("RCS  X = P2*R^2", rcs, 1);

        // ===== §19.5 overlap 校正: X' = X / O(R) =====
        std::vector<double> corrected = step5_overlap_correct(rcs, sim.overlap);
        print_step("5");
        print_vec("corrected (RCS/O)", corrected, 1);

        // ===== §19.6–7 Klett 远->近递推反演 =====
        // 先重建分子廓线 (用于剔除分子分量) ——瑞利标高 8 km
        double el_rad = deg2rad(elevation_deg);
        std::vector<double> mol_ext(ranges.size());
        std::vector<double> mol_beta(ranges.size());
        for (std::size_t i = 0; i < ranges.size(); ++i) {
            double alt = ranges[i] * std::sin(el_rad);
            mol_ext[i] = 0.012 * std::exp(-alt / 8000.0);
            mol_beta[i] = mol_ext[i] / 8.0;
        }
        // ref_beta_aerosol 决定绝对量级, 这里取 0.0008 (城市背景轻度气溶胶)
        KlettResult klett = step6_7_klett(corrected, ranges, mol_ext, mol_beta, LIDAR_RATIO_SR, 0.0008);
        print_step("6-7");
        std::cout << "    Klett inversion S=" << LIDAR_RATIO_SR << " ref_beta_aerosol=0.0008\n";
        print_vec("beta  (km^-1 sr^-1)", klett.beta, 5);
        print_vec("alpha (km^-1)      ", klett.alpha, 4);

        // ===== §19.8 湿度修正 f(RH): α_dry = α / f(RH) =====
        std::vector<double> alpha_dry = step8_humidity_correct(klett.alpha, sim.rh_percent, 0.45, 0.35);
        print_step("8");
        print_vec("RH (%)  ", sim.rh_percent, 1);
        print_vec("dry alpha (km^-1)", alpha_dry, 4);

        // ===== §19.9 PM 估算: 线性定标 =====
        PmResult pm = step9_pm_estimate(alpha_dry);
        print_step("9");
        std::cout << "    PM2.5 = 500 * alpha_dry + 10,  PM10 = 1.6 * PM2.5\n";
        print_vec("PM2.5", pm.pm25, 2);
        print_vec("PM10 ", pm.pm10, 2);

        // ===== §19.10 阈值分割: mask = (PM2.5 > 60 μg/m³) =====
        std::vector<int> mask(pm.pm25.size(), 0);
        for (std::size_t i = 0; i < pm.pm25.size(); ++i) mask[i] = (pm.pm25[i] > PM25_THRESHOLD) ? 1 : 0;
        print_step("10");
        std::cout << "    threshold = PM2.5 > " << PM25_THRESHOLD << " ug/m^3\n";
        print_mask("mask", mask);

        // ===== §19.11 极坐标 -> ENU =====
        //   E = R·cos(el)·sin(az)
        //   N = R·cos(el)·cos(az)
        //   U = R·sin(el)
        std::vector<std::vector<double>> enu(ranges.size(), std::vector<double>(3, 0.0));
        print_step("11");
        std::cout << "                range   PM2.5    east     north    up\n";
        for (std::size_t i = 0; i < ranges.size(); ++i) {
            polar_to_enu(ranges[i], azimuth_deg, elevation_deg, enu[i][0], enu[i][1], enu[i][2]);
            printf("                  %3.0fm  %6.2f  %7.2f  %7.2f  %6.2f\n",
                   ranges[i], pm.pm25[i], enu[i][0], enu[i][1], enu[i][2]);
        }

        // ===== §19.12 PM2.5 加权质心 C = Σ w·r / Σ w (w = PM2.5) =====
        print_step("12");
        double sx = 0, sy = 0, sz = 0, sw = 0;
        for (std::size_t i = 0; i < pm.pm25.size(); ++i) {
            if (mask[i] == 0) continue;
            double w = pm.pm25[i];
            sx += w * enu[i][0]; sy += w * enu[i][1]; sz += w * enu[i][2]; sw += w;
        }
        HotspotEvent event;
        event.cell_count = 0;
        for (int m : mask) event.cell_count += m;
        if (sw > 0) {
            event.centroid_east = sx / sw;
            event.centroid_north = sy / sw;
            event.centroid_up = sz / sw;
        }
        printf("    centroid ENU = (%7.2f, %7.2f, %6.2f) m\n",
               event.centroid_east, event.centroid_north, event.centroid_up);

        // ===== §19.13 形成热点事件 + 喷雾目标角 =====
        //   target_az = atan2(E, N)        (北顺为 +)
        //   target_el = atan2(U, √(E²+N²))
        print_step("13");
        double horizontal = std::sqrt(event.centroid_east * event.centroid_east +
                                      event.centroid_north * event.centroid_north);
        event.target_az_deg = (event.cell_count > 0)
            ? std::atan2(event.centroid_east, event.centroid_north) * 180.0 / PI : 0.0;
        event.target_el_deg = (event.cell_count > 0 && horizontal > 1e-6)
            ? std::atan2(event.centroid_up, horizontal) * 180.0 / PI : 0.0;

        // peak / mean 统计 (只在 hot cell 范围内)
        double peak = 0.0; int cnt = 0; double sum = 0.0;
        for (std::size_t i = 0; i < pm.pm25.size(); ++i) {
            if (mask[i] == 0) continue;
            peak = std::max(peak, pm.pm25[i]);
            sum += pm.pm25[i]; ++cnt;
        }
        event.peak_pm25 = peak;
        event.mean_pm25 = cnt > 0 ? sum / cnt : 0.0;
        // 推荐动作: 平均 > 60 → 启动喷雾; 否则持续监测
        event.action = (event.mean_pm25 > 60.0) ? "spray" : "monitor";

        printf("    target (az, el)   = (%6.2f deg, %5.2f deg)\n", event.target_az_deg, event.target_el_deg);
        printf("    peak  PM2.5       = %6.2f ug/m^3\n", event.peak_pm25);
        printf("    mean  PM2.5       = %6.2f ug/m^3\n", event.mean_pm25);
        printf("    recommended action = %s\n", event.action.c_str());

        // ===== 写 JSON: 整条链路的中间量 + 事件汇总 =====
        std::vector<std::pair<std::string, JsonValue>> intermediate;
        intermediate.push_back({"after_background", JsonValue::array(doubles_to_json(round6(p1)))});
        intermediate.push_back({"after_energy_norm", JsonValue::array(doubles_to_json(round6(p2)))});
        intermediate.push_back({"rcs", JsonValue::array(doubles_to_json(round6(rcs)))});
        intermediate.push_back({"after_overlap_corr", JsonValue::array(doubles_to_json(round6(corrected)))});
        intermediate.push_back({"beta", JsonValue::array(doubles_to_json(round6(klett.beta)))});
        intermediate.push_back({"alpha", JsonValue::array(doubles_to_json(round6(klett.alpha)))});
        intermediate.push_back({"alpha_dry", JsonValue::array(doubles_to_json(round6(alpha_dry)))});
        intermediate.push_back({"pm25", JsonValue::array(doubles_to_json(round6(pm.pm25)))});
        intermediate.push_back({"pm10", JsonValue::array(doubles_to_json(round6(pm.pm10)))});
        std::vector<JsonValue> mask_json;
        for (int m : mask) mask_json.push_back(JsonValue::integer(m));
        intermediate.push_back({"hotspot_mask", JsonValue::array(mask_json)});

        std::vector<JsonValue> raw_counts_json;
        for (double v : sim.raw_counts) raw_counts_json.push_back(JsonValue::number(round6(v)));

        std::vector<JsonValue> enu_points_json;
        for (auto& row : enu) enu_points_json.push_back(JsonValue::array(doubles_to_json(round6(row))));

        std::vector<std::pair<std::string, JsonValue>> root;
        root.push_back({"scenario", JsonValue::object({
            {"azimuth_deg", JsonValue::number(azimuth_deg)},
            {"elevation_deg", JsonValue::number(elevation_deg)},
            {"device_height_m", JsonValue::number(DEVICE_HEIGHT_M)},
            {"ranges_m", JsonValue::array(doubles_to_json(ranges))}
        })});
        root.push_back({"raw_counts", JsonValue::array(raw_counts_json)});
        root.push_back({"intermediate", JsonValue::object(intermediate)});
        root.push_back({"enu_points_m", JsonValue::array(enu_points_json)});
        root.push_back({"centroid_enu_m", JsonValue::array({
            JsonValue::number(round6(event.centroid_east)),
            JsonValue::number(round6(event.centroid_north)),
            JsonValue::number(round6(event.centroid_up))
        })});
        root.push_back({"hotspot_event", JsonValue::object({
            {"cell_count", JsonValue::integer(event.cell_count)},
            {"target_azimuth_deg", JsonValue::number(round6(event.target_az_deg))},
            {"target_elevation_deg", JsonValue::number(round6(event.target_el_deg))},
            {"peak_pm25_ugm3", JsonValue::number(round6(event.peak_pm25))},
            {"mean_pm25_ugm3", JsonValue::number(round6(event.mean_pm25))},
            {"recommended_action", JsonValue::string(event.action)}
        })});

        JsonValue root_json = JsonValue::object(root);
        std::filesystem::path out = std::filesystem::current_path() / "data" / "examples" / "1d_ray_result.json";
        write_json_file(out, root_json);
        std::cout << "\n[OK] JSON written to " << out.string() << "\n";

        return event;
}

} // namespace example_1d_ray
