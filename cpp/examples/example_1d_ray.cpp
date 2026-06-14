// cpp/examples/example_1d_ray.cpp
//
// 第 19 章一维算例: 一条射线从原始 photon counts 一路走到热点告警。
// 对应《激光颗粒物监测系统入门手册》19.1 – 19.14 节的 13 步算法链:
//
//   1. 场景设定 + 物理模型仿真 (Gaussian plume + LiDAR 方程 + 射击噪声)
//   2. 背景光扣除
//   3. 发射能量归一化
//   4. RCS 距离平方校正
//   5. overlap 校正
//   6-7. 简化 Klett 远->近递推反演 -> alpha, beta
//   8. 湿度修正 -> alpha_dry
//   9. 干态消光 -> PM2.5 / PM10 (教学线性模型)
//   10. 阈值分割
//   11. 极坐标 -> ENU
//   12. PM2.5 加权质心
//   13. 形成热点事件 + 喷雾目标角
//
// 编译运行:
//   cmake -S . -B build
//   cmake --build build --config Release --target lidar_example_1d_ray
//   build\lidar_example_1d_ray
//
// 输出: 在 data/examples/1d_ray_result.json 写入完整 JSON 结果,
//       并在终端打印每一步的中间结果。

#include "example_common.hpp"

#include <cmath>
#include <string>
#include <vector>

using namespace example_common;

namespace {

constexpr double LIDAR_RATIO_SR = 45.0;        // 气溶胶 lidar ratio
constexpr double SYSTEM_CONSTANT = 6.0e5;      // 系统常数 C·E 计算时用
constexpr double BACKGROUND_COUNTS = 20.0;
constexpr double LASER_ENERGY_MJ = 2.0;
constexpr double PLUME_CENTER_M = 120.0;
constexpr double PLUME_STRENGTH_DRY = 0.55;    // 干态气溶胶消光强度峰值 km^-1
constexpr double HUMIDITY_GROWTH = 1.35;       // 湿度散射放大倍率
constexpr double PM25_THRESHOLD = 60.0;        // 业务阈值
constexpr double DEVICE_HEIGHT_M = 18.0;

struct SingleRayProfile {
    std::vector<double> ranges_m;
    std::vector<double> raw_counts;
    double background = 0.0;
    double energy_mj = 0.0;
    std::vector<double> overlap;
    std::vector<double> rh_percent;
    double azimuth_deg = 0.0;
    double elevation_deg = 0.0;
    // 真值, 仅用于事后核对, 算法不用
    std::vector<double> true_ext_dry;
    std::vector<double> true_ext_wet;
    std::vector<double> true_beta;
};

// --------------------------------------------------------------------------
// 第 1 步: 物理模型正向仿真
//   - 分子分量按 exp 衰减
//   - 120m 处放一个 Gaussian plume
//   - 真值 alpha/beta 已知 -> 反向求 photon counts (加噪声)
// --------------------------------------------------------------------------

SingleRayProfile simulate_single_ray(const std::vector<double>& ranges,
                                     double azimuth_deg, double elevation_deg, Rng& rng) {
    SingleRayProfile p;
    p.ranges_m = ranges;
    p.azimuth_deg = azimuth_deg;
    p.elevation_deg = elevation_deg;
    p.background = BACKGROUND_COUNTS;
    p.energy_mj = LASER_ENERGY_MJ;

    std::size_t n = ranges.size();
    double dr_km = (ranges.size() > 1 ? ranges[1] - ranges[0] : 30.0) / 1000.0;
    double el_rad = deg2rad(elevation_deg);

    std::vector<double> altitude(n);
    std::vector<double> mol_ext(n);
    std::vector<double> mol_beta(n);
    std::vector<double> aerosol_ext_dry(n);
    std::vector<double> aerosol_ext_wet(n);
    std::vector<double> true_beta_total(n);
    std::vector<double> true_ext_wet(n);
    std::vector<double> overlap(n);

    for (std::size_t i = 0; i < n; ++i) {
        altitude[i] = ranges[i] * std::sin(el_rad);
        mol_ext[i] = 0.012 * std::exp(-altitude[i] / 8000.0);
        mol_beta[i] = mol_ext[i] / 8.0;

        double boundary = 0.070 * std::exp(-altitude[i] / 800.0);
        double plume = PLUME_STRENGTH_DRY * gaussian1d(ranges[i], PLUME_CENTER_M, 25.0);
        aerosol_ext_dry[i] = 0.04 + boundary + plume;
        aerosol_ext_wet[i] = aerosol_ext_dry[i] * HUMIDITY_GROWTH;

        true_beta_total[i] = mol_beta[i] + aerosol_ext_wet[i] / LIDAR_RATIO_SR;
        true_ext_wet[i] = mol_ext[i] + aerosol_ext_wet[i];

        // overlap 函数: 近距小、远距趋 1
        overlap[i] = clamp(std::pow(ranges[i] / 220.0, 0.82), 0.32, 1.0);
    }

    // LiDAR 方程正向 photon counts
    std::vector<double> raw_counts(n);
    double optical_depth = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        optical_depth += true_ext_wet[i] * dr_km;
        double range_km = ranges[i] / 1000.0;
        double signal = SYSTEM_CONSTANT * LASER_ENERGY_MJ * overlap[i] *
                        true_beta_total[i] / (range_km * range_km + 1e-12) *
                        std::exp(-2.0 * optical_depth);
        double shot_sigma = std::sqrt(std::max(signal, 1.0)) * 0.08;
        double noisy = signal + p.background + rng.next_gauss(0.0, shot_sigma);
        raw_counts[i] = std::max(noisy, p.background + 0.1);
    }

    p.raw_counts = raw_counts;
    p.overlap = overlap;
    p.true_ext_dry = aerosol_ext_dry;
    p.true_ext_wet = true_ext_wet;
    p.true_beta = true_beta_total;

    // 相对湿度沿距离略变 (72 -> 82 %)
    p.rh_percent.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        p.rh_percent[i] = 72.0 + 10.0 * static_cast<double>(i) / static_cast<double>(n - 1);
    }

    return p;
}

// --------------------------------------------------------------------------
// 第 2 步: 背景扣除
// --------------------------------------------------------------------------
std::vector<double> step2_background_subtract(const std::vector<double>& raw, double bg) {
    std::vector<double> out(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
        out[i] = std::max(raw[i] - bg, 0.0);
    }
    return out;
}

// --------------------------------------------------------------------------
// 第 3 步: 发射能量归一化
// --------------------------------------------------------------------------
std::vector<double> step3_energy_normalize(const std::vector<double>& in, double energy_mj) {
    std::vector<double> out(in.size());
    double e = std::max(energy_mj, 1e-6);
    for (std::size_t i = 0; i < in.size(); ++i) out[i] = in[i] / e;
    return out;
}

// --------------------------------------------------------------------------
// 第 4 步: RCS = P2 * R^2 (km^2)
// --------------------------------------------------------------------------
std::vector<double> step4_rcs(const std::vector<double>& p2, const std::vector<double>& ranges) {
    std::vector<double> out(p2.size());
    for (std::size_t i = 0; i < p2.size(); ++i) {
        double rkm = ranges[i] / 1000.0;
        out[i] = p2[i] * rkm * rkm;
    }
    return out;
}

// --------------------------------------------------------------------------
// 第 5 步: overlap 校正: corrected = RCS / O(R)
// --------------------------------------------------------------------------
std::vector<double> step5_overlap_correct(const std::vector<double>& rcs, const std::vector<double>& overlap) {
    std::vector<double> out(rcs.size());
    for (std::size_t i = 0; i < rcs.size(); ++i) {
        out[i] = rcs[i] / std::max(overlap[i], 0.15);
    }
    return out;
}

// --------------------------------------------------------------------------
// 第 6-7 步: 简化 Klett 远->近递推
//   beta(i) = X(i) / ( X_ref/beta_ref + 2S * 积分 X(r)dr )
//   alpha = mol_ext + S * (beta - mol_beta)
//
//   ref_beta_aerosol 决定整条廓线绝对量级 (Klett 固有特性)
// --------------------------------------------------------------------------
struct KlettResult {
    std::vector<double> alpha;
    std::vector<double> beta;
};

KlettResult step6_7_klett(const std::vector<double>& corrected,
                          const std::vector<double>& ranges,
                          const std::vector<double>& mol_ext,
                          const std::vector<double>& mol_beta,
                          double lidar_ratio_sr,
                          double ref_beta_aerosol) {
    std::size_t n = corrected.size();
    double dr_km = (n > 1 ? ranges[1] - ranges[0] : 30.0) / 1000.0;

    std::size_t ref_index = (n >= 2 ? n - 2 : 0);
    double x_ref = 0.0;
    int ref_count = 0;
    for (std::size_t i = ref_index; i < n; ++i) { x_ref += corrected[i]; ++ref_count; }
    x_ref = std::max(x_ref / std::max(ref_count, 1), 1e-9);

    double mol_beta_ref = 0.0;
    for (std::size_t i = ref_index; i < n; ++i) mol_beta_ref += mol_beta[i];
    mol_beta_ref /= std::max(ref_count, 1);

    double beta_ref = mol_beta_ref + ref_beta_aerosol;
    double seed = x_ref / std::max(beta_ref, 1e-9);

    std::vector<double> beta(n, 0.0), alpha(n, 0.0);
    double cum_integral = 0.0;
    for (int idx = static_cast<int>(n) - 1; idx >= 0; --idx) {
        std::size_t i = static_cast<std::size_t>(idx);
        if (idx < static_cast<int>(n) - 1) {
            double seg = (corrected[i] + corrected[i + 1]) / 2.0 * dr_km;
            cum_integral += seg;
        }
        double denom = seed + 2.0 * lidar_ratio_sr * cum_integral;
        double beta_total = corrected[i] / std::max(denom, 1e-12);
        double beta_aerosol = std::max(beta_total - mol_beta[i], 0.0);
        beta[i] = mol_beta[i] + beta_aerosol;
        alpha[i] = mol_ext[i] + lidar_ratio_sr * beta_aerosol;
        // 物理约束: 不低于分子背景, 不高于 0.45 (理想厚云)
        alpha[i] = clamp(alpha[i], mol_ext[i], 0.45);
    }
    return {alpha, beta};
}

// --------------------------------------------------------------------------
// 第 8 步: 湿度修正 (标准 f(RH) 模型, 文档 12.8)
//   f(RH) = ((1 - RH_ref) / (1 - RH))^gamma
//   alpha_dry = alpha / f(RH)
// --------------------------------------------------------------------------
std::vector<double> step8_humidity_correct(const std::vector<double>& alpha,
                                           const std::vector<double>& rh_percent,
                                           double gamma = 0.45, double rh_ref = 0.35) {
    std::vector<double> out(alpha.size());
    for (std::size_t i = 0; i < alpha.size(); ++i) {
        double rh = clamp(rh_percent[i] / 100.0, 0.05, 0.98);
        double f = std::pow((1.0 - rh_ref) / std::max(1.0 - rh, 1e-3), gamma);
        out[i] = alpha[i] / f;
    }
    return out;
}

// --------------------------------------------------------------------------
// 第 9 步: 干态消光 -> PM2.5 / PM10
//   PM2.5 = 500 * alpha_dry + 10
//   PM10  = 1.6 * PM2.5
// --------------------------------------------------------------------------
struct PmResult { std::vector<double> pm25; std::vector<double> pm10; };

PmResult step9_pm_estimate(const std::vector<double>& alpha_dry) {
    std::vector<double> pm25(alpha_dry.size());
    std::vector<double> pm10(alpha_dry.size());
    for (std::size_t i = 0; i < alpha_dry.size(); ++i) {
        pm25[i] = 500.0 * alpha_dry[i] + 10.0;
        pm10[i] = 1.6 * pm25[i];
    }
    return {pm25, pm10};
}

// --------------------------------------------------------------------------
// 第 10-13 步: 阈值 + ENU + 质心 + 事件
// --------------------------------------------------------------------------

struct HotspotEvent {
    int cell_count = 0;
    double centroid_east = 0.0;
    double centroid_north = 0.0;
    double centroid_up = 0.0;
    double target_az_deg = 0.0;
    double target_el_deg = 0.0;
    double peak_pm25 = 0.0;
    double mean_pm25 = 0.0;
    std::string action;
};

void print_vec(const std::string& label, const std::vector<double>& v, int precision = 4) {
    std::cout << "    " << label << " = [";
    for (std::size_t i = 0; i < v.size(); ++i) {
        std::cout << std::fixed << std::setprecision(precision) << v[i];
        if (i + 1 < v.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

void print_mask(const std::string& label, const std::vector<int>& v) {
    std::cout << "    " << label << " = [";
    for (std::size_t i = 0; i < v.size(); ++i) {
        std::cout << v[i];
        if (i + 1 < v.size()) std::cout << ", ";
    }
    int sum = 0; for (int x : v) sum += x;
    std::cout << "]  hot bin count = " << sum << "\n";
}

HotspotEvent run_1d_ray_demo() {
    print_header("1D ray example (sections 19.1-19.15)");

    Rng rng(7);
    std::vector<double> ranges = {30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
    double azimuth_deg = 40.0, elevation_deg = 10.0;

    // 1. 仿真 (物理模型正向 + Gaussian plume + LiDAR 方程 + 射击噪声)
    SingleRayProfile sim = simulate_single_ray(ranges, azimuth_deg, elevation_deg, rng);
    std::cout << "[1] scenario + raw signal\n";
        std::cout << "    azimuth=" << azimuth_deg << " deg  elevation=" << elevation_deg
                  << " deg  device_height=" << DEVICE_HEIGHT_M << " m\n";
        std::cout << "    ranges m = [30, 60, 90, 120, 150, 180]\n";
        print_vec("raw photon counts", sim.raw_counts, 1);
        std::cout << "    background = " << sim.background << "  laser_energy = "
                  << sim.energy_mj << " mJ\n";

        // 2. 背景扣除
        std::vector<double> p1 = step2_background_subtract(sim.raw_counts, sim.background);
        print_step("2");
        print_vec("P1 (after background)", p1, 3);

        // 3. 能量归一
        std::vector<double> p2 = step3_energy_normalize(p1, sim.energy_mj);
        print_step("3");
        print_vec("P2 (per mJ)", p2, 3);

        // 4. RCS
        std::vector<double> rcs = step4_rcs(p2, ranges);
        print_step("4");
        print_vec("RCS  X = P2*R^2", rcs, 1);

        // 5. overlap 校正
        std::vector<double> corrected = step5_overlap_correct(rcs, sim.overlap);
        print_step("5");
        print_vec("corrected (RCS/O)", corrected, 1);

        // 6-7. Klett
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

        // 8. 湿度修正
        std::vector<double> alpha_dry = step8_humidity_correct(klett.alpha, sim.rh_percent, 0.45, 0.35);
        print_step("8");
        print_vec("RH (%)  ", sim.rh_percent, 1);
        print_vec("dry alpha (km^-1)", alpha_dry, 4);

        // 9. PM 估算
        PmResult pm = step9_pm_estimate(alpha_dry);
        print_step("9");
        std::cout << "    PM2.5 = 500 * alpha_dry + 10,  PM10 = 1.6 * PM2.5\n";
        print_vec("PM2.5", pm.pm25, 2);
        print_vec("PM10 ", pm.pm10, 2);

        // 10. 阈值
        std::vector<int> mask(pm.pm25.size(), 0);
        for (std::size_t i = 0; i < pm.pm25.size(); ++i) mask[i] = (pm.pm25[i] > PM25_THRESHOLD) ? 1 : 0;
        print_step("10");
        std::cout << "    threshold = PM2.5 > " << PM25_THRESHOLD << " ug/m^3\n";
        print_mask("mask", mask);

        // 11. ENU
        std::vector<std::vector<double>> enu(ranges.size(), std::vector<double>(3, 0.0));
        print_step("11");
        std::cout << "                range   PM2.5    east     north    up\n";
        for (std::size_t i = 0; i < ranges.size(); ++i) {
            polar_to_enu(ranges[i], azimuth_deg, elevation_deg, enu[i][0], enu[i][1], enu[i][2]);
            printf("                  %3.0fm  %6.2f  %7.2f  %7.2f  %6.2f\n",
                   ranges[i], pm.pm25[i], enu[i][0], enu[i][1], enu[i][2]);
        }

        // 12. 加权质心
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

        // 13. 事件
        print_step("13");
        double horizontal = std::sqrt(event.centroid_east * event.centroid_east +
                                      event.centroid_north * event.centroid_north);
        event.target_az_deg = (event.cell_count > 0)
            ? std::atan2(event.centroid_east, event.centroid_north) * 180.0 / PI : 0.0;
        event.target_el_deg = (event.cell_count > 0 && horizontal > 1e-6)
            ? std::atan2(event.centroid_up, horizontal) * 180.0 / PI : 0.0;

        // peak/mean
        double peak = 0.0; int cnt = 0; double sum = 0.0;
        for (std::size_t i = 0; i < pm.pm25.size(); ++i) {
            if (mask[i] == 0) continue;
            peak = std::max(peak, pm.pm25[i]);
            sum += pm.pm25[i]; ++cnt;
        }
        event.peak_pm25 = peak;
        event.mean_pm25 = cnt > 0 ? sum / cnt : 0.0;
        event.action = (event.mean_pm25 > 60.0) ? "spray" : "monitor";

        printf("    target (az, el)   = (%6.2f deg, %5.2f deg)\n", event.target_az_deg, event.target_el_deg);
        printf("    peak  PM2.5       = %6.2f ug/m^3\n", event.peak_pm25);
        printf("    mean  PM2.5       = %6.2f ug/m^3\n", event.mean_pm25);
        printf("    recommended action = %s\n", event.action.c_str());

        // 写 JSON
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

}  // namespace

int main() {
    enable_ansi_color();
    try {
        run_1d_ray_demo();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
    return 0;
}
