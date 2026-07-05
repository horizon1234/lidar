/**
 * @file Example1dRayPhysics.hpp
 * @brief One-dimensional ray teaching example physics model and retrieval steps.
 */
#pragma once

#include "ExampleCommon.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace example_1d_ray {
using namespace example_common;

// ---- §19.1 物理常量与场景参数 (城市边界层轻度气溶胶典型值) ----
constexpr double LIDAR_RATIO_SR = 45.0;        ///< 气溶胶 lidar ratio S = α/β (单位 sr), 城市约 40–70
constexpr double SYSTEM_CONSTANT = 6.0e5;      ///< 系统综合常数 C·E 中的 C (含探测器增益、光学效率)
constexpr double BACKGROUND_COUNTS = 20.0;     ///< 背景光子计数 (太阳/电子学暗计数)
constexpr double LASER_ENERGY_MJ = 2.0;        ///< 单脉冲激光能量 E (mJ)
constexpr double PLUME_CENTER_M = 120.0;       ///< Gaussian 烟羽中心距离 (m)
constexpr double PLUME_STRENGTH_DRY = 0.55;    ///< 干态气溶胶消光强度峰值 (km^-1)
constexpr double HUMIDITY_GROWTH = 1.35;       ///< 湿度散射放大倍率 (湿/干 β 比值)
constexpr double PM25_THRESHOLD = 60.0;        ///< 业务热点阈值, 超过即判 hot cell
constexpr double DEVICE_HEIGHT_M = 18.0;       ///< LiDAR 设备离地高度, 用于 ENU 上向分量

/**
 * @brief 单条射线的全部输入与"真值"廓线。
 *
 * 包含 LiDAR 实测可见量 (raw_counts, background, energy, overlap, rh)
 * 以及真值 (true_ext_dry / true_ext_wet / true_beta)。真值仅供事后
 * 与反演结果对照学习之用, 反演算法**不读真值**, 完全从原始信号出发。
 */
struct SingleRayProfile {
    std::vector<double> ranges_m;       ///< 距离 bin 中心点序列 (单位 m)
    std::vector<double> raw_counts;     ///< 探测器原始 photon counts
    double background = 0.0;             ///< 背景光子计数 (常数, 所有 bin 共享)
    double energy_mj = 0.0;              ///< 单脉冲激光能量 (mJ)
    std::vector<double> overlap;         ///< overlap 函数 O(R) ∈ (0,1], 近距小、远距→1
    std::vector<double> rh_percent;      ///< 每个 bin 的相对湿度 RH (%)
    double azimuth_deg = 0.0;            ///< 射线方位角 (度), 北顺时针
    double elevation_deg = 0.0;          ///< 射线仰角 (度)
    /// @name 真值 (仅供反演结果对照), 算法本身不使用
    /// @{
    std::vector<double> true_ext_dry;   ///< 干态气溶胶消光系数 α_dry (km^-1)
    std::vector<double> true_ext_wet;   ///< 湿态总消光系数 α_mol+α_aerwet (km^-1)
    std::vector<double> true_beta;      ///< 总后向散射系数 β_total (km^-1 sr^-1)
    /// @}
};

// --------------------------------------------------------------------------
// ---- §19.1 正向仿真: 构造单条射线的气溶胶场 + LiDAR 方程生成 photon counts ----
// --------------------------------------------------------------------------

/**
 * @brief 正向仿真: 给定 (az, el, ranges) 构造一条"被测"信号。
 *
 * 物理模型:
 *  - 分子分量按指数随高度衰减 (标高 8 km):
 *      α_mol(h) = 0.012 · exp(-h/8000),  β_mol = α_mol / 8   (瑞利 lidar ratio ≈ 8 sr)
 *  - 气溶胶干态 = 本底(0.04) + 边界层(0.070·exp(-h/800)) + Gaussian 烟羽:
 *      α_aer,dry(R) = 0.04 + boundary(h) + PLUME_STRENGTH · exp(-(R-120)^2 / (2·25^2))
 *  - 湿度使气溶胶消光放大: α_aer,wet = α_aer,dry · HUMIDITY_GROWTH
 *  - 总后向散射 β = β_mol + α_aer,wet / S   (S = 气溶胶 lidar ratio)
 *  - overlap 函数: O(R) = clamp((R/220)^0.82, 0.32, 1.0)   (近距收光不全)
 *
 * LiDAR 方程 (单脉冲 photon counts):
 * @note
 *   N(R) = C·E·O(R) · β(R) / R^2 · exp(-2·∫₀^R α(r) dr) + N_bg + noise
 *   - 1/R^2     来自球面几何 (散开立体角 ~ 1/R^2)
 *   - exp(-2τ)  双程透过率, τ = 累积光学厚度
 *   - noise     ~ sqrt(N_signal)·0.08  (近似 shot + 探测器噪声, 标准差 8 %)
 *
 * @param ranges 距离 bin 中心 (m)
 * @param azimuth_deg 方位角 (度)
 * @param elevation_deg 仰角 (度)
 * @param rng   随机数发生器, 用于射击噪声
 * @return 单条射线的可见量与 (供对照的) 真值
 */
inline SingleRayProfile simulate_single_ray(const std::vector<double>& ranges,
                                     double azimuth_deg, double elevation_deg, Rng& rng) {
    SingleRayProfile p;
    p.ranges_m = ranges;
    p.azimuth_deg = azimuth_deg;
    p.elevation_deg = elevation_deg;
    p.background = BACKGROUND_COUNTS;
    p.energy_mj = LASER_ENERGY_MJ;

    std::size_t n = ranges.size();
    double dr_km = (ranges.size() > 1 ? ranges[1] - ranges[0] : 30.0) / 1000.0;  // 距离 bin 步长 (km)
    double el_rad = deg2rad(elevation_deg);

    std::vector<double> altitude(n);          // 每个 bin 对应的离地高度 h = R·sin(el)
    std::vector<double> mol_ext(n);           // 分子消光系数 α_mol
    std::vector<double> mol_beta(n);          // 分子后向散射 β_mol
    std::vector<double> aerosol_ext_dry(n);   // 干态气溶胶消光 α_aer,dry
    std::vector<double> aerosol_ext_wet(n);   // 湿态气溶胶消光 α_aer,wet
    std::vector<double> true_beta_total(n);   // 真值 β_total = β_mol + β_aer
    std::vector<double> true_ext_wet(n);      // 真值 α_total = α_mol + α_aer,wet
    std::vector<double> overlap(n);           // overlap 函数 O(R)

    for (std::size_t i = 0; i < n; ++i) {
        altitude[i] = ranges[i] * std::sin(el_rad);
        // 分子 (瑞利) 分量: α_mol = 0.012·exp(-h/8km),  β_mol ≈ α_mol/8
        mol_ext[i] = 0.012 * std::exp(-altitude[i] / 8000.0);
        mol_beta[i] = mol_ext[i] / 8.0;

        // 边界层本底随高度按标高 800 m 衰减
        double boundary = 0.070 * std::exp(-altitude[i] / 800.0);
        // Gaussian 烟羽: 中心 120 m, σ=25 m
        double plume = PLUME_STRENGTH_DRY * gaussian1d(ranges[i], PLUME_CENTER_M, 25.0);
        // 干态气溶胶消光 = 本底 + 边界层 + 烟羽
        aerosol_ext_dry[i] = 0.04 + boundary + plume;
        // 湿态放大 (RH 越高, 亲水性气溶胶吸湿增长, 散射增强)
        aerosol_ext_wet[i] = aerosol_ext_dry[i] * HUMIDITY_GROWTH;

        // 总后向散射: β = β_mol + α_aer,wet / S   (S = LIDAR_RATIO_SR)
        true_beta_total[i] = mol_beta[i] + aerosol_ext_wet[i] / LIDAR_RATIO_SR;
        // 总消光: α = α_mol + α_aer,wet
        true_ext_wet[i] = mol_ext[i] + aerosol_ext_wet[i];

        // overlap 函数 O(R): 近距(<220 m)几何收光不完整, 远距→1
        overlap[i] = clamp(std::pow(ranges[i] / 220.0, 0.82), 0.32, 1.0);
    }

    // ---- 由真值 α/β 经 LiDAR 方程正向生成 photon counts ----
    std::vector<double> raw_counts(n);
    double optical_depth = 0.0;   // 累积光学厚度 τ(R) = Σ α(r)·dr
    for (std::size_t i = 0; i < n; ++i) {
        // 累积光学厚度 (前向求和, 用于双程衰减 exp(-2τ))
        optical_depth += true_ext_wet[i] * dr_km;
        double range_km = ranges[i] / 1000.0;
        // LiDAR 方程: N_signal = C·E·O·β/R² · exp(-2τ)
        double signal = SYSTEM_CONSTANT * LASER_ENERGY_MJ * overlap[i] *
                        true_beta_total[i] / (range_km * range_km + 1e-12) *
                        std::exp(-2.0 * optical_depth);
        // 射击噪声: σ ≈ sqrt(N)·0.08 (8 % 相对精度, 典型探测器)
        double shot_sigma = std::sqrt(std::max(signal, 1.0)) * 0.08;
        // 加性噪声: 信号 + 背景 + 高斯抖动
        double noisy = signal + p.background + rng.next_gauss(0.0, shot_sigma);
        // 物理约束: 至少不会被背景"压成负数"
        raw_counts[i] = std::max(noisy, p.background + 0.1);
    }

    p.raw_counts = raw_counts;
    p.overlap = overlap;
    p.true_ext_dry = aerosol_ext_dry;
    p.true_ext_wet = true_ext_wet;
    p.true_beta = true_beta_total;

    // RH 沿距离作缓变假设 72%->82%, 用于后续湿度修正
    p.rh_percent.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        p.rh_percent[i] = 72.0 + 10.0 * static_cast<double>(i) / static_cast<double>(n - 1);
    }

    return p;
}

// --------------------------------------------------------------------------
// ---- §19.2 背景光扣除:  N'(R) = N(R) - N_bg --------------------------------
// --------------------------------------------------------------------------

/**
 * @brief 从每个 bin 中扣除背景光子计数。
 *
 * 背景包含太阳散射光与探测器暗计数, 在一条射线上近似为常数 N_bg。
 * 扣除后的净信号 N' 才是真正的 LiDAR 回波。下限保护避免出现负数
 * (实际数据中噪声可能使个别 bin 略低于背景)。
 *
 * @param raw 原始 photon counts
 * @param bg  背景计数 N_bg (常数)
 * @return 扣背景后的净信号 N' (每 mJ 之前的"counts")
 */
inline std::vector<double> step2_background_subtract(const std::vector<double>& raw, double bg) {
    std::vector<double> out(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
        out[i] = std::max(raw[i] - bg, 0.0);
    }
    return out;
}

// --------------------------------------------------------------------------
// ---- §19.3 发射能量归一化:  P1(R) = N'(R) / E -----------------------------
// --------------------------------------------------------------------------

/**
 * @brief 用激光单脉冲能量做归一化。
 *
 * 不同脉冲能量 E 不同时, 直接比较 photon counts 会有系统性偏差。
 * 归一化后 P1 = N'/E 表示"每毫焦能量"对应的回波, 跨脉冲可比。
 *
 * @param in        净信号 N'
 * @param energy_mj 单脉冲激光能量 (mJ)
 * @return 归一化信号 P1
 */
inline std::vector<double> step3_energy_normalize(const std::vector<double>& in, double energy_mj) {
    std::vector<double> out(in.size());
    double e = std::max(energy_mj, 1e-6);   // 防止除零
    for (std::size_t i = 0; i < in.size(); ++i) out[i] = in[i] / e;
    return out;
}

// --------------------------------------------------------------------------
// ---- §19.4 RCS 距离平方校正:  X(R) = P1(R) · R²  (km²) -------------------
// --------------------------------------------------------------------------

/**
 * @brief 距离平方校正得到 LiDAR 量 X (Range-Corrected Signal)。
 *
 * LiDAR 方程中回波按 1/R² 衰减 (球面发散), 为了让信号"主要反映散射体本身",
 * 必须乘以 R² 把几何因子消掉。X(R) 量纲为 km², 后续用于 Klett 反演。
 *
 * X 与 β 的关系:   X(R) = C·O(R)·β(R)·exp(-2τ(R))
 *
 * @param p2     归一化信号 P1=R^0 校正前的信号
 * @param ranges 距离序列 (m)
 * @return 距离平方校正信号 X = P1·R^2  (km^2)
 */
inline std::vector<double> step4_rcs(const std::vector<double>& p2, const std::vector<double>& ranges) {
    std::vector<double> out(p2.size());
    for (std::size_t i = 0; i < p2.size(); ++i) {
        double rkm = ranges[i] / 1000.0;
        out[i] = p2[i] * rkm * rkm;
    }
    return out;
}

// --------------------------------------------------------------------------
// ---- §19.5 overlap 校正:  X'(R) = X(R) / O(R) ----------------------------
// --------------------------------------------------------------------------

/**
 * @brief 补偿 LiDAR 近距 overlap 不完整。
 *
 * 近距 (本例 <220 m) 激光束与望远镜视场没完全重合, 导致探测器漏收一部分回波。
 * 用经验函数 O(R) 做除法校正后, X'(R) ∝ β(R)·exp(-2τ(R)), 可以直接送入 Klett。
 *
 * @param rcs     距离平方校正信号 X
 * @param overlap overlap 函数 O(R)
 * @return overlap 校正后的洁净信号 X' = X / O(R)
 */
inline std::vector<double> step5_overlap_correct(const std::vector<double>& rcs, const std::vector<double>& overlap) {
    std::vector<double> out(rcs.size());
    for (std::size_t i = 0; i < rcs.size(); ++i) {
        out[i] = rcs[i] / std::max(overlap[i], 0.15);   // O 下限保护
    }
    return out;
}

// --------------------------------------------------------------------------
// ---- §19.6–19.7 简化 Klett 远->近 递推反演 -------------------------------
//   beta_total(i) = X'(i) / (X'_ref/beta_ref + 2S · ∫_{R_i}^{R_far} X'(r) dr)
//   beta_aerosol  = beta_total - beta_mol              (剔除分子分量)
//   alpha         = alpha_mol + S · beta_aerosol       (利用 lidar ratio S)
//
//   ref_beta_aerosol 决定整条廓线的绝对量级——这是 Klett 法的固有特性
//   (反演对远端边界条件 β_ref 敏感, 远端假设越接近真值越准)。
// --------------------------------------------------------------------------

/**
 * @brief Klett 反演的输出: α (消光) 与 β (后向散射) 廓线。
 */
struct KlettResult {
    std::vector<double> alpha;   ///< 总消光系数 α (km^-1)
    std::vector<double> beta;    ///< 总后向散射系数 β (km^-1 sr^-1)
};

/**
 * @brief 用简化 Klett 法从校正信号 X' 反演 α、β。
 *
 * Klett 算法的核心是把 LiDAR 方程转换成伯努利型 ODE, 解析解为:
 * @note
 *   β(R) = X(R) / ( X(R_ref)/β_ref + 2·S · ∫_{R}^{R_ref} X(r) dr )
 * 其中:
 *   - X(R)   是 §19.5 校正后的信号
 *   - S      是 lidar ratio = α/β (气溶胶约 40–70 sr)
 *   - β_ref  远端边界处的 β, 反演只能确定相对廓线, β_ref 决定绝对标尺
 *
 * 本实现采用"远端附近几 bin 的平均"作为参考量 X_ref、β_ref, 然后从远端
 * 往近端做累加积分 (cum_integral 从远往近增加), 完成远->近递推。
 *
 * 物理约束:
 *   - α 不低于分子背景 α_mol
 *   - α 不高于 0.45 km^-1 (相当于水云般浓密的极端情形, 避免反演发散)
 *
 * @param corrected           §19.5 校正信号 X'
 * @param ranges              距离序列 (m)
 * @param mol_ext             分子消光廓线 α_mol (km^-1)
 * @param mol_beta            分子后向散射廓线 β_mol
 * @param lidar_ratio_sr      气溶胶 lidar ratio S (sr)
 * @param ref_beta_aerosol    远端参考的"气溶胶"部分 β_aer,ref (绝对量级锚点)
 * @return α 与 β 廓线 (km^-1 / km^-1 sr^-1)
 */
inline KlettResult step6_7_klett(const std::vector<double>& corrected,
                          const std::vector<double>& ranges,
                          const std::vector<double>& mol_ext,
                          const std::vector<double>& mol_beta,
                          double lidar_ratio_sr,
                          double ref_beta_aerosol) {
    std::size_t n = corrected.size();
    double dr_km = (n > 1 ? ranges[1] - ranges[0] : 30.0) / 1000.0;

    // 远端参考: 取倒数 2 个 bin 的均值 (作为 β_ref 的锚点)
    std::size_t ref_index = (n >= 2 ? n - 2 : 0);
    double x_ref = 0.0;
    int ref_count = 0;
    for (std::size_t i = ref_index; i < n; ++i) { x_ref += corrected[i]; ++ref_count; }
    x_ref = std::max(x_ref / std::max(ref_count, 1), 1e-9);   // 防 0

    // 参考处的分子 β 均值, 用于把气溶胶参考与分子参考合并
    double mol_beta_ref = 0.0;
    for (std::size_t i = ref_index; i < n; ++i) mol_beta_ref += mol_beta[i];
    mol_beta_ref /= std::max(ref_count, 1);

    // β_ref = β_mol,ref + β_aer,ref  (远端总后向散射的"标尺")
    double beta_ref = mol_beta_ref + ref_beta_aerosol;
    // seed = X_ref / β_ref, 进入伯努利解的分母
    double seed = x_ref / std::max(beta_ref, 1e-9);

    std::vector<double> beta(n, 0.0), alpha(n, 0.0);
    double cum_integral = 0.0;   // 从远端往近端累加的 ∫X dr
    for (int idx = static_cast<int>(n) - 1; idx >= 0; --idx) {
        std::size_t i = static_cast<std::size_t>(idx);
        if (idx < static_cast<int>(n) - 1) {
            // 梯形积分 X(r)·dr (相邻两 bin 平均 × 步长)
            double seg = (corrected[i] + corrected[i + 1]) / 2.0 * dr_km;
            cum_integral += seg;
        }
        // Klett 解析解分母: seed + 2S·∫X dr
        double denom = seed + 2.0 * lidar_ratio_sr * cum_integral;
        // 解析解 β_total = X / 分母
        double beta_total = corrected[i] / std::max(denom, 1e-12);
        // 减掉分子分量得到气溶胶 β (≥ 0)
        double beta_aerosol = std::max(beta_total - mol_beta[i], 0.0);
        beta[i] = mol_beta[i] + beta_aerosol;
        // 重新加回分子消光, α = α_mol + S·β_aer
        alpha[i] = mol_ext[i] + lidar_ratio_sr * beta_aerosol;
        // 物理约束: 不低于分子背景, 不高于 0.45 (理想厚云)
        alpha[i] = clamp(alpha[i], mol_ext[i], 0.45);
    }
    return {alpha, beta};
}

// --------------------------------------------------------------------------
// ---- §19.8 湿度修正:  f(RH) = ((1-RH_ref)/(1-RH))^γ, α_dry = α / f(RH) -----
// --------------------------------------------------------------------------

/**
 * @brief 用 f(RH) 经验模型把湿态消光反推成干态消光。
 *
 * 亲水性气溶胶 (硫酸盐、硝酸盐、海盐) 在高湿下吸水增长, 光被额外散射,
 * LiDAR 反演到的 α 是"湿态"。要让 LiDAR 的 α 与地面干燥 PM 监测站可比,
 * 必须"折干"。
 *
 * 国际通用的双参 f(RH) 公式:
 * @note
 *   f(RH) = ((1 - RH_ref) / (1 - RH))^γ
 *   - RH_ref = 0.35 (35 % 干燥参考)
 *   - γ      = 0.45 (城市混合型气溶胶典型经验值)
 *   - RH 越接近 1, f(RH) 越大, 折干后 α_dry 越小
 *
 * @param alpha     湿态消光 α (km^-1)
 * @param rh_percent 相对湿度序列 (%)
 * @param gamma     f(RH) 经验指数, 默认 0.45
 * @param rh_ref    干燥参考湿度 (0–1), 默认 0.35
 * @return 干态消光 α_dry = α / f(RH)
 */
inline std::vector<double> step8_humidity_correct(const std::vector<double>& alpha,
                                           const std::vector<double>& rh_percent,
                                           double gamma = 0.45, double rh_ref = 0.35) {
    std::vector<double> out(alpha.size());
    for (std::size_t i = 0; i < alpha.size(); ++i) {
        double rh = clamp(rh_percent[i] / 100.0, 0.05, 0.98);   // RH ∈ [5 %, 98 %]
        // f(RH) = ((1-RH_ref)/(1-RH))^γ
        double f = std::pow((1.0 - rh_ref) / std::max(1.0 - rh, 1e-3), gamma);
        out[i] = alpha[i] / f;   // 折干
    }
    return out;
}

// --------------------------------------------------------------------------
// ---- §19.9 干态消光 -> PM2.5 / PM10 (教学线性模型) ------------------------
//   PM2.5 = 500 · α_dry + 10            (μg/m³, 经验线性定标)
//   PM10  = 1.6 · PM2.5                 (粗略 PM10 ≈ 1.6 × PM2.5)
// --------------------------------------------------------------------------

/**
 * @brief PM 反算结果容器。
 */
struct PmResult {
    std::vector<double> pm25;   ///< PM2.5 浓度序列 (μg/m³)
    std::vector<double> pm10;   ///< PM10  浓度序列 (μg/m³)
};

/**
 * @brief 把干态消光线性映射为 PM 浓度。
 *
 * 干态气溶胶消光系数 α_dry 与 PM 质量浓度近似线性 (单位质量散射效率
 * 在边界层变化较小, 一阶定标即可), 故用线性模型:
 *   PM2.5 = 500·α_dry + 10     (offset 10 表示背景细颗粒本底)
 *   PM10  = 1.6 · PM2.5        (经验比值, 与组分有关)
 *
 * @note 实际工程里这步通常用机器学习模型, 这里为教学保留线性形式。
 * @param alpha_dry §19.8 折干后的消光 α_dry (km^-1)
 * @return PM2.5 与 PM10 序列 (μg/m³)
 */
inline PmResult step9_pm_estimate(const std::vector<double>& alpha_dry) {
    std::vector<double> pm25(alpha_dry.size());
    std::vector<double> pm10(alpha_dry.size());
    for (std::size_t i = 0; i < alpha_dry.size(); ++i) {
        pm25[i] = 500.0 * alpha_dry[i] + 10.0;   // 线性定标
        pm10[i] = 1.6 * pm25[i];                 // 经验比
    }
    return {pm25, pm10};
}

} // namespace example_1d_ray
