// Internal include fragment: preprocessing, inversion, humidity, projection, and calibration primitives.
// ====================================================================================
// ---- L1 预处理 / L1 preprocessing ----------------------------------------------------------------
// 将探测器原始计数的"伪信号"还原为可比对的物理量：
// 1. 减背景 → 2. 除以激光能量归一 → 3. 除 overlap → 4. 乘 r² 校正几何发散 → 衰减后向散射。
// 同时输出 SNR 与质控标志（近场 partial overlap、激光能量过低、近场 SNR 过低）。
// ====================================================================================

/**
 * @brief 对单条 LiDAR 廓线执行 L1 预处理。
 *
 * 处理链路：
 *   background_corrected = max(raw - background, 0)
 *   energy_normalized    = background_corrected / laser_energy
 *   overlap_corrected    = energy_normalized / overlap(r)
 *   attenuated_backscatter = overlap_corrected · r²
 *   SNR                  = background_corrected / sqrt(raw + background)
 *
 * 其中乘以 r² 反转了 LiDAR 方程中的 1/r² 几何衰减（未校正透过率，因此仍含双程大气透过率，称为"衰减"后向散射）。
 * SNR 用 Poisson 噪声近似（σ=√N），用于决定哪一段 bin 可进入反演。
 *
 * @param[in] profile 单条原始 LiDAR 廓线。
 * @return 预处理结果（L1 信号、衰减后向散射、SNR、QC 标志）。
 */
PreprocessResult preprocess_profile(const LidarProfile& profile) {
    PreprocessResult output;
    for (std::size_t index = 0; index < profile.raw_counts.size(); ++index) {
        // 减背景计数，并截断到 0，避免负值
        double background_corrected = std::max(profile.raw_counts[index] - profile.background_counts, 0.0);
        // 按脉冲能量归一，抵消每次发射能量抖动
        double energy_normalized = background_corrected / std::max(profile.laser_energy_mj, 1e-6);
        // 修正双轴 overlap，还原真实接收信号（下限 0.15 避免近场过度放大）
        double overlap_corrected = energy_normalized / std::max(profile.overlap[index], 0.15);
        double range_km = profile.ranges_m[index] / 1000.0;
        // 乘 r² 抵消 LiDAR 方程中的几何扩散衰减（至此得到含大气透过率的衰减后向散射）
        double attenuated = overlap_corrected * range_km * range_km;
        // 信噪比：基于 Poisson 计数噪声近似 σ ≈ √(raw + background)
        double signal_to_noise = background_corrected / std::max(std::sqrt(profile.raw_counts[index] + profile.background_counts), 1.0);

        output.l1_signal.push_back(background_corrected);
        output.attenuated_backscatter.push_back(std::max(attenuated, 1e-9));
        output.snr.push_back(signal_to_noise);
    }

    // ---- 质控标志 / QC flags ----
    // 近场 overlap 过低：前 3 个 bin 最小 overlap < 0.4 表示近场盲区
    if (profile.overlap.size() >= 3 && *std::min_element(profile.overlap.begin(), profile.overlap.begin() + 3) < 0.4) {
        output.qc_flags.push_back("near-range-partial-overlap");
    }
    // 激光能量过低：脉冲退化或老化导致信噪下降
    if (profile.laser_energy_mj < 0.93) {
        output.qc_flags.push_back("low-laser-energy");
    }
    // 近场 SNR 过弱：前 4 bin 平均 SNR < 3 通常意味着近距离回波质量不可用
    if (output.snr.size() >= 4 && mean(std::vector<double>(output.snr.begin(), output.snr.begin() + 4)) < 3.0) {
        output.qc_flags.push_back("weak-near-range-snr");
    }
    return output;
}

// ====================================================================================
// ---- Fernald 反演 / Fernald inversion ------------------------------------------------------------
// 由"衰减后向散射"反推气溶胶消光与后向散射廓线。
// 因 LiDAR 方程求解是 ill-posed（含双程透过率 exp 项），需要远端参考点锚定边界。
// 本实现采用从远端到近端的反向积分法。
// ====================================================================================

/**
 * @brief 对衰减后向散射执行 Fernald 反演，求气溶胶消光与气溶胶后向散射廓线。
 *
 * 利用远端（远场）假设：远端处气溶胶很弱，可以认为信号近似对应纯分子+少量气溶胶参考值。
 * 算法步骤：
 *   1. 取末端 5 个 bin 平均信号 ref_signal，配合参考点后向散射 ref_beta 求标定系数 scale = ref_beta / ref_signal；
 *   2. 将整段衰减后向散射乘 scale 得到估计的总后向散射；
 *   3. 自远端向近端反向迭代：先以透过率 exp(2τ) 复原（注意这里用 exp(+2τ)，因为衰减后向散射是被透过率压缩过的），
 *      再减去分子后向散射得到气溶胶 β；
 *   4. 由 β·S（气溶胶激光比）计算气溶胶消光，加上分子消光得到总消光；
 *   5. 限制总消光在 [分子消光, 0.45] 之间，防止数值不稳。
 *
 * @param[in] profile 单条 LiDAR 廓线（用于读取分子后向散射/消光与距离 bin）。
 * @param[in] attenuated_backscatter 预处理后的衰减后向散射廓线。
 * @param[in] aerosol_lidar_ratio_sr 气溶胶激光比（sr），典型 PM 用 40~60 sr。
 * @param[in] reference_aerosol_backscatter 远端参考点的气溶胶后向散射（1/m·sr），通常取很小的本底值。
 * @return (总消光, 气溶胶后向散射) 廓线对。
 */
std::pair<std::vector<double>, std::vector<double>> run_fernald_inversion(
    const LidarProfile& profile,
    const std::vector<double>& attenuated_backscatter,
    double aerosol_lidar_ratio_sr,
    double reference_aerosol_backscatter
) {
    // 参考点：取末端 5 个 bin（远场气溶胶稀薄）作为锚定
    int ref_index = std::max<int>(static_cast<int>(attenuated_backscatter.size()) - 5, 0);
    std::vector<double> ref_signal_slice(attenuated_backscatter.begin() + ref_index, attenuated_backscatter.end());
    std::vector<double> ref_beta_slice(profile.molecular_backscatter.begin() + ref_index, profile.molecular_backscatter.end());
    double ref_signal = std::max(mean(ref_signal_slice), 1e-9);
    // 参考点后向散射：分子 + 给定的小量气溶胶本底
    double ref_beta = mean(ref_beta_slice) + reference_aerosol_backscatter;
    // 尺度因子：把后续衰减后向散射缩放到近似总后向散射
    double scale = ref_beta / ref_signal;

    std::vector<double> extinction(attenuated_backscatter.size(), 0.0);
    std::vector<double> aerosol_backscatter(attenuated_backscatter.size(), 0.0);
    std::vector<double> scaled_signal;
    scaled_signal.reserve(attenuated_backscatter.size());
    for (double value : attenuated_backscatter) {
        scaled_signal.push_back(std::max(value * scale, 1e-9));
    }
    double step_km = profile.ranges_m.size() > 1 ? (profile.ranges_m[1] - profile.ranges_m[0]) / 1000.0 : 0.05;
    double optical_depth = 0.0;
    // 反向积分：从远端（信号最弱且气溶胶稀薄）向近端反推
    for (int index = static_cast<int>(scaled_signal.size()) - 1; index >= 0; --index) {
        // 复原总后向散射：衰减信号 × exp(+2τ) 抵消 LiDAR 方程的双程透过率
        double total_backscatter = std::max(scaled_signal[index] * std::exp(2.0 * optical_depth), profile.molecular_backscatter[index]);
        // 气溶胶后向散射 = 总 - 分子
        double aerosol_beta = std::max(total_backscatter - profile.molecular_backscatter[index], 0.0);
        // 总消光 = 分子消光 + S·气溶胶 β（S 为气溶胶激光比）
        double total_extinction = profile.molecular_extinction[index] + aerosol_lidar_ratio_sr * aerosol_beta;
        // 限制总消光：下限为分子消光（不允许低于纯大气），上限 0.45（防止数值爆炸）
        total_extinction = std::min(std::max(total_extinction, profile.molecular_extinction[index]), 0.45);
        aerosol_backscatter[index] = aerosol_beta;
        extinction[index] = total_extinction;
        // 累积光学厚度 τ（向近端积分）
        optical_depth += total_extinction * step_km;
    }
    return {extinction, aerosol_backscatter};
}

// ====================================================================================
// ---- 湿度校正 / Humidity correction ----------------------------------------------------------------
// 相对湿度升高会使气溶胶颗粒吸水增长（κ-Köhler 物理过程），增强其散射/消光截面。
// 反演过程中如果直接使用"湿"消光，会用错误的 PM 浓度。本节按典型吸湿增长因子将湿消光除回"干"参考状态。
// ====================================================================================

/**
 * @brief 计算吸湿增长因子 g(RH)（无量纲）。
 *
 * 采用简化的 κ-Köhler 形式：
 *   g(RH) = 1 + κ · max( RH/(1-RH) - RH_dry/(1-RH_dry), 0 ) · 0.18
 * 其中 RH/(1-RH) 称为水分活度对应量，κ 反映气溶胶化学组分吸湿性（硫酸盐大、矿尘小）。
 * 因子 0.18 为压缩系数（防止 g 增长过快超出消光实测范围）。
 *
 * @param[in] relative_humidity 当前 RH（0~1）。
 * @param[in] dry_reference_rh 干参考状态的 RH（约 0.35~0.4），即认为是"无吸湿"基线。
 * @param[in] hygroscopicity 吸湿性参数 κ（典型 0.1~0.5）。
 * @return 增长因子（≥ 1）。
 */
double growth_factor(double relative_humidity, double dry_reference_rh, double hygroscopicity) {
    // RH 夹到 [0.05, 0.98]：过低无意义，过高会导致 1-RH 发散
    double rh = std::min(std::max(relative_humidity, 0.05), 0.98);
    double dry_ratio = dry_reference_rh / std::max(1.0 - dry_reference_rh, 0.02);
    double humid_ratio = rh / std::max(1.0 - rh, 0.02);
    return std::max(1.0, 1.0 + hygroscopicity * std::max(humid_ratio - dry_ratio, 0.0) * 0.18);
}

/**
 * @brief 对消光廓线做吸湿校正，从"湿"消光还原为"干"消光。
 *
 * 数学：干消光 = 湿消光 / g(RH)。后续 PM 反演使用干消光更稳定（去除了 RH 引起的虚假变化）。
 *
 * @param[in] extinction 原始（湿）消光廓线。
 * @param[in] relative_humidity 当前 RH。
 * @param[in] dry_reference_rh 干参考 RH。
 * @param[in] hygroscopicity 吸湿参数 κ。
 * @return 逐 bin 的干消光廓线。
 */
std::vector<double> apply_humidity_correction(const std::vector<double>& extinction, double relative_humidity, double dry_reference_rh, double hygroscopicity) {
    double factor = growth_factor(relative_humidity, dry_reference_rh, hygroscopicity);
    std::vector<double> output;
    output.reserve(extinction.size());
    for (double value : extinction) {
        output.push_back(value / factor);
    }
    return output;
}

// ====================================================================================
// ---- ENU 坐标转换 / ENU coordinate conversion ----------------------------------------------------
// 将 LiDAR 量测的距离 bin 与方位/仰角转换为以站点为原点的 ENU（East-North-Up）笛卡尔坐标，
// 用于三维热点定位与可视化。
// ====================================================================================

/**
 * @brief 把每条距离 bin 转换为 ENU 三维点。
 *
 * 转换公式（球坐标 → 直角 ENU）：
 *   East  = r · cos(elev) · sin(azim)
 *   North = r · cos(elev) · cos(azim)
 *   Up    = r · sin(elev)
 * 其中仰角沿天顶为 90°，方位角沿正北为 0° 顺时针增加。
 *
 * @param[in] profile 含方位角、仰角、距离的 LiDAR 廓线。
 * @return ENU 坐标点列表（每行 [East_m, North_m, Up_m]）。
 */
std::vector<std::vector<double>> profile_bins_to_enu(const LidarProfile& profile) {
    std::vector<std::vector<double>> output;
    output.reserve(profile.ranges_m.size());
    double azimuth_rad = profile.azimuth_deg * std::numbers::pi / 180.0;
    double elevation_rad = profile.elevation_deg * std::numbers::pi / 180.0;
    for (double range_m : profile.ranges_m) {
        // 水平投影 = r · cos(elev)，再分解到 East/North
        double east = range_m * std::cos(elevation_rad) * std::sin(azimuth_rad);
        double north = range_m * std::cos(elevation_rad) * std::cos(azimuth_rad);
        // 铅垂分量 = r · sin(elev)
        double up = range_m * std::sin(elevation_rad);
        output.push_back({east, north, up});
    }
    return output;
}

// ====================================================================================
// ---- PM 校准特征表 / PM calibration feature table -------------------------------------------------
// 将一次完整战役中按时间戳聚合：合并 stare 反演的近地面干消光、衰减后向散射、
// 以及同时间戳下 PPI 的 hotspot 代理量，并匹配地面站真值，得到用于校准 PM 模型的特征向量。
// ====================================================================================

/**
 * @brief 按时间戳构建 PM 校准用的特征样本表。
 *
 * 流程：
 *   1. 把地面观测按时间戳建立索引；
 *   2. 把 processed_profiles 按时间戳分离为 stare / PPI 两个子集；
 *   3. 对每个有 ground 配对的 stare 时间戳：
 *      - 取近地面若干 bin（surface_bin_count）的干消光均值作为 surface feature；
 *      - 若有 PPI，用所有 PPI 方位中 surface 切片的最大干消光作为 hotspot_proxy；
 *      - 否则用 stare 切片的最大值代替 hotspot_proxy；
 *      - 组装特征向量 [1.0 (截距), 干消光均值, RH, hotspot_proxy]，并附 PM2.5/PM10 真值；
 *
 * @param[in] processed_profiles 已反演的所有处理廓线（含 stare 与 PPI）。
 * @param[in] ground_measurements 同步期地面观测序列。
 * @param[in] surface_bin_count 近地面 bin 数量（商用尺度下一般 4 个，对应约 400 m）。
 * @return 时间序列特征样本列表，每条对应一个时刻。
 */
std::vector<FeatureSample> build_timestamp_feature_table(
    const std::vector<ProcessedProfile>& processed_profiles,
    const std::vector<GroundMeasurement>& ground_measurements,
    int surface_bin_count
) {
    // 1) 按 timestamp 索引地面观测
    std::map<std::string, GroundMeasurement> ground_by_timestamp;
    for (const auto& measurement : ground_measurements) {
        ground_by_timestamp[measurement.timestamp] = measurement;
    }

    // 2) 按 timestamp 分离 stare/PPI 已处理廓线（用指针避免深拷贝）
    std::map<std::string, std::vector<const ProcessedProfile*>> stare_by_timestamp;
    std::map<std::string, std::vector<const ProcessedProfile*>> ppi_by_timestamp;
    for (const auto& processed : processed_profiles) {
        if (processed.profile.scan_mode == "stare") {
            stare_by_timestamp[processed.profile.timestamp].push_back(&processed);
        } else if (processed.profile.scan_mode == "ppi") {
            ppi_by_timestamp[processed.profile.timestamp].push_back(&processed);
        }
    }

    std::vector<FeatureSample> samples;
    for (const auto& [timestamp, stare_profiles] : stare_by_timestamp) {
        // 仅当存在地面观测时才生成样本（无 ground 无法训练）
        if (stare_profiles.empty() || !ground_by_timestamp.contains(timestamp)) {
            continue;
        }
        const auto& ground = ground_by_timestamp.at(timestamp);
        const auto* stare_profile = stare_profiles.front();
        // 取近地面 bin 的干消光与衰减后向散射切片
        std::vector<double> stare_slice(stare_profile->dry_extinction.begin(), stare_profile->dry_extinction.begin() + std::min<int>(surface_bin_count, static_cast<int>(stare_profile->dry_extinction.size())));
        std::vector<double> attenuated_slice(stare_profile->attenuated_backscatter.begin(), stare_profile->attenuated_backscatter.begin() + std::min<int>(surface_bin_count, static_cast<int>(stare_profile->attenuated_backscatter.size())));
        // 热点代理：若 PPI 存在，取所有方位 surface 切片的最大值；否则退回 stare 切片最大值
        double hotspot_proxy = 0.0;
        if (ppi_by_timestamp.contains(timestamp)) {
            for (const auto* profile : ppi_by_timestamp.at(timestamp)) {
                hotspot_proxy = std::max(hotspot_proxy, *std::max_element(profile->dry_extinction.begin(), profile->dry_extinction.begin() + std::min<int>(surface_bin_count, static_cast<int>(profile->dry_extinction.size()))));
            }
        } else {
            hotspot_proxy = *std::max_element(stare_slice.begin(), stare_slice.end());
        }
        // features = [截距 1.0, 近地面干消光均值, RH, hotspot_proxy]
        samples.push_back(FeatureSample{
            timestamp,
            {1.0, mean(stare_slice), ground.relative_humidity, hotspot_proxy},
            mean(stare_slice),
            mean(attenuated_slice),
            hotspot_proxy,
            !stare_profile->profile.site_id.empty() ? stare_profile->profile.site_id : (!ground.site_id.empty() ? ground.site_id : "default-site"),
            ground.pm25_ugm3,
            ground.pm10_ugm3,
            ground.relative_humidity,
            ground.wind_speed_ms,
        });
    }
    return samples;
}

// ====================================================================================
// ---- 最小二乘求解 / Linear algebra primitives for OLS ---------------------------------------------
// PM 校准本质上是最小二乘问题 min ||Xw - y||²，其闭式解的对应正规方程为 X^T X w = X^T y。
// 这里用 Gauss-Jordan 全主元消元解该线性方程组，并给 Gram 矩阵加微小正则项防止奇异。
// ====================================================================================

/**
 * @brief 用 Gauss-Jordan 全主元消元法解线性方程组 A·x = b。
 *
 * 实现细节：
 *   - 把 b 拼接到 A 末列形成增广矩阵；
 *   - 每个主元位置选列中绝对值最大者作行交换（部分主元），改善数值稳定性；
 *   - 主元归一后，把其他行该列清零，最终得到对角单位阵，最后一列即为解。
 *
 * @param[in] matrix 系数方阵 A（按值传递以便原地修改）。
 * @param[in] vector 右端向量 b。
 * @return 解向量 x，长度等于 vector.size()。
 * @throw std::runtime_error 当矩阵奇异（主元绝对值 < 1e-9）时抛出。
 */
std::vector<double> solve_linear_system(std::vector<std::vector<double>> matrix, std::vector<double> vector) {
    int size = static_cast<int>(vector.size());
    // 构造增广矩阵：把 b 拼到 A 的最后一列
    for (int row = 0; row < size; ++row) {
        matrix[row].push_back(vector[row]);
    }
    for (int pivot = 0; pivot < size; ++pivot) {
        // 选第 pivot 列下方最大绝对值作为新主元（部分主元法）
        int max_row = pivot;
        for (int row = pivot + 1; row < size; ++row) {
            if (std::abs(matrix[row][pivot]) > std::abs(matrix[max_row][pivot])) {
                max_row = row;
            }
        }
        std::swap(matrix[pivot], matrix[max_row]);
        double pivot_value = matrix[pivot][pivot];
        if (std::abs(pivot_value) < 1e-9) {
            throw std::runtime_error("Singular matrix encountered during PM calibration");
        }
        // 归一化主元行使主元为 1
        for (int column = pivot; column <= size; ++column) {
            matrix[pivot][column] /= pivot_value;
        }
        // 用主元行消去其他行该列
        for (int row = 0; row < size; ++row) {
            if (row == pivot) {
                continue;
            }
            double factor = matrix[row][pivot];
            for (int column = pivot; column <= size; ++column) {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
        }
    }

    // 增广矩阵最后一列即为解
    std::vector<double> solution(size, 0.0);
    for (int row = 0; row < size; ++row) {
        solution[row] = matrix[row][size];
    }
    return solution;
}

/**
 * @brief 普通最小二乘（OLS）线性回归：求解 X^T X · w = X^T y。
 *
 * 实现：
 *   - 累加 Gram 矩阵 G = X^T X 与右端项 h = X^T y；
 *   - 给对角线加微小正则 1e-6（岭回归项），避免 G 退化；
 *   - 调用 solve_linear_system 得到权重 w。
 *
 * @param[in] feature_matrix 设计矩阵 X，每行为一条样本的特征向量。
 * @param[in] target 对应的标签向量 y（每条样本一个标量）。
 * @return 权重向量 w，长度等于行内特征数。
 */
std::vector<double> fit_linear_regression(const std::vector<std::vector<double>>& feature_matrix, const std::vector<double>& target) {
    int feature_count = static_cast<int>(feature_matrix.front().size());
    std::vector<std::vector<double>> gram(feature_count, std::vector<double>(feature_count, 0.0));
    std::vector<double> rhs(feature_count, 0.0);
    // 累加正式方程 X^T X 与 X^T y
    for (std::size_t row = 0; row < feature_matrix.size(); ++row) {
        for (int left = 0; left < feature_count; ++left) {
            rhs[left] += feature_matrix[row][left] * target[row];
            for (int right = 0; right < feature_count; ++right) {
                gram[left][right] += feature_matrix[row][left] * feature_matrix[row][right];
            }
        }
    }
    // L2 正则项（岭回归）：对角加 1e-6 防止 G 不可逆
    for (int diagonal = 0; diagonal < feature_count; ++diagonal) {
        gram[diagonal][diagonal] += 1e-6;
    }
    return solve_linear_system(gram, rhs);
}

/**
 * @brief 用线性模型对单条样本打分：output = Σ coef_i · feat_i。
 *
 * @param[in] coefficients 权重（含截距对应首特征 1.0）。
 * @param[in] features 与系数等长的特征向量。
 * @return 预测值（如 PM2.5 浓度）。
 */
double predict(const std::vector<double>& coefficients, const std::vector<double>& features) {
    double output = 0.0;
    for (std::size_t index = 0; index < coefficients.size(); ++index) {
        output += coefficients[index] * features[index];
    }
    return output;
}

/**
 * @brief 把样本按模 5 分组划分为 train(3)/val(1)/test(1) 三段。
 *
 * 用 index % 5 而非随机抽样，避免演示数据中加入额外随机源；同时保证时间维度上的相对均匀，
 * 有助于后续漂移测试。若某段为空，从 train 末尾移入一条避免上报告错。
 *
 * @param[in] samples 待划分样本（按生成顺序）。
 * @return 三段样本及对应时间戳索引。
 */
CalibrationSplit split_samples(const std::vector<FeatureSample>& samples) {
    CalibrationSplit split;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        int bucket = static_cast<int>(index % 5);
        if (bucket <= 2) {
            // 桶 0,1,2 → 训练（60%）
            split.train.push_back(samples[index]);
        } else if (bucket == 3) {
            // 桶 3 → 验证（20%）
            split.val.push_back(samples[index]);
        } else {
            // 桶 4 → 测试（20%）
            split.test.push_back(samples[index]);
        }
    }
    // 兜底：测试/验证为空时从 train 末尾补一条
    if (split.val.empty() && !split.train.empty()) {
        split.val.push_back(split.train.back());
        split.train.pop_back();
    }
    if (split.test.empty() && !split.train.empty()) {
        split.test.push_back(split.train.back());
        split.train.pop_back();
    }
    // 顺手记下各自时间戳，供后续漂移监控使用
    for (const auto& sample : split.train) split.train_timestamps.push_back(sample.timestamp);
    for (const auto& sample : split.val) split.val_timestamps.push_back(sample.timestamp);
    for (const auto& sample : split.test) split.test_timestamps.push_back(sample.timestamp);
    return split;
}

/**
 * @brief 端到端 PM 模型校准：划分样本 → 训练 PM2.5/PM10 各一个线性模型。
 *
 * 这是 PM 反演的"训练"流程：
 *   1. 通过 split_samples 切分训练/验证/测试集；
 *   2. 用训练集的 [1.0, 干消光均值, RH, hotspot_proxy] 拟合两个目标（PM2.5、PM10）的最小二乘直线。
 *
 * @param[in] samples 时间戳特征样本列表（来自 build_timestamp_feature_table）。
 * @return (PM2.5/PM10 模型权重, 切分集合)。后续 apply_pm_models 会使用这些模型对整条廓线推 PM。
 */
std::pair<CalibrationModels, CalibrationSplit> fit_pm_models(const std::vector<FeatureSample>& samples) {
    CalibrationSplit split = split_samples(samples);
    std::vector<std::vector<double>> features;
    std::vector<double> pm25_target;
    std::vector<double> pm10_target;
    for (const auto& sample : split.train) {
        features.push_back(sample.features);
        pm25_target.push_back(sample.pm25_true);
        pm10_target.push_back(sample.pm10_true);
    }
    CalibrationModels models;
    models.pm25 = fit_linear_regression(features, pm25_target);
    models.pm10 = fit_linear_regression(features, pm10_target);
    return {models, split};
}

std::map<std::string, StationOffset> fit_station_offsets(
    const CalibrationModels& models,
    const CalibrationSplit& split
) {
    std::map<std::string, std::vector<double>> pm25_grouped;
    std::map<std::string, std::vector<double>> pm10_grouped;
    for (const auto& sample : split.train) {
        std::string site_id = sample.site_id.empty() ? "default-site" : sample.site_id;
        pm25_grouped[site_id].push_back(sample.pm25_true - predict(models.pm25, sample.features));
        pm10_grouped[site_id].push_back(sample.pm10_true - predict(models.pm10, sample.features));
    }

    std::map<std::string, StationOffset> output;
    for (const auto& [site_id, residuals] : pm25_grouped) {
        output[site_id] = StationOffset{
            mean(residuals),
            mean(pm10_grouped[site_id]),
            static_cast<int>(residuals.size()),
        };
    }
    return output;
}

