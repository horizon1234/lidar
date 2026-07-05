// Internal include fragment: PM application and hotspot detection.
// ---- PM 浓度反演与热点检测 ----

/**
 * @brief 把已拟合的 PM2.5/PM10 线性模型应用到每条已处理廓线上，得到逐距离门的颗粒物浓度。
 *
 * 算法流程：
 *  1. 按 timestamp 把特征表组织成哈希表，O(1) 查找每条廓线对应时刻的地面特征；
 *  2. 对每条廓线，确定其 site_id（特征样本优先，廓线次之，最后回退 default-site），
 *     从而取得该站点的偏置校准 StationOffset；
 *  3. 逐距离门构造特征向量 [1, dry_ext, RH, hotspot_proxy]，用线性模型预测基线 PM 值，
 *     再叠加站点偏置；
 *  4. 当本距离门干消光高于地面背景时，按 "excess × 1250" 的线性系数叠加额外热点增强，
 *     用于补偿远场局部污染团块；最后用 std::max(0, ...) 截断负值。
 *
 * @param processed_profiles 已完成的反演廓线（dry_extinction 已就绪），结果会写入 pm25/pm10 字段。
 * @param models             由 fit_pm_models 拟合得到的 PM2.5/PM10 线性模型。
 * @param feature_table      每个时间步的地面特征样本（含 hotspot_proxy 与 surface_dry_ext）。
 * @param station_offsets    站点 ID 到偏置校准的映射，缺省时使用零偏置。
 */
void apply_pm_models(
    std::vector<ProcessedProfile>& processed_profiles,
    const CalibrationModels& models,
    const std::vector<FeatureSample>& feature_table,
    const std::map<std::string, StationOffset>& station_offsets
) {
    // 用 timestamp 做索引，便于按时间步 O(1) 取到对应的地面特征
    std::map<std::string, FeatureSample> feature_by_timestamp;
    for (const auto& sample : feature_table) {
        feature_by_timestamp[sample.timestamp] = sample;
    }

    for (auto& processed : processed_profiles) {
        const auto& timestamp_features = feature_by_timestamp.at(processed.profile.timestamp);
        // 站点 ID 的优先级：地面特征 > 廓线自身 > 默认站点
        std::string site_id = !timestamp_features.site_id.empty() ? timestamp_features.site_id : (!processed.profile.site_id.empty() ? processed.profile.site_id : "default-site");
        StationOffset offset = station_offsets.contains(site_id) ? station_offsets.at(site_id) : StationOffset{};
        processed.pm25.clear();
        processed.pm10.clear();
        for (std::size_t index = 0; index < processed.dry_extinction.size(); ++index) {
            // 本距离门相对地面背景的超额消光，>0 表示存在局部污染增强
            double local_excess = std::max(0.0, processed.dry_extinction[index] - timestamp_features.surface_dry_ext);
            // 特征向量：[偏置项, 干消光, 相对湿度, 热点代理]
            std::vector<double> features{1.0, processed.dry_extinction[index], processed.profile.relative_humidity, timestamp_features.hotspot_proxy};
            double base_pm25 = predict(models.pm25, features) + offset.pm25_offset;
            double base_pm10 = predict(models.pm10, features) + offset.pm10_offset;
            double hotspot_boost = 1250.0 * local_excess;  ///< 经验线性系数，反映超额消光到 PM 质量浓度的转换
            // PM10 与 PM2.5 的增强近似按 1.18 倍比例放大（粗细颗粒物混合比的经验值）
            processed.pm25.push_back(std::max(0.0, base_pm25 + hotspot_boost));
            processed.pm10.push_back(std::max(0.0, base_pm10 + 1.18 * hotspot_boost));
        }
    }
}

// 前向声明：单层检测实现见下方，多仰角分组器先调用它
DetectionResult detect_hotspots_single_layer(
    const std::vector<const ProcessedProfile*>& profiles,
    double threshold_ugm3,
    double relative_pm25_threshold,
    double relative_dry_ext_threshold,
    int min_cells,
    double volume_baseline_pm25,
    double volume_baseline_dry_ext);

/**
 * @brief 在 PPI 体积扫描（可含多个仰角层）上检测污染热点。
 *
 * 多仰角体积扫描时，输入 profile 可能来自不同仰角层。若把所有层混在一起做
 * 连通域分析，方位角步长会被错算、环形邻域会跨层粘连，产生错误的伪热点。
 * 因此本函数按 elevation_deg 把 profile 分组，每层独立调用
 * detect_hotspots_single_layer，再合并各层的热点（保留 ENU 三维质心，
 * 上游可用质心 Up 分量区分层位）。预测/真值掩膜按各层顺序拼接。
 *
 * 单层场景（所有 profile 同仰角）退化为直接调用一次单层检测，行为与旧版一致。
 *
 * @param profiles                       指向同一时间步 PPI 廓线的指针集合（可多仰角）。
 * @param threshold_ugm3                 绝对热点 PM2.5 阈值（µg/m³）。
 * @param relative_pm25_threshold        相对基线的 PM2.5 超出阈值。
 * @param relative_dry_ext_threshold     相对基线的干消光超出阈值（必须同时满足）。
 * @param min_cells                      一个有效热点簇所需的最小网格数。
 * @return DetectionResult               含预测/真值掩码（各层拼接）及热点列表。
 */
DetectionResult detect_hotspots(
    const std::vector<ProcessedProfile*>& profiles,
    double threshold_ugm3,
    double relative_pm25_threshold,
    double relative_dry_ext_threshold,
    int min_cells
) {
    DetectionResult result;
    if (profiles.empty()) {
        return result;
    }

    // 按仰角分桶（用四舍五入到 0.01° 作为 key，避免浮点抖动导致同层被拆分）
    std::map<int, std::vector<const ProcessedProfile*>> by_elevation_milli_deg;
    for (const auto* profile : profiles) {
        int key = static_cast<int>(std::round(profile->profile.elevation_deg * 100.0));
        by_elevation_milli_deg[key].push_back(profile);
    }

    int global_component_index = 0;
    for (auto& [elev_key, layer_profiles] : by_elevation_milli_deg) {
        // 层内按方位角排序后做单层检测（各层用各自基线，单层场景行为不变）
        std::sort(layer_profiles.begin(), layer_profiles.end(),
                  [](const ProcessedProfile* left, const ProcessedProfile* right) {
                      return left->profile.azimuth_deg < right->profile.azimuth_deg;
                  });
        DetectionResult layer_result = detect_hotspots_single_layer(
            layer_profiles,
            threshold_ugm3,
            relative_pm25_threshold,
            relative_dry_ext_threshold,
            min_cells,
            -1.0,
            -1.0);

        // 拼接掩膜
        result.predicted_mask.insert(result.predicted_mask.end(),
            layer_result.predicted_mask.begin(), layer_result.predicted_mask.end());
        result.truth_mask.insert(result.truth_mask.end(),
            layer_result.truth_mask.begin(), layer_result.truth_mask.end());

        // 合并热点，并重新编号为全局连续 ID
        for (auto& hotspot : layer_result.hotspots) {
            ++global_component_index;
            std::ostringstream hotspot_id;
            hotspot_id << "hotspot-" << std::setw(3) << std::setfill('0') << global_component_index;
            hotspot.hotspot_id = hotspot_id.str();
            result.hotspots.push_back(std::move(hotspot));
        }
    }
    return result;
}

/**
 * @brief 在**单层** PPI 扫描上检测污染热点（hotspot）。
 *
 * 算法分两大阶段：
 *  1. **逐距离门预测掩码生成**：先用绝对阈值（PM2.5 ≥ threshold_ugm3）或相对阈值
 *     （PM2.5 与 dry_extinction 同时相对基线超出）判断每个网格是否为热点；
 *  2. **连通域分析（4-邻域 BFS）**：在 (方位角 × 距离) 网格上做连通域搜索，
 *     丢弃小于 min_cells 的孤立簇，对剩余簇按 PM2.5 加权求质心、面积与峰值，
 *     并根据峰值 PM2.5 给出 "medium/high/critical" 三档严重性。
 *
 * 完整 360° PPI 会把首尾方位角相连；扇区扫描不会跨首尾相连。面积按径向距离
 * × 实际方位角步长 × 距离步长 投影到水平面近似（cos(elevation)）。
 *
 * @note 本函数假设输入 profile 全部位于**同一仰角层**；多仰角体积扫描请用
 *       上层 detect_hotspots() 分层后分别调用本函数。
 *
 * @param profiles                       指向同一时间步、同一仰角层 PPI 廓线的指针集合。
 * @param threshold_ugm3                 绝对热点 PM2.5 阈值（µg/m³）。
 * @param relative_pm25_threshold        相对基线的 PM2.5 超出阈值。
 * @param relative_dry_ext_threshold     相对基线的干消光超出阈值（必须同时满足）。
 * @param min_cells                      一个有效热点簇所需的最小网格数。
 * @param volume_baseline_pm25           整个体扫的 PM2.5 基线（中位数）；<0 时用本层自行计算。
 *                                       多仰角场景下传入体扫基线可避免高空层因本底极低而误触发相对阈值。
 * @param volume_baseline_dry_ext        整个体扫的干消光基线（中位数）；<0 时用本层自行计算。
 * @return DetectionResult               含预测/真值掩码及热点列表。
 */
DetectionResult detect_hotspots_single_layer(
    const std::vector<const ProcessedProfile*>& profiles,
    double threshold_ugm3,
    double relative_pm25_threshold,
    double relative_dry_ext_threshold,
    int min_cells,
    double volume_baseline_pm25,
    double volume_baseline_dry_ext
) {
    DetectionResult result;
    if (profiles.empty()) {
        return result;
    }

    // 按方位角排序，确保 PPI 网格在方位方向上单调
    std::vector<const ProcessedProfile*> sorted_profiles = profiles;

    int range_count = static_cast<int>(sorted_profiles.front()->profile.ranges_m.size());
    int azimuth_count = static_cast<int>(sorted_profiles.size());
    // mask 采用 row-major 布局：azimuth_index * range_count + range_index
    result.predicted_mask.assign(range_count * azimuth_count, 0);
    result.truth_mask.assign(range_count * azimuth_count, 0);

    // 基线：随距离变化的环境本底（per-range-bin 方位角中位数）。
    // 物理动机：边界层气溶胶随高度指数衰减，固定仰角的 PPI 廓线天然具有
    // "近场高、远场低"的距离衰减结构。若用整层单一中位数做基线，则近场所有
    // 方位都会被误判为"相对基线偏高"（误报），而真正的烟羽是"在某一距离上
    // 比该距离处的其它方位更亮"。因此对每个距离 bin 单独取所有方位的中位数
    // 作为该距离的环境本底，仅方位向的局部增强（烟羽）才会超过基线。
    // 体积级基线（volume_baseline_*）仅用于绝对值模式；相对阈值始终用 per-range 基线。
    (void)volume_baseline_pm25;   // 体积级基线参数保留兼容，相对阈值改用 per-range 基线
    (void)volume_baseline_dry_ext;
    std::vector<double> baseline_pm25_by_range(range_count);
    std::vector<double> baseline_atten_back_by_range(range_count);
    for (int range_index = 0; range_index < range_count; ++range_index) {
        std::vector<double> pm25_at_range;
        std::vector<double> atten_at_range;
        pm25_at_range.reserve(azimuth_count);
        atten_at_range.reserve(azimuth_count);
        for (const auto* profile : sorted_profiles) {
            if (range_index < static_cast<int>(profile->pm25.size())) {
                pm25_at_range.push_back(profile->pm25[range_index]);
            }
            if (range_index < static_cast<int>(profile->attenuated_backscatter.size())) {
                atten_at_range.push_back(profile->attenuated_backscatter[range_index]);
            }
        }
        baseline_pm25_by_range[range_index] = pm25_at_range.empty() ? 0.0 : median(pm25_at_range);
        baseline_atten_back_by_range[range_index] = atten_at_range.empty() ? 0.0 : median(atten_at_range);
    }

    // 第一阶段：逐网格判定，得到预测掩码；同时记录真值掩码用于后续评估。
    // 相对判据使用【衰减后向散射】而非 Fernald 反演的干消光：
    //   - 衰减后向散射是距离校正后的直接观测量，烟羽在其上有清晰的局部增强（典型 20~30%）；
    //   - Fernald 反演在 6 km 长距离上因 exp(2τ) 反向递推不稳定，干消光几乎不携带烟羽对比度。
    // 因此采用"该距离 bin 的方位角中位数"作为环境本底，要求该网格的衰减后向散射相对本底
    // 超出 relative_dry_ext_threshold（此处语义为【相对超出的比例】，如 0.15 表示 +15%），
    // 同时 PM2.5 相对本底超出 relative_pm25_threshold，二者同时满足才判为相对热点。
    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        const auto* processed = sorted_profiles[azimuth_index];
        for (int range_index = 0; range_index < range_count; ++range_index) {
            int linear_index = azimuth_index * range_count + range_index;
            result.truth_mask[linear_index] = (range_index < static_cast<int>(processed->profile.true_hotspot_mask.size()))
                ? processed->profile.true_hotspot_mask[range_index] : 0;
            bool absolute_hotspot = processed->pm25[range_index] >= threshold_ugm3;
            double base_atten = baseline_atten_back_by_range[range_index];
            double atten_ratio = base_atten > 1e-9
                ? (processed->attenuated_backscatter[range_index] - base_atten) / base_atten
                : 0.0;
            bool relative_hotspot = (processed->pm25[range_index] - baseline_pm25_by_range[range_index] >= relative_pm25_threshold)
                && (atten_ratio >= relative_dry_ext_threshold);
            if (absolute_hotspot || relative_hotspot) {
                result.predicted_mask[linear_index] = 1;
            }
        }
    }

    // 第二阶段：4-邻域连通域分析（BFS）。完整 360° PPI 才允许方位首尾相连；
    // 扇区扫描（如 0-180°）不能把两个边界方位粘成同一个热点。
    // 注意：评估指标（precision/recall/F1）基于 predicted_mask 计算。为保证 min_cells
    // 过滤真正作用于评估结果，这里先保存第一阶段原始掩码，随后清空 predicted_mask，
    // 仅在通过 min_cells 阈值的连通簇单元上重新置 1（丢弃过小的孤立噪声簇）。
    std::vector<int> raw_mask = result.predicted_mask;
    std::fill(result.predicted_mask.begin(), result.predicted_mask.end(), 0);
    std::vector<std::vector<bool>> visited(azimuth_count, std::vector<bool>(range_count, false));
    int component_index = 0;
    std::vector<double> azimuth_diffs;
    azimuth_diffs.reserve(std::max(azimuth_count - 1, 0));
    for (int index = 1; index < azimuth_count; ++index) {
        double diff = sorted_profiles[index]->profile.azimuth_deg - sorted_profiles[index - 1]->profile.azimuth_deg;
        if (diff > 1e-6) {
            azimuth_diffs.push_back(diff);
        }
    }
    double azimuth_step_deg = azimuth_diffs.empty() ? 360.0 / std::max(azimuth_count, 1) : median(azimuth_diffs);
    double first_azimuth = sorted_profiles.front()->profile.azimuth_deg;
    double last_azimuth = sorted_profiles.back()->profile.azimuth_deg;
    bool wraps_azimuth = azimuth_count > 2
        && (last_azimuth - first_azimuth + azimuth_step_deg >= 360.0 - 0.5 * azimuth_step_deg);
    double range_step_m = range_count > 1 ? sorted_profiles.front()->profile.ranges_m[1] - sorted_profiles.front()->profile.ranges_m[0] : 50.0;
    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        for (int range_index = 0; range_index < range_count; ++range_index) {
            if (raw_mask[azimuth_index * range_count + range_index] == 0 || visited[azimuth_index][range_index]) {
                continue;
            }

            // 经典 BFS 队列：收集当前簇内所有连通的网格
            std::queue<std::pair<int, int>> queue;
            std::vector<std::pair<int, int>> component_cells;
            queue.push({azimuth_index, range_index});
            visited[azimuth_index][range_index] = true;
            while (!queue.empty()) {
                auto [current_azimuth, current_range] = queue.front();
                queue.pop();
                component_cells.push_back({current_azimuth, current_range});
                std::vector<std::pair<int, int>> neighbors;
                neighbors.reserve(4);
                if (current_azimuth > 0) {
                    neighbors.push_back({current_azimuth - 1, current_range});
                } else if (wraps_azimuth) {
                    neighbors.push_back({azimuth_count - 1, current_range});
                }
                if (current_azimuth + 1 < azimuth_count) {
                    neighbors.push_back({current_azimuth + 1, current_range});
                } else if (wraps_azimuth) {
                    neighbors.push_back({0, current_range});
                }
                neighbors.push_back({current_azimuth, current_range - 1});
                neighbors.push_back({current_azimuth, current_range + 1});
                for (const auto& [next_azimuth, next_range] : neighbors) {
                    if (next_range < 0 || next_range >= range_count || visited[next_azimuth][next_range]) {
                        continue;
                    }
                    if (raw_mask[next_azimuth * range_count + next_range] == 0) {
                        continue;
                    }
                    visited[next_azimuth][next_range] = true;
                    queue.push({next_azimuth, next_range});
                }
            }

            // 太小的簇视为噪声丢弃（既不进入热点列表，也不写回 predicted_mask）
            if (static_cast<int>(component_cells.size()) < min_cells) {
                continue;
            }

            // 该连通簇通过了 min_cells 过滤，将其单元写回 predicted_mask（用于指标评估）
            for (const auto& [cell_azimuth, cell_range] : component_cells) {
                result.predicted_mask[cell_azimuth * range_count + cell_range] = 1;
            }

            ++component_index;
            // 为质心、面积、峰值统计累积各方向统计量
            std::vector<double> weights;
            std::vector<double> east_values;
            std::vector<double> north_values;
            std::vector<double> up_values;
            std::vector<double> pm_values;
            double area_m2 = 0.0;
            std::string scan_id = sorted_profiles[component_cells.front().first]->profile.scan_id;
            for (const auto& [component_azimuth, component_range] : component_cells) {
                const auto* processed = sorted_profiles[component_azimuth];
                double pm25_value = processed->pm25[component_range];
                const auto& point = processed->enu_points_m[component_range];
                // 用 PM2.5 值做加权，使质心更贴近高污染核心区
                weights.push_back(pm25_value);
                east_values.push_back(point[0]);
                north_values.push_back(point[1]);
                up_values.push_back(point[2]);
                pm_values.push_back(pm25_value);
                // 单元格面积：径向距离投影到水平面 × 方位角弧度 × 距离步长，最小取 10 m 防止近场退化
                double radial_distance = processed->profile.ranges_m[component_range] * std::cos(processed->profile.elevation_deg * std::numbers::pi / 180.0);
                area_m2 += std::max(radial_distance, 10.0) * (azimuth_step_deg * std::numbers::pi / 180.0) * range_step_m;
            }

            // 加权质心（East/North/Up），用 std::max 分母防止零除
            double total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);
            std::vector<double> centroid{
                std::inner_product(east_values.begin(), east_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
                std::inner_product(north_values.begin(), north_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
                std::inner_product(up_values.begin(), up_values.end(), weights.begin(), 0.0) / std::max(total_weight, 1e-6),
            };
            double peak_pm25 = *std::max_element(pm_values.begin(), pm_values.end());
            // 严重性按峰值相对绝对阈值的超出量分档：+35 µg/m³ 进入 high，+70 进入 critical
            std::string severity = peak_pm25 >= threshold_ugm3 + 70.0 ? "critical" : (peak_pm25 >= threshold_ugm3 + 35.0 ? "high" : "medium");

            std::ostringstream hotspot_id;
            hotspot_id << "hotspot-" << std::setw(3) << std::setfill('0') << component_index;  ///< 形如 hotspot-001 的固定宽度 ID
            result.hotspots.push_back(Hotspot{
                sorted_profiles[component_cells.front().first]->profile.timestamp,
                scan_id,
                hotspot_id.str(),
                centroid,
                peak_pm25,
                mean(pm_values),
                area_m2,
                static_cast<int>(component_cells.size()),
                severity,
            });
        }
    }
    return result;
}

