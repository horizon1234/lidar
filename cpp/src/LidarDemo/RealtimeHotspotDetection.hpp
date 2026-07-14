// 已标定 YLJ5 PPI 产品的分层连通域热点检测。
// 输入只允许来自同时具备接收机标定和站点 PM 标定的 ProcessedProfile。

/**
 * @brief 判断一个距离门是否有资格参与定量热点检测。
 *
 * PM 和衰减后向散射都必须存在且有限；bin_quality 为空时兼容旧数据，否则必须通过比
 * Fernald 更严格的 quantitative 门控，即 partial_overlap 也会被排除。ENU 坐标长度在
 * 计算连通域统计时另行校验。
 */
bool hotspot_bin_usable(const ProcessedProfile& profile, std::size_t index) {
    return index < profile.pm25.size()
        && index < profile.attenuated_backscatter.size()
        && std::isfinite(profile.pm25[index])
        && std::isfinite(profile.attenuated_backscatter[index])
        && (profile.bin_quality.empty()
            || (index < profile.bin_quality.size()
                && bin_is_usable_for_quantitative_product(profile.bin_quality[index])));
}

/**
 * @brief 在一个固定仰角层上检测二维极坐标污染热点。
 *
 * 算法把每条 PPI 射线视为方位维、每个距离门视为距离维，形成规则的 azimuth x range
 * 网格。候选像元同时支持绝对 PM 阈值和“相对同距离环背景增强”两条路径，再用四邻域
 * 连通域去除孤立噪声，最终计算浓度加权 ENU 质心、峰值、均值和近似水平面积。
 */
std::vector<Hotspot> detect_hotspots_single_layer(
    std::vector<const ProcessedProfile*> profiles,
    double threshold_ugm3,
    double relative_pm25_threshold,
    double relative_backscatter_threshold,
    int min_cells) {
    if (profiles.empty()) return {};

    // 连通域的方位邻接依赖稳定顺序，先按方位角从小到大排列射线。
    std::sort(profiles.begin(), profiles.end(),
        [](const ProcessedProfile* left, const ProcessedProfile* right) {
            return left->profile.azimuth_deg < right->profile.azimuth_deg;
        });

    // 取所有射线和所有必需产品的共同最短长度，保证后续矩阵索引永不越界。
    std::size_t range_count = std::numeric_limits<std::size_t>::max();
    for (const auto* profile : profiles) {
        range_count = std::min({
            range_count,
            profile->profile.ranges_m.size(),
            profile->pm25.size(),
            profile->attenuated_backscatter.size(),
            profile->enu_points_m.size(),
        });
    }
    if (range_count == 0 || range_count == std::numeric_limits<std::size_t>::max()) {
        return {};
    }

    const int azimuth_count = static_cast<int>(profiles.size());
    const int bin_count = static_cast<int>(range_count);

    /*
     * 对每个固定距离环，跨所有方位计算 PM 和后向散射中位数基线。这样可以消除随距离的
     * 系统性衰减/标定残差，只把某一方向相对同距离其他方向显著增强的区域作为局地候选。
     * 无效 bin 不进入基线；median(empty)=0，后续相对后向散射分支会因 baseline<=1e-9 关闭。
     */
    std::vector<double> pm_baseline(range_count, 0.0);
    std::vector<double> backscatter_baseline(range_count, 0.0);
    for (std::size_t range_index = 0; range_index < range_count; ++range_index) {
        std::vector<double> pm_values;
        std::vector<double> backscatter_values;
        pm_values.reserve(profiles.size());
        backscatter_values.reserve(profiles.size());
        for (const auto* profile : profiles) {
            if (!hotspot_bin_usable(*profile, range_index)) continue;
            pm_values.push_back(profile->pm25[range_index]);
            backscatter_values.push_back(profile->attenuated_backscatter[range_index]);
        }
        pm_baseline[range_index] = median(std::move(pm_values));
        backscatter_baseline[range_index] = median(std::move(backscatter_values));
    }

    // candidate_mask 使用一维连续存储，索引公式为 azimuth_index*bin_count+range_index。
    std::vector<int> candidate_mask(
        static_cast<std::size_t>(azimuth_count * bin_count), 0);
    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        const auto* profile = profiles[static_cast<std::size_t>(azimuth_index)];
        for (int range_index = 0; range_index < bin_count; ++range_index) {
            const std::size_t bin = static_cast<std::size_t>(range_index);
            if (!hotspot_bin_usable(*profile, bin)) continue;
            // 绝对条件捕获全圈普遍偏高时仍然超过环境/业务阈值的污染区。
            const bool absolute_hotspot = profile->pm25[bin] >= threshold_ugm3;
            const double baseline = backscatter_baseline[bin];
            const double enhancement = baseline > 1e-9
                ? (profile->attenuated_backscatter[bin] - baseline) / baseline
                : 0.0;
            /*
             * 相对条件要求 PM 增量和光学后向散射增量同时成立。双条件能减少 PM 线性模型
             * 偏差或单个反演异常导致的假热点。
             */
            const bool relative_hotspot =
                profile->pm25[bin] - pm_baseline[bin] >= relative_pm25_threshold
                && enhancement >= relative_backscatter_threshold;
            if (absolute_hotspot || relative_hotspot) {
                candidate_mask[static_cast<std::size_t>(
                    azimuth_index * bin_count + range_index)] = 1;
            }
        }
    }

    // 用相邻射线方位差的中位数估计实际扫描步长，容忍少量抖动或缺帧。
    std::vector<double> azimuth_steps;
    for (int index = 1; index < azimuth_count; ++index) {
        const double step = profiles[static_cast<std::size_t>(index)]->profile.azimuth_deg
            - profiles[static_cast<std::size_t>(index - 1)]->profile.azimuth_deg;
        if (step > 1e-6) azimuth_steps.push_back(step);
    }
    const double azimuth_step_deg = azimuth_steps.empty()
        ? 360.0 / std::max(azimuth_count, 1)
        : median(std::move(azimuth_steps));
    /*
     * 只有覆盖接近完整 360 度时，首尾方位才互为邻居。扇区扫描不能环绕连接，否则会把
     * 扇区两侧物理上相距很远的候选区域错误合并。
     */
    const bool wraps_azimuth = azimuth_count > 2
        && profiles.back()->profile.azimuth_deg
            - profiles.front()->profile.azimuth_deg
            + azimuth_step_deg >= 360.0 - 0.5 * azimuth_step_deg;
    // 当前面积估算假设距离门近似等间距，取第一对 bin 的间隔。
    const double range_step_m = range_count > 1
        ? profiles.front()->profile.ranges_m[1]
            - profiles.front()->profile.ranges_m[0]
        : 3.75;

    // 四邻域 BFS：距离方向连接前/后 bin，方位方向连接前/后射线。
    std::vector<std::vector<bool>> visited(
        static_cast<std::size_t>(azimuth_count),
        std::vector<bool>(range_count, false));
    std::vector<Hotspot> hotspots;
    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        for (int range_index = 0; range_index < bin_count; ++range_index) {
            const auto linear_index = static_cast<std::size_t>(
                azimuth_index * bin_count + range_index);
            if (candidate_mask[linear_index] == 0
                || visited[static_cast<std::size_t>(azimuth_index)]
                          [static_cast<std::size_t>(range_index)]) {
                continue;
            }

            // 从一个尚未访问的候选像元开始，收集其完整连通分量。
            std::queue<std::pair<int, int>> queue;
            std::vector<std::pair<int, int>> component;
            queue.push({azimuth_index, range_index});
            visited[static_cast<std::size_t>(azimuth_index)]
                   [static_cast<std::size_t>(range_index)] = true;
            while (!queue.empty()) {
                const auto current = queue.front();
                queue.pop();
                component.push_back(current);
                // 先添加同一射线上的近/远距离邻居。
                std::vector<std::pair<int, int>> neighbors{
                    {current.first, current.second - 1},
                    {current.first, current.second + 1},
                };
                if (current.first > 0) {
                    // 普通情况下连接前一方位；完整圆的第 0 条还要连接最后一条。
                    neighbors.push_back({current.first - 1, current.second});
                } else if (wraps_azimuth) {
                    neighbors.push_back({azimuth_count - 1, current.second});
                }
                if (current.first + 1 < azimuth_count) {
                    // 同理连接后一方位，必要时从最后一条环绕到第 0 条。
                    neighbors.push_back({current.first + 1, current.second});
                } else if (wraps_azimuth) {
                    neighbors.push_back({0, current.second});
                }
                for (const auto& neighbor : neighbors) {
                    if (neighbor.second < 0 || neighbor.second >= bin_count) continue;
                    const auto neighbor_azimuth = static_cast<std::size_t>(neighbor.first);
                    const auto neighbor_range = static_cast<std::size_t>(neighbor.second);
                    const auto neighbor_index = static_cast<std::size_t>(
                        neighbor.first * bin_count + neighbor.second);
                    if (!visited[neighbor_azimuth][neighbor_range]
                        && candidate_mask[neighbor_index] != 0) {
                        visited[neighbor_azimuth][neighbor_range] = true;
                        queue.push(neighbor);
                    }
                }
            }
            // 小于最小像元数的孤立分量更可能是随机噪声或单射线伪影，直接丢弃。
            if (static_cast<int>(component.size()) < std::max(min_cells, 1)) continue;

            // 对通过面积门槛的连通分量计算空间位置、浓度和面积统计。
            double weighted_east = 0.0;
            double weighted_north = 0.0;
            double weighted_up = 0.0;
            double total_weight = 0.0;
            double peak_pm25 = 0.0;
            double pm_sum = 0.0;
            double area_m2 = 0.0;
            for (const auto& cell : component) {
                const auto* profile = profiles[static_cast<std::size_t>(cell.first)];
                const std::size_t bin = static_cast<std::size_t>(cell.second);
                if (profile->enu_points_m[bin].size() < 3) continue;
                const double pm25 = profile->pm25[bin];
                // 用 PM 作为质心权重，使事件位置更接近污染核心而不是几何边界中心。
                const double weight = std::max(pm25, 1e-6);
                weighted_east += profile->enu_points_m[bin][0] * weight;
                weighted_north += profile->enu_points_m[bin][1] * weight;
                weighted_up += profile->enu_points_m[bin][2] * weight;
                total_weight += weight;
                peak_pm25 = std::max(peak_pm25, pm25);
                pm_sum += pm25;
                // 斜距投影到水平面，单像元面积近似为 r_horizontal*dtheta*dr。
                const double horizontal_range = profile->profile.ranges_m[bin]
                    * std::cos(profile->profile.elevation_deg * std::numbers::pi / 180.0);
                // 近原点用 10 m 下限，避免极坐标扇形面积退化为 0。
                area_m2 += std::max(horizontal_range, 10.0)
                    * (azimuth_step_deg * std::numbers::pi / 180.0)
                    * range_step_m;
            }
            if (total_weight <= 0.0) continue;
            // hotspot_id 在合并所有仰角层后统一生成，这里暂时留空。
            hotspots.push_back(Hotspot{
                profiles[static_cast<std::size_t>(component.front().first)]->profile.timestamp,
                profiles[static_cast<std::size_t>(component.front().first)]->profile.scan_id,
                "",
                {weighted_east / total_weight,
                 weighted_north / total_weight,
                 weighted_up / total_weight},
                peak_pm25,
                pm_sum / static_cast<double>(component.size()),
                area_m2,
                static_cast<int>(component.size()),
                // 严重度使用相对绝对阈值的固定增量分级，后续可替换为站点业务规则。
                peak_pm25 >= threshold_ugm3 + 70.0
                    ? "critical"
                    : (peak_pm25 >= threshold_ugm3 + 35.0 ? "high" : "medium"),
            });
        }
    }
    return hotspots;
}

/**
 * @brief 按仰角分层运行热点检测，并生成跨层唯一事件编号。
 *
 * 不同仰角对应不同高度锥面，不能直接在二维四邻域中连接。这里把仰角量化到 0.01 度
 * 分组，各层独立检测；当前实现不会把不同高度层的分量融合成三维体热点。
 */
std::vector<Hotspot> detect_hotspots(
    const std::vector<const ProcessedProfile*>& profiles,
    double threshold_ugm3,
    double relative_pm25_threshold,
    double relative_backscatter_threshold,
    int min_cells) {
    std::map<int, std::vector<const ProcessedProfile*>> elevation_layers;
    for (const auto* profile : profiles) {
        if (profile != nullptr && profile->profile.scan_mode == "ppi") {
            // round(elevation*100) 抑制编码器微小浮点扰动，同时保留 0.01 度层分辨率。
            elevation_layers[static_cast<int>(
                std::llround(profile->profile.elevation_deg * 100.0))].push_back(profile);
        }
    }

    std::vector<Hotspot> hotspots;
    for (auto& entry : elevation_layers) {
        // 每个高度层使用完全相同的阈值与最小连通像元数。
        auto layer_hotspots = detect_hotspots_single_layer(
            std::move(entry.second),
            threshold_ugm3,
            relative_pm25_threshold,
            relative_backscatter_threshold,
            min_cells);
        for (auto& hotspot : layer_hotspots) {
            // 按最终输出顺序生成稳定、便于 UI 展示的三位十进制编号。
            std::ostringstream id;
            id << "hotspot-" << std::setw(3) << std::setfill('0')
               << hotspots.size() + 1;
            hotspot.hotspot_id = id.str();
            hotspots.push_back(std::move(hotspot));
        }
    }
    return hotspots;
}
