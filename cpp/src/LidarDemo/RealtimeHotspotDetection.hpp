// 已标定 YLJ5 PPI 产品的分层连通域热点检测。

std::vector<Hotspot> detect_hotspots_single_layer(
    std::vector<const ProcessedProfile*> profiles,
    double threshold_ugm3,
    double relative_pm25_threshold,
    double relative_backscatter_threshold,
    int min_cells) {
    if (profiles.empty()) return {};
    std::sort(profiles.begin(), profiles.end(),
        [](const ProcessedProfile* left, const ProcessedProfile* right) {
            return left->profile.azimuth_deg < right->profile.azimuth_deg;
        });

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
    std::vector<double> pm_baseline(range_count, 0.0);
    std::vector<double> backscatter_baseline(range_count, 0.0);
    for (std::size_t range_index = 0; range_index < range_count; ++range_index) {
        std::vector<double> pm_values;
        std::vector<double> backscatter_values;
        pm_values.reserve(profiles.size());
        backscatter_values.reserve(profiles.size());
        for (const auto* profile : profiles) {
            pm_values.push_back(profile->pm25[range_index]);
            backscatter_values.push_back(profile->attenuated_backscatter[range_index]);
        }
        pm_baseline[range_index] = median(std::move(pm_values));
        backscatter_baseline[range_index] = median(std::move(backscatter_values));
    }

    std::vector<int> candidate_mask(
        static_cast<std::size_t>(azimuth_count * bin_count), 0);
    for (int azimuth_index = 0; azimuth_index < azimuth_count; ++azimuth_index) {
        const auto* profile = profiles[static_cast<std::size_t>(azimuth_index)];
        for (int range_index = 0; range_index < bin_count; ++range_index) {
            const std::size_t bin = static_cast<std::size_t>(range_index);
            const bool absolute_hotspot = profile->pm25[bin] >= threshold_ugm3;
            const double baseline = backscatter_baseline[bin];
            const double enhancement = baseline > 1e-9
                ? (profile->attenuated_backscatter[bin] - baseline) / baseline
                : 0.0;
            const bool relative_hotspot =
                profile->pm25[bin] - pm_baseline[bin] >= relative_pm25_threshold
                && enhancement >= relative_backscatter_threshold;
            if (absolute_hotspot || relative_hotspot) {
                candidate_mask[static_cast<std::size_t>(
                    azimuth_index * bin_count + range_index)] = 1;
            }
        }
    }

    std::vector<double> azimuth_steps;
    for (int index = 1; index < azimuth_count; ++index) {
        const double step = profiles[static_cast<std::size_t>(index)]->profile.azimuth_deg
            - profiles[static_cast<std::size_t>(index - 1)]->profile.azimuth_deg;
        if (step > 1e-6) azimuth_steps.push_back(step);
    }
    const double azimuth_step_deg = azimuth_steps.empty()
        ? 360.0 / std::max(azimuth_count, 1)
        : median(std::move(azimuth_steps));
    const bool wraps_azimuth = azimuth_count > 2
        && profiles.back()->profile.azimuth_deg
            - profiles.front()->profile.azimuth_deg
            + azimuth_step_deg >= 360.0 - 0.5 * azimuth_step_deg;
    const double range_step_m = range_count > 1
        ? profiles.front()->profile.ranges_m[1]
            - profiles.front()->profile.ranges_m[0]
        : 3.75;

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

            std::queue<std::pair<int, int>> queue;
            std::vector<std::pair<int, int>> component;
            queue.push({azimuth_index, range_index});
            visited[static_cast<std::size_t>(azimuth_index)]
                   [static_cast<std::size_t>(range_index)] = true;
            while (!queue.empty()) {
                const auto current = queue.front();
                queue.pop();
                component.push_back(current);
                std::vector<std::pair<int, int>> neighbors{
                    {current.first, current.second - 1},
                    {current.first, current.second + 1},
                };
                if (current.first > 0) {
                    neighbors.push_back({current.first - 1, current.second});
                } else if (wraps_azimuth) {
                    neighbors.push_back({azimuth_count - 1, current.second});
                }
                if (current.first + 1 < azimuth_count) {
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
            if (static_cast<int>(component.size()) < std::max(min_cells, 1)) continue;

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
                const double weight = std::max(pm25, 1e-6);
                weighted_east += profile->enu_points_m[bin][0] * weight;
                weighted_north += profile->enu_points_m[bin][1] * weight;
                weighted_up += profile->enu_points_m[bin][2] * weight;
                total_weight += weight;
                peak_pm25 = std::max(peak_pm25, pm25);
                pm_sum += pm25;
                const double horizontal_range = profile->profile.ranges_m[bin]
                    * std::cos(profile->profile.elevation_deg * std::numbers::pi / 180.0);
                area_m2 += std::max(horizontal_range, 10.0)
                    * (azimuth_step_deg * std::numbers::pi / 180.0)
                    * range_step_m;
            }
            if (total_weight <= 0.0) continue;
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
                peak_pm25 >= threshold_ugm3 + 70.0
                    ? "critical"
                    : (peak_pm25 >= threshold_ugm3 + 35.0 ? "high" : "medium"),
            });
        }
    }
    return hotspots;
}

std::vector<Hotspot> detect_hotspots(
    const std::vector<const ProcessedProfile*>& profiles,
    double threshold_ugm3,
    double relative_pm25_threshold,
    double relative_backscatter_threshold,
    int min_cells) {
    std::map<int, std::vector<const ProcessedProfile*>> elevation_layers;
    for (const auto* profile : profiles) {
        if (profile != nullptr && profile->profile.scan_mode == "ppi") {
            elevation_layers[static_cast<int>(
                std::llround(profile->profile.elevation_deg * 100.0))].push_back(profile);
        }
    }

    std::vector<Hotspot> hotspots;
    for (auto& entry : elevation_layers) {
        auto layer_hotspots = detect_hotspots_single_layer(
            std::move(entry.second),
            threshold_ugm3,
            relative_pm25_threshold,
            relative_backscatter_threshold,
            min_cells);
        for (auto& hotspot : layer_hotspots) {
            std::ostringstream id;
            id << "hotspot-" << std::setw(3) << std::setfill('0')
               << hotspots.size() + 1;
            hotspot.hotspot_id = id.str();
            hotspots.push_back(std::move(hotspot));
        }
    }
    return hotspots;
}
