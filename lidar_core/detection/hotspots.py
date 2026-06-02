from __future__ import annotations

import math
import statistics
from collections import deque

from lidar_core.models import Hotspot, ProcessedProfile


def _mean(values: list[float]) -> float:
    return sum(values) / max(len(values), 1)


def detect_hotspots(
    profiles: list[ProcessedProfile],
    threshold_ugm3: float,
    relative_pm25_threshold: float,
    relative_dry_ext_threshold: float,
    min_cells: int,
) -> tuple[list[Hotspot], list[int], list[int]]:
    if not profiles:
        return [], [], []

    sorted_profiles = sorted(profiles, key=lambda item: item.profile.azimuth_deg)
    range_count = len(sorted_profiles[0].profile.ranges_m)
    azimuth_count = len(sorted_profiles)
    predicted_mask = [0] * (azimuth_count * range_count)
    truth_mask = [0] * (azimuth_count * range_count)
    visited: set[tuple[int, int]] = set()
    hotspots: list[Hotspot] = []
    azimuth_step_deg = 360.0 / max(azimuth_count, 1)
    range_step_m = sorted_profiles[0].profile.ranges_m[1] - sorted_profiles[0].profile.ranges_m[0] if range_count > 1 else 50.0
    all_pm25 = [value for profile in sorted_profiles for value in profile.pm25]
    all_dry_ext = [value for profile in sorted_profiles for value in profile.dry_extinction]
    baseline_pm25 = statistics.median(all_pm25)
    baseline_dry_ext = statistics.median(all_dry_ext)

    for azimuth_index, processed in enumerate(sorted_profiles):
        for range_index, pm25_value in enumerate(processed.pm25):
            linear_index = azimuth_index * range_count + range_index
            truth_mask[linear_index] = int(processed.profile.true_hotspot_mask[range_index])
            is_absolute_hotspot = pm25_value >= threshold_ugm3
            is_relative_hotspot = (
                pm25_value - baseline_pm25 >= relative_pm25_threshold
                and processed.dry_extinction[range_index] - baseline_dry_ext >= relative_dry_ext_threshold
            )
            if is_absolute_hotspot or is_relative_hotspot:
                predicted_mask[linear_index] = 1

    component_index = 0
    for azimuth_index in range(azimuth_count):
        for range_index in range(range_count):
            if predicted_mask[azimuth_index * range_count + range_index] == 0 or (azimuth_index, range_index) in visited:
                continue

            queue = deque([(azimuth_index, range_index)])
            component_cells: list[tuple[int, int]] = []
            visited.add((azimuth_index, range_index))

            while queue:
                current_azimuth, current_range = queue.popleft()
                component_cells.append((current_azimuth, current_range))
                neighbors = [
                    ((current_azimuth - 1) % azimuth_count, current_range),
                    ((current_azimuth + 1) % azimuth_count, current_range),
                    (current_azimuth, current_range - 1),
                    (current_azimuth, current_range + 1),
                ]
                for next_azimuth, next_range in neighbors:
                    if next_range < 0 or next_range >= range_count:
                        continue
                    if (next_azimuth, next_range) in visited:
                        continue
                    if predicted_mask[next_azimuth * range_count + next_range] == 0:
                        continue
                    visited.add((next_azimuth, next_range))
                    queue.append((next_azimuth, next_range))

            if len(component_cells) < min_cells:
                continue

            component_index += 1
            weights = []
            east_values = []
            north_values = []
            up_values = []
            pm_values = []
            area_m2 = 0.0
            scan_id = sorted_profiles[component_cells[0][0]].profile.scan_id

            for component_azimuth, component_range in component_cells:
                processed = sorted_profiles[component_azimuth]
                pm25_value = processed.pm25[component_range]
                east, north, up = processed.enu_points_m[component_range]
                weights.append(pm25_value)
                east_values.append(east)
                north_values.append(north)
                up_values.append(up)
                pm_values.append(pm25_value)
                radial_distance = processed.profile.ranges_m[component_range] * math.cos(math.radians(processed.profile.elevation_deg))
                area_m2 += max(radial_distance, 10.0) * math.radians(azimuth_step_deg) * range_step_m

            total_weight = sum(weights)
            centroid = [
                sum(value * weight for value, weight in zip(east_values, weights)) / max(total_weight, 1e-6),
                sum(value * weight for value, weight in zip(north_values, weights)) / max(total_weight, 1e-6),
                sum(value * weight for value, weight in zip(up_values, weights)) / max(total_weight, 1e-6),
            ]
            peak_pm25 = max(pm_values)
            severity = "high" if peak_pm25 >= threshold_ugm3 + 35.0 else "medium"
            if peak_pm25 >= threshold_ugm3 + 70.0:
                severity = "critical"

            hotspots.append(
                Hotspot(
                    timestamp=sorted_profiles[component_cells[0][0]].profile.timestamp,
                    scan_id=scan_id,
                    hotspot_id=f"hotspot-{component_index:03d}",
                    centroid_enu_m=centroid,
                    peak_pm25_ugm3=peak_pm25,
                    mean_pm25_ugm3=_mean(pm_values),
                    estimated_area_m2=area_m2,
                    cell_count=len(component_cells),
                    severity=severity,
                )
            )

    return hotspots, predicted_mask, truth_mask
