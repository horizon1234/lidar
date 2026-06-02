from __future__ import annotations

import math

from lidar_core.models import LidarProfile


def _mean(values: list[float]) -> float:
    return sum(values) / max(len(values), 1)


def run_fernald_inversion(
    profile: LidarProfile,
    attenuated_backscatter: list[float],
    aerosol_lidar_ratio_sr: float,
    reference_aerosol_backscatter: float,
) -> tuple[list[float], list[float]]:
    ref_index = max(len(attenuated_backscatter) - 5, 0)
    ref_signal = max(_mean(attenuated_backscatter[ref_index:]), 1e-9)
    ref_beta = _mean(profile.molecular_backscatter[ref_index:]) + reference_aerosol_backscatter
    scale = ref_beta / ref_signal
    scaled_signal = [max(value * scale, 1e-9) for value in attenuated_backscatter]
    step_km = (profile.ranges_m[1] - profile.ranges_m[0]) / 1000.0 if len(profile.ranges_m) > 1 else 0.05

    extinction = [0.0] * len(scaled_signal)
    aerosol_backscatter = [0.0] * len(scaled_signal)
    optical_depth = 0.0

    for index in range(len(scaled_signal) - 1, -1, -1):
        total_backscatter = max(scaled_signal[index] * math.exp(2.0 * optical_depth), profile.molecular_backscatter[index])
        aerosol_beta = max(total_backscatter - profile.molecular_backscatter[index], 0.0)
        total_extinction = profile.molecular_extinction[index] + aerosol_lidar_ratio_sr * aerosol_beta
        total_extinction = min(max(total_extinction, profile.molecular_extinction[index]), 0.45)
        aerosol_backscatter[index] = aerosol_beta
        extinction[index] = total_extinction
        optical_depth += total_extinction * step_km

    return extinction, aerosol_backscatter
