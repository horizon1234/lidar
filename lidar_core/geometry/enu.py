from __future__ import annotations

import math

from lidar_core.models import LidarProfile


def polar_to_enu(range_m: float, azimuth_deg: float, elevation_deg: float) -> tuple[float, float, float]:
    azimuth_rad = math.radians(azimuth_deg)
    elevation_rad = math.radians(elevation_deg)
    east = range_m * math.cos(elevation_rad) * math.sin(azimuth_rad)
    north = range_m * math.cos(elevation_rad) * math.cos(azimuth_rad)
    up = range_m * math.sin(elevation_rad)
    return east, north, up


def profile_bins_to_enu(profile: LidarProfile) -> list[list[float]]:
    return [list(polar_to_enu(range_m, profile.azimuth_deg, profile.elevation_deg)) for range_m in profile.ranges_m]
