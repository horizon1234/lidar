from __future__ import annotations

import math
import random
from datetime import datetime, timedelta

from lidar_core.models import GroundMeasurement, LidarProfile, SiteInfo


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _gaussian(distance: float, center: float, sigma: float) -> float:
    return math.exp(-((distance - center) ** 2) / max(2.0 * sigma * sigma, 1e-6))


def _azimuth_delta(left_deg: float, right_deg: float) -> float:
    raw = abs(left_deg - right_deg) % 360.0
    return min(raw, 360.0 - raw)


def _mean(values: list[float]) -> float:
    return sum(values) / max(len(values), 1)


def _build_overlap(ranges_m: list[float]) -> list[float]:
    overlap = []
    for distance_m in ranges_m:
        ratio = (distance_m / 180.0) ** 0.82
        overlap.append(_clamp(ratio, 0.22, 1.0))
    return overlap


def _simulate_profile_fields(
    ranges_m: list[float],
    azimuth_deg: float,
    elevation_deg: float,
    step_index: int,
    total_steps: int,
    relative_humidity: float,
    lidar_ratio_sr: float,
) -> tuple[list[float], list[float], list[float], list[float], list[float], list[int]]:
    time_phase = 2.0 * math.pi * step_index / max(total_steps, 1)
    molecular_extinction = []
    molecular_backscatter = []
    true_backscatter = []
    true_extinction = []
    true_pm25 = []
    true_pm10 = []
    true_hotspot_mask = []
    humidity_growth = 1.0 + 0.35 * max(relative_humidity - 0.55, 0.0) * 3.0

    for range_m in ranges_m:
        elevation_rad = math.radians(elevation_deg)
        altitude_m = range_m * math.sin(elevation_rad)
        horizontal_m = range_m * math.cos(elevation_rad)
        molecular_ext = 0.012 * math.exp(-altitude_m / 8000.0)
        molecular_beta = molecular_ext / 8.0

        boundary_layer = (0.017 + 0.018 * (1.0 + math.sin(time_phase)) / 2.0) * math.exp(-altitude_m / 550.0)
        lofted_layer = 0.018 * _gaussian(altitude_m, 500.0 + 120.0 * math.sin(time_phase), 160.0)
        plume_1 = 0.080 * _gaussian(horizontal_m, 650.0 + 80.0 * math.cos(time_phase), 130.0) * _gaussian(_azimuth_delta(azimuth_deg, 80.0), 0.0, 18.0) * _gaussian(altitude_m, 85.0, 45.0)
        plume_2 = 0.045 * _gaussian(horizontal_m, 1050.0, 150.0) * _gaussian(_azimuth_delta(azimuth_deg, 170.0), 0.0, 22.0) * _gaussian(altitude_m, 120.0, 65.0)

        aerosol_dry_ext = boundary_layer + lofted_layer + plume_1 + plume_2
        aerosol_ext = aerosol_dry_ext * humidity_growth
        aerosol_beta = aerosol_ext / lidar_ratio_sr

        local_pm25 = 640.0 * aerosol_dry_ext + 210.0 * plume_1 + 15.0
        local_pm10 = 920.0 * aerosol_dry_ext + 260.0 * (plume_1 + plume_2) + 22.0
        hotspot_flag = 1 if plume_1 + plume_2 > 0.025 else 0

        molecular_extinction.append(molecular_ext)
        molecular_backscatter.append(molecular_beta)
        true_backscatter.append(molecular_beta + aerosol_beta)
        true_extinction.append(molecular_ext + aerosol_ext)
        true_pm25.append(local_pm25)
        true_pm10.append(local_pm10)
        true_hotspot_mask.append(hotspot_flag)

    return (
        molecular_extinction,
        molecular_backscatter,
        true_backscatter,
        true_extinction,
        true_pm25,
        true_pm10,
        true_hotspot_mask,
    )


def _simulate_raw_counts(
    true_backscatter: list[float],
    true_extinction: list[float],
    overlap: list[float],
    ranges_m: list[float],
    laser_energy_mj: float,
    background_counts: float,
    system_constant: float,
    rng: random.Random,
) -> list[float]:
    raw_counts = []
    optical_depth = 0.0
    step_km = (ranges_m[1] - ranges_m[0]) / 1000.0 if len(ranges_m) > 1 else 0.05

    for index, range_m in enumerate(ranges_m):
        optical_depth += true_extinction[index] * step_km
        range_km = range_m / 1000.0
        signal = system_constant * laser_energy_mj * overlap[index] * true_backscatter[index] * math.exp(-2.0 * optical_depth) / max(range_km * range_km, 1e-6)
        noise_sigma = max(background_counts * 0.07, signal * 0.05)
        noisy_signal = max(signal + background_counts + rng.gauss(0.0, noise_sigma), background_counts + 0.1)
        raw_counts.append(noisy_signal)
    return raw_counts


def simulate_campaign(config: dict) -> tuple[SiteInfo, list[LidarProfile], list[GroundMeasurement]]:
    site = SiteInfo(**config["site"])
    simulation = config["simulation"]
    rng = random.Random(simulation["seed"])
    base_time = datetime(2026, 5, 30, 8, 0, 0)
    time_steps = int(simulation["time_steps"])
    minutes_per_step = int(simulation["minutes_per_step"])
    ranges_m = [simulation["range_bin_m"] * (index + 1) for index in range(int(simulation["range_bin_count"]))]
    overlap = _build_overlap(ranges_m)
    lidar_ratio_sr = float(simulation["lidar_ratio_sr"])
    profiles: list[LidarProfile] = []
    ground_measurements: list[GroundMeasurement] = []

    for step_index in range(time_steps):
        timestamp = (base_time + timedelta(minutes=step_index * minutes_per_step)).isoformat(timespec="minutes")
        time_phase = 2.0 * math.pi * step_index / max(time_steps, 1)
        relative_humidity = _clamp(0.48 + 0.20 * math.sin(time_phase - 0.6) + rng.gauss(0.0, 0.015), 0.28, 0.90)
        temperature_c = 27.0 - 7.0 * math.sin(time_phase - 0.2) + rng.gauss(0.0, 0.3)
        wind_speed_ms = _clamp(2.8 + 1.2 * math.cos(time_phase + 0.3) + rng.gauss(0.0, 0.2), 0.6, 6.5)
        wind_dir_deg = (120.0 + 45.0 * math.sin(time_phase) + rng.gauss(0.0, 4.0)) % 360.0
        background_counts = 10.5 + rng.gauss(0.0, 0.5)
        laser_energy_mj = 1.0 + rng.gauss(0.0, 0.03)

        stare_fields = _simulate_profile_fields(ranges_m, 0.0, 90.0, step_index, time_steps, relative_humidity, lidar_ratio_sr)
        stare_raw_counts = _simulate_raw_counts(
            stare_fields[2],
            stare_fields[3],
            overlap,
            ranges_m,
            laser_energy_mj,
            background_counts,
            simulation["system_constant"],
            rng,
        )
        profiles.append(
            LidarProfile(
                site_id=site.name.lower().replace(" ", "-"),
                timestamp=timestamp,
                scan_id=f"{timestamp}_stare",
                scan_mode="stare",
                source_kind="synthetic_stare",
                azimuth_deg=0.0,
                elevation_deg=90.0,
                ranges_m=ranges_m,
                raw_counts=stare_raw_counts,
                laser_energy_mj=laser_energy_mj,
                background_counts=background_counts,
                overlap=overlap,
                relative_humidity=relative_humidity,
                temperature_c=temperature_c,
                wind_speed_ms=wind_speed_ms,
                wind_dir_deg=wind_dir_deg,
                molecular_backscatter=stare_fields[1],
                molecular_extinction=stare_fields[0],
                true_backscatter=stare_fields[2],
                true_extinction=stare_fields[3],
                true_pm25=stare_fields[4],
                true_pm10=stare_fields[5],
                true_hotspot_mask=stare_fields[6],
            )
        )

        ppi_profiles_for_timestamp: list[LidarProfile] = []
        azimuth_step_deg = float(simulation["ppi_azimuth_step_deg"])
        ppi_elevation_deg = float(simulation["ppi_elevation_deg"])
        azimuth = 0.0
        while azimuth < 360.0:
            ppi_fields = _simulate_profile_fields(ranges_m, azimuth, ppi_elevation_deg, step_index, time_steps, relative_humidity, lidar_ratio_sr)
            ppi_raw_counts = _simulate_raw_counts(
                ppi_fields[2],
                ppi_fields[3],
                overlap,
                ranges_m,
                laser_energy_mj,
                background_counts,
                simulation["system_constant"],
                rng,
            )
            profile = LidarProfile(
                site_id=site.name.lower().replace(" ", "-"),
                timestamp=timestamp,
                scan_id=f"{timestamp}_ppi_{int(azimuth):03d}",
                scan_mode="ppi",
                source_kind="synthetic_ppi",
                azimuth_deg=azimuth,
                elevation_deg=ppi_elevation_deg,
                ranges_m=ranges_m,
                raw_counts=ppi_raw_counts,
                laser_energy_mj=laser_energy_mj,
                background_counts=background_counts,
                overlap=overlap,
                relative_humidity=relative_humidity,
                temperature_c=temperature_c,
                wind_speed_ms=wind_speed_ms,
                wind_dir_deg=wind_dir_deg,
                molecular_backscatter=ppi_fields[1],
                molecular_extinction=ppi_fields[0],
                true_backscatter=ppi_fields[2],
                true_extinction=ppi_fields[3],
                true_pm25=ppi_fields[4],
                true_pm10=ppi_fields[5],
                true_hotspot_mask=ppi_fields[6],
            )
            profiles.append(profile)
            ppi_profiles_for_timestamp.append(profile)
            azimuth += azimuth_step_deg

        near_surface_dry = _mean([value / (1.0 + 0.35 * max(relative_humidity - 0.55, 0.0) * 3.0) for value in stare_fields[3][:6]])
        hotspot_proxy = max(max(value / (1.0 + 0.35 * max(relative_humidity - 0.55, 0.0) * 3.0) for value in profile.true_extinction[:6]) for profile in ppi_profiles_for_timestamp)
        pm25_ugm3 = max(24.0, 430.0 * near_surface_dry + 305.0 * hotspot_proxy + 0.24 * relative_humidity * 100.0 + rng.gauss(0.0, 1.8))
        pm10_ugm3 = max(34.0, 610.0 * near_surface_dry + 410.0 * hotspot_proxy + 0.31 * relative_humidity * 100.0 + rng.gauss(0.0, 2.4))
        ground_measurements.append(
            GroundMeasurement(
                site_id=site.name.lower().replace(" ", "-"),
                timestamp=timestamp,
                pm25_ugm3=pm25_ugm3,
                pm10_ugm3=pm10_ugm3,
                relative_humidity=relative_humidity,
                temperature_c=temperature_c,
                wind_speed_ms=wind_speed_ms,
                wind_dir_deg=wind_dir_deg,
            )
        )

    return site, profiles, ground_measurements


__all__ = [
    "simulate_campaign",
    "_build_overlap",
    "_simulate_profile_fields",
    "_simulate_raw_counts",
]
