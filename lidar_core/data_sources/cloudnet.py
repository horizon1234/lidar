from __future__ import annotations

import csv
import json
import math
import random
import shutil
import ssl
import urllib.parse
import urllib.request
from datetime import datetime, timedelta
from pathlib import Path

from lidar_core.io import read_json, write_json
from lidar_core.models import GroundMeasurement, LidarProfile, SiteInfo
from lidar_core.simulation.scene import _build_overlap, _simulate_profile_fields, _simulate_raw_counts


def _import_netcdf4():
    """延迟导入 netCDF4。模拟模式不需要它，只有 Cloudnet hybrid 模式才需要。"""
    try:
        from netCDF4 import Dataset, num2date
    except ImportError as exc:  # pragma: no cover - 环境相关
        raise ImportError(
            "Cloudnet hybrid 模式需要 netCDF4，请运行: pip install netCDF4"
        ) from exc
    return Dataset, num2date


def _safe_float(value) -> float:
    if hasattr(value, "filled"):
        value = value.filled(0.0)
    try:
        output = float(value)
    except (TypeError, ValueError):
        return 0.0
    if math.isnan(output):
        return 0.0
    return output


def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def _netcdf_compatible_path(path: Path) -> str:
    path = path.resolve()
    path_text = str(path)
    if path_text.isascii():
        return path_text

    cache_root = Path(path.anchor) / "temp" / "lidar_cloudnet_cache"
    mirrored_path = cache_root / path.name
    _ensure_parent(mirrored_path)
    if not mirrored_path.exists() or mirrored_path.stat().st_size != path.stat().st_size:
        shutil.copy2(path, mirrored_path)
    return str(mirrored_path)


def _urlopen_json(url: str, verify_ssl: bool = True) -> dict | list:
    if verify_ssl:
        with urllib.request.urlopen(url) as response:
            return json.loads(response.read().decode("utf-8"))
    context = ssl._create_unverified_context()
    with urllib.request.urlopen(url, context=context) as response:
        return json.loads(response.read().decode("utf-8"))


def _download_file(url: str, output_path: Path, verify_ssl: bool = True) -> Path:
    _ensure_parent(output_path)
    if output_path.exists():
        return output_path
    if verify_ssl:
        with urllib.request.urlopen(url) as response:
            output_path.write_bytes(response.read())
    else:
        context = ssl._create_unverified_context()
        with urllib.request.urlopen(url, context=context) as response:
            output_path.write_bytes(response.read())
    return output_path


def _fetch_open_meteo_ground_records(lat: float, lon: float, date: str, output_root: Path) -> list[dict]:
    day_token = date.replace("-", "")
    json_path = output_root / f"open_meteo_{day_token}_ground_pm_meteo.json"
    csv_path = output_root / f"open_meteo_{day_token}_ground_pm_meteo.csv"
    manifest_path = output_root / f"open_meteo_{day_token}_ground_pm_meteo_manifest.json"
    if json_path.exists():
        return read_json(json_path)

    air_quality_url = "https://air-quality-api.open-meteo.com/v1/air-quality?" + urllib.parse.urlencode(
        {
            "latitude": lat,
            "longitude": lon,
            "start_date": date,
            "end_date": date,
            "hourly": "pm10,pm2_5",
            "timezone": "UTC",
        }
    )
    weather_url = "https://archive-api.open-meteo.com/v1/archive?" + urllib.parse.urlencode(
        {
            "latitude": lat,
            "longitude": lon,
            "start_date": date,
            "end_date": date,
            "hourly": "temperature_2m,relative_humidity_2m,windspeed_10m,winddirection_10m",
            "timezone": "UTC",
        }
    )
    air_quality = _urlopen_json(air_quality_url)
    weather = _urlopen_json(weather_url)

    records = []
    for index, timestamp in enumerate(air_quality["hourly"]["time"]):
        pm25 = air_quality["hourly"]["pm2_5"][index]
        pm10 = air_quality["hourly"]["pm10"][index]
        records.append(
            {
                "timestamp": timestamp,
                "pm25": pm25,
                "pm10": pm10,
                "temperature_c": weather["hourly"]["temperature_2m"][index],
                "relative_humidity": weather["hourly"]["relative_humidity_2m"][index],
                "wind_speed_ms": round(weather["hourly"]["windspeed_10m"][index] / 3.6, 4),
                "wind_dir_deg": weather["hourly"]["winddirection_10m"][index],
            }
        )

    _ensure_parent(json_path)
    write_json(json_path, records)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)
    write_json(
        manifest_path,
        {
            "source": "Open-Meteo",
            "latitude": lat,
            "longitude": lon,
            "date": date,
            "record_count": len(records),
            "pm25_non_null": sum(record["pm25"] is not None for record in records),
            "pm10_non_null": sum(record["pm10"] is not None for record in records),
            "json": str(json_path),
            "csv": str(csv_path),
        },
    )
    return records


def _parse_cloudnet_base_time(time_units: str) -> datetime:
    base_text = time_units.split("since", 1)[1].strip()
    return datetime.fromisoformat(base_text)


def _select_even_indices(length: int, count: int) -> list[int]:
    if length <= count:
        return list(range(length))
    return sorted({round(index * (length - 1) / max(count - 1, 1)) for index in range(count)})


def _to_iso_minute(timestamp: datetime) -> str:
    return timestamp.replace(second=0, microsecond=0).isoformat(timespec="minutes")


def _nearest_ground_record(target: datetime, records: list[dict]) -> dict:
    best_record = records[0]
    best_delta = None
    for record in records:
        current = datetime.fromisoformat(record["timestamp"])
        delta = abs((current - target).total_seconds())
        if best_delta is None or delta < best_delta:
            best_delta = delta
            best_record = record
    return best_record


def _standard_molecular_fields(ranges_m: list[float], site_altitude_m: float, elevation_deg: float) -> tuple[list[float], list[float]]:
    molecular_extinction = []
    molecular_backscatter = []
    elevation_rad = math.radians(elevation_deg)
    for range_m in ranges_m:
        altitude_m = site_altitude_m + range_m * math.sin(elevation_rad)
        molecular_ext = 0.012 * math.exp(-altitude_m / 8000.0)
        molecular_extinction.append(molecular_ext)
        molecular_backscatter.append(molecular_ext / 8.0)
    return molecular_extinction, molecular_backscatter


def _filter_range_indices(range_values_m: list[float], min_range_m: float, max_range_m: float, target_count: int) -> list[int]:
    candidates = [index for index, value in enumerate(range_values_m) if min_range_m <= value <= max_range_m]
    if not candidates:
        raise ValueError("No Cloudnet range bins fell inside the configured min/max range")
    chosen_offsets = _select_even_indices(len(candidates), target_count)
    return [candidates[offset] for offset in chosen_offsets]


def _build_real_stare_profiles(config: dict, site: SiteInfo, ground_measurements: list[GroundMeasurement], source_root: Path) -> list[LidarProfile]:
    Dataset, num2date = _import_netcdf4()
    cloudnet = config["source"]["cloudnet"]
    file_path = source_root / cloudnet["local_file"]
    dataset = Dataset(_netcdf_compatible_path(file_path))

    time_values = num2date(
        dataset.variables["time"][:],
        units=dataset.variables["time"].units,
        only_use_cftime_datetimes=False,
    )
    range_values_m = [float(value) for value in dataset.variables["range"][:]]
    beta = dataset.variables["beta"]
    zenith_angle_deg = float(dataset.variables["zenith_angle"][:]) if "zenith_angle" in dataset.variables else 0.0
    elevation_deg = 90.0 - zenith_angle_deg
    time_indices = _select_even_indices(len(time_values), int(cloudnet["time_steps"]))
    range_indices = _filter_range_indices(
        range_values_m,
        float(cloudnet["min_range_m"]),
        float(cloudnet["max_range_m"]),
        int(cloudnet["range_bin_count"]),
    )
    ranges_m = [range_values_m[index] for index in range_indices]
    overlap = _build_overlap(ranges_m)
    pseudo_scale = float(cloudnet["pseudo_signal_scale"])
    molecular_extinction, molecular_backscatter = _standard_molecular_fields(ranges_m, site.altitude_m, elevation_deg)
    ground_by_timestamp = {measurement.timestamp: measurement for measurement in ground_measurements}
    profiles = []

    for order, time_index in enumerate(time_indices):
        timestamp = _to_iso_minute(time_values[time_index])
        ground = ground_by_timestamp[timestamp]
        beta_row = beta[time_index, range_indices]
        if hasattr(beta_row, "filled"):
            beta_values = [max(float(value), 0.0) for value in beta_row.filled(0.0)]
        else:
            beta_values = [max(float(value), 0.0) for value in beta_row]

        background_counts = 5.0 + 0.15 * (order % 5)
        laser_energy_mj = 1.0 + 0.01 * math.sin(order)
        raw_counts = []
        approx_extinction = []
        for beta_value, range_m, overlap_value, molecular_ext in zip(beta_values, ranges_m, overlap, molecular_extinction):
            range_km = range_m / 1000.0
            raw_counts.append(background_counts + beta_value * pseudo_scale * overlap_value / max(range_km * range_km, 1e-6))
            aerosol_ext = beta_value * float(config["retrieval"]["aerosol_lidar_ratio_sr"])
            approx_extinction.append(molecular_ext + aerosol_ext)

        profiles.append(
            LidarProfile(
                site_id=cloudnet["site_id"],
                timestamp=timestamp,
                scan_id=f"{timestamp}_stare_real",
                scan_mode="stare",
                source_kind="cloudnet_real_stare",
                azimuth_deg=0.0,
                elevation_deg=elevation_deg,
                ranges_m=ranges_m,
                raw_counts=raw_counts,
                laser_energy_mj=laser_energy_mj,
                background_counts=background_counts,
                overlap=overlap,
                relative_humidity=ground.relative_humidity,
                temperature_c=ground.temperature_c,
                wind_speed_ms=ground.wind_speed_ms,
                wind_dir_deg=ground.wind_dir_deg,
                molecular_backscatter=molecular_backscatter,
                molecular_extinction=molecular_extinction,
                true_backscatter=[molecular + observed for molecular, observed in zip(molecular_backscatter, beta_values)],
                true_extinction=approx_extinction,
                true_pm25=[0.0 for _ in ranges_m],
                true_pm10=[0.0 for _ in ranges_m],
                true_hotspot_mask=[0 for _ in ranges_m],
            )
        )

    dataset.close()
    return profiles


def _build_ground_measurements(config: dict, site: SiteInfo, source_root: Path) -> list[GroundMeasurement]:
    Dataset, num2date = _import_netcdf4()
    cloudnet = config["source"]["cloudnet"]
    date = cloudnet["date"]
    records = _fetch_open_meteo_ground_records(site.latitude_deg, site.longitude_deg, date, source_root / "data" / "public" / "cloudnet")
    dataset = Dataset(_netcdf_compatible_path(source_root / cloudnet["local_file"]))
    time_values = num2date(
        dataset.variables["time"][:],
        units=dataset.variables["time"].units,
        only_use_cftime_datetimes=False,
    )
    time_indices = _select_even_indices(len(time_values), int(cloudnet["time_steps"]))
    ground_measurements = []
    for time_index in time_indices:
        timestamp = _to_iso_minute(time_values[time_index])
        matched = _nearest_ground_record(datetime.fromisoformat(timestamp), records)
        pm25 = matched["pm25"]
        pm10 = matched["pm10"]
        if pm25 is None:
            pm25 = 12.0
        if pm10 is None:
            pm10 = 20.0
        ground_measurements.append(
            GroundMeasurement(
                site_id=cloudnet["site_id"],
                timestamp=timestamp,
                pm25_ugm3=float(pm25),
                pm10_ugm3=float(pm10),
                relative_humidity=float(matched["relative_humidity"] or 60.0) / 100.0,
                temperature_c=float(matched["temperature_c"] or 18.0),
                wind_speed_ms=float(matched["wind_speed_ms"] or 2.0),
                wind_dir_deg=float(matched["wind_dir_deg"] or 0.0),
            )
        )
    dataset.close()
    return ground_measurements


def _build_synthetic_ppi_profiles(config: dict, site: SiteInfo, ground_measurements: list[GroundMeasurement], ranges_m: list[float]) -> list[LidarProfile]:
    cloudnet = config["source"]["cloudnet"]
    simulation = config["simulation"]
    rng = random.Random(int(simulation["seed"]) + 101)
    overlap = _build_overlap(ranges_m)
    lidar_ratio_sr = float(simulation["lidar_ratio_sr"])
    ppi_profiles: list[LidarProfile] = []
    for step_index, ground in enumerate(ground_measurements):
        azimuth = 0.0
        while azimuth < 360.0:
            fields = _simulate_profile_fields(
                ranges_m,
                azimuth,
                float(simulation["ppi_elevation_deg"]),
                step_index,
                len(ground_measurements),
                ground.relative_humidity,
                lidar_ratio_sr,
            )
            background_counts = 10.2 + rng.gauss(0.0, 0.35)
            laser_energy_mj = 1.0 + rng.gauss(0.0, 0.02)
            raw_counts = _simulate_raw_counts(
                fields[2],
                fields[3],
                overlap,
                ranges_m,
                laser_energy_mj,
                background_counts,
                float(simulation["system_constant"]),
                rng,
            )
            ppi_profiles.append(
                LidarProfile(
                    site_id=cloudnet["site_id"],
                    timestamp=ground.timestamp,
                    scan_id=f"{ground.timestamp}_ppi_{int(azimuth):03d}",
                    scan_mode="ppi",
                    source_kind="synthetic_ppi_hybrid",
                    azimuth_deg=azimuth,
                    elevation_deg=float(simulation["ppi_elevation_deg"]),
                    ranges_m=ranges_m,
                    raw_counts=raw_counts,
                    laser_energy_mj=laser_energy_mj,
                    background_counts=background_counts,
                    overlap=overlap,
                    relative_humidity=ground.relative_humidity,
                    temperature_c=ground.temperature_c,
                    wind_speed_ms=ground.wind_speed_ms,
                    wind_dir_deg=ground.wind_dir_deg,
                    molecular_backscatter=fields[1],
                    molecular_extinction=fields[0],
                    true_backscatter=fields[2],
                    true_extinction=fields[3],
                    true_pm25=fields[4],
                    true_pm10=fields[5],
                    true_hotspot_mask=fields[6],
                )
            )
            azimuth += float(simulation["ppi_azimuth_step_deg"])
    return ppi_profiles


def load_cloudnet_hybrid_campaign(config: dict):
    Dataset, _num2date = _import_netcdf4()
    source_root = Path(config["source"].get("root", ".")).resolve()
    cloudnet = config["source"]["cloudnet"]
    local_file = source_root / cloudnet["local_file"]
    if not local_file.exists():
        _download_file(cloudnet["download_url"], local_file, verify_ssl=bool(cloudnet.get("verify_ssl", True)))

    dataset = Dataset(_netcdf_compatible_path(local_file))
    site = SiteInfo(
        name=cloudnet["site_name"],
        latitude_deg=float(dataset.variables["latitude"][:]),
        longitude_deg=float(dataset.variables["longitude"][:]),
        altitude_m=float(dataset.variables["altitude"][:]),
        site_id=cloudnet["site_id"],
    )
    dataset.close()

    ground_measurements = _build_ground_measurements(config, site, source_root)
    stare_profiles = _build_real_stare_profiles(config, site, ground_measurements, source_root)
    ranges_m = stare_profiles[0].ranges_m
    ppi_profiles = _build_synthetic_ppi_profiles(config, site, ground_measurements, ranges_m)
    profiles = []
    by_timestamp = {profile.timestamp: [] for profile in stare_profiles}
    for profile in stare_profiles:
        by_timestamp[profile.timestamp].append(profile)
    for profile in ppi_profiles:
        by_timestamp.setdefault(profile.timestamp, []).append(profile)
    for timestamp in sorted(by_timestamp):
        profiles.extend(by_timestamp[timestamp])
    return site, profiles, ground_measurements, {
        "mode": "cloudnet_hybrid",
        "site_id": site.site_id,
        "site_name": site.name,
        "cloudnet_file": str(local_file),
        "measurement_date": cloudnet["date"],
        "ground_provider": "open-meteo",
        "real_stare_profile_count": len(stare_profiles),
        "synthetic_ppi_profile_count": len(ppi_profiles),
    }
