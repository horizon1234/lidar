from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass
class SiteInfo:
    name: str
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    site_id: str = ""


@dataclass
class LidarProfile:
    site_id: str
    timestamp: str
    scan_id: str
    scan_mode: str
    source_kind: str
    azimuth_deg: float
    elevation_deg: float
    ranges_m: list[float]
    raw_counts: list[float]
    laser_energy_mj: float
    background_counts: float
    overlap: list[float]
    relative_humidity: float
    temperature_c: float
    wind_speed_ms: float
    wind_dir_deg: float
    molecular_backscatter: list[float]
    molecular_extinction: list[float]
    true_backscatter: list[float]
    true_extinction: list[float]
    true_pm25: list[float]
    true_pm10: list[float]
    true_hotspot_mask: list[int] = field(default_factory=list)


@dataclass
class GroundMeasurement:
    site_id: str
    timestamp: str
    pm25_ugm3: float
    pm10_ugm3: float
    relative_humidity: float
    temperature_c: float
    wind_speed_ms: float
    wind_dir_deg: float


@dataclass
class ProcessedProfile:
    profile: LidarProfile
    l1_signal: list[float]
    attenuated_backscatter: list[float]
    snr: list[float]
    extinction: list[float]
    dry_extinction: list[float]
    pm25: list[float]
    pm10: list[float]
    enu_points_m: list[list[float]]
    qc_flags: list[str]
    latency_ms: float


@dataclass
class Hotspot:
    timestamp: str
    scan_id: str
    hotspot_id: str
    centroid_enu_m: list[float]
    peak_pm25_ugm3: float
    mean_pm25_ugm3: float
    estimated_area_m2: float
    cell_count: int
    severity: str


def to_serializable(value: Any) -> Any:
    if hasattr(value, "__dataclass_fields__"):
        return asdict(value)
    return value
