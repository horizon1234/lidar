from __future__ import annotations


def growth_factor(relative_humidity: float, dry_reference_rh: float, hygroscopicity: float) -> float:
    rh = min(max(relative_humidity, 0.05), 0.98)
    dry_ratio = dry_reference_rh / max(1.0 - dry_reference_rh, 0.02)
    humid_ratio = rh / max(1.0 - rh, 0.02)
    return max(1.0, 1.0 + hygroscopicity * max(humid_ratio - dry_ratio, 0.0) * 0.18)


def apply_humidity_correction(
    extinction: list[float],
    relative_humidity: float,
    dry_reference_rh: float,
    hygroscopicity: float,
) -> list[float]:
    factor = growth_factor(relative_humidity, dry_reference_rh, hygroscopicity)
    return [value / factor for value in extinction]
