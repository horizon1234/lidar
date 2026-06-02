from __future__ import annotations

from lidar_core.models import LidarProfile


def _mean(values: list[float]) -> float:
    return sum(values) / max(len(values), 1)


def preprocess_profile(profile: LidarProfile) -> dict:
    l1_signal = []
    attenuated_backscatter = []
    snr = []
    qc_flags: list[str] = []

    for index, raw_count in enumerate(profile.raw_counts):
        background_corrected = max(raw_count - profile.background_counts, 0.0)
        energy_normalized = background_corrected / max(profile.laser_energy_mj, 1e-6)
        overlap_corrected = energy_normalized / max(profile.overlap[index], 0.15)
        range_km = profile.ranges_m[index] / 1000.0
        attenuated = overlap_corrected * range_km * range_km
        signal_to_noise = background_corrected / max((raw_count + profile.background_counts) ** 0.5, 1.0)

        l1_signal.append(background_corrected)
        attenuated_backscatter.append(max(attenuated, 1e-9))
        snr.append(signal_to_noise)

    if min(profile.overlap[:3]) < 0.4:
        qc_flags.append("near-range-partial-overlap")
    if profile.laser_energy_mj < 0.93:
        qc_flags.append("low-laser-energy")
    if _mean(snr[:4]) < 3.0:
        qc_flags.append("weak-near-range-snr")

    return {
        "l1_signal": l1_signal,
        "attenuated_backscatter": attenuated_backscatter,
        "snr": snr,
        "qc_flags": qc_flags,
    }
