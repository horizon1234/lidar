from __future__ import annotations

from typing import Iterable

from lidar_core.models import GroundMeasurement, ProcessedProfile


def _mean(values: Iterable[float]) -> float:
    items = list(values)
    return sum(items) / max(len(items), 1)


def _solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    size = len(vector)
    augmented = [row[:] + [vector[index]] for index, row in enumerate(matrix)]
    for pivot in range(size):
        max_row = max(range(pivot, size), key=lambda row_index: abs(augmented[row_index][pivot]))
        augmented[pivot], augmented[max_row] = augmented[max_row], augmented[pivot]
        pivot_value = augmented[pivot][pivot]
        if abs(pivot_value) < 1e-9:
            raise ValueError("Singular matrix encountered during PM calibration")
        for column in range(pivot, size + 1):
            augmented[pivot][column] /= pivot_value
        for row_index in range(size):
            if row_index == pivot:
                continue
            factor = augmented[row_index][pivot]
            for column in range(pivot, size + 1):
                augmented[row_index][column] -= factor * augmented[pivot][column]
    return [augmented[row_index][-1] for row_index in range(size)]


def _fit_linear_regression(feature_matrix: list[list[float]], target: list[float]) -> list[float]:
    feature_count = len(feature_matrix[0])
    gram = [[0.0 for _ in range(feature_count)] for _ in range(feature_count)]
    rhs = [0.0 for _ in range(feature_count)]

    for row, target_value in zip(feature_matrix, target):
        for left in range(feature_count):
            rhs[left] += row[left] * target_value
            for right in range(feature_count):
                gram[left][right] += row[left] * row[right]

    for diagonal in range(feature_count):
        gram[diagonal][diagonal] += 1e-6

    return _solve_linear_system(gram, rhs)


def _predict(coefficients: list[float], features: list[float]) -> float:
    return sum(weight * value for weight, value in zip(coefficients, features))


def build_timestamp_feature_table(
    processed_profiles: list[ProcessedProfile],
    ground_measurements: list[GroundMeasurement],
    surface_bin_count: int,
) -> list[dict]:
    ground_by_timestamp = {measurement.timestamp: measurement for measurement in ground_measurements}
    by_timestamp: dict[str, dict[str, list[ProcessedProfile] | ProcessedProfile | GroundMeasurement]] = {}

    for processed in processed_profiles:
        container = by_timestamp.setdefault(processed.profile.timestamp, {"stare": [], "ppi": []})
        container[processed.profile.scan_mode].append(processed)

    samples = []
    for timestamp in sorted(by_timestamp):
        ground = ground_by_timestamp.get(timestamp)
        modes = by_timestamp[timestamp]
        if ground is None or not modes["stare"]:
            continue
        stare_profile = modes["stare"][0]
        ppi_profiles = modes["ppi"]
        hotspot_proxy = max((max(profile.dry_extinction[:surface_bin_count]) for profile in ppi_profiles), default=max(stare_profile.dry_extinction[:surface_bin_count]))
        surface_dry_ext = _mean(stare_profile.dry_extinction[:surface_bin_count])
        surface_attenuated = _mean(stare_profile.attenuated_backscatter[:surface_bin_count])
        features = [1.0, surface_dry_ext, ground.relative_humidity, hotspot_proxy]
        samples.append(
            {
                "timestamp": timestamp,
                "features": features,
                "surface_dry_ext": surface_dry_ext,
                "surface_attenuated": surface_attenuated,
                "hotspot_proxy": hotspot_proxy,
                "site_id": stare_profile.profile.site_id or ground.site_id or "default-site",
                "pm25_true": ground.pm25_ugm3,
                "pm10_true": ground.pm10_ugm3,
                "relative_humidity": ground.relative_humidity,
                "wind_speed_ms": ground.wind_speed_ms,
            }
        )
    return samples


def _split_samples(samples: list[dict], train_ratio: float, val_ratio: float) -> dict:
    train = []
    val = []
    test = []
    for index, sample in enumerate(samples):
        bucket = index % 5
        if bucket in {0, 1, 2}:
            train.append(sample)
        elif bucket == 3:
            val.append(sample)
        else:
            test.append(sample)

    if not val and train:
        val.append(train.pop())
    if not test and train:
        test.append(train.pop())

    return {
        "train": train,
        "val": val,
        "test": test,
        "train_timestamps": [sample["timestamp"] for sample in train],
        "val_timestamps": [sample["timestamp"] for sample in val],
        "test_timestamps": [sample["timestamp"] for sample in test],
    }


def fit_pm_models(samples: list[dict], train_ratio: float, val_ratio: float) -> tuple[dict, dict]:
    split = _split_samples(samples, train_ratio, val_ratio)
    train_samples = split["train"]
    feature_matrix = [sample["features"] for sample in train_samples]
    pm25_target = [sample["pm25_true"] for sample in train_samples]
    pm10_target = [sample["pm10_true"] for sample in train_samples]
    models = {
        "pm25": _fit_linear_regression(feature_matrix, pm25_target),
        "pm10": _fit_linear_regression(feature_matrix, pm10_target),
    }
    return models, split


def fit_station_offsets(samples: list[dict], models: dict, split: dict) -> dict:
    grouped: dict[str, dict[str, list[float]]] = {}
    for sample in split["train"]:
        site_id = sample.get("site_id") or "default-site"
        current = grouped.setdefault(site_id, {"pm25": [], "pm10": []})
        current["pm25"].append(sample["pm25_true"] - _predict(models["pm25"], sample["features"]))
        current["pm10"].append(sample["pm10_true"] - _predict(models["pm10"], sample["features"]))

    return {
        site_id: {
            "pm25_offset": _mean(values["pm25"]),
            "pm10_offset": _mean(values["pm10"]),
            "sample_count": len(values["pm25"]),
        }
        for site_id, values in grouped.items()
    }


def apply_pm_models(processed_profiles: list[ProcessedProfile], models: dict, feature_table: list[dict], station_offsets: dict | None = None) -> None:
    feature_by_timestamp = {sample["timestamp"]: sample for sample in feature_table}
    for processed in processed_profiles:
        timestamp_features = feature_by_timestamp[processed.profile.timestamp]
        station_offset = (station_offsets or {}).get(timestamp_features.get("site_id") or processed.profile.site_id or "default-site", {})
        pm25_offset = station_offset.get("pm25_offset", 0.0)
        pm10_offset = station_offset.get("pm10_offset", 0.0)
        processed.pm25 = []
        processed.pm10 = []
        for dry_extinction, attenuated in zip(processed.dry_extinction, processed.attenuated_backscatter):
            local_excess = max(0.0, dry_extinction - timestamp_features["surface_dry_ext"])
            features = [
                1.0,
                dry_extinction,
                processed.profile.relative_humidity,
                timestamp_features["hotspot_proxy"],
            ]
            base_pm25 = _predict(models["pm25"], features) + pm25_offset
            base_pm10 = _predict(models["pm10"], features) + pm10_offset
            hotspot_boost = 1250.0 * local_excess
            processed.pm25.append(max(0.0, base_pm25 + hotspot_boost))
            processed.pm10.append(max(0.0, base_pm10 + 1.18 * hotspot_boost))
