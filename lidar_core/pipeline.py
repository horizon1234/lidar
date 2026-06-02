from __future__ import annotations

import copy
import time
from datetime import datetime
from pathlib import Path

from lidar_core.calibration import apply_humidity_correction, apply_pm_models, build_timestamp_feature_table, fit_pm_models, fit_station_offsets
from lidar_core.data_sources import load_campaign
from lidar_core.detection import detect_hotspots
from lidar_core.geometry import profile_bins_to_enu
from lidar_core.io import write_json
from lidar_core.models import ProcessedProfile, to_serializable
from lidar_core.preprocessing import preprocess_profile
from lidar_core.qa import classification_metrics, mae, r2_score, rmse
from lidar_core.retrieval import run_fernald_inversion


def _mean(values: list[float]) -> float:
    return sum(values) / max(len(values), 1)


def _group_by_timestamp(processed_profiles: list[ProcessedProfile]) -> dict[str, list[ProcessedProfile]]:
    grouped: dict[str, list[ProcessedProfile]] = {}
    for processed in processed_profiles:
        grouped.setdefault(processed.profile.timestamp, []).append(processed)
    return grouped


def _sampling_minutes(timestamps: list[str]) -> int:
    if len(timestamps) < 2:
        return 0
    deltas = []
    for index in range(1, len(timestamps)):
        current = datetime.fromisoformat(timestamps[index])
        previous = datetime.fromisoformat(timestamps[index - 1])
        deltas.append((current - previous).total_seconds() / 60.0)
    return round(_mean(deltas)) if deltas else 0


def _predict_sample(sample: dict, models: dict, station_offsets: dict | None = None) -> tuple[float, float]:
    station_offset = (station_offsets or {}).get(sample.get("site_id") or "default-site", {})
    pm25_prediction = sum(weight * value for weight, value in zip(models["pm25"], sample["features"])) + station_offset.get("pm25_offset", 0.0)
    pm10_prediction = sum(weight * value for weight, value in zip(models["pm10"], sample["features"])) + station_offset.get("pm10_offset", 0.0)
    return pm25_prediction, pm10_prediction


def _extract_ground_predictions(feature_table: list[dict], models: dict, timestamps: list[str], station_offsets: dict | None = None) -> dict:
    feature_by_timestamp = {sample["timestamp"]: sample for sample in feature_table}
    pm25_truth = []
    pm25_pred = []
    pm10_truth = []
    pm10_pred = []
    residual_rows = []
    for timestamp in timestamps:
        sample = feature_by_timestamp[timestamp]
        predicted_pm25, predicted_pm10 = _predict_sample(sample, models, station_offsets)
        pm25_truth.append(sample["pm25_true"])
        pm10_truth.append(sample["pm10_true"])
        pm25_pred.append(predicted_pm25)
        pm10_pred.append(predicted_pm10)
        residual_rows.append(
            {
                "timestamp": timestamp,
                "site_id": sample.get("site_id") or "default-site",
                "pm25_residual": sample["pm25_true"] - predicted_pm25,
                "pm10_residual": sample["pm10_true"] - predicted_pm10,
            }
        )
    return {
        "pm25_truth": pm25_truth,
        "pm25_pred": pm25_pred,
        "pm10_truth": pm10_truth,
        "pm10_pred": pm10_pred,
        "residual_rows": residual_rows,
    }


def _summarize_drift_monitoring(residual_rows: list[dict]) -> dict:
    grouped: dict[str, list[dict]] = {}
    for row in residual_rows:
        grouped.setdefault(row["site_id"], []).append(row)

    summary = {"stations": {}, "alerts": []}
    for site_id, rows in grouped.items():
        ordered_rows = sorted(rows, key=lambda item: item["timestamp"])
        window_size = min(3, len(ordered_rows))
        windows = []
        for start_index in range(0, len(ordered_rows) - window_size + 1):
            window = ordered_rows[start_index:start_index + window_size]
            pm25_bias = _mean([item["pm25_residual"] for item in window])
            pm10_bias = _mean([item["pm10_residual"] for item in window])
            alert_level = "ok"
            if abs(pm25_bias) > 6.0 or abs(pm10_bias) > 8.0:
                alert_level = "warning"
            window_summary = {
                "start": window[0]["timestamp"],
                "end": window[-1]["timestamp"],
                "pm25_bias": round(pm25_bias, 3),
                "pm10_bias": round(pm10_bias, 3),
                "alert_level": alert_level,
            }
            windows.append(window_summary)
            if alert_level != "ok":
                summary["alerts"].append({"site_id": site_id, **window_summary})
        latest_window = windows[-1] if windows else {"pm25_bias": 0.0, "pm10_bias": 0.0, "alert_level": "ok"}
        summary["stations"][site_id] = {
            "sample_count": len(ordered_rows),
            "latest_pm25_bias": latest_window["pm25_bias"],
            "latest_pm10_bias": latest_window["pm10_bias"],
            "latest_alert_level": latest_window["alert_level"],
            "windows": windows,
        }
    return summary


def _apply_failure_case(profiles, ground_measurements, case_name: str) -> None:
    if case_name == "high-background-light":
        for profile in profiles:
            extra_background = profile.background_counts * 1.9
            profile.background_counts += extra_background
            profile.raw_counts = [value + extra_background * 2.2 for value in profile.raw_counts]
        return
    if case_name == "overlap-miscalibration":
        for profile in profiles:
            for index in range(min(6, len(profile.overlap))):
                profile.overlap[index] = max(profile.overlap[index] * 0.65, 0.1)
        return
    if case_name == "humidity-surge":
        for profile in profiles:
            profile.relative_humidity = min(0.95, profile.relative_humidity + 0.22)
        for measurement in ground_measurements:
            measurement.relative_humidity = min(0.95, measurement.relative_humidity + 0.22)
        return
    raise ValueError(f"Unsupported failure case: {case_name}")


def _run_failure_case_suite(config: dict, site, profiles, ground_measurements) -> list[dict]:
    results = []
    for case_name in ["high-background-light", "overlap-miscalibration", "humidity-surge"]:
        case_profiles = copy.deepcopy(profiles)
        case_ground = copy.deepcopy(ground_measurements)
        _apply_failure_case(case_profiles, case_ground, case_name)
        case_output = _run_pipeline_on_dataset(config, site, case_profiles, case_ground)
        results.append(
            {
                "name": case_name,
                "pm25_rmse": case_output["metrics"]["pm25"]["rmse"],
                "pm10_rmse": case_output["metrics"]["pm10"]["rmse"],
                "hotspot_f1": case_output["metrics"]["hotspot"]["f1"],
                "throughput_profiles_per_s": case_output["metrics"]["runtime"]["throughput_profiles_per_s"],
            }
        )
    return results


def _build_demo_payload(site, processed_profiles: list[ProcessedProfile], feature_table: list[dict], split: dict, hotspots_by_timestamp: dict, metrics: dict, ablation: list[dict], sensitivity: list[dict], source_metadata: dict, station_calibration: dict, drift_monitoring: dict, failure_cases: list[dict]) -> dict:
    grouped = _group_by_timestamp(processed_profiles)
    timestamps = sorted(grouped)
    stare_profiles = [next(profile for profile in grouped[timestamp] if profile.profile.scan_mode == "stare") for timestamp in timestamps]
    curtain = {
        "times": timestamps,
        "heights_m": [round(value[2], 1) for value in stare_profiles[0].enu_points_m],
        "pm25": [[round(value, 2) for value in profile.pm25] for profile in stare_profiles],
        "extinction": [[round(value, 4) for value in profile.extinction] for profile in stare_profiles],
    }

    latest_timestamp = timestamps[-1]
    latest_ppi_profiles = sorted(
        [profile for profile in grouped[latest_timestamp] if profile.profile.scan_mode == "ppi"],
        key=lambda item: item.profile.azimuth_deg,
    )
    ppi_cells = []
    for processed in latest_ppi_profiles:
        for index, point in enumerate(processed.enu_points_m):
            ppi_cells.append(
                {
                    "x_m": round(point[0], 2),
                    "y_m": round(point[1], 2),
                    "z_m": round(point[2], 2),
                    "pm25": round(processed.pm25[index], 2),
                    "pm10": round(processed.pm10[index], 2),
                    "is_true_hotspot": bool(processed.profile.true_hotspot_mask[index]),
                }
            )

    qc_times = timestamps
    qc_mean_snr = [round(_mean(next(profile for profile in grouped[timestamp] if profile.profile.scan_mode == "stare").snr[:8]), 2) for timestamp in qc_times]
    qc_latency = [round(_mean([profile.latency_ms for profile in grouped[timestamp]]), 2) for timestamp in qc_times]
    qc_background = [round(next(profile for profile in grouped[timestamp] if profile.profile.scan_mode == "stare").profile.background_counts, 2) for timestamp in qc_times]
    qc_energy = [round(next(profile for profile in grouped[timestamp] if profile.profile.scan_mode == "stare").profile.laser_energy_mj, 3) for timestamp in qc_times]

    alerts = []
    for timestamp in timestamps:
        for hotspot in hotspots_by_timestamp.get(timestamp, []):
            alerts.append(to_serializable(hotspot))

    return {
        "site": to_serializable(site),
        "dataset_summary": {
            "profile_count": len(processed_profiles),
            "timestamp_count": len(timestamps),
            "sampling_minutes": _sampling_minutes(timestamps),
            "range_bin_count": len(stare_profiles[0].profile.ranges_m),
            "train_count": len(split["train"]),
            "val_count": len(split["val"]),
            "test_count": len(split["test"]),
        },
        "source": source_metadata,
        "curtain": curtain,
        "ppi": {
            "timestamp": latest_timestamp,
            "cells": ppi_cells,
        },
        "hotspots": [to_serializable(hotspot) for hotspot in hotspots_by_timestamp.get(latest_timestamp, [])],
        "qc": {
            "times": qc_times,
            "mean_snr": qc_mean_snr,
            "latency_ms": qc_latency,
            "background_counts": qc_background,
            "laser_energy_mj": qc_energy,
        },
        "alerts": alerts,
        "metrics": metrics,
        "sensitivity": sensitivity,
        "ablation": ablation,
        "failure_cases": failure_cases,
        "station_calibration": station_calibration,
        "drift_monitoring": drift_monitoring,
        "ground_series": feature_table,
    }


def _run_pipeline_on_dataset(config: dict, site, profiles, ground_measurements, lidar_ratio_override: float | None = None, disable_humidity: bool = False) -> dict:
    retrieval_config = config["retrieval"]
    humidity_config = config["humidity"]
    calibration_config = config["pm_calibration"]
    hotspot_config = config["hotspot"]
    processed_profiles: list[ProcessedProfile] = []

    total_start = time.perf_counter()
    for profile in profiles:
        started_at = time.perf_counter()
        preprocessed = preprocess_profile(profile)
        extinction, _ = run_fernald_inversion(
            profile,
            preprocessed["attenuated_backscatter"],
            float(lidar_ratio_override or retrieval_config["aerosol_lidar_ratio_sr"]),
            float(retrieval_config["reference_aerosol_backscatter"]),
        )
        dry_extinction = list(extinction)
        if not disable_humidity:
            dry_extinction = apply_humidity_correction(
                extinction,
                profile.relative_humidity,
                float(humidity_config["dry_reference_rh"]),
                float(humidity_config["hygroscopicity"]),
            )
        processed_profiles.append(
            ProcessedProfile(
                profile=profile,
                l1_signal=preprocessed["l1_signal"],
                attenuated_backscatter=preprocessed["attenuated_backscatter"],
                snr=preprocessed["snr"],
                extinction=extinction,
                dry_extinction=dry_extinction,
                pm25=[],
                pm10=[],
                enu_points_m=profile_bins_to_enu(profile),
                qc_flags=preprocessed["qc_flags"],
                latency_ms=(time.perf_counter() - started_at) * 1000.0,
            )
        )

    feature_table = build_timestamp_feature_table(processed_profiles, ground_measurements, int(calibration_config["surface_bin_count"]))
    models, split = fit_pm_models(feature_table, float(calibration_config["train_ratio"]), float(calibration_config["val_ratio"]))
    station_offsets = fit_station_offsets(feature_table, models, split)
    apply_pm_models(processed_profiles, models, feature_table, station_offsets=station_offsets)

    hotspots_by_timestamp: dict[str, list] = {}
    predicted_mask: list[int] = []
    truth_mask: list[int] = []
    grouped = _group_by_timestamp(processed_profiles)
    for timestamp, profiles_at_timestamp in grouped.items():
        ppi_profiles = [profile for profile in profiles_at_timestamp if profile.profile.scan_mode == "ppi"]
        hotspots, predicted, truth = detect_hotspots(
            ppi_profiles,
            float(hotspot_config["pm25_threshold_ugm3"]),
            float(hotspot_config["scan_relative_pm25_threshold_ugm3"]),
            float(hotspot_config["scan_relative_dry_ext_threshold"]),
            int(hotspot_config["min_cells"]),
        )
        hotspots_by_timestamp[timestamp] = hotspots
        predicted_mask.extend(predicted)
        truth_mask.extend(truth)

    elapsed_s = time.perf_counter() - total_start
    holdout_timestamps = split["val_timestamps"] + split["test_timestamps"]
    test_timestamps = holdout_timestamps or split["train_timestamps"]
    predictions = _extract_ground_predictions(feature_table, models, test_timestamps, station_offsets=station_offsets)
    hotspot_scores = classification_metrics(truth_mask, predicted_mask)
    drift_monitoring = _summarize_drift_monitoring(predictions["residual_rows"])

    metrics = {
        "pm25": {
            "mae": round(mae(predictions["pm25_truth"], predictions["pm25_pred"]), 3),
            "rmse": round(rmse(predictions["pm25_truth"], predictions["pm25_pred"]), 3),
            "r2": round(r2_score(predictions["pm25_truth"], predictions["pm25_pred"]), 3),
        },
        "pm10": {
            "mae": round(mae(predictions["pm10_truth"], predictions["pm10_pred"]), 3),
            "rmse": round(rmse(predictions["pm10_truth"], predictions["pm10_pred"]), 3),
            "r2": round(r2_score(predictions["pm10_truth"], predictions["pm10_pred"]), 3),
        },
        "hotspot": {key: round(value, 3) if isinstance(value, float) else value for key, value in hotspot_scores.items()},
        "drift": {
            "station_count": len(drift_monitoring["stations"]),
            "alert_count": len(drift_monitoring["alerts"]),
        },
        "runtime": {
            "mean_latency_ms": round(_mean([profile.latency_ms for profile in processed_profiles]), 3),
            "throughput_profiles_per_s": round(len(processed_profiles) / max(elapsed_s, 1e-9), 3),
        },
    }

    return {
        "site": site,
        "profiles": profiles,
        "ground_measurements": ground_measurements,
        "processed_profiles": processed_profiles,
        "feature_table": feature_table,
        "split": split,
        "station_offsets": station_offsets,
        "drift_monitoring": drift_monitoring,
        "hotspots_by_timestamp": hotspots_by_timestamp,
        "metrics": metrics,
    }


def run_end_to_end(config: dict, output_root: str | Path | None = None) -> dict:
    site, profiles, ground_measurements, source_metadata = load_campaign(config, output_root=output_root)
    primary = _run_pipeline_on_dataset(config, site, profiles, ground_measurements)

    ablation = []
    full_metrics = primary["metrics"]
    ablation.append({
        "name": "full-pipeline",
        "pm25_rmse": full_metrics["pm25"]["rmse"],
        "hotspot_f1": full_metrics["hotspot"]["f1"],
    })

    humidity_disabled = _run_pipeline_on_dataset(config, site, copy.deepcopy(profiles), copy.deepcopy(ground_measurements), disable_humidity=True)
    ablation.append({
        "name": "without-humidity-correction",
        "pm25_rmse": humidity_disabled["metrics"]["pm25"]["rmse"],
        "hotspot_f1": humidity_disabled["metrics"]["hotspot"]["f1"],
    })

    sensitivity = []
    for lidar_ratio in config["evaluation"]["sensitivity_lidar_ratios"]:
        perturbed = _run_pipeline_on_dataset(config, site, copy.deepcopy(profiles), copy.deepcopy(ground_measurements), lidar_ratio_override=float(lidar_ratio))
        sensitivity.append(
            {
                "lidar_ratio_sr": float(lidar_ratio),
                "pm25_rmse": perturbed["metrics"]["pm25"]["rmse"],
                "pm10_rmse": perturbed["metrics"]["pm10"]["rmse"],
                "hotspot_f1": perturbed["metrics"]["hotspot"]["f1"],
            }
        )

    stability_span = max(item["pm25_rmse"] for item in sensitivity) - min(item["pm25_rmse"] for item in sensitivity)
    primary["metrics"]["retrieval_stability"] = {
        "pm25_rmse_span": round(stability_span, 3),
        "reference_lidar_ratio_sr": config["retrieval"]["aerosol_lidar_ratio_sr"],
    }
    failure_cases = _run_failure_case_suite(config, site, profiles, ground_measurements)

    demo_payload = _build_demo_payload(
        primary["site"],
        primary["processed_profiles"],
        primary["feature_table"],
        primary["split"],
        primary["hotspots_by_timestamp"],
        primary["metrics"],
        ablation,
        sensitivity,
        source_metadata,
        primary["station_offsets"],
        primary["drift_monitoring"],
        failure_cases,
    )
    demo_payload["dataset_summary"]["source_mode"] = source_metadata["mode"]
    demo_payload["dataset_summary"]["source_breakdown"] = {
        "stare_real": sum(1 for profile in primary["profiles"] if profile.source_kind == "cloudnet_real_stare"),
        "stare_synthetic": sum(1 for profile in primary["profiles"] if profile.source_kind == "synthetic_stare"),
        "ppi_hybrid": sum(1 for profile in primary["profiles"] if profile.source_kind == "synthetic_ppi_hybrid"),
        "ppi_synthetic": sum(1 for profile in primary["profiles"] if profile.source_kind == "synthetic_ppi"),
    }

    if output_root is not None:
        root = Path(output_root)
        write_json(root / "data" / "raw" / "simulated_demo_campaign.json", [to_serializable(profile) for profile in primary["profiles"]])
        write_json(root / "data" / "l1" / "demo_preprocessed.json", [to_serializable(profile) for profile in primary["processed_profiles"]])
        write_json(root / "data" / "l2" / "demo_results.json", demo_payload)

    return demo_payload
