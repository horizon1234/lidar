from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lidar_core.io import write_json


def _fetch_json(base_url: str, params: dict) -> dict:
    query = urlencode(params)
    with urlopen(f"{base_url}?{query}") as response:
        return json.loads(response.read().decode("utf-8"))


def main() -> None:
    latitude = 39.9042
    longitude = 116.4074
    start_date = "2026-05-30"
    end_date = "2026-05-31"

    air_quality = _fetch_json(
        "https://air-quality-api.open-meteo.com/v1/air-quality",
        {
            "latitude": latitude,
            "longitude": longitude,
            "hourly": "pm10,pm2_5",
            "start_date": start_date,
            "end_date": end_date,
            "timezone": "Asia/Shanghai",
        },
    )
    weather = _fetch_json(
        "https://archive-api.open-meteo.com/v1/archive",
        {
            "latitude": latitude,
            "longitude": longitude,
            "start_date": start_date,
            "end_date": end_date,
            "hourly": "temperature_2m,relative_humidity_2m,windspeed_10m,winddirection_10m",
            "timezone": "Asia/Shanghai",
        },
    )

    records = []
    for index, timestamp in enumerate(air_quality["hourly"]["time"]):
        records.append(
            {
                "timestamp": timestamp,
                "pm25": air_quality["hourly"]["pm2_5"][index],
                "pm10": air_quality["hourly"]["pm10"][index],
                "temperature_c": weather["hourly"]["temperature_2m"][index],
                "relative_humidity": weather["hourly"]["relative_humidity_2m"][index],
                "wind_speed_ms": round(weather["hourly"]["windspeed_10m"][index] / 3.6, 3),
                "wind_dir_deg": weather["hourly"]["winddirection_10m"][index],
            }
        )

    output_dir = ROOT / "data" / "public"
    output_dir.mkdir(parents=True, exist_ok=True)
    write_json(output_dir / "open_meteo_beijing_ground_pm_meteo.json", records)
    write_json(
        output_dir / "open_meteo_beijing_fetch_manifest.json",
        {
            "source": "Open-Meteo",
            "latitude": latitude,
            "longitude": longitude,
            "start_date": start_date,
            "end_date": end_date,
            "record_count": len(records),
            "fields": list(records[0].keys()) if records else [],
        },
    )

    csv_path = output_dir / "open_meteo_beijing_ground_pm_meteo.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)

    print(f"Fetched {len(records)} hourly records into {csv_path}")


if __name__ == "__main__":
    main()