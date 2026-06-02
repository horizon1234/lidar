from __future__ import annotations

import json
import ssl
import sys
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _download(url: str, output_path: Path, verify_ssl: bool) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if verify_ssl:
        with urllib.request.urlopen(url) as response:
            output_path.write_bytes(response.read())
    else:
        context = ssl._create_unverified_context()
        with urllib.request.urlopen(url, context=context) as response:
            output_path.write_bytes(response.read())


def _fetch_json(url: str) -> dict:
    with urllib.request.urlopen(url) as response:
        return json.loads(response.read().decode("utf-8"))


def main() -> None:
    cloudnet_url = "https://cloudnet.fmi.fi/api/download/product/7d0ade91-3fec-4d70-b79f-c332a77c21a9/20250423_bucharest_chm15k_c60c931f.nc"
    cloudnet_path = ROOT / "data" / "public" / "cloudnet" / "20250423_bucharest_chm15k_c60c931f.nc"
    _download(cloudnet_url, cloudnet_path, verify_ssl=False)

    params_air = {
        "latitude": 44.344,
        "longitude": 26.012,
        "start_date": "2025-04-23",
        "end_date": "2025-04-23",
        "hourly": "pm10,pm2_5",
        "timezone": "UTC",
    }
    params_weather = {
        "latitude": 44.344,
        "longitude": 26.012,
        "start_date": "2025-04-23",
        "end_date": "2025-04-23",
        "hourly": "temperature_2m,relative_humidity_2m,windspeed_10m,winddirection_10m",
        "timezone": "UTC",
    }
    air = _fetch_json("https://air-quality-api.open-meteo.com/v1/air-quality?" + urllib.parse.urlencode(params_air))
    weather = _fetch_json("https://archive-api.open-meteo.com/v1/archive?" + urllib.parse.urlencode(params_weather))
    output_dir = ROOT / "data" / "public" / "cloudnet"
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "20250423_bucharest_open_meteo_air_quality.json").write_text(json.dumps(air, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_dir / "20250423_bucharest_open_meteo_weather.json").write_text(json.dumps(weather, ensure_ascii=False, indent=2), encoding="utf-8")
    manifest = {
        "cloudnet_lidar": str(cloudnet_path),
        "open_meteo_air_quality": str(output_dir / "20250423_bucharest_open_meteo_air_quality.json"),
        "open_meteo_weather": str(output_dir / "20250423_bucharest_open_meteo_weather.json"),
    }
    (output_dir / "20250423_bucharest_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()