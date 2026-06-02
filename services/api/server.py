from __future__ import annotations

import argparse
import json
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lidar_core.io import read_json
from lidar_core.pipeline import run_end_to_end


def _build_summary(config_path: Path) -> dict:
    config = read_json(config_path)
    results = run_end_to_end(config)
    return {
        "dataset_summary": results["dataset_summary"],
        "metrics": results["metrics"],
        "latest_hotspots": results["hotspots"],
        "alert_count": len(results["alerts"]),
    }


class DemoHandler(BaseHTTPRequestHandler):
    config_path = ROOT / "configs" / "default_pipeline.json"

    def do_GET(self) -> None:
        summary = _build_summary(self.config_path)
        if self.path not in {"/api/summary", "/api/hotspots"}:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not Found")
            return
        payload = summary if self.path == "/api/summary" else summary["latest_hotspots"]
        content = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format: str, *args) -> None:
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve atmospheric lidar demo API")
    parser.add_argument("--config", default=str(ROOT / "configs" / "default_pipeline.json"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    config_path = Path(args.config)
    if args.once:
        print(json.dumps(_build_summary(config_path), ensure_ascii=False, indent=2))
        return

    DemoHandler.config_path = config_path
    with HTTPServer((args.host, args.port), DemoHandler) as server:
        print(f"Serving on http://{args.host}:{args.port}/api/summary")
        server.serve_forever()


if __name__ == "__main__":
    main()
