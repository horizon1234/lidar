from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lidar_core.io import read_json, write_json
from lidar_core.pipeline import run_end_to_end


def main() -> None:
    parser = argparse.ArgumentParser(description="Run batch lidar processing pipeline")
    parser.add_argument("--config", default=str(ROOT / "configs" / "default_pipeline.json"))
    parser.add_argument("--output", default=str(ROOT))
    args = parser.parse_args()

    config = read_json(args.config)
    results = run_end_to_end(config, output_root=args.output)
    summary_path = Path(args.output) / "data" / "l2" / "worker_summary.json"
    write_json(summary_path, results["metrics"])
    print(json.dumps(results["metrics"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
