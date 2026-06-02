from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lidar_core.io import read_json
from lidar_core.pipeline import run_end_to_end


class PipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        config = read_json(ROOT / "configs" / "default_pipeline.json")
        cls.results = run_end_to_end(config)

    def test_metrics_exist(self) -> None:
        metrics = self.results["metrics"]
        self.assertIn("pm25", metrics)
        self.assertIn("pm10", metrics)
        self.assertIn("hotspot", metrics)
        self.assertGreater(metrics["runtime"]["throughput_profiles_per_s"], 0.0)

    def test_pm_quality_is_reasonable(self) -> None:
        self.assertLess(self.results["metrics"]["pm25"]["rmse"], 20.0)
        self.assertLess(self.results["metrics"]["pm10"]["rmse"], 25.0)
        self.assertGreater(self.results["metrics"]["pm25"]["r2"], 0.7)

    def test_hotspot_detection_is_reasonable(self) -> None:
        self.assertGreater(self.results["metrics"]["hotspot"]["f1"], 0.55)
        self.assertGreaterEqual(len(self.results["alerts"]), 1)

    def test_demo_payload_shapes(self) -> None:
        curtain = self.results["curtain"]
        self.assertEqual(len(curtain["times"]), self.results["dataset_summary"]["timestamp_count"])
        self.assertEqual(len(curtain["pm25"]), len(curtain["times"]))
        self.assertGreater(len(self.results["ppi"]["cells"]), 100)


if __name__ == "__main__":
    unittest.main()