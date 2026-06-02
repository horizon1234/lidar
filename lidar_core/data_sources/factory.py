from __future__ import annotations

from lidar_core.data_sources.cloudnet import load_cloudnet_hybrid_campaign
from lidar_core.simulation.scene import simulate_campaign


def load_campaign(config: dict, output_root=None):
    source = config.get("source", {})
    mode = source.get("mode", "simulation")
    if mode == "simulation":
        site, profiles, ground_measurements = simulate_campaign(config)
        return site, profiles, ground_measurements, {
            "mode": "simulation",
            "site_id": site.site_id or site.name.lower().replace(" ", "-"),
            "site_name": site.name,
            "real_stare_profile_count": 0,
            "synthetic_ppi_profile_count": sum(1 for profile in profiles if profile.scan_mode == "ppi"),
        }
    if mode == "cloudnet_hybrid":
        return load_cloudnet_hybrid_campaign(config)
    raise ValueError(f"Unsupported source mode: {mode}")
