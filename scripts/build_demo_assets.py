from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lidar_core.io import read_json, write_json
from lidar_core.pipeline import run_end_to_end


def _build_line_chart_svg(values: list[float], width: int, height: int, stroke: str) -> str:
    minimum = min(values)
    maximum = max(values)
    spread = maximum - minimum or 1.0
    step_x = width / max(len(values) - 1, 1)
    points = []
    for index, value in enumerate(values):
        x_coord = index * step_x
        y_coord = height - ((value - minimum) / spread) * (height - 20) - 10
        points.append(f"{x_coord:.1f},{y_coord:.1f}")
    polyline = " ".join(points)
    return f'<svg viewBox="0 0 {width} {height}" class="chart"><polyline fill="none" stroke="{stroke}" stroke-width="3" points="{polyline}" /></svg>'


def _render_dashboard(data: dict) -> str:
    embedded = json.dumps(data, ensure_ascii=False)
    snr_chart = _build_line_chart_svg(data["qc"]["mean_snr"], 520, 180, "#0f766e")
    latency_chart = _build_line_chart_svg(data["qc"]["latency_ms"], 520, 180, "#b45309")
    return f"""<!DOCTYPE html>
<html lang=\"zh-CN\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>Atmospheric Lidar Demo</title>
  <style>
    :root {{
      --bg: #f7f4ed;
      --panel: rgba(255, 252, 247, 0.9);
      --ink: #1f2937;
      --muted: #6b7280;
      --accent: #0f766e;
      --accent-soft: #d6f0ec;
      --danger: #c2410c;
      --shadow: 0 18px 42px rgba(15, 23, 42, 0.08);
    }}
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; font-family: 'IBM Plex Sans', 'Segoe UI', sans-serif; background: radial-gradient(circle at top left, #fff8ec, var(--bg)); color: var(--ink); }}
    header {{ padding: 32px 36px 16px; }}
    h1 {{ margin: 0; font-size: 30px; }}
    p {{ color: var(--muted); line-height: 1.6; }}
    nav {{ display: flex; gap: 12px; padding: 0 36px 18px; flex-wrap: wrap; }}
    nav button {{ border: 0; border-radius: 999px; padding: 10px 16px; background: white; box-shadow: var(--shadow); cursor: pointer; color: var(--ink); }}
    nav button.active {{ background: var(--accent); color: white; }}
    main {{ display: grid; gap: 18px; padding: 0 36px 36px; }}
    section {{ display: none; gap: 18px; }}
    section.active {{ display: grid; }}
    .grid {{ display: grid; gap: 18px; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); }}
    .card {{ background: var(--panel); backdrop-filter: blur(14px); border: 1px solid rgba(15, 23, 42, 0.06); border-radius: 22px; padding: 18px; box-shadow: var(--shadow); }}
    .metric {{ font-size: 28px; font-weight: 700; margin-top: 8px; }}
    .muted {{ color: var(--muted); font-size: 14px; }}
    .heatmap {{ display: grid; gap: 2px; }}
    .heatmap-row {{ display: grid; gap: 2px; grid-template-columns: repeat(auto-fit, minmax(12px, 1fr)); }}
    .heatmap-cell {{ height: 12px; border-radius: 2px; }}
    .legend {{ display: flex; justify-content: space-between; color: var(--muted); font-size: 12px; margin-top: 10px; }}
    svg.chart {{ width: 100%; height: 180px; background: linear-gradient(180deg, rgba(15,118,110,0.08), rgba(255,255,255,0)); border-radius: 16px; }}
    .ppi {{ position: relative; width: 100%; min-height: 480px; background: linear-gradient(180deg, #fdfaf4, #eef7f5); border-radius: 20px; overflow: hidden; }}
    .ppi-point {{ position: absolute; width: 8px; height: 8px; border-radius: 50%; transform: translate(-50%, -50%); opacity: 0.9; }}
    table {{ width: 100%; border-collapse: collapse; }}
    th, td {{ padding: 10px 8px; border-bottom: 1px solid rgba(107,114,128,0.18); text-align: left; font-size: 14px; }}
    .severity-medium {{ color: #0f766e; }}
    .severity-high {{ color: #b45309; }}
    .severity-critical {{ color: var(--danger); }}
    .pill {{ display: inline-block; padding: 4px 10px; border-radius: 999px; background: var(--accent-soft); color: var(--accent); font-size: 12px; font-weight: 600; }}
    @media (max-width: 768px) {{
      header, nav, main {{ padding-left: 18px; padding-right: 18px; }}
      .ppi {{ min-height: 340px; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>颗粒物大气激光雷达 Demo</h1>
    <p>一个自包含的静态演示页面，展示时间高度图、PPI 热点、质量控制、历史告警和关键评测指标。</p>
  </header>
  <nav>
    <button class=\"tab active\" data-target=\"overview\">实时总览</button>
    <button class=\"tab\" data-target=\"ppi\">PPI 与热点</button>
    <button class=\"tab\" data-target=\"qc\">质量控制</button>
    <button class=\"tab\" data-target=\"history\">历史回放</button>
  </nav>
  <main>
    <section id=\"overview\" class=\"active\">
      <div class=\"grid\">
        <div class=\"card\"><div class=\"muted\">PM2.5 RMSE</div><div class=\"metric\">{data['metrics']['pm25']['rmse']}</div></div>
        <div class=\"card\"><div class=\"muted\">PM10 RMSE</div><div class=\"metric\">{data['metrics']['pm10']['rmse']}</div></div>
        <div class=\"card\"><div class=\"muted\">热点 F1</div><div class=\"metric\">{data['metrics']['hotspot']['f1']}</div></div>
        <div class=\"card\"><div class=\"muted\">吞吐量</div><div class=\"metric\">{data['metrics']['runtime']['throughput_profiles_per_s']}</div></div>
      </div>
      <div class=\"card\">
        <div class=\"pill\">时间高度图</div>
        <div id=\"curtain\" class=\"heatmap\"></div>
        <div class=\"legend\"><span>低</span><span>PM2.5 估算</span><span>高</span></div>
      </div>
    </section>
    <section id=\"ppi\">
      <div class=\"grid\">
        <div class=\"card\">
          <div class=\"pill\">PPI 热点定位</div>
          <div id=\"ppi-layer\" class=\"ppi\"></div>
        </div>
        <div class=\"card\">
          <div class=\"pill\">热点事件</div>
          <table>
            <thead><tr><th>ID</th><th>峰值 PM2.5</th><th>面积</th><th>等级</th></tr></thead>
            <tbody id=\"hotspot-table\"></tbody>
          </table>
        </div>
      </div>
    </section>
    <section id=\"qc\">
      <div class=\"grid\">
        <div class=\"card\"><div class=\"pill\">平均 SNR</div>{snr_chart}</div>
        <div class=\"card\"><div class=\"pill\">平均处理时延</div>{latency_chart}</div>
      </div>
      <div class=\"grid\">
        <div class=\"card\"><div class=\"muted\">背景光基线</div><div class=\"metric\">{max(data['qc']['background_counts'])}</div></div>
        <div class=\"card\"><div class=\"muted\">激光能量波动</div><div class=\"metric\">{max(data['qc']['laser_energy_mj']) - min(data['qc']['laser_energy_mj']):.3f}</div></div>
        <div class=\"card\"><div class=\"muted\">反演稳定性跨度</div><div class=\"metric\">{data['metrics']['retrieval_stability']['pm25_rmse_span']}</div></div>
      </div>
    </section>
    <section id=\"history\">
      <div class=\"grid\">
        <div class=\"card\">
          <div class=\"pill\">数据划分</div>
          <table>
            <tbody>
              <tr><th>训练集</th><td>{data['dataset_summary']['train_count']}</td></tr>
              <tr><th>验证集</th><td>{data['dataset_summary']['val_count']}</td></tr>
              <tr><th>测试集</th><td>{data['dataset_summary']['test_count']}</td></tr>
              <tr><th>时序步数</th><td>{data['dataset_summary']['timestamp_count']}</td></tr>
            </tbody>
          </table>
        </div>
        <div class=\"card\">
          <div class=\"pill\">消融和敏感性</div>
          <table>
            <thead><tr><th>实验</th><th>PM2.5 RMSE</th><th>热点 F1</th></tr></thead>
            <tbody id=\"analysis-table\"></tbody>
          </table>
        </div>
      </div>
      <div class=\"card\">
        <div class=\"pill\">历史告警</div>
        <table>
          <thead><tr><th>时间</th><th>峰值 PM2.5</th><th>质心 ENU</th><th>等级</th></tr></thead>
          <tbody id=\"alert-table\"></tbody>
        </table>
      </div>
    </section>
  </main>
  <script id=\"demo-data\" type=\"application/json\">{embedded}</script>
  <script>
    const data = JSON.parse(document.getElementById('demo-data').textContent);
    const tabs = document.querySelectorAll('.tab');
    tabs.forEach((button) => {{
      button.addEventListener('click', () => {{
        tabs.forEach((item) => item.classList.remove('active'));
        document.querySelectorAll('section').forEach((section) => section.classList.remove('active'));
        button.classList.add('active');
        document.getElementById(button.dataset.target).classList.add('active');
      }});
    }});

    const curtain = document.getElementById('curtain');
    const curtainValues = data.curtain.pm25.flat();
    const curtainMin = Math.min(...curtainValues);
    const curtainMax = Math.max(...curtainValues);
    data.curtain.pm25.forEach((row) => {{
      const rowElement = document.createElement('div');
      rowElement.className = 'heatmap-row';
      row.forEach((value) => {{
        const normalized = (value - curtainMin) / Math.max(curtainMax - curtainMin, 1e-6);
        const cell = document.createElement('div');
        cell.className = 'heatmap-cell';
        const hue = 190 - normalized * 165;
        const lightness = 86 - normalized * 36;
        cell.style.background = 'hsl(' + hue + ', 72%, ' + lightness + '%)';
        cell.title = value.toFixed(2);
        rowElement.appendChild(cell);
      }});
      curtain.appendChild(rowElement);
    }});

    const ppiLayer = document.getElementById('ppi-layer');
    const xValues = data.ppi.cells.map((cell) => cell.x_m);
    const yValues = data.ppi.cells.map((cell) => cell.y_m);
    const pmValues = data.ppi.cells.map((cell) => cell.pm25);
    const xMin = Math.min(...xValues);
    const xMax = Math.max(...xValues);
    const yMin = Math.min(...yValues);
    const yMax = Math.max(...yValues);
    const pmMin = Math.min(...pmValues);
    const pmMax = Math.max(...pmValues);
    data.ppi.cells.forEach((cell) => {{
      const node = document.createElement('div');
      node.className = 'ppi-point';
      const left = ((cell.x_m - xMin) / Math.max(xMax - xMin, 1e-6)) * 100;
      const top = 100 - ((cell.y_m - yMin) / Math.max(yMax - yMin, 1e-6)) * 100;
      const normalized = (cell.pm25 - pmMin) / Math.max(pmMax - pmMin, 1e-6);
      node.style.left = left + '%';
      node.style.top = top + '%';
      node.style.background = cell.is_true_hotspot ? '#c2410c' : 'hsl(' + (190 - normalized * 160) + ', 75%, ' + (80 - normalized * 35) + '%)';
      node.style.width = (6 + normalized * 10) + 'px';
      node.style.height = node.style.width;
      node.title = 'PM2.5 ' + cell.pm25.toFixed(1) + ' ug/m3';
      ppiLayer.appendChild(node);
    }});

    const hotspotTable = document.getElementById('hotspot-table');
    data.hotspots.forEach((hotspot) => {{
      const row = document.createElement('tr');
      row.innerHTML = '<td>' + hotspot.hotspot_id + '</td><td>' + hotspot.peak_pm25_ugm3.toFixed(1) + '</td><td>' + hotspot.estimated_area_m2.toFixed(1) + '</td><td class="severity-' + hotspot.severity + '">' + hotspot.severity + '</td>';
      hotspotTable.appendChild(row);
    }});

    const analysisTable = document.getElementById('analysis-table');
    [...data.ablation, ...data.sensitivity.map((item) => ({{ name: 'lidar_ratio_' + item.lidar_ratio_sr, pm25_rmse: item.pm25_rmse, hotspot_f1: item.hotspot_f1 }}))].forEach((item) => {{
      const row = document.createElement('tr');
      row.innerHTML = '<td>' + item.name + '</td><td>' + item.pm25_rmse + '</td><td>' + item.hotspot_f1 + '</td>';
      analysisTable.appendChild(row);
    }});

    const alertTable = document.getElementById('alert-table');
    data.alerts.slice(0, 12).forEach((alert) => {{
      const row = document.createElement('tr');
      row.innerHTML = '<td>' + alert.timestamp + '</td><td>' + alert.peak_pm25_ugm3.toFixed(1) + '</td><td>' + alert.centroid_enu_m.map((value) => value.toFixed(1)).join(', ') + '</td><td class="severity-' + alert.severity + '">' + alert.severity + '</td>';
      alertTable.appendChild(row);
    }});
  </script>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate demo dashboard assets")
    parser.add_argument("--config", default=str(ROOT / "configs" / "default_pipeline.json"))
    parser.add_argument("--output-root", default=str(ROOT))
    parser.add_argument("--dashboard", default=None)
    args = parser.parse_args()

    config_path = Path(args.config)
    output_root = Path(args.output_root)
    config = read_json(config_path)
    if "source" in config and "root" in config["source"]:
        config["source"]["root"] = str(ROOT)
    results = run_end_to_end(config, output_root=output_root)
    write_json(output_root / "data" / "l2" / "demo_results.json", results)
    dashboard_path = Path(args.dashboard) if args.dashboard else output_root / "web" / "demo_dashboard.html"
    dashboard_path.parent.mkdir(parents=True, exist_ok=True)
    dashboard_path.write_text(_render_dashboard(results), encoding="utf-8")
    print(f"Generated {dashboard_path}")


if __name__ == "__main__":
    main()

