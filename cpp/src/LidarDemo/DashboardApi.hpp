// Public API implementation fragment: dashboard and summary rendering.
// ---- 前端 HTML 仪表盘渲染 ----

/**
 * @brief 用一次 run_end_to_end 得到的 demo payload 渲染单页 HTML 仪表盘（中文）。
 *
 * 这是最重要的对外 API 之一，产出可直接落盘到 web/demo_dashboard.html 风格的页面。
 * 渲染策略：
 *  1. 从 payload 的 qc.mean_snr / qc.latency_ms 提取两条时间序列，用 build_line_chart_svg
 *     各生成一张内嵌 SVG 折线图（颜色分别对应 telemetry 蓝/琥珀）；
 *  2. 把整个 payload 以紧凑 JSON 嵌入到 <script type="application/json"> 中，
 *     页面加载时一段 JS 把 source/drift_monitoring/failure_cases 三段以 <pre> 美化输出；
 *  3. 顶部三段卡片：四类关键指标（PM2.5 RMSE、PM10 RMSE、热点 F1、吞吐量）。
 *
 * CSS 内嵌在 <style> 中，单文件即可离线使用。
 *
 * @param data 完整的 demo payload（来自 run_end_to_end）。
 * @return std::string 完整的 HTML 文档字符串。
 */
std::string render_dashboard(const Json& data) {
    std::vector<double> snr_values = json_to_double_vector(data.at("qc").at("mean_snr"));
    std::vector<double> latency_values = json_to_double_vector(data.at("qc").at("latency_ms"));
    // 紧凑 JSON（无缩进）以减小 HTML 体积
    std::string embedded = dump_json(data, 0);
    std::string snr_chart = build_line_chart_svg(snr_values, 520, 180, "#0f766e");     ///< teal 主色调
    std::string latency_chart = build_line_chart_svg(latency_values, 520, 180, "#b45309"); ///< 琥珀色对比

    std::ostringstream html;
    html << "<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\" /><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />"
         << "<title>Atmospheric Lidar Demo</title><style>"
         << ":root{--bg:#f7f4ed;--panel:rgba(255,252,247,.92);--ink:#1f2937;--muted:#6b7280;--accent:#0f766e;--shadow:0 18px 42px rgba(15,23,42,.08);}"
         << "*{box-sizing:border-box}body{margin:0;font-family:'IBM Plex Sans','Segoe UI',sans-serif;background:radial-gradient(circle at top left,#fff8ec,var(--bg));color:var(--ink)}"
         << "header{padding:32px 36px 16px}main{display:grid;gap:18px;padding:0 36px 36px}.grid{display:grid;gap:18px;grid-template-columns:repeat(auto-fit,minmax(260px,1fr))}"
         << ".card{background:var(--panel);border:1px solid rgba(15,23,42,.06);border-radius:22px;padding:18px;box-shadow:var(--shadow)}.metric{font-size:28px;font-weight:700;margin-top:8px}"
         << ".muted{color:var(--muted);font-size:14px}.pill{display:inline-block;padding:4px 10px;border-radius:999px;background:#d6f0ec;color:var(--accent);font-size:12px;font-weight:600}"
         << "svg.chart{width:100%;height:180px;background:linear-gradient(180deg,rgba(15,118,110,.08),rgba(255,255,255,0));border-radius:16px}.panel{white-space:pre-wrap;line-height:1.6;font-size:13px}"
         << "@media (max-width:768px){header,main{padding-left:18px;padding-right:18px}}</style></head><body>"
         << "<header><h1>颗粒物大气激光雷达 Demo</h1><p class=\"muted\">C++ 版本输出的静态结果页面，展示指标、质量控制、站点校准和失败案例摘要。</p></header><main>"
         // 顶部四个关键指标卡片
         << "<section class=\"grid\">"
         << "<div class=\"card\"><div class=\"muted\">PM2.5 RMSE</div><div class=\"metric\">" << data.at("metrics").at("pm25").at("rmse").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">PM10 RMSE</div><div class=\"metric\">" << data.at("metrics").at("pm10").at("rmse").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">热点 F1</div><div class=\"metric\">" << data.at("metrics").at("hotspot").at("f1").number_value() << "</div></div>"
         << "<div class=\"card\"><div class=\"muted\">吞吐量</div><div class=\"metric\">" << data.at("metrics").at("runtime").at("throughput_profiles_per_s").number_value() << "</div></div>"
         // QC 折线卡片
         << "</section><section class=\"grid\">"
         << "<div class=\"card\"><div class=\"pill\">平均 SNR</div>" << snr_chart << "</div>"
         << "<div class=\"card\"><div class=\"pill\">平均处理时延</div>" << latency_chart << "</div>"
         // 三个 JSON 文本面板，由下方 JS 在加载时填充
         << "</section><section class=\"grid\">"
         << "<div class=\"card\"><div class=\"pill\">Source</div><div class=\"panel\" id=\"source-panel\"></div></div>"
         << "<div class=\"card\"><div class=\"pill\">Drift Monitoring</div><div class=\"panel\" id=\"drift-panel\"></div></div>"
         << "<div class=\"card\"><div class=\"pill\">Failure Cases</div><div class=\"panel\" id=\"failure-panel\"></div></div>"
         // 把整个 payload 嵌入到 script tag，前端用 JSON.parse 读取
         << "</section><script id=\"demo-data\" type=\"application/json\">" << embedded << "</script><script>"
         << "const data=JSON.parse(document.getElementById('demo-data').textContent);"
         << "document.getElementById('source-panel').textContent=JSON.stringify(data.source,null,2);"
         << "document.getElementById('drift-panel').textContent=JSON.stringify(data.drift_monitoring,null,2);"
         << "document.getElementById('failure-panel').textContent=JSON.stringify(data.failure_cases,null,2);"
         << "</script></main></body></html>";
    return html.str();
}

/**
 * @brief 从完整的 demo payload 中提取一个轻量摘要（便于上层 API 返回）。
 *
 * 仅保留 dataset_summary、metrics、最新热点列表与告警计数四个关键字段，
 * 适合用于 REST API 的"概览"端点。
 *
 * @param results 完整的 demo payload。
 * @return Json   摘要对象。
 */
Json build_summary_payload(const Json& results) {
    return Json::Object{
        {"dataset_summary", results.at("dataset_summary")},
        {"metrics", results.at("metrics")},
        {"latest_hotspots", results.at("hotspots")},
        {"alert_count", static_cast<int>(results.at("alerts").array_items().size())},
    };
}

