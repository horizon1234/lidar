# 公开数据资产

当前仓库的公开数据资产分成两类：

1. 直接可抓取的地面 PM / 气象样例
2. 已登记入口、后续可扩展接入的公开 LiDAR 数据源

## 已实现样例

运行下面任一入口都可以抓取北京区域的小时级 PM2.5、PM10、温度、湿度、风速、风向：

```powershell
python scripts/fetch_public_ground_data.py
build/lidar_fetch_public_ground_data --output-root .
```

输出文件：

- `data/public/open_meteo_beijing_ground_pm_meteo.json`
- `data/public/open_meteo_beijing_ground_pm_meteo.csv`
- `data/public/open_meteo_beijing_fetch_manifest.json`

用途：

- 演示真实公开来源的地面 PM / 气象对齐
- 给 PM 标定和数据资产部分提供一个可复现样例

Cloudnet Bucharest 公开样例也已有 C++ 抓取入口：

```powershell
build/lidar_fetch_cloudnet_public_sample --config configs/CloudnetHybridPipeline.json --output-root .
```

对应输出会落到 `data/public/cloudnet/`，包括 `.nc` 原始文件、Open-Meteo 原始 JSON、对齐后的 ground JSON/CSV，以及 manifest。

## 已登记数据源

见 `data/public/public_dataset_registry.json`。

当前重点保留两个来源：

- Open-Meteo：小时级地面 PM / 气象样例
- Cloudnet Data Portal：可公开检索的大气遥感廓线产品入口

## 对齐建议

真实项目接入公开 LiDAR 数据时，建议按下面顺序做：

1. 先把 LiDAR profile 时间轴规整到固定时间栅格
2. 再把地面 PM / 气象做最近邻或窗口聚合
3. 保留缺测标志，不要盲目插值
4. 单独存站点元数据和质量标志
