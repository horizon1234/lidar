# Atmospheric Lidar Pollution

这个仓库现在包含一套最小可运行的颗粒物大气激光雷达算法骨架，主链已经同步落了一版 C++ 实现，覆盖以下链路：

- 回波模拟
- L0 到 L2 预处理
- Fernald 近似反演
- 湿度修正
- PM2.5 和 PM10 标定
- 热点检测
- ENU 坐标映射
- 评测、消融和敏感性分析
- 静态 Demo 资产生成

## C++ 主入口

C++ 代码位于 `cpp/`，构建入口位于仓库根目录的 `CMakeLists.txt`。

如果本机有 CMake 和 C++20 编译器，可以使用下面的命令：

```powershell
cmake -S . -B build
cmake --build build --config Release
```

常用入口：

```powershell
build/lidar_build_demo_assets --config configs/default_pipeline.json --output-root .
build/lidar_run_batch --config configs/default_pipeline.json --output .
build/lidar_api_server --config configs/default_pipeline.json --once
build/lidar_api_server --config configs/default_pipeline.json --host 127.0.0.1 --port 8765
build/lidar_fetch_public_ground_data --output-root .
build/lidar_fetch_cloudnet_public_sample --config configs/cloudnet_hybrid_pipeline.json --output-root .
```

说明：当前 C++ 端已覆盖默认 simulation 主链、批处理、静态 Demo、live HTTP API、Open-Meteo 公开样例抓取，以及基于 NetCDF 的 Cloudnet hybrid 本地读取路径。要跑 `configs/cloudnet_hybrid_pipeline.json`，需要在编译时链接 NetCDF；在 Windows C++ 构建下，可以先用 `lidar_fetch_cloudnet_public_sample` 抓取样例，也可以让 Cloudnet loader 在缺文件时自动下载并生成对齐后的 Open-Meteo 资产。Python 抓取脚本和 Python API 仍保留作参考实现。

## Python 参考入口

1. 生成样例数据、L1 和 L2 结果、静态 Demo 页面：

```powershell
python scripts/build_demo_assets.py
```

1. 运行单元测试：

```powershell
python -m unittest discover tests -v
```

1. 启动参考版 Python API：

```powershell
python services/api/server.py --once
```

## 主要目录

- `cpp/` C++ 版本算法主链和入口程序
- `configs/` 默认配置
- `lidar_core/` 算法主链
- `services/` API 和批处理入口
- `scripts/` 复现实验和 Demo 资产脚本
- `tests/` 单元测试

## 补充文档

- `docs/project_supplement.md` 六项补充内容的正式落地说明
- `docs/12_week_plan.md` 12 周落地计划
- `docs/resume_project_entry.md` 简历项目版本
- `data/public/README.md` 公开数据资产说明
