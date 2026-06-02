# 项目补充说明

本文把当前仓库从“方案文档”补到“可投中高级算法工程师岗位的项目骨架”。重点不是 UI，而是能回答算法面试中的六类问题：算法主链、数据资产、评测、工程化、演示闭环和中高级深度。当前仓库在保留 Python 参考实现的同时，已经补了一版 C++ 主链骨架，便于往中高级算法工程和工程化落地方向继续推进。

## 1. 算法主链

当前最小可运行算法链已经落地在代码里，对应关系如下：

| 模块 | 代码位置 | 当前能力 |
| --- | --- | --- |
| 回波模拟 | `lidar_core/simulation/scene.py` | 生成固定点 stare 和水平 PPI 扫描，输出原始回波、真值 backscatter/extinction、真值 PM 和热点 mask |
| L0 -> L1 预处理 | `lidar_core/preprocessing/pipeline.py` | 背景扣除、发射能量归一化、overlap 校正、RCS 风格的距离平方补偿、SNR 和 QC flag |
| Fernald 近似反演 | `lidar_core/retrieval/fernald.py` | 基于参考区和固定 lidar ratio 的后向积分，恢复 extinction 与 aerosol backscatter |
| 湿度修正 | `lidar_core/calibration/humidity.py` | 用吸湿增长因子把湿态 extinction 拉回 dry extinction |
| PM2.5 / PM10 标定 | `lidar_core/calibration/pm.py` | 基于对齐后的地面 PM 与气象观测做线性标定，并在 bin 级别加入局部异常增强 |
| 热点检测 | `lidar_core/detection/hotspots.py` | 绝对 PM 阈值 + 扫描内相对异常联合判定，之后做连通域、面积、质心和等级计算 |
| ENU 坐标映射 | `lidar_core/geometry/enu.py` | 把极坐标 range/azimuth/elevation 映射到 ENU 三维坐标 |
| 端到端编排 | `lidar_core/pipeline.py` | 串联仿真、预处理、反演、标定、热点、评测、消融、敏感性和 Demo 输出 |

当前默认配置下，端到端输出包含：

- L0：模拟原始回波与元数据
- L1：背景扣除后的信号、attenuated backscatter、SNR、QC flag
- L2：extinction、dry extinction、PM2.5、PM10、热点事件、ENU 三维位置
- Demo：时间高度图、PPI 热点、质量控制指标、告警和分析表

## 2. 数据资产

### 2.1 可控模拟数据

模拟数据由 `scripts/build_demo_assets.py` 触发生成，输出到以下目录：

- `data/raw/simulated_demo_campaign.json`
- `data/l1/demo_preprocessed.json`
- `data/l2/demo_results.json`

默认数据规模：

- 站点：1 个固定站点，北京示例站
- 时间步：18 个时间步
- 采样间隔：20 分钟
- 扫描模式：每个时间步 1 条 stare + 12 条 PPI 射线
- 距离 bin：30 个，分辨率 50 m
- 总 profile 数：234

模拟场景包含：

- 近地边界层背景气溶胶
- 时间变化的高架污染层
- 两个局部 plume 源
- 湿度、温度、风速风向的同步变化
- 可直接用来做热点真值评测的 `true_hotspot_mask`

### 2.2 公开数据

公开数据资产分成两类：

1. 已落地到仓库的地面 PM + 气象样例
2. 已登记到仓库的数据源清单

对应文件：

- `data/public/public_dataset_registry.json`
- `data/public/README.md`
- `scripts/fetch_public_ground_data.py`
- `cpp/apps/fetch_public_ground_data.cpp`
- `cpp/apps/fetch_cloudnet_public_sample.cpp`

当前公开样例策略：

- 用 Open-Meteo 抓取北京区域的小时级 PM2.5、PM10、温度、湿度、风速、风向
- 把结果写成一份原始 JSON 和一份对齐后的 CSV
- 作为地面观测对齐样例和 Demo 数据资产

对公开激光雷达数据，当前仓库先登记入口而不是内置大文件：

- Cloudnet 数据门户，可下载公开的地基遥感廓线数据和可视化结果
- 该类数据适合用来替换 `lidar_core/simulation/scene.py` 的模拟输入，验证真实 backscatter / extinction 处理链

### 2.3 对齐方式

当前对齐粒度是“按时间戳对齐的地面 PM 与气象”。后续接真实 LiDAR 数据时建议按下面方式扩展：

1. LiDAR 使用 profile 时间戳或 scan 时间戳为主键
2. 地面 PM 和气象按最近邻或固定窗口聚合到同一时间栅格
3. 若真实设备时钟偏移明显，先做时间漂移校正
4. 若存在多个地面站，保留站点 ID 与距离权重

### 2.4 训练集 / 验证集 / 测试集划分

当前离线基线采用交错时间切分：

- 训练集：时间序列中 `index % 5` 落在 `0, 1, 2` 的样本
- 验证集：时间序列中 `index % 5 == 3` 的样本
- 测试集：时间序列中 `index % 5 == 4` 的样本
- 离线指标：使用验证集和测试集合并后的 holdout 结果，避免小样本下单一子集方差过低

这种切分比简单地切最后几段更合理，因为当前数据是带周期变化的时序样本。如果完全按尾段切分，容易得到方差过小的测试集，导致 $R^2$ 被严重低估。

### 2.5 事件定义

当前热点事件定义为：

1. 单元级别满足“绝对 PM 阈值”或“扫描内相对异常阈值”
2. 相邻单元形成连通域
3. 连通域大小达到 `min_cells`
4. 输出热点质心、峰值、面积和等级

这套定义更接近真实业务，因为热点在很多场景下不是绝对浓度极高，而是相对局部背景显著抬升。

## 3. 评测指标

当前仓库已经内置以下评测：

- PM2.5：MAE、RMSE、$R^2$
- PM10：MAE、RMSE、$R^2$
- 热点检测：Precision、Recall、F1
- 反演稳定性：不同 lidar ratio 下 PM RMSE 的跨度
- 工程性能：平均单 profile 时延、profiles/s 吞吐量

默认固定随机种子下，当前离线 baseline 为：

| 指标 | 数值 |
| --- | --- |
| PM2.5 RMSE | 2.555 |
| PM2.5 $R^2$ | 0.819 |
| PM10 RMSE | 2.863 |
| PM10 $R^2$ | 0.892 |
| Hotspot Precision | 0.628 |
| Hotspot Recall | 0.754 |
| Hotspot F1 | 0.685 |
| 平均单 profile 时延 | 0.122 ms |
| 吞吐量 | 4721 profiles/s |

这些数字是 Demo baseline，不应直接包装成真实设备线上指标。简历上应写成“模拟和样例数据上的离线基线”，后续再用真实 LiDAR 数据替换。

## 4. 工程化

### 4.1 当前目录落地情况

| 目标 | 当前实现 |
| --- | --- |
| C++ 主链 | `cpp/` + `CMakeLists.txt`，覆盖 simulation 主链、Cloudnet hybrid 本地读取、batch、demo、live HTTP API 和 `--once` 摘要输出 |
| 算法模块 | `lidar_core/` |
| 配置 | `configs/default_pipeline.json` |
| API | `services/api/server.py` + `cpp/apps/api_server.cpp` |
| 任务调度 / 批处理 | `services/workers/run_batch.py` |
| 可复现实验脚本 | `scripts/build_demo_assets.py`、`scripts/fetch_public_ground_data.py`、`cpp/apps/fetch_public_ground_data.cpp`、`cpp/apps/fetch_cloudnet_public_sample.cpp` |
| 测试 | `tests/test_pipeline.py` |
| 公开数据资产说明 | `data/public/README.md` |
| 实验记录模板 | `experiments/experiment_log_template.md` |

### 4.2 当前工程闭环

当前已经具备一个最小闭环：

1. 生成模拟 LiDAR campaign
2. 运行 Python 或 C++ 的 L0 -> L2 处理链
3. 输出评测指标、漂移监控和热点事件
4. 生成静态 Demo 页面
5. 通过 Python 或 C++ 的 HTTP API 读取摘要结果，也可以用 C++ `--once` 输出摘要 JSON
6. 用单元测试验证 Python 基线结构，并保留 C++ 断言测试入口

### 4.3 当前仍缺的工程项

如果要进一步变成真实交付项目，下一步应该补：

- 地面站多源对齐服务
- 持久化数据库和实验追踪
- 流式处理或消息总线
- 更完整的配置分层
- CI、代码格式化和类型检查
- 真正的历史回放和用户权限

## 5. Demo 闭环

当前 Demo 已经定义并生成以下页面：

- 实时总览：时间高度图 + 关键指标卡片
- PPI 与热点：二维平面热点定位 + 事件表
- 质量控制：SNR、处理时延、背景光、激光能量波动
- 历史回放：数据切分、消融、敏感性、历史告警

生成方式：

```powershell
python scripts/build_demo_assets.py
```

或使用 C++ 入口：

```powershell
build/lidar_build_demo_assets --config configs/default_pipeline.json --output-root .
```

输出文件：

- `web/demo_dashboard.html`
- `data/l2/demo_results.json`

API 摘要：

```powershell
python services/api/server.py --once
```

对应的 C++ live API / 摘要输出：

```powershell
build/lidar_api_server --config configs/default_pipeline.json --once
build/lidar_api_server --config configs/default_pipeline.json --host 127.0.0.1 --port 8765
```

Cloudnet hybrid 在 C++ 端的调用方式与默认链相同，只需要换配置：

```powershell
build/lidar_fetch_cloudnet_public_sample --config configs/cloudnet_hybrid_pipeline.json --output-root .
build/lidar_run_batch --config configs/cloudnet_hybrid_pipeline.json --output .
build/lidar_api_server --config configs/cloudnet_hybrid_pipeline.json --once
build/lidar_api_server --config configs/cloudnet_hybrid_pipeline.json --host 127.0.0.1 --port 8765
```

北京地面 PM / 气象公开样例现在也可以直接用 C++ 入口抓取：

```powershell
build/lidar_fetch_public_ground_data --output-root .
```

当前这条 C++ 真实数据路径依赖 NetCDF 库；在 Windows C++ 构建下，既可以显式运行 `lidar_fetch_cloudnet_public_sample` 先落地 `data/public/cloudnet/` 里的样例，也可以让 loader 在缺少 `.nc` 或对齐后的 Open-Meteo JSON 时自动补齐。Python 抓取脚本仍然保留作参考和兜底。

对 Cloudnet hybrid 真实样例，当前已把热点阈值调到更保守的配置：`pm25_threshold_ugm3 = 80.0`、`scan_relative_pm25_threshold_ugm3 = 8.0`、`min_cells = 6`。这组参数在当前 Bucharest 单日样例上把热点 F1 从 0.128 提升到 0.176，同时减少了事件级告警数量。

## 6. 中高级层面的深度

当前仓库已经不是单纯“复现公式”，而是把几个中高级面试经常会问到的点提前落到了代码结构里：

### 6.1 不确定度与稳定性

- 用不同 lidar ratio 做敏感性分析
- 用 `retrieval_stability` 记录对结果的影响跨度

### 6.2 消融实验

- `full-pipeline`
- `without-humidity-correction`

下一步可以继续补：

- 不做 overlap 校正
- 不做局部热点增强
- 不做相对异常判定

### 6.3 模型漂移监控

建议后续上线时记录：

- 每日 PM 标定系数漂移
- 不同湿度区间下残差变化
- 风速 / 风向条件变化下的误差偏移
- 热点误报率随季节和工地状态变化的趋势

### 6.4 线上校准策略

建议把 PM 标定拆成两层：

1. 全局基础模型
2. 站点级增量校准

站点级模型可按周或按月滚动更新，但必须保留回滚能力和版本号。

### 6.5 多源融合

当前代码里已经给 PM 标定留出了多源扩展位，后续可以继续接：

- 多地面站
- 风场再分析数据
- 道路 / 工地边界
- 相机或视频的扬尘告警
- 车载扫描轨迹

### 6.6 失败案例分析

后续必须单独建一个失败案例集，至少覆盖：

- 雨雾天气
- 背景光过强
- 低空 overlap 失配
- plume 非常贴地导致局部饱和
- 地面站位置与 plume 不共址

如果你把这部分补齐，面试官会更容易把你看成“能做系统闭环的算法工程师”，而不是只会写一个回归脚本的人。
