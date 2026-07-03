# TODO: 完整逼真演示资产

当前仓库只提交轻量默认演示资产，避免 GitHub 单文件 100MB 限制。

需要完整工地/城市固定站仿真资产时，在本地执行：

```bash
./bin/lidar_build_demo_assets --config configs/field_scanning_lidar.json --output-root .
```

需要完整走航车仿真资产时，在本地执行：

```bash
./bin/lidar_build_demo_assets --config configs/mobile_mapping_lidar.json --output-root .
```

完整资产会覆盖：

- `data/raw/simulated_demo_campaign.json`
- `data/l1/demo_preprocessed.json`
- `data/l2/demo_results.json`
- `data/vendor/device_product_schema.json`
- `web/demo_dashboard.html`

注意：`field_scanning_lidar.json` 生成的 L1 JSON 可能超过 100MB，不应直接提交到普通 Git 仓库。若要长期保存完整资产，应改用 Git LFS、对象存储或发布包。
