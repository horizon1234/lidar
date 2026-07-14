# ============================================================================
# Makefile —— 基于 cmake 的便捷编译入口
#
# 用法：
#   make              编译全部（首次会自动 cmake 配置）
#   make server       只编译 lidar_sim_server
#   make client       只编译 Linux Qt 客户端 lidar_gui
#   make protocol     只编译 lidar_protocol 静态库
#   make core         只编译 lidar_core（接口库）
#   make tests        编译测试目标
#   make run-server   编译 server 后启动（端口 19850）
#   make run-client   编译 client 后启动
#   make test         运行 ctest
#   make clean        清理 build/
#   make pristine     清理 build/ 和 bin/
# ============================================================================

# 构建/输出目录
BUILD_DIR   := build
BIN_DIR     := bin
# 并行编译核数（取 CPU 核数，最多 16）
JOBS        := $(shell nproc 2>/dev/null || echo 4)
JOBS        := $(shell echo "$(JOBS) > 16" | bc 2>/dev/null || echo "$(JOBS)")
# cmake 的生成器（make -jN）
CMAKE_BUILD := cmake --build $(BUILD_DIR) -j$(JOBS)

# 所有可执行文件列表（与 bin/ 中的产物对应）
EXES := \
	lidar_sim_server        \
	lidar_gui               \
	lidar_run_full_demo     \
	lidar_pipeline_test     \
	lidar_closed_loop_test  \
	lidar_example_1d_ray    \
	lidar_example_2d_ppi    \
	lidar_example_3d_rhi    \
	lidar_run_batch         \
	lidar_build_demo_assets \
	lidar_api_server        \
	lidar_fetch_public_ground_data \
	lidar_fetch_cloudnet_public_sample

# ---- 首次配置：如果 build/ 不存在或 CMakeCache.txt 缺失则自动配置 ----
.PHONY: config
config:
	@if [ ! -f $(BUILD_DIR)/CMakeCache.txt ]; then \
		echo "[config] 首次运行，执行 cmake 配置 ..."; \
		cmake -S . -B $(BUILD_DIR); \
	else \
		echo "[config] build/ 已配置，跳过"; \
	fi

# ============================================================================
# 编译目标
# ============================================================================

# 默认目标：编译全部
.PHONY: all
all: config
	$(CMAKE_BUILD)

# --- 子项目目标（各自的可执行文件 + 依赖的静态库）---

.PHONY: server
server: config
	$(CMAKE_BUILD) --target lidar_sim_server
	@echo "✓ server 已编译 → $(BIN_DIR)/lidar_sim_server"

.PHONY: client
client: config
	$(CMAKE_BUILD) --target lidar_gui
	@echo "✓ Linux Qt client 已编译 → $(BIN_DIR)/lidar_gui"

.PHONY: protocol
protocol: config
	$(CMAKE_BUILD) --target lidar_protocol
	@echo "✓ lidar_protocol 静态库已编译"

.PHONY: core
core: config
	$(CMAKE_BUILD) --target lidar_core
	@echo "✓ lidar_core (interface) 已就绪"

.PHONY: examples
examples: config
	$(CMAKE_BUILD) --target lidar_example_1d_ray lidar_example_2d_ppi lidar_example_3d_rhi
	@echo "✓ 教学算例已编译 → $(BIN_DIR)/"

.PHONY: apps
apps: config
	$(CMAKE_BUILD) --target lidar_run_batch lidar_build_demo_assets lidar_api_server
	@echo "✓ 工具程序已编译 → $(BIN_DIR)/"

# ============================================================================
# 测试
# ============================================================================

.PHONY: tests test
tests test: all
	cd $(BUILD_DIR) && ctest --output-on-failure

# ============================================================================
# 运行（先编译再启动）
# ============================================================================

.PHONY: run-server
run-server: server
	./$(BIN_DIR)/lidar_sim_server

.PHONY: run-client
run-client: client
	./$(BIN_DIR)/lidar_gui

# ============================================================================
# 清理
# ============================================================================

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	@echo "✓ build/ 已清理"

.PHONY: pristine
pristine:
	rm -rf $(BUILD_DIR) $(BIN_DIR)
	@echo "✓ build/ 和 bin/ 已清理"

# 列出所有可编译目标
.PHONY: list
list:
	@echo "可用目标："
	@echo "  all          编译全部"
	@echo "  server       只编译仿真服务器 lidar_sim_server"
	@echo "  client       只编译 Linux Qt 主控客户端 lidar_gui"
	@echo "  protocol     只编译协议库"
	@echo "  core         只编译核心库"
	@echo "  examples     编译第 19 章教学算例"
	@echo "  apps         编译工具程序"
	@echo "  tests/test   编译并运行测试"
	@echo "  run-server   编译并启动服务器"
	@echo "  run-client   编译并启动客户端"
	@echo "  clean        清理 build/"
	@echo "  pristine     清理 build/ 和 bin/"
